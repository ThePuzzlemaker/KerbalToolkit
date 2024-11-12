use std::{
    cmp::Ordering,
    collections::{HashMap, HashSet},
    sync::Arc,
};

use kerbtk::{
    arena::Arena,
    bodies::SolarSystem,
    ffs::{self, FuelFlowSimulation, Resource, ResourceId, SimVessel},
    kepler::orbits::StateVector,
    maneuver::Maneuver,
    time::{GET, UT},
    vessel::{TrackedId, Vessel, VesselClass, VesselClassId, VesselId},
};
use nalgebra::Vector3;
use parking_lot::{RwLock, RwLockReadGuard};
use serde::{Deserialize, Serialize};
use time::Duration;

use crate::{find_sv, DisplaySelect};

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Mission {
    pub system: Arc<SolarSystem>,
    pub classes: Arena<VesselClassId, VesselClass>,
    pub vessels: Arena<VesselId, Vessel>,
    #[serde(default)]
    pub plan: HashMap<VesselId, MissionPlan>,
    #[serde(skip)]
    pub was_replaced: bool,
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct MissionPlan {
    pub events: Vec<PlannedEvent>,
    pub anchor_vector_slot: String,
    pub sim_vessel: Option<SimVessel>,
    pub nodal_targets: Option<NodalTargets>,
}

impl Mission {
    pub fn state_at(
        &self,
        vessel_id: VesselId,
        get: GET,
        calculate_fuel: bool,
        collocation: Collocation,
        mut track_updates: impl FnMut(GET, &VesselState),
    ) -> Option<VesselState> {
        let vessel = &self.vessels[vessel_id];
        let plan = self.plan.get(&vessel_id)?;
        let class_id = vessel.class?;
        let class = &self.classes[class_id];

        let mut resources: Resources = HashMap::new();
        for ((pid, rid), res) in &vessel.resources {
            let tid = class.parts[*pid].tracked_id;
            resources
                .entry((tid, vessel_id))
                .or_default()
                .insert(*rid, res.clone());
        }

        let sim_vessel = plan.sim_vessel.clone()?;
        let sv = find_sv(Some(vessel), &plan.anchor_vector_slot).ok()?;
        let mut state = VesselState {
            get: GET::from_duration(sv.time - vessel.get_base),
            sv,
            class: vessel.class?,
            resources,
            fuel_stats: CompactFuelStats {
                // no engines set, ΔVrem is 0
                dvrem: 0.0,
                start_mass: sim_vessel.mass,
                end_mass: sim_vessel.mass,
                bt: 0.0,
            },
            sim_vessel,
        };

        let ix = plan.events.binary_search_by_key(&get, |evt| evt.get());
        let mut excess = Duration::ZERO;
        let range = match (ix, collocation) {
            (Err(ix), _) => {
                if ix > 0 {
                    excess = get - plan.events[ix - 1].get();
                }
                ..ix
            }
            (Ok(ix), Collocation::BeforeEvent) => ..ix,
            (Ok(ix), Collocation::AfterEvent) => ..(ix + 1),
        };
        for evt in &plan.events[range] {
            if evt.get() < state.get {
                // TODO: warn or something?
                continue;
            }

            match evt {
                PlannedEvent::Maneuver(mnv) => {
                    state = mnv.propagate_state(state, calculate_fuel, self, vessel_id)?;
                    track_updates(mnv.inner.geti, &state);
                }
                PlannedEvent::Separation(sep) => {
                    state = sep.propagate_state(state, calculate_fuel, self, vessel_id)?;
                    track_updates(sep.get, &state);
                }
            }
        }

        Some(state.propagate(excess, self)?)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Collocation {
    BeforeEvent,
    AfterEvent,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VesselState {
    pub get: GET,
    pub sv: StateVector,
    pub class: VesselClassId,
    pub resources: Resources,
    pub sim_vessel: SimVessel,
    pub fuel_stats: CompactFuelStats,
}

impl VesselState {
    pub fn propagate(mut self, deltat: Duration, mission: &Mission) -> Option<Self> {
        self.get = self.get + deltat;
        self.sv = self
            .sv
            .propagate_with_soi(&mission.system, deltat, 1e-7, 30000)?;
        Some(self)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum NodalTargets {
    Translunar {
        soi_ut: UT,
        lat_pe: f64,
        lng_pe: f64,
        h_pe: f64,
        i: f64,
        // sv_soi: StateVector,
        mnv_base_code: String,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum PlannedEvent {
    Maneuver(PlannedManeuver),
    Separation(PlannedSeparation),
}

impl PlannedEvent {
    pub fn mnv_ref(&self) -> Option<&PlannedManeuver> {
        let Self::Maneuver(mnv) = self else {
            return None;
        };
        Some(mnv)
    }

    pub fn sep_ref(&self) -> Option<&PlannedSeparation> {
        let Self::Separation(sep) = self else {
            return None;
        };
        Some(sep)
    }

    pub fn get(&self) -> GET {
        match self {
            PlannedEvent::Maneuver(mnv) => mnv.inner.geti,
            PlannedEvent::Separation(sep) => sep.get,
        }
    }

    pub fn resources(&self) -> Option<&Resources> {
        match self {
            PlannedEvent::Maneuver(mnv) => Some(&mnv.resources),
            PlannedEvent::Separation(sep) => Some(&sep.resources),
        }
    }

    pub fn source(&self) -> DisplaySelect {
        match self {
            PlannedEvent::Maneuver(mnv) => mnv.source,
            PlannedEvent::Separation(sep) => sep.source,
        }
    }
}

pub type Resources = HashMap<(TrackedId, VesselId), HashMap<ResourceId, Resource>>;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlannedSeparation {
    pub get: GET,
    pub new_vessel: VesselId,
    pub new_class: VesselClassId,
    pub new_sim: SimVessel,
    pub resources: Resources,
    pub source: DisplaySelect,
    pub fuel_stats: CompactFuelStats,
    pub engines: HashSet<(TrackedId, VesselId)>,
}

pub fn run_ffs(
    prev_resources: Option<Resources>,
    engines: HashSet<(TrackedId, VesselId)>,
    vessel_id: VesselId,
    class: &VesselClass,
    mut sim_vessel: SimVessel,
    deltav: Vector3<f64>,
) -> (CompactFuelStats, Resources) {
    if let Some(prev_resources) = prev_resources {
        for ((tid, vid), resources) in prev_resources {
            if let Some((_, part)) = sim_vessel
                .parts
                .iter_mut()
                .find(|x| x.1.tracked_id == tid && x.1.on_vessel == vid)
            {
                part.resources = resources;
            }
        }
    }

    for (_, part) in class.parts.iter() {
        if engines.contains(&(part.tracked_id, vessel_id)) {
            let sid = sim_vessel
                .parts
                .iter()
                .find_map(|(i, x)| (x.tracked_id == part.tracked_id).then_some(i))
                .expect("oops");
            sim_vessel
                .active_engines
                .extend(part.engines.iter().cloned().map(|mut x| {
                    x.part = sid;
                    x
                }));
        }
    }

    let mut ffs = FuelFlowSimulation::default();

    ffs.run(&mut sim_vessel, Some(deltav.norm() * 1000.0), false);

    let resources = sim_vessel
        .parts
        .iter()
        .map(|(_, x)| ((x.tracked_id, x.on_vessel), x.resources.clone()))
        .collect::<HashMap<_, _>>();
    ffs.run(&mut sim_vessel, None, true);
    tracing::trace!("{ffs:#?}");
    //ffs.run(&mut sim_vessel, None);

    let mut deltav_ach = deltav.norm() * 1000.0;
    let mut bt = 0.0;
    let start_mass_tons = ffs.segments[0].start_mass;
    let mut end_mass = ffs.segments[0].start_mass * 1000.0;
    for segment in &ffs.segments {
        match segment.deltav.total_cmp(&deltav_ach) {
            Ordering::Less | Ordering::Equal => {
                bt += segment.delta_time;
                deltav_ach -= segment.deltav;
                end_mass = segment.end_mass * 1000.0;
            }
            Ordering::Greater => {
                // Calculate end mass with Tsiolkovsky rocket equation

                let start_mass = end_mass;
                let exhvel = segment.isp * ffs::G0;
                let alpha = libm::exp(-deltav_ach / exhvel);
                end_mass = start_mass * alpha;

                bt += (start_mass * exhvel) / (1000.0 * segment.thrust) * (1.0 - alpha);
                deltav_ach = 0.0;
                break;
            }
        }
    }

    let dvrem = if deltav_ach > 1e-6 {
        -deltav_ach
    } else {
        ffs.segments
            .iter()
            .fold(-deltav.norm() * 1000.0, |acc, x| acc + x.deltav)
    };

    (
        CompactFuelStats {
            dvrem,
            bt,
            start_mass: start_mass_tons,
            end_mass: end_mass / 1000.0,
        },
        resources,
    )
}

impl PlannedSeparation {
    pub fn propagate_state(
        &self,
        state: VesselState,
        calculate_fuel: bool,
        mission: &Mission,
        vessel_id: VesselId,
    ) -> Option<VesselState> {
        let get = self.get;
        let ut = mission.vessels[vessel_id].get_base + get.into_duration();
        let sv = if (ut.into_duration().as_seconds_f64()
            - state.sv.time.into_duration().as_seconds_f64())
        .abs()
            < 1e-3
        {
            state.sv
        } else {
            state
                .sv
                .propagate_with_soi(&mission.system, get - state.get, 1e-7, 30000)?
        };

        let class = self.new_class;
        let mut sim_vessel = self.new_sim.clone();
        let mut resources = state.resources.clone();

        let set = mission.classes[class]
            .parts
            .iter()
            .map(|(_, p)| p.tracked_id)
            .collect::<HashSet<_>>();
        resources.retain(|(tid, _), _| set.contains(tid));
        sim_vessel.active_engines.retain(|e| {
            let tid = sim_vessel.parts[e.part].tracked_id;
            set.contains(&tid)
        });
        let mut removed_parts = HashSet::new();
        sim_vessel.parts.retain(|pid, p| {
            let tid = p.tracked_id;
            if set.contains(&tid) {
                true
            } else {
                removed_parts.insert(pid);
                false
            }
        });
        for (_, part) in sim_vessel.parts.iter_mut() {
            part.crossfeed_part_set
                .retain(|pid| !removed_parts.contains(pid));
        }

        let (fuel_stats, resources) = if calculate_fuel {
            run_ffs(
                Some(resources),
                self.engines.clone(),
                vessel_id,
                &mission.classes[class],
                sim_vessel.clone(),
                Vector3::zeros(),
            )
        } else {
            (self.fuel_stats, resources)
        };

        Some(VesselState {
            get,
            sv,
            class,
            resources,
            sim_vessel,
            fuel_stats,
        })
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PlannedManeuver {
    pub inner: Maneuver,
    pub engines: HashSet<(TrackedId, VesselId)>,
    pub resources: HashMap<(TrackedId, VesselId), HashMap<ResourceId, Resource>>,
    pub fuel_stats: CompactFuelStats,
    pub source: DisplaySelect,
}

impl PlannedManeuver {
    pub fn propagate_state(
        &self,
        state: VesselState,
        calculate_fuel: bool,
        mission: &Mission,
        vessel_id: VesselId,
    ) -> Option<VesselState> {
        let get = self.inner.geti;
        let ut = mission.vessels[vessel_id].get_base + get.into_duration();
        let mut sv = if (ut.into_duration().as_seconds_f64()
            - state.sv.time.into_duration().as_seconds_f64())
        .abs()
            < 1e-3
        {
            state.sv
        } else {
            state
                .sv
                .propagate_with_soi(&mission.system, get - state.get, 1e-7, 30000)?
        };
        sv.velocity += self.inner.deltav_bci();
        let vessel = &mission.vessels[vessel_id];
        let class = vessel.class?;
        let sim_vessel = state.sim_vessel.clone();

        let (fuel_stats, resources) = if calculate_fuel {
            run_ffs(
                Some(state.resources.clone()),
                self.engines.clone(),
                vessel_id,
                &mission.classes[class],
                sim_vessel.clone(),
                self.inner.deltav_bci(),
            )
        } else {
            (self.fuel_stats, self.resources.clone())
        };

        Some(VesselState {
            get,
            sv,
            class,
            resources,
            sim_vessel,
            fuel_stats,
        })
    }
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
pub struct CompactFuelStats {
    /// ΔV remaining (m/s)
    pub dvrem: f64,
    /// Start mass (tons)
    pub start_mass: f64,
    /// End mass (tons)
    pub end_mass: f64,
    /// Burn time (s)
    pub bt: f64,
}

#[derive(Clone, Debug)]
pub struct MissionRef(pub(crate) Arc<RwLock<Mission>>);
impl MissionRef {
    pub fn new(mission: Arc<RwLock<Mission>>) -> Self {
        Self(mission)
    }
    pub fn read(&'_ self) -> RwLockReadGuard<'_, Mission> {
        self.0.read()
    }
}
