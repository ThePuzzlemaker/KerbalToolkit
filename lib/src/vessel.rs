#![allow(clippy::type_complexity)]
use std::collections::{HashMap, HashSet};

use color_eyre::eyre::{self, OptionExt};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::{
    arena::{Arena, IdLike},
    ffs::{Engine, FlowMode, Propellant, Resource, ResourceId, SimPartId},
    kepler::orbits::StateVector,
    krpc::{self, Client, ModifierChangeWhen, SpaceCenter},
    time::UT,
};

#[derive(
    Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
#[repr(transparent)]
pub struct VesselClassId(u64);

impl IdLike for VesselClassId {
    fn from_raw(index: usize) -> Self {
        Self(index as u64)
    }

    fn into_raw(self) -> usize {
        self.0 as usize
    }
}

#[derive(
    Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize,
)]
#[repr(transparent)]
pub struct VesselId(u64);

impl IdLike for VesselId {
    fn from_raw(index: usize) -> Self {
        Self(index as u64)
    }

    fn into_raw(self) -> usize {
        self.0 as usize
    }
}

#[derive(Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Vessel {
    pub name: String,
    pub description: String,
    #[serde(skip)]
    pub link: Option<krpc::Vessel>,
    pub class: Option<VesselClassId>,
    pub resources: HashMap<(PartId, ResourceId), Resource>,
    #[serde(default)]
    pub tracked_persistent_id_map: HashMap<TrackedId, PersistentId>,
    #[serde(default)]
    pub svs: HashMap<String, StateVector>,
    pub get_base: UT,
}

#[derive(Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct VesselClass {
    pub name: String,
    pub description: String,
    pub shortcode: String,
    pub parts: Arena<PartId, Part>,
    pub tracked_id_map: HashMap<TrackedId, PartId>,
    pub root: Option<PartId>,
}

impl VesselClass {
    pub fn load_parts_from_editor(
        client: &mut Client,
    ) -> eyre::Result<(
        Arena<PartId, Part>,
        Option<PartId>,
        HashMap<TrackedId, PartId>,
    )> {
        let mut map = HashMap::new();
        let mut parts = Arena::new();
        let mut tracked_id_map = HashMap::new();
        let mut root = None;

        let mut sc = client.space_center();
        for part in sc
            .get_editor()?
            .get_current_ship(&mut sc)?
            // TODO: i18n
            .ok_or_eyre("No ship in editor.")?
            .get_parts(&mut sc)?
            .get_all(&mut sc)?
        {
            Self::load_part(
                part,
                &mut sc,
                &mut map,
                &mut parts,
                &mut root,
                &mut tracked_id_map,
            )?;
        }

        Ok((parts, root, tracked_id_map))
    }

    pub fn load_parts_from_flight(
        client: &mut Client,
    ) -> eyre::Result<(
        Arena<PartId, Part>,
        Option<PartId>,
        HashMap<TrackedId, PartId>,
    )> {
        let mut map = HashMap::new();
        let mut parts = Arena::new();
        let mut tracked_id_map = HashMap::new();
        let mut root = None;

        let mut sc = client.space_center();
        for part in sc
            .get_active_vessel()?
            // TODO: i18n
            .ok_or_eyre("No flight in progress.")?
            .get_parts(&mut sc)?
            .get_all(&mut sc)?
        {
            Self::load_part(
                part,
                &mut sc,
                &mut map,
                &mut parts,
                &mut root,
                &mut tracked_id_map,
            )?;
        }

        Ok((parts, root, tracked_id_map))
    }

    fn load_part(
        part: krpc::Part,
        sc: &mut SpaceCenter,
        map: &mut HashMap<krpc::Part, PartId>,
        parts: &mut Arena<PartId, Part>,
        root: &mut Option<PartId>,
        tracked_id_map: &mut HashMap<TrackedId, PartId>,
    ) -> eyre::Result<()> {
        let name = part.get_name(sc)?;
        let title = part.get_title(sc)?;
        let tag = part.get_tag(sc)?;

        let pfdecoupler = part.get_pf_decoupler(sc)?;
        let rodecoupler = part.get_ro_decoupler(sc)?;
        let decoupler = part.get_decoupler(sc)?;
        let decouplers = if pfdecoupler.is_some() {
            Some(Decouplers::ProceduralFairing(ProceduralFairingDecoupler))
        } else if let Some(rodecoupler) = rodecoupler {
            // TODO: make these non-nullable
            let top = rodecoupler.get_top_decoupler(sc)?.unwrap();
            let bot = rodecoupler.get_bottom_decoupler(sc)?.unwrap();
            Some(Decouplers::RODecoupler {
                top: Decoupler {
                    is_omni_decoupler: top.get_is_omni_decoupler(sc)?,
                    attached_part: top.get_attached_part(sc)?.map(|part| {
                        *map.entry(part)
                            .or_insert_with(|| parts.push(Part::default()))
                    }),
                },
                bot: Decoupler {
                    is_omni_decoupler: bot.get_is_omni_decoupler(sc)?,
                    attached_part: bot.get_attached_part(sc)?.map(|part| {
                        *map.entry(part)
                            .or_insert_with(|| parts.push(Part::default()))
                    }),
                },
            })
        } else {
            decoupler
                .map(|decoupler| -> eyre::Result<_> {
                    Ok(Decouplers::Single(Decoupler {
                        is_omni_decoupler: decoupler.get_is_omni_decoupler(sc)?,
                        attached_part: decoupler.get_attached_part(sc)?.map(|part| {
                            *map.entry(part)
                                .or_insert_with(|| parts.push(Part::default()))
                        }),
                    }))
                })
                .transpose()?
        };

        let children = part
            .get_children(sc)?
            .into_iter()
            .map(|child| {
                *map.entry(child)
                    .or_insert_with(|| parts.push(Part::default()))
            })
            .collect();
        let parent = part.get_parent(sc)?.map(|parent| {
            *map.entry(parent)
                .or_insert_with(|| parts.push(Part::default()))
        });

        let attachment = if part.get_radially_attached(sc)? {
            Attachment::Radial
        } else if part.get_axially_attached(sc)? {
            Attachment::Axial
        } else {
            Attachment::None
        };

        let crossfeed_part_set = part
            .get_crossfeed_part_set(sc)?
            .into_iter()
            .map(|part| {
                *map.entry(part)
                    .or_insert_with(|| parts.push(Part::default()))
            })
            .collect();

        let (mass, dry_mass, crew_mass) = part.get_part_masses(sc)?;

        let params_curve = part.get_engine_params_curve(sc)?;
        let params_bool = part.get_engine_params_bool(sc)?;
        let params_f64 = part.get_engine_params_f64(sc)?;
        let ttm = part.get_engine_thrust_transform_multipliers(sc)?;
        let ttv = part.get_engine_thrust_transforms(sc)?;
        let prop = part.get_engine_propellants(sc)?;
        let mut engines = vec![];

        // TODO: engine names for display
        for engine_id in params_curve.keys() {
            let propellants = prop
                .get(engine_id)
                .unwrap()
                .iter()
                .map(|(id, ignore_for_isp, ratio, flow_mode, density)| {
                    (
                        ResourceId(*id),
                        Propellant {
                            ignore_for_isp: *ignore_for_isp,
                            ratio: *ratio as f64,
                            flow_mode: FlowMode::from(*flow_mode),
                            density: *density as f64,
                        },
                    )
                })
                .collect();

            let params_f64 = params_f64.get(engine_id).unwrap();
            let params_bool = params_bool.get(engine_id).unwrap();
            let params_curve = params_curve.get(engine_id).unwrap();

            let engine = Engine {
                propellants,
                propellant_flow_modes: HashMap::new(),
                resource_consumptions: HashMap::new(),

                thrust_transform_multipliers: ttm
                    .get(engine_id)
                    .unwrap()
                    .iter()
                    .map(|x| *x as f64)
                    .collect(),
                thrust_direction_vectors: ttv
                    .get(engine_id)
                    .unwrap()
                    .iter()
                    .map(|(x, y, z)| Vector3::new(*x, *y, *z))
                    .collect(),

                // TODO
                is_operational: true,
                flow_multiplier: 1.0,

                thrust_current: Vector3::zeros(),
                thrust_max: Vector3::zeros(),
                thrust_min: Vector3::zeros(),
                mass_flow_rate: 0.0,
                isp: 0.0,
                module_residuals: *params_f64.get("moduleResiduals").unwrap(),
                module_spoolup_time: *params_f64.get("moduleSpoolupTime").unwrap(),
                is_module_engines_rf: *params_bool.get("isModuleEnginesRf").unwrap(),
                no_propellants: false,
                is_unrestartable_dead_engine: false,

                g: *params_f64.get("g").unwrap(),
                max_fuel_flow: *params_f64.get("maxFuelFlow").unwrap(),
                max_thrust: *params_f64.get("maxThrust").unwrap(),
                min_fuel_flow: *params_f64.get("minFuelFlow").unwrap(),
                min_thrust: *params_f64.get("minThrust").unwrap(),
                mult_isp: *params_f64.get("multIsp").unwrap(),
                clamp: *params_f64.get("clamp").unwrap(),
                flow_mult_cap: *params_f64.get("flowMultCap").unwrap(),
                flow_mult_cap_sharpness: *params_f64.get("flowMultCapSharpness").unwrap(),
                throttle_limiter: *params_f64.get("throttleLimiter").unwrap(),

                throttle_locked: *params_bool.get("throttleLocked").unwrap(),
                atm_change_flow: *params_bool.get("atmChangeFlow").unwrap(),
                use_atm_curve: *params_bool.get("useAtmCurve").unwrap(),
                use_atm_curve_isp: *params_bool.get("useAtmCurveIsp").unwrap(),
                use_throttle_isp_curve: *params_bool.get("useThrottleIspCurve").unwrap(),
                use_vel_curve: *params_bool.get("useVelCurve").unwrap(),
                use_vel_curve_isp: *params_bool.get("useVelCurveIsp").unwrap(),

                throttle_isp_curve: params_curve
                    .get("throttleIspCurve")
                    .unwrap()
                    .iter()
                    .copied()
                    .collect(),
                throttle_isp_curve_atm_strength: params_curve
                    .get("throttleIspCurveAtmStrength")
                    .unwrap()
                    .iter()
                    .copied()
                    .collect(),
                vel_curve: params_curve
                    .get("velCurve")
                    .unwrap()
                    .iter()
                    .copied()
                    .collect(),
                vel_curve_isp: params_curve
                    .get("velCurveIsp")
                    .unwrap()
                    .iter()
                    .copied()
                    .collect(),
                atm_curve: params_curve
                    .get("atmCurve")
                    .unwrap()
                    .iter()
                    .copied()
                    .collect(),
                atm_curve_isp: params_curve
                    .get("atmCurveIsp")
                    .unwrap()
                    .iter()
                    .copied()
                    .collect(),
                atmosphere_curve: params_curve
                    .get("atmosphereCurve")
                    .unwrap()
                    .iter()
                    .copied()
                    .collect(),

                is_sepratron: false,
                part: SimPartId::from_raw(usize::MAX),
            };
            engines.push(engine);
        }

        let resources = part.get_resources(sc)?;
        let resources = resources.get_all(sc)?;
        let mut part_resources = HashMap::new();
        let mut disabled_resource_mass = 0.0;
        for resource in resources {
            let id = ResourceId(resource.get_id(sc)?);
            let density = resource.get_density(sc)?;
            let amount = resource.get_amount(sc)?;
            part_resources.insert(
                id,
                Resource {
                    free: density <= f32::EPSILON,
                    max_amount: resource.get_max_amount(sc)? as f64,
                    amount: amount as f64,
                    density: density as f64,
                    residual: 0.0,
                    enabled: resource.get_enabled(sc)?,
                    name: resource.get_name(sc)?.into(),
                },
            );
            if !resource.get_enabled(sc)? {
                disabled_resource_mass += amount as f64 * density as f64;
            }
        }

        let mass_modifiers = part
            .get_part_mass_modifiers(sc)?
            .into_iter()
            .map(|x| {
                let changes_when = x.get_module_mass_change_when(sc)?;
                let current_mass =
                    x.get_module_mass(sc, dry_mass as f32, krpc::StagingSituation::Current)? as f64;
                let staged_mass =
                    x.get_module_mass(sc, dry_mass as f32, krpc::StagingSituation::Staged)? as f64;
                let unstaged_mass =
                    x.get_module_mass(sc, dry_mass as f32, krpc::StagingSituation::Unstaged)?
                        as f64;
                Ok(MassModifier {
                    current_mass,
                    staged_mass,
                    unstaged_mass,
                    changes_when,
                    module_name: x.get_name(sc)?,
                })
            })
            .collect::<Result<Vec<_>, eyre::Report>>()?;

        let tracked_id = TrackedId::from_raw(part.get_tracked_id(sc)? as usize);

        let part1 = Part {
            tracked_id,
            parent,
            children,
            name,
            title,
            tag,
            decouplers,
            attachment,
            crossfeed_part_set,
            resources: part_resources,
            resource_priority: part.get_resource_priority(sc)?,
            resource_request_remaining_threshold: part
                .get_resource_request_remaining_threshold(sc)?,
            mass,
            dry_mass,
            crew_mass,
            disabled_resource_mass,
            engines,
            is_launch_clamp: part.get_launch_clamp(sc)?.is_some(),
            mass_modifiers,
        };

        if let Some(id) = map.get(&part) {
            if parent.is_none() {
                *root = Some(*id);
            }
            parts[*id] = part1;
            tracked_id_map.insert(tracked_id, *id);
        } else {
            let id = parts.push(part1);
            if parent.is_none() {
                *root = Some(id);
            }
            map.insert(part, id);

            tracked_id_map.insert(tracked_id, id);
        }

        Ok(())
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Part {
    pub tracked_id: TrackedId,
    pub parent: Option<PartId>,
    pub children: Vec<PartId>,
    pub name: String,
    pub title: String,
    pub tag: String,
    pub decouplers: Option<Decouplers>,
    pub attachment: Attachment,
    pub crossfeed_part_set: Vec<PartId>,
    pub resources: HashMap<ResourceId, Resource>,
    pub resource_priority: i32,
    pub resource_request_remaining_threshold: f64,
    pub mass: f64,
    pub dry_mass: f64,
    pub crew_mass: f64,
    pub disabled_resource_mass: f64,
    pub is_launch_clamp: bool,
    pub engines: Vec<Engine>,
    pub mass_modifiers: Vec<MassModifier>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MassModifier {
    pub current_mass: f64,
    pub staged_mass: f64,
    pub unstaged_mass: f64,
    pub changes_when: ModifierChangeWhen,
    pub module_name: String,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Attachment {
    Radial,
    Axial,
    None,
}

impl Default for Attachment {
    fn default() -> Self {
        Self::None
    }
}

#[derive(
    Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Deserialize, Serialize,
)]
pub struct PartId(u32);

impl IdLike for PartId {
    fn from_raw(index: usize) -> Self {
        Self(index as u32)
    }

    fn into_raw(self) -> usize {
        self.0 as usize
    }
}

#[derive(
    Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Deserialize, Serialize,
)]
pub struct PersistentId(u32);

impl IdLike for PersistentId {
    fn from_raw(index: usize) -> Self {
        Self(index as u32)
    }

    fn into_raw(self) -> usize {
        self.0 as usize
    }
}

#[derive(
    Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Deserialize, Serialize,
)]
pub struct TrackedId(u32);

impl IdLike for TrackedId {
    fn from_raw(index: usize) -> Self {
        Self(index as u32)
    }

    fn into_raw(self) -> usize {
        self.0 as usize
    }
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub enum Decouplers {
    Single(Decoupler),
    RODecoupler { top: Decoupler, bot: Decoupler },
    ProceduralFairing(ProceduralFairingDecoupler),
}

#[derive(Copy, Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct ProceduralFairingDecoupler;

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct Decoupler {
    pub is_omni_decoupler: bool,
    pub attached_part: Option<PartId>,
}

#[allow(clippy::implicit_hasher)]
pub fn decoupled_vessels(
    vessel: &VesselClass,
    fired_decouplers: &[PartId],
    ro_decouplers: &HashMap<PartId, (bool, bool)>,
) -> Vec<HashSet<PartId>> {
    let parts: HashSet<PartId> = vessel.parts.iter().map(|(x, _)| x).collect();
    let mut worklist = vec![parts];
    let mut vessels = vec![];
    let mut visited_decouplers = HashSet::new();
    let fired_decouplers = fired_decouplers
        .iter()
        .copied()
        .chain(ro_decouplers.keys().copied())
        .collect::<Vec<_>>();

    while let Some(subvessel) = worklist.pop() {
        let mut found_decoupler = false;
        for &decoupler in &fired_decouplers {
            if !visited_decouplers.contains(&decoupler) && subvessel.contains(&decoupler) {
                found_decoupler = true;
                visited_decouplers.insert(decoupler);

                match vessel.parts[decoupler].decouplers.as_ref().expect("oops") {
                    Decouplers::Single(d) => {
                        let mut explosive_node = HashSet::new();
                        let mut secondary_node = HashSet::new();

                        if let Some(attached_part) = d.attached_part {
                            traverse_until(
                                vessel,
                                attached_part,
                                &visited_decouplers,
                                &mut explosive_node,
                            );
                        }
                        if let Some(parent) = vessel.parts[decoupler].parent {
                            if Some(parent) != d.attached_part {
                                traverse_until(
                                    vessel,
                                    parent,
                                    &visited_decouplers,
                                    &mut secondary_node,
                                );
                            }
                        }

                        if d.is_omni_decoupler {
                            for &child in &vessel.parts[decoupler].children {
                                if Some(child) == d.attached_part {
                                    continue;
                                }

                                if vessel.parts[child].attachment == Attachment::Radial {
                                    let mut set = HashSet::new();
                                    traverse_until(vessel, child, &visited_decouplers, &mut set);
                                    worklist.push(set);
                                } else {
                                    traverse_until(
                                        vessel,
                                        child,
                                        &visited_decouplers,
                                        &mut secondary_node,
                                    );
                                }
                            }
                            let mut set = HashSet::new();
                            set.insert(decoupler);
                            worklist.push(set);
                        } else {
                            for &child in &vessel.parts[decoupler].children {
                                if Some(child) == d.attached_part {
                                    continue;
                                }

                                traverse_until(
                                    vessel,
                                    child,
                                    &visited_decouplers,
                                    &mut secondary_node,
                                );
                            }
                            secondary_node.insert(decoupler);
                        }

                        worklist.extend([explosive_node, secondary_node]);
                    }
                    Decouplers::RODecoupler { top, bot } => {
                        let (top_en, bot_en) = ro_decouplers
                            .get(&decoupler)
                            .copied()
                            .unwrap_or((false, false));

                        let mut explosive_node = HashSet::new();
                        let mut secondary_node = HashSet::new();

                        let mut omni = false;
                        let attached_part = match (top_en, bot_en) {
                            (true, true) => {
                                omni = true;
                                top.attached_part
                            }
                            (true, false) => top.attached_part,
                            (false, true) => bot.attached_part,
                            (false, false) => None,
                        };

                        if let Some(attached_part) = attached_part {
                            traverse_until(
                                vessel,
                                attached_part,
                                &visited_decouplers,
                                &mut explosive_node,
                            );
                        }
                        if let Some(parent) = vessel.parts[decoupler].parent {
                            if Some(parent) != attached_part {
                                traverse_until(
                                    vessel,
                                    parent,
                                    &visited_decouplers,
                                    &mut secondary_node,
                                );
                            }
                        }

                        if omni {
                            for &child in &vessel.parts[decoupler].children {
                                if Some(child) == attached_part {
                                    continue;
                                }

                                if vessel.parts[child].attachment == Attachment::Radial {
                                    let mut set = HashSet::new();
                                    traverse_until(vessel, child, &visited_decouplers, &mut set);
                                    worklist.push(set);
                                } else {
                                    traverse_until(
                                        vessel,
                                        child,
                                        &visited_decouplers,
                                        &mut secondary_node,
                                    );
                                }
                            }
                            let mut set = HashSet::new();
                            set.insert(decoupler);
                            worklist.push(set);
                        } else {
                            for &child in &vessel.parts[decoupler].children {
                                if Some(child) == attached_part {
                                    continue;
                                }

                                traverse_until(
                                    vessel,
                                    child,
                                    &visited_decouplers,
                                    &mut secondary_node,
                                );
                            }
                            secondary_node.insert(decoupler);
                        }

                        worklist.extend([explosive_node, secondary_node]);
                    }
                    // Decouple the fairing pieces, i.e. do nothing.
                    Decouplers::ProceduralFairing(_) => {}
                }

                break;
            }
        }

        if !found_decoupler && !subvessel.is_empty() {
            vessels.push(subvessel);
        }
    }

    vessels
}

fn traverse_until(
    vessel: &VesselClass,
    target: PartId,
    visited: &HashSet<PartId>,
    set: &mut HashSet<PartId>,
) {
    let mut worklist = vec![target];
    while let Some(target) = worklist.pop() {
        if set.contains(&target) {
            continue;
        }

        if !visited.contains(&target) {
            set.insert(target);
        }

        for &child in &vessel.parts[target].children {
            if !visited.contains(&child) {
                worklist.push(child);
            }
        }

        if let Some(parent) = vessel.parts[target].parent {
            if !visited.contains(&parent) {
                worklist.push(parent);
            }
        }
    }
}
