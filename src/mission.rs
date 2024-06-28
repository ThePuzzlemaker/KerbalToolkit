use std::{collections::HashMap, sync::Arc};

use kerbtk::{
    arena::Arena,
    bodies::SolarSystem,
    ffs::{Resource, ResourceId, SimVessel},
    kepler::orbits::StateVector,
    maneuver::Maneuver,
    time::UT,
    vessel::{PartId, Vessel, VesselClass, VesselClassId, VesselId},
};
use parking_lot::{RwLock, RwLockReadGuard};
use serde::{Deserialize, Serialize};

use crate::DisplaySelect;

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Mission {
    pub system: Arc<SolarSystem>,
    pub classes: Arena<VesselClassId, VesselClass>,
    pub vessels: Arena<VesselId, Vessel>,
    #[serde(default)]
    pub plan: HashMap<VesselId, MissionPlan>,
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct MissionPlan {
    pub maneuvers: Vec<PlannedManeuver>,
    pub anchor_vector_slot: String,
    pub sim_vessel: Option<SimVessel>,
    pub nodal_targets: Option<NodalTargets>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum NodalTargets {
    Translunar {
        soi_ut: UT,
        lat_pe: f64,
        lng_pe: f64,
        h_pe: f64,
        // sv_soi: StateVector,
        mnv_base_code: String,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlannedManeuver {
    pub inner: Maneuver,
    pub engines: Vec<PartId>,
    pub resources: HashMap<u32, HashMap<ResourceId, Resource>>,
    /// ΔV remaining (m/s)
    pub dvrem: f64,
    /// Start mass (tons)
    pub start_mass: f64,
    /// End mass (tons)
    pub end_mass: f64,
    /// Burn time (s)
    pub bt: f64,
    pub source: DisplaySelect,
}

#[derive(Debug, Clone)]
pub struct MissionRef(Arc<RwLock<Mission>>);
impl MissionRef {
    pub fn new(mission: Arc<RwLock<Mission>>) -> Self {
        Self(mission)
    }
    pub fn read(&'_ self) -> RwLockReadGuard<'_, Mission> {
        self.0.read()
    }
}
