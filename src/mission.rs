use std::{collections::HashMap, sync::Arc};

use kerbtk::{
    arena::Arena,
    bodies::SolarSystem,
    maneuver::Maneuver,
    vessel::{PartId, Vessel, VesselClass, VesselClassId, VesselId},
};
use parking_lot::{RwLock, RwLockReadGuard};
use serde::{Deserialize, Serialize};

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
    pub maneuvers: HashMap<String, PlannedManeuver>,
    pub anchor_vector_slot: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct PlannedManeuver {
    pub inner: Maneuver,
    pub engines: Vec<PartId>,
    pub dvrem: f64,
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
