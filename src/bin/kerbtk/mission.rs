use std::{collections::HashMap, sync::Arc};

use kerbtk::{
    bodies::SolarSystem,
    ffs::FuelStats,
    maneuver::Maneuver,
    vessel::{PartId, Vessel, VesselClass, VesselRef},
};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Mission {
    pub system: SolarSystem,
    pub classes: Vec<Arc<RwLock<VesselClass>>>,
    pub vessels: Vec<Arc<RwLock<Vessel>>>,
    #[serde(default)]
    pub plan: MissionPlan,
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct MissionPlan {
    pub maneuvers: HashMap<String, PlannedManeuver>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct PlannedManeuver {
    pub maneuver: Maneuver,
    pub vessel: VesselRef,
    pub engines: Vec<PartId>,
    pub stats: FuelStats,
}
