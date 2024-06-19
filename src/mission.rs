use std::{collections::HashMap, sync::Arc};

use kerbtk::{
    arena::{Arena, IdLike},
    bodies::SolarSystem,
    maneuver::Maneuver,
    vessel::{PartId, Vessel, VesselClass, VesselRef},
};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Mission {
    pub system: Arc<SolarSystem>,
    pub classes: Arena<VesselClassId, Arc<RwLock<VesselClass>>>,
    pub vessels: Arena<VesselId, Arc<RwLock<Vessel>>>,
    #[serde(default)]
    pub plan: HashMap<VesselId, MissionPlan>,
}

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

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct MissionPlan {
    pub maneuvers: HashMap<String, PlannedManeuver>,
    pub anchor_vector_slot: String,
    pub vessel: VesselRef,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct PlannedManeuver {
    pub inner: Maneuver,
    pub engines: Vec<PartId>,
    pub dvrem: f64,
}
