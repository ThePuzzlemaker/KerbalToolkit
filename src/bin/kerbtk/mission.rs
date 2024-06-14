use std::sync::Arc;

use kerbtk::{
    bodies::SolarSystem,
    kepler::orbits::StateVector,
    vessel::{Vessel, VesselClass, VesselRef},
};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Mission {
    pub system: SolarSystem,
    pub classes: Vec<Arc<RwLock<VesselClass>>>,
    pub vessels: Vec<Arc<RwLock<Vessel>>>,
}
