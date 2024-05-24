use std::sync::Arc;

use kerbtk::{bodies::SolarSystem, vessel::VesselClass};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Mission {
    pub system: SolarSystem,
    pub classes: Vec<Arc<RwLock<VesselClass>>>,
}
