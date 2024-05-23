use std::{cell::RefCell, rc::Rc};

use kerbtk::{bodies::SolarSystem, vessel::VesselClass};
use serde::{Deserialize, Serialize};

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Mission {
    pub system: SolarSystem,
    pub classes: Vec<Rc<RefCell<VesselClass>>>,
}
