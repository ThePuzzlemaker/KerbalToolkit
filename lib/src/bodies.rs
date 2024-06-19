//! Definitions of celestial bodies.

use std::{collections::HashMap, sync::Arc};

use serde::{Deserialize, Serialize};

use crate::kepler::orbits::Orbit;

/// A celestial body.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Body {
    /// Standard gravitational parameter (`km^3/s^2`)
    pub mu: f64,
    /// Mean radius of the body's sphere (`km`)
    pub radius: f64,
    /// Ephemerides at starting epoch
    pub ephem: Orbit,
    /// Rotational period, length of sidereal day (`sec`)
    pub rotperiod: f64,
    /// Initial rotation about the body spin axis at given epoch
    /// (`rad`)
    pub rotini: f64,
    /// A list of names of bodies orbiting this body.
    pub satellites: Arc<[Arc<str>]>,
    /// The name of the parent body of this body, if any.
    pub parent: Option<Arc<str>>,
    /// Name of this body as displayed in KSP
    pub name: Arc<str>,
    /// Is this a star?
    pub is_star: bool,
    /// Radius of this body's sphere of influence (`km`)
    pub soi: f64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SolarSystem {
    pub bodies: HashMap<Arc<str>, Arc<Body>>,
}
