//! General purpose maneuver calculations
//!
//! This (currently) includes:
//! - Circularization
//!   - Fixed altitude (presuming periapsis <= the desired altitude <= apoapsis)
//!   - At periapsis
//!   - At apoapsis
//!   - Fixed delta-T to maneuver

use std::f64::consts;

use nalgebra::Vector3;
use time::Duration;

use crate::{
    bodies::SolarSystem,
    kepler::orbits::{self, StateVector},
    time::{GET, UT},
};

use super::{frenet, Maneuver, ManeuverKind};

pub enum CircMode {
    FixedAltitude(f64),
    Periapsis,
    Apoapsis,
    FixedDeltaT(Duration),
}

pub fn circ(
    system: &SolarSystem,
    sv: StateVector,
    mode: CircMode,
    get_base: UT,
) -> Option<Maneuver> {
    let (tig_vector, deltav) = match mode {
        CircMode::FixedAltitude(alt) => circ_fixed_alt(system, sv, alt, false)?,
        CircMode::Periapsis => circ_periapsis(system, sv)?,
        CircMode::Apoapsis => circ_apoapsis(system, sv)?,
        CircMode::FixedDeltaT(delta_t) => circ_fixed_delta_t(system, sv, delta_t)?,
    };

    Some(Maneuver {
        geti: GET::from_duration(tig_vector.time - get_base),
        deltav,
        tig_vector,
        kind: ManeuverKind::GeneralPurpose,
    })
}

fn circ_fixed_alt(
    system: &SolarSystem,
    sv: StateVector,
    alt: f64,
    already_propagated: bool,
) -> Option<(StateVector, Vector3<f64>)> {
    let tig_vector = if already_propagated {
        sv
    } else {
        let obt = sv.clone().into_orbit(1e-8);
        let r = alt + sv.body.radius;
        let ta = libm::acos((obt.p - r) / (r * obt.e));
        if ta.is_nan() {
            return None;
        }
        let tof = orbits::time_of_flight(sv.position.norm(), r, obt.ta, ta, obt.p, sv.body.mu);
        if obt.e >= 1.0 {
            // FIXME: not sure yet what to do in this case
            todo!();
        }
        sv.propagate_with_soi(system, Duration::seconds_f64(tof), 1e-7, 30000)?
    };

    let mut obt = tig_vector.clone().into_orbit(1e-8);
    obt.e = 0.0;
    obt.p = alt + tig_vector.body.radius;
    let tig_vector_after = obt.sv_bci(&tig_vector.body);
    let deltav = tig_vector_after.velocity - tig_vector.velocity;
    let deltav = frenet(&tig_vector).try_inverse()? * deltav;

    Some((tig_vector, deltav))
}

fn circ_periapsis(system: &SolarSystem, sv: StateVector) -> Option<(StateVector, Vector3<f64>)> {
    let obt = sv.clone().into_orbit(1e-8);
    let mut tof = orbits::time_of_flight(
        sv.position.norm(),
        obt.periapsis_radius(),
        obt.ta,
        0.0,
        obt.p,
        sv.body.mu,
    );
    if obt.e >= 1.0 {
        // FIXME: not sure yet what to do in this case
        todo!();
    }
    tof = tof.rem_euclid(obt.period(sv.body.mu));
    circ_fixed_alt(
        system,
        sv.clone()
            .propagate_with_soi(system, Duration::seconds_f64(tof), 1e-7, 30000)?,
        obt.periapsis_radius() - sv.body.radius,
        true,
    )
}

fn circ_apoapsis(system: &SolarSystem, sv: StateVector) -> Option<(StateVector, Vector3<f64>)> {
    let obt = sv.clone().into_orbit(1e-8);
    let mut tof = orbits::time_of_flight(
        sv.position.norm(),
        obt.apoapsis_radius(),
        obt.ta,
        consts::PI,
        obt.p,
        sv.body.mu,
    );
    if obt.e >= 1.0 {
        // FIXME: not sure yet what to do in this case
        todo!();
    }
    tof = tof.rem_euclid(obt.period(sv.body.mu));
    circ_fixed_alt(
        system,
        sv.clone()
            .propagate_with_soi(system, Duration::seconds_f64(tof), 1e-7, 30000)?,
        obt.apoapsis_radius() - sv.body.radius,
        true,
    )
}

fn circ_fixed_delta_t(
    system: &SolarSystem,
    sv: StateVector,
    delta_t: Duration,
) -> Option<(StateVector, Vector3<f64>)> {
    let tig_vector = sv.propagate_with_soi(system, delta_t, 1e-7, 30000)?;

    let circ_altitude = tig_vector.position.norm() - tig_vector.body.radius;

    circ_fixed_alt(system, tig_vector, circ_altitude, true)
}
