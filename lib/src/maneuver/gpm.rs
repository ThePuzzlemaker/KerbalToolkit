//! General purpose maneuver calculations
//!
//! This (currently) includes:
//! - Circularization
//!   - Fixed altitude (presuming periapsis <= the desired altitude <= apoapsis)
//!   - At periapsis
//!   - At apoapsis
//!   - Fixed delta-T to maneuver

use std::{cmp, f64::consts};

use nalgebra::Vector3;
use time::Duration;

use crate::{
    bodies::SolarSystem,
    kepler::orbits::{self, Apsis, StateVector},
    time::{GET, UT},
};

use super::{frenet, Maneuver, ManeuverKind};

pub enum ApsisChangeMode {
    Optimum,
    FixedDeltaT(Duration),
    AtPeriapsis,
    AtApoapsis,
}

pub fn change_apsis(
    system: &SolarSystem,
    sv: StateVector,
    apsis: Apsis,
    to: f64,
    mode: ApsisChangeMode,
    get_base: UT,
) -> Option<Maneuver> {
    let (tig_vector, deltav) = match mode {
        ApsisChangeMode::AtApoapsis => {
            change_apsis_at_apsis(system, Apsis::Apoapsis, apsis, to, sv)?
        }
        ApsisChangeMode::AtPeriapsis => {
            change_apsis_at_apsis(system, Apsis::Periapsis, apsis, to, sv)?
        }
        // ApsisChangeMode::FixedDeltaT(delta_t) => {
        //     change_apsis_fixed_deltat(system, apsis, to, sv, delta_t)?
        // }
        ApsisChangeMode::Optimum => change_apsis_optimum(system, apsis, to, sv)?,
        _ => todo!(),
    };

    Some(Maneuver {
        geti: GET::from_duration(tig_vector.time - get_base),
        deltav,
        tig_vector,
        kind: ManeuverKind::GeneralPurpose,
    })
}

fn change_apsis_at_apsis(
    system: &SolarSystem,
    at: Apsis,
    apsis: Apsis,
    to: f64,
    sv: StateVector,
) -> Option<(StateVector, Vector3<f64>)> {
    if apsis != at {
        return change_apsis_optimum(system, apsis, to, sv);
    }

    let sv_at_apsis = sv.clone().propagate_to_apsis(system, at, 1e-7, 30000)?;
    let obt = sv_at_apsis.clone().into_orbit(1e-8);
    let twomu_over_r = 2.0 * sv.body.mu / sv_at_apsis.position.norm();
    let old_a = obt.semimajor_axis();
    let new_a = (obt.apsis_radius(apsis) + to) * 0.5;
    let deltav = libm::sqrt(twomu_over_r - sv.body.mu / new_a)
        - libm::sqrt(twomu_over_r - sv.body.mu / old_a);
    let deltav = Vector3::new(deltav, 0.0, 0.0);

    Some((sv_at_apsis, deltav))
}

fn change_apsis_optimum(
    system: &SolarSystem,
    apsis: Apsis,
    to: f64,
    sv: StateVector,
) -> Option<(StateVector, Vector3<f64>)> {
    let obt = sv.clone().into_orbit(1e-8);
    if apsis == Apsis::Apoapsis && (obt.e - 1.0).abs() < 1e-6 {
        return None;
    }
    let sv_at_apsis = sv.clone().propagate_to_apsis(system, !apsis, 1e-7, 30000)?;
    let obt = sv_at_apsis.clone().into_orbit(1e-8);
    let twomu_over_r = 2.0 * sv.body.mu / sv_at_apsis.position.norm();
    let old_a = obt.semimajor_axis();
    let new_a = (obt.apsis_radius(!apsis) + to) * 0.5;
    let deltav = libm::sqrt(twomu_over_r - sv.body.mu / new_a)
        - libm::sqrt(twomu_over_r - sv.body.mu / old_a);
    let deltav = Vector3::new(deltav, 0.0, 0.0);

    Some((sv_at_apsis, deltav))
}

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
