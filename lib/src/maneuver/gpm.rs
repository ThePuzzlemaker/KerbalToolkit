//! General purpose maneuver calculations
//!
//! This (currently) includes:
//! - Circularization
//!   - Fixed altitude (presuming periapsis <= the desired altitude <= apoapsis)
//!   - At periapsis
//!   - At apoapsis
//!   - Fixed delta-T to maneuver

use std::f64::consts;

use nalgebra::{Rotation3, Vector3};
use nlopt::{Algorithm, FailState, Nlopt, Target};
use time::Duration;
use tracing::trace;

use crate::{
    bodies::SolarSystem,
    kepler::orbits::{self, Apsis, StateVector},
    time::{GET, UT},
    translunar::approximate_gradient,
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
        ApsisChangeMode::FixedDeltaT(delta_t) => {
            change_apsis_fixed_deltat(system, apsis, to, sv, delta_t)?
        }
        ApsisChangeMode::Optimum => change_apsis_optimum(system, apsis, to, sv)?,
    };

    Some(Maneuver {
        geti: GET::from_duration(tig_vector.time - get_base),
        deltav,
        tig_vector,
        kind: ManeuverKind::GeneralPurpose,
    })
}

fn change_apsis_fixed_deltat(
    system: &SolarSystem,
    apsis: Apsis,
    to: f64,
    sv: StateVector,
    dt: Duration,
) -> Option<(StateVector, Vector3<f64>)> {
    struct ChangeApsisProblem {
        sv: StateVector,
        to: f64,
        apsis: Apsis,
    }

    impl ChangeApsisProblem {
        fn cost(&self, v: &[f64; 2]) -> f64 {
            v[0] * v[0] + v[1] * v[1]
        }

        fn constraint(x: &[f64], g: Option<&mut [f64]>, this: &mut &Self) -> f64 {
            if let Some(g) = g {
                approximate_gradient(x, |v| Self::constraint_inner(v, this), g)
            }
            Self::constraint_inner(x, this)
        }

        fn constraint_inner(x: &[f64], this: &mut &Self) -> f64 {
            let deltav = Vector3::new(x[0], 0.0, x[1]);
            let frenet = frenet(&this.sv);
            let deltav = frenet * deltav;
            let mut sv = this.sv.clone();
            sv.velocity += deltav;
            (sv.into_orbit(1e-8).apsis_radius(this.apsis) - this.to).abs()
        }
    }

    let sv_at_dt = sv.clone().propagate_with_soi(system, dt, 1e-7, 30000)?;
    let obt = sv_at_dt.clone().into_orbit(1e-8);
    let twomu_over_r = 2.0 * sv.body.mu / sv_at_dt.position.norm();
    let old_a = obt.semimajor_axis();
    let new_a = (obt.apsis_radius(!apsis) + to) * 0.5;
    let deltav = libm::sqrt(twomu_over_r - sv.body.mu / new_a)
        - libm::sqrt(twomu_over_r - sv.body.mu / old_a);
    let deltav =
        Rotation3::from_axis_angle(&Vector3::y_axis(), obt.ta) * Vector3::new(deltav, 0.0, 0.0);

    let problem = ChangeApsisProblem {
        sv: sv_at_dt.clone(),
        to,
        apsis,
    };

    let mut opt = Nlopt::new(
        Algorithm::Cobyla,
        2,
        |v, _, p| p.cost(&[v[0], v[1]]),
        Target::Minimize,
        &problem,
    );
    opt.set_maxeval(30000 as u32).expect("misconfig");
    opt.set_ftol_rel(1e-7).expect("misconfig");
    opt.set_lower_bounds(&[-100.0, -100.0]).expect("misconfig");
    opt.set_upper_bounds(&[100.0, 100.0]).expect("misconfig");
    opt.add_inequality_constraint(ChangeApsisProblem::constraint, &problem, 0.001)
        .expect("misconfig");

    let mut x_init = [deltav.x, deltav.y];
    let res = opt.optimize(&mut x_init);
    match res {
        Ok(res) => trace!("ChangeApsisProblem: opt success: {res:?}"),
        Err(res @ (FailState::RoundoffLimited, _)) => {
            trace!("ChangeApsisProblem: warn: roundoff limited: {res:?}");
        }
        Err(e) => {
            trace!("ChangeApsisProblem: error: {e:?}");
            return None;
        }
    };

    let deltav = Vector3::new(x_init[0], 0.0, x_init[1]);

    Some((sv_at_dt, deltav))
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

    let sv_at_apsis = sv
        .clone()
        .propagate_to_apsis(system, at, 1e-7, 30000, false)?;
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
    let sv_at_apsis = sv
        .clone()
        .propagate_to_apsis(system, !apsis, 1e-7, 30000, true)?;
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
