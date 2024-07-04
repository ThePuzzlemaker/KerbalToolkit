//! Translunar trajectory generation and correction
#![allow(non_snake_case)]

use std::{f64::consts, ops::Range, sync::Arc};

use argmin::{
    core::{CostFunction, Executor},
    solver::{
        particleswarm::ParticleSwarm,
        simulatedannealing::{Anneal, SATempFunc, SimulatedAnnealing},
    },
};
use nalgebra::{Matrix3, Rotation3, Vector3};
use nlopt::{Algorithm, FailState, Nlopt, Target};
use ordered_float::OrderedFloat;
use rand::{distributions::Slice, thread_rng, Rng};
use time::Duration;
use tracing::trace;

use crate::{
    bodies::Body,
    kepler::{
        lambert,
        orbits::{time_of_flight, Orbit, ReferenceFrame, StateVector},
    },
    maneuver::{self, frenet},
    time::UT,
};

pub struct TliSolver2 {
    cs: TliConstraintSet2,
    central: Arc<Body>,
    moon: Arc<Body>,
    moon_sv_t0: StateVector,
    opt_periapse: bool,
}

#[derive(Clone)]
pub struct TliConstraintSet2 {
    pub central_sv: StateVector,
    pub flight_time: Range<Duration>,
    pub moon_periapse_radius: Range<f64>,
    pub coast_time: Range<Duration>,
    pub pe_lat: Range<f64>,
    pub pe_lng: Range<f64>,
    pub moon_inclination: Range<f64>,
}

struct TliProblem3 {
    cs: TliConstraintSet2,
    moon: Arc<Body>,
    moon_sv_t0: StateVector,
    period: f64,
}

struct TliProblem3Vals {
    coast_time: f64,
    delta_v_tot: Vector3<f64>,
}

impl From<&'_ [f64]> for TliProblem3Vals {
    fn from(value: &'_ [f64]) -> Self {
        Self {
            coast_time: value[0],
            delta_v_tot: Vector3::new(value[1], value[2], value[3]),
        }
    }
}

impl From<[f64; 4]> for TliProblem3Vals {
    fn from(value: [f64; 4]) -> Self {
        Self {
            coast_time: value[0],
            delta_v_tot: Vector3::new(value[1], value[2], value[3]),
        }
    }
}

impl From<TliProblem3Vals> for [f64; 4] {
    fn from(value: TliProblem3Vals) -> Self {
        [
            value.coast_time,
            value.delta_v_tot.x,
            value.delta_v_tot.y,
            value.delta_v_tot.z,
        ]
    }
}

/// Helper function to calculate gradient of function numerically.
/// Can be useful when a gradient must be provided to the optimization
/// algorithm and a closed-form derivative cannot be obtained
// from NLOpt crate w/ patch for Fn->FnMut
// TODO: multi-constraint version
pub fn approximate_gradient<F>(x0: &[f64], mut f: F, grad: &mut [f64])
where
    F: FnMut(&[f64]) -> f64,
{
    let n = x0.len();
    let mut x0 = x0.to_vec();
    let eps = f64::EPSILON.powf(1.0 / 3.0);
    for i in 0..n {
        let x0i = x0[i];
        x0[i] = x0i - eps;
        let fl = f(&x0);
        x0[i] = x0i + eps;
        let fh = f(&x0);
        grad[i] = (fh - fl) / (2.0 * eps);
        x0[i] = x0i;
    }
}

impl TliProblem3 {
    #[allow(clippy::needless_pass_by_value)]
    fn objective(x: &[f64], g: Option<&mut [f64]>, _this: &mut &Self) -> f64 {
        //trace!("objective: x={x:?}");
        let v = TliProblem3Vals::from(x);
        //trace!("CT={}", v.coast_time);
        if let Some(g) = g {
            g[0] = 0.0;
            g[1] = v.delta_v_tot.x / v.delta_v_tot.norm();
            g[2] = v.delta_v_tot.y / v.delta_v_tot.norm();
            g[3] = v.delta_v_tot.z / v.delta_v_tot.norm();
        }
        v.delta_v_tot.norm()
    }

    // fn nonzero_constraint(x: &[f64], g: Option<&mut [f64]>, this: &mut ()) -> f64 {
    //     if let Some(g) = g {
    //         g[0] = 0.0;
    //         g[1] = 0.0;
    //         g[2] = -2.0 / x[2].powi(3);
    //         g[3] = -2.0 / x[3].powi(3);
    //         g[4] = -2.0 / x[4].powi(3);
    //     }
    //     1.0 / x[2].powi(2) + 1.0 / x[3].powi(2) + 1.0 / x[4].powi(2)
    // }

    // fn pe_lat_constraint(x: &[f64], g: Option<&mut [f64]>, this: &mut &Self) -> f64 {
    //     //trace!("hp_constraint: x={x:?}");
    //     if let Some(g) = g {
    //         approximate_gradient(x, |v| Self::pe_latlng_constraint_inner(v, this).0, g);
    //     }
    //     Self::pe_latlng_constraint_inner(x, this).0
    // }

    fn inc_constraint(x: &[f64], g: Option<&mut [f64]>, this: &mut &Self) -> f64 {
        //trace!("hp_constraint: x={x:?}");
        if let Some(g) = g {
            approximate_gradient(x, |v| Self::inc_constraint_inner(v, this), g);
        }
        Self::inc_constraint_inner(x, this)
    }

    fn inc_constraint_inner(x: &[f64], this: &mut &Self) -> f64 {
        //trace!("hp_constraint_inner: x={x:?}");
        let v = TliProblem3Vals::from(x);

        let Some(pe_sv) = Self::integrate(&v, this) else {
            //return (this.moon.soi + 10_000.0).powi(2);
            return 10_000.0 * consts::TAU;
            //return f64::NAN;
        };

        let obt = pe_sv.into_orbit(1e-8);
        // trace!(
        //     "Pe inc = {:.3}, CV={}",
        //     obt.i.to_degrees(),
        //     (obt.i - 0.5 * (this.cs.moon_inclination.end + this.cs.moon_inclination.start))
        //         .to_degrees()
        // );

        (obt.i - 0.5 * (this.cs.moon_inclination.end + this.cs.moon_inclination.start)).powi(2)
    }

    // fn pe_lng_constraint(x: &[f64], g: Option<&mut [f64]>, this: &mut &Self) -> f64 {
    //     //trace!("hp_constraint: x={x:?}");
    //     if let Some(g) = g {
    //         approximate_gradient(x, |v| Self::pe_latlng_constraint_inner(v, this).1, g);
    //     }
    //     Self::pe_latlng_constraint_inner(x, this).1
    // }

    // fn pe_latlng_constraint_inner(x: &[f64], this: &mut &Self) -> (f64, f64) {
    //     //trace!("hp_constraint_inner: x={x:?}");
    //     let v = TliProblem3Vals::from(x);

    //     let Some(pe_sv) = Self::integrate(&v, this) else {
    //         //return (this.moon.soi + 10_000.0).powi(2);
    //         // return 10_000.0 * consts::TAU;
    //         return (10_000.0 * consts::TAU, 10_000.0 * consts::TAU);
    //     };

    //     let (lat, lng) = pe_sv.latlng();

    //     // trace!(
    //     //     "Pe lat = {:.3}, lng = {:.3} CV={},{}",
    //     //     lat.to_degrees(),
    //     //     lng.to_degrees(),
    //     //     (lat - 0.5 * (this.cs.pe_lat.end + this.cs.pe_lat.start)).to_degrees(),
    //     //     (lng - 0.5 * (this.cs.pe_lng.end + this.cs.pe_lng.start)).to_degrees(),
    //     // );

    //     (
    //         (lat - 0.5 * (this.cs.pe_lat.end + this.cs.pe_lat.start)).powi(2),
    //         (lng - 0.5 * (this.cs.pe_lng.end + this.cs.pe_lng.start)).powi(2),
    //     )
    // }

    fn hp_constraint(x: &[f64], g: Option<&mut [f64]>, this: &mut &Self) -> f64 {
        //trace!("hp_constraint: x={x:?}");
        if let Some(g) = g {
            approximate_gradient(x, |v| Self::hp_constraint_inner(v, this), g);
        }
        Self::hp_constraint_inner(x, this)
    }

    fn hp_constraint_inner(x: &[f64], this: &mut &Self) -> f64 {
        //trace!("hp_constraint_inner: x={x:?}");
        let v = TliProblem3Vals::from(x);

        let Some(pe_sv) = Self::integrate(&v, this) else {
            return (this.moon.soi + 10_000.0).powi(2);
            //return f64::NAN;
        };

        // trace!("Pe sv rad = {:.3}", pe_sv.position.norm());
        let pe_rad = pe_sv.into_orbit(1e-8).periapsis_radius();
        // trace!(
        //     "Pe rad = {pe_rad:.3}, CV={}",
        //     pe_rad - 0.5 * (this.cs.moon_periapse_radius.end + this.cs.moon_periapse_radius.start)
        // );

        (pe_rad - 0.5 * (this.cs.moon_periapse_radius.end + this.cs.moon_periapse_radius.start))
            .powi(2)
    }

    /// Integrate given the initial state and hyperparameters,
    /// returning the state vector at lunar periapsis
    fn integrate(v: &TliProblem3Vals, this: &mut &Self) -> Option<StateVector> {
        let coast_time = v.coast_time * this.period;
        //trace!("CT={}", v.coast_time);
        let mut central_sv =
            this.cs
                .central_sv
                .clone()
                .propagate(Duration::seconds_f64(coast_time), 1e-7, 500)?;
        let moon_sv =
            this.moon_sv_t0
                .clone()
                .propagate(Duration::seconds_f64(coast_time), 1e-7, 500)?;
        // trace!(
        //     "ΔV = {}",
        //     frenet(&central_sv).try_inverse().unwrap() * v.delta_v_tot * 1000.0
        // );

        central_sv.velocity += v.delta_v_tot;

        let soi = central_sv.intersect_soi_child(&moon_sv, &this.moon, 1e-7, 30000)?;

        let r0 = soi.position.norm();
        let mut soi = soi.into_orbit(1e-8);
        // TODO: check this value
        let tof = time_of_flight(
            r0,
            soi.periapsis_radius(),
            soi.ta - consts::TAU,
            0.0,
            soi.p,
            this.moon.mu,
        );
        soi.ta = 0.0;
        // trace!("tof0={tof:.3}");
        soi.epoch = soi.epoch + Duration::seconds_f64(tof);
        let soi = soi.sv_bci(&this.moon);

        // trace!("tof={}", soi.time - central_sv.time);
        // if !this.cs.flight_time.contains(&(soi.time - central_sv.time)) {
        //     return None;
        // }

        Some(soi)
    }
}

struct TLIProblem4<'a> {
    solver: &'a TliSolver2,
}

impl<'a> CostFunction for TLIProblem4<'a> {
    type Param = [f64; 2];

    type Output = f64;

    fn cost(&self, param: &Self::Param) -> Result<Self::Output, argmin::core::Error> {
        // Round to ms precision
        let flight_time = Duration::seconds_f64((param[1] * 1e+3).round() / 1e+3);
        let coast_time = Duration::seconds_f64((param[0] * 1e+3).round() / 1e+3);

        let Some(sv) = self
            .solver
            .cs
            .central_sv
            .clone()
            .propagate(coast_time, 1e-7, 500)
        else {
            return Ok(f64::MAX);
        };

        let Some(moon_sv) =
            self.solver
                .moon_sv_t0
                .clone()
                .propagate(flight_time + coast_time, 1e-7, 500)
        else {
            return Ok(f64::MAX);
        };
        let Some((v0, _v2)) = lambert::lambert(
            sv.position,
            moon_sv.position,
            flight_time.as_seconds_f64(),
            self.solver.central.mu,
            1e-15,
            500,
        )
        .min_by_key(|x| OrderedFloat((x.0 - sv.velocity).norm())) else {
            return Ok(f64::MAX);
        };

        let deltav = (v0 - sv.velocity).norm();

        Ok(deltav)
    }
}

impl TliSolver2 {
    pub fn new(
        cs: TliConstraintSet2,
        central: Arc<Body>,
        moon: Arc<Body>,
        t0: UT,
        opt_periapse: bool,
    ) -> Self {
        let moon_sv_t0 = moon
            .ephem
            .sv_bci(&central)
            .propagate(t0 - moon.ephem.epoch, 1e-7, 500)
            .expect("Propagate moon SV");
        Self {
            cs,
            central,
            moon,
            moon_sv_t0,
            opt_periapse,
        }
    }

    pub fn run(&mut self, maxiter: u64) -> Option<TLISolution> {
        let problem = TLIProblem4 { solver: self };

        let mut opt = Nlopt::new(
            Algorithm::Cobyla,
            2,
            |v, _, p| p.cost(&[v[0], v[1]]).unwrap_or(f64::NAN),
            Target::Minimize,
            &problem,
        );
        opt.set_maxeval(maxiter as u32).expect("misconfig");
        opt.set_ftol_rel(1e-7).expect("misconfig");
        opt.set_lower_bounds(&[
            self.cs.coast_time.start.as_seconds_f64(),
            self.cs.flight_time.start.as_seconds_f64(),
        ])
        .expect("misconfig");
        opt.set_upper_bounds(&[
            self.cs.coast_time.end.as_seconds_f64(),
            self.cs.flight_time.end.as_seconds_f64(),
        ])
        .expect("misconfig");

        let mut x_init = [
            (self.cs.coast_time.start + self.cs.coast_time.end).as_seconds_f64() / 2.0,
            (self.cs.flight_time.start + self.cs.flight_time.end).as_seconds_f64() / 2.0,
        ];
        trace!("x_init: {x_init:?}");
        let res = opt.optimize(&mut x_init);
        match res {
            Ok(res) => trace!("TliSolver::run: opt success: {res:?}"),
            Err(res @ (FailState::RoundoffLimited, _)) => {
                trace!("TliSolver::run: warn: roundoff limited: {res:?}");
            }
            Err(e) => {
                trace!("TliSolver::run: error: {e:?}");
                return None;
            }
        };

        let params = x_init;
        // Round to ms precision
        let flight_time = (params[1] * 1e+3).round() / 1e+3;
        let coast_time = (params[0] * 1e+3).round() / 1e+3;
        let sv_init =
            self.cs
                .central_sv
                .clone()
                .propagate(Duration::seconds_f64(coast_time), 1e-7, 500)?;

        let moon_sv = self.moon_sv_t0.clone().propagate(
            Duration::seconds_f64(flight_time + coast_time),
            1e-7,
            500,
        )?;

        let r0 = sv_init.position;
        let (v0, _v2) = lambert::lambert(r0, moon_sv.position, flight_time, self.central.mu, 1e-15, 500)
                .min_by_key(|x| OrderedFloat((x.0 - sv_init.velocity).norm()))
                // TODO: error?
                ?;

        let mut deltav = v0 - sv_init.velocity;
        // Round to 0.01m/s precision
        deltav.x = (deltav.x * 1e+05).round() / 1e+05;
        deltav.y = (deltav.y * 1e+05).round() / 1e+05;
        deltav.z = (deltav.z * 1e+05).round() / 1e+05;
        // We'll assume >100km/s ΔV is *probably* a bad trajectory.
        if deltav.norm() > 100.0 {
            return None;
        }

        let obt = self.cs.central_sv.clone().into_orbit(1e-8);
        let period = obt.period(self.central.mu);
        let problem = TliProblem3 {
            cs: self.cs.clone(),
            moon: self.moon.clone(),
            moon_sv_t0: self.moon_sv_t0.clone(),
            period,
        };
        let mut opt = Nlopt::new(
            Algorithm::Auglag,
            4,
            TliProblem3::objective,
            Target::Minimize,
            &problem,
        );
        opt.set_maxeval(maxiter as u32).expect("misconfig");
        opt.set_ftol_rel(1e-7).expect("misconfig");
        opt.set_lower_bounds(&<[f64; 4]>::from(TliProblem3Vals {
            coast_time: self.cs.coast_time.start.as_seconds_f64() / period,
            delta_v_tot: Vector3::new(deltav.x, deltav.y, deltav.z)
                - 0.1 * deltav.norm() * Vector3::new(1.0, 1.0, 1.0),
        }))
        .expect("misconfig");
        opt.set_upper_bounds(&<[f64; 4]>::from(TliProblem3Vals {
            coast_time: self.cs.coast_time.end.as_seconds_f64() / period,
            delta_v_tot: Vector3::new(deltav.x, deltav.y, deltav.z)
                + 0.1 * deltav.norm() * Vector3::new(1.0, 1.0, 1.0),
        }))
        .expect("misconfig");

        let mut opt1 = Nlopt::new(Algorithm::Bobyqa, 4, |_, _, ()| 0.0, Target::Minimize, ());

        opt1.set_maxeval(maxiter as u32).expect("misconfig");
        opt1.set_ftol_rel(1e-7).expect("misconfig");
        opt.set_local_optimizer(opt1).expect("misconfig");

        if self.opt_periapse {
            opt.add_inequality_constraint(
                TliProblem3::hp_constraint,
                &problem,
                (0.5 * (self.cs.moon_periapse_radius.end - self.cs.moon_periapse_radius.start))
                    .powi(2),
            )
            .expect("misconfig");
        }
        opt.add_inequality_constraint(
            TliProblem3::inc_constraint,
            &problem,
            (0.5 * (self.cs.moon_inclination.end - self.cs.moon_inclination.start)).powi(2),
        )
        .expect("misconfig");

        // opt.add_inequality_constraint(
        //     TliProblem3::pe_lat_constraint,
        //     &problem,
        //     (0.5 * (self.cs.pe_lat.end - self.cs.pe_lat.start)).powi(2),
        // )
        // .expect("misconfig");
        // opt.add_inequality_constraint(
        //     TliProblem3::pe_lng_constraint,
        //     &problem,
        //     (0.5 * (self.cs.pe_lng.end - self.cs.pe_lng.start)).powi(2),
        // )
        // .expect("misconfig");

        // opt.add_inequality_constraint(TliProblem3::nonzero_constraint, (), 1e+6)
        //     .expect("misconfig");

        let mut x_init = <[f64; 4]>::from(TliProblem3Vals {
            coast_time: coast_time / period,
            delta_v_tot: deltav,
        });
        trace!("x_init: {x_init:?}");
        let res = opt.optimize(&mut x_init);
        match res {
            Ok(res) => trace!("TliSolver::run: opt success: {res:?}"),
            Err(res @ (FailState::RoundoffLimited, _)) => {
                trace!("TliSolver::run: warn: roundoff limited: {res:?}");
            }
            Err(e) => {
                trace!("TliSolver::run: error: {e:?}");
                return None;
            }
        };
        let mut x_final = TliProblem3Vals::from(x_init);
        let mut coast_time = x_final.coast_time * period;

        let mut sv_init =
            self.cs
                .central_sv
                .clone()
                .propagate(Duration::seconds_f64(coast_time), 1e-7, 500)?;
        let moon_sv_init =
            self.moon_sv_t0
                .clone()
                .propagate(Duration::seconds_f64(coast_time), 1e-7, 500)?;
        let mut sv_final = sv_init.clone();
        sv_final.velocity += x_final.delta_v_tot;

        let sv = sv_final.intersect_soi_child(&moon_sv_init, &self.moon, 1e-7, 30000)?;
        let obt = sv.clone().into_orbit(1e-8);
        trace!("Pe rad = {}", obt.periapsis_radius());

        if !self
            .cs
            .moon_periapse_radius
            .contains(&obt.periapsis_radius())
            || !self.cs.moon_inclination.contains(&obt.i)
        {
            let mut opt = Nlopt::new(
                Algorithm::Cobyla,
                4,
                TliProblem3::objective,
                Target::Minimize,
                &problem,
            );
            opt.set_maxeval(maxiter as u32).expect("misconfig");
            opt.set_ftol_rel(1e-7).expect("misconfig");
            opt.set_lower_bounds(&<[f64; 4]>::from(TliProblem3Vals {
                coast_time: self.cs.coast_time.start.as_seconds_f64() / period,
                delta_v_tot: Vector3::new(deltav.x, deltav.y, deltav.z)
                    - 0.1 * deltav.norm() * Vector3::new(1.0, 1.0, 1.0),
            }))
            .expect("misconfig");
            opt.set_upper_bounds(&<[f64; 4]>::from(TliProblem3Vals {
                coast_time: self.cs.coast_time.end.as_seconds_f64() / period,
                delta_v_tot: Vector3::new(deltav.x, deltav.y, deltav.z)
                    + 0.1 * deltav.norm() * Vector3::new(1.0, 1.0, 1.0),
            }))
            .expect("misconfig");

            if self.opt_periapse {
                opt.add_inequality_constraint(
                    TliProblem3::hp_constraint,
                    &problem,
                    (0.5 * (self.cs.moon_periapse_radius.end - self.cs.moon_periapse_radius.start))
                        .powi(2),
                )
                .expect("misconfig");
            }
            opt.add_inequality_constraint(
                TliProblem3::inc_constraint,
                &problem,
                (0.5 * (self.cs.moon_inclination.end - self.cs.moon_inclination.start)).powi(2),
            )
            .expect("misconfig");
            trace!("x_init: {x_init:?}");
            let res = opt.optimize(&mut x_init);
            match res {
                Ok(res) => trace!("TliSolver::run: opt success: {res:?}"),
                Err(res @ (FailState::RoundoffLimited, _)) => {
                    trace!("TliSolver::run: warn: roundoff limited: {res:?}");
                }
                Err(e) => {
                    trace!("TliSolver::run: error: {e:?}");
                    return None;
                }
            };
            x_final = TliProblem3Vals::from(x_init);
            coast_time = x_final.coast_time * period;

            sv_init = self.cs.central_sv.clone().propagate(
                Duration::seconds_f64(coast_time),
                1e-7,
                500,
            )?;
            let moon_sv_init =
                self.moon_sv_t0
                    .clone()
                    .propagate(Duration::seconds_f64(coast_time), 1e-7, 500)?;
            let mut sv_final = sv_init.clone();
            sv_final.velocity += x_final.delta_v_tot;

            let sv = sv_final.intersect_soi_child(&moon_sv_init, &self.moon, 1e-7, 30000)?;
            let obt = sv.clone().into_orbit(1e-8);
            trace!("Pe rad = {}", obt.periapsis_radius());
            if !self
                .cs
                .moon_periapse_radius
                .contains(&obt.periapsis_radius())
                || !self.cs.moon_inclination.contains(&obt.i)
            {
                return None;
            }
        }

        Some(TLISolution {
            sv_init,
            // TODO: fix this number
            flight_time: Duration::seconds_f64(flight_time),
            deltav: x_final.delta_v_tot,
        })
    }
}

#[allow(non_snake_case)]
pub struct TLISolver {
    cs: TLIConstraintSet,
    central: Arc<Body>,
    moon: Arc<Body>,
    moon_sv_t0: StateVector,
    opt_periapse: bool,
    allow_retrograde: bool,
}

#[allow(non_snake_case)]
pub struct TLIConstraintSet {
    pub central_sv: StateVector,
    pub flight_time: Range<Duration>,
    pub moon_periapse_radius: Range<f64>,
    pub coast_time: Range<Duration>,
}

#[derive(Debug)]
pub struct TLISolution {
    pub sv_init: StateVector,
    pub flight_time: Duration,
    pub deltav: Vector3<f64>,
}

impl TLISolver {
    pub fn new(
        cs: TLIConstraintSet,
        central: Arc<Body>,
        moon: Arc<Body>,
        t0: UT,
        opt_periapse: bool,
        allow_retrograde: bool,
    ) -> Self {
        let moon_sv_t0 = moon
            .ephem
            .sv_bci(&central)
            .propagate(t0 - moon.ephem.epoch, 1e-7, 500)
            .expect("Propagate moon SV");
        Self {
            cs,
            central,
            moon,
            moon_sv_t0,
            opt_periapse,
            allow_retrograde,
        }
    }

    pub fn run(&mut self, maxiter: u64, temp: f64) -> Option<TLISolution> {
        let problem = TLIProblem1 { solver: self };
        let solver = ParticleSwarm::new(
            (
                vec![
                    self.cs.flight_time.start.as_seconds_f64(),
                    self.cs.coast_time.start.as_seconds_f64(),
                ],
                vec![
                    self.cs.flight_time.end.as_seconds_f64(),
                    self.cs.coast_time.end.as_seconds_f64(),
                ],
            ),
            10,
        );

        let res = Executor::new(problem, solver)
            .configure(|state| state.max_iters(maxiter))
            .run()
            .ok()?;
        trace!("{res}");

        let best = res.state.clone().best_individual?;
        // We'll assume >100km/s ΔV is *probably* a bad trajectory.
        if best.cost > 100.0 {
            return None;
        }

        let params = best.position;
        // Round to ms precision
        let flight_time = (params[0] * 1e+3).round() / 1e+3;
        let coast_time = (params[1] * 1e+3).round() / 1e+3;
        let sv_init =
            self.cs
                .central_sv
                .clone()
                .propagate(Duration::seconds_f64(coast_time), 1e-7, 500)?;

        let moon_sv = self.moon_sv_t0.clone().propagate(
            Duration::seconds_f64(flight_time + coast_time),
            1e-7,
            500,
        )?;

        let r0 = sv_init.position;
        let (v0, _v2) = lambert::lambert(r0, moon_sv.position, flight_time, self.central.mu, 1e-15, 500)
                .min_by_key(|x| OrderedFloat((x.0 - sv_init.velocity).norm()))
                // TODO: error?
                ?;

        let mut deltav = v0 - sv_init.velocity;
        // Round to 0.01m/s precision
        deltav.x = (deltav.x * 1e+05).round() / 1e+05;
        deltav.y = (deltav.y * 1e+05).round() / 1e+05;
        deltav.z = (deltav.z * 1e+05).round() / 1e+05;

        let moon_sv_init =
            self.moon_sv_t0
                .clone()
                .propagate(Duration::seconds_f64(coast_time), 1e-7, 500)?;

        let deltav = if self.opt_periapse {
            let frenet = maneuver::frenet(&sv_init);
            let frenet_inv = frenet.try_inverse().expect("oops");
            let deltav = frenet_inv * deltav;

            let problem = TLIProblem2 {
                solver: self,
                sv_init: sv_init.clone(),
                moon_sv_init: moon_sv_init.clone(),
                allow_retrograde: self.allow_retrograde,
                frenet,
                maxiter,
            };

            let solver = SimulatedAnnealing::new(temp)
                .ok()?
                .with_temp_func(SATempFunc::Boltzmann)
                .with_reannealing_fixed(1000)
                .with_reannealing_accepted(500)
                .with_reannealing_best(800);

            let res = Executor::new(problem, solver)
                .configure(|state| {
                    state
                        .param(vec![deltav.x, deltav.y, deltav.z])
                        .max_iters(maxiter)
                })
                .run()
                .ok()?;
            trace!("{res}");

            let params = res.state.best_param?;

            let mut deltav = Vector3::new(params[0], params[1], params[2]);
            // Round to 0.01m/s precision
            deltav.x = (deltav.x * 1e+5).round() / 1e+5;
            deltav.y = (deltav.y * 1e+5).round() / 1e+5;
            deltav.z = (deltav.z * 1e+5).round() / 1e+5;

            let mut sv = sv_init.clone();
            sv.velocity += frenet * deltav;
            let sv = sv.intersect_soi_child(&moon_sv_init, &self.moon, 1e-7, maxiter)?;
            let obt = sv.clone().into_orbit(1e-8);
            trace!("Pe rad = {}", obt.periapsis_radius());
            if !self
                .cs
                .moon_periapse_radius
                .contains(&obt.periapsis_radius())
            {
                return None;
            }
            frenet * deltav
        } else {
            deltav
        };

        Some(TLISolution {
            flight_time: Duration::seconds_f64(flight_time),
            deltav,
            sv_init,
        })
    }
}

struct TLIProblem2<'a> {
    solver: &'a TLISolver,
    sv_init: StateVector,
    moon_sv_init: StateVector,
    allow_retrograde: bool,
    frenet: Matrix3<f64>,
    maxiter: u64,
}

impl<'a> CostFunction for TLIProblem2<'a> {
    type Param = Vec<f64>;

    type Output = f64;

    fn cost(&self, param: &Self::Param) -> Result<Self::Output, argmin_math::Error> {
        // Round to .01m/s precision
        let dvx = (param[0] * 1e+05).round() / 1e+05;
        let dvy = (param[1] * 1e+05).round() / 1e+05;
        let dvz = (param[2] * 1e+05).round() / 1e+05;

        let mut sv = self.sv_init.clone();
        sv.velocity += self.frenet * Vector3::new(dvx, dvy, dvz);
        let Some(sv) =
            sv.intersect_soi_child(&self.moon_sv_init, &self.solver.moon, 1e-7, self.maxiter)
        else {
            return Ok(f64::MAX);
        };
        let obt = sv.clone().into_orbit(1e-8);

        let soi_factor = (sv.position.norm() - sv.body.soi).abs();
        let pe = obt.periapsis_radius();
        let lo_periapsis_factor =
            (self.solver.cs.moon_periapse_radius.start - pe).clamp(0.0, f64::MAX);
        let hi_periapsis_factor =
            (pe - self.solver.cs.moon_periapse_radius.end).clamp(0.0, f64::MAX);
        let deltav_factor = Vector3::new(dvx, dvy, dvz).norm_squared();

        Ok(100.0 * soi_factor
            + 50.0 * lo_periapsis_factor
            + 50.0 * hi_periapsis_factor
           + 1.0 * deltav_factor
	   // Discourage large radial and normal components
	   + 100.0 * dvy.abs() + 100.0 * dvz.abs())
    }
}

impl<'a> Anneal for TLIProblem2<'a> {
    type Param = Vec<f64>;

    type Output = Vec<f64>;

    type Float = f64;

    #[allow(clippy::cast_sign_loss)]
    fn anneal(&self, param: &Vec<f64>, temp: f64) -> Result<Self::Output, argmin_math::Error> {
        let mut param = param.clone();
        let distr = Slice::new(&[0usize, 0, 0, 0, 0, 0, 0, 1, 1, 2, 2]).unwrap();
        let step = Slice::new(&[-0.01 / 1000.0, 0.01 / 1000.0]).unwrap();
        let mut x = [-0.01 / 1000.0; 10];
        x[8] = 0.01 / 1000.0;
        x[9] = 0.01 / 1000.0;
        // 80% negative, 20% positive
        let weighted = Slice::new(&x).unwrap();
        // TODO: use a faster rng
        let mut rng = thread_rng();

        for _ in 0..=(temp.floor() as u64) {
            let idx = *rng.sample(distr);
            // Bias negative prograde when looking for non-retrograde orbits
            let val = if !self.allow_retrograde && idx == 0 {
                *rng.sample(weighted)
            } else {
                // Step size of .01 m/s
                *rng.sample(step)
            };

            param[idx] += val;
        }
        Ok(param)
    }
}

struct TLIProblem1<'a> {
    solver: &'a TLISolver,
}

impl<'a> CostFunction for TLIProblem1<'a> {
    type Param = Vec<f64>;

    type Output = f64;

    fn cost(&self, param: &Self::Param) -> Result<Self::Output, argmin::core::Error> {
        // Round to ms precision
        let flight_time = Duration::seconds_f64((param[0] * 1e+3).round() / 1e+3);
        let coast_time = Duration::seconds_f64((param[1] * 1e+3).round() / 1e+3);

        let Some(sv) = self
            .solver
            .cs
            .central_sv
            .clone()
            .propagate(coast_time, 1e-7, 500)
        else {
            return Ok(f64::MAX);
        };

        let Some(moon_sv) =
            self.solver
                .moon_sv_t0
                .clone()
                .propagate(flight_time + coast_time, 1e-7, 500)
        else {
            return Ok(f64::MAX);
        };
        let Some((v0, _v2)) = lambert::lambert(
            sv.position,
            moon_sv.position,
            flight_time.as_seconds_f64(),
            self.solver.central.mu,
            1e-15,
            500,
        )
        .min_by_key(|x| OrderedFloat((x.0 - sv.velocity).norm())) else {
            return Ok(f64::MAX);
        };

        let deltav = (v0 - sv.velocity).norm();

        Ok(deltav)
    }
}

// fn stumpff_s(x: f64) -> f64 {
//     if x < -1e-6 {
//         (libm::sinh(libm::sqrt(-x)) - libm::sqrt(-x)) / libm::sqrt((-x).powi(3))
//     } else if x.abs() <= 1e-6 {
//         1.0 / 6.0
//     } else {
//         (libm::sqrt(x) - libm::sin(libm::sqrt(x))) / libm::sqrt(x.powi(3))
//     }
// }

// fn stumpff_c(x: f64) -> f64 {
//     if x < -1e-6 {
//         (1.0 - libm::cosh(libm::sqrt(-x))) / x
//     } else if x.abs() <= 1e-6 {
//         1.0 / 2.0
//     } else {
//         (1.0 - libm::cos(libm::sqrt(x))) / x
//     }
// }

// fn b_matrix(
//     t: f64,
//     t0: f64,
//     body: &Arc<Body>,
//     r: Vector3<f64>,
//     r0: Vector3<f64>,
//     v: Vector3<f64>,
//     v0: Vector3<f64>,
// ) -> Matrix3<f64> {
//     let (f, g, f_dot, g_dot, psi, alpha) =
//         StateVector::universal_variables(r0, v0, body, t - t0, 1e-7, 500);
//     let alpha = -alpha;

//     let lambda = alpha * psi.powi(2);
//     let c_2 = stumpff_c(lambda);
//     let c_3 = stumpff_s(lambda);
//     let c_4 = if lambda <= 1e-6 {
//         1.0 / 24.0
//     } else {
//         (c_2 - 1.0 / 2.0) / lambda
//     };
//     let c_5 = if lambda <= 1e-6 {
//         1.0 / 120.0
//     } else {
//         (c_3 - 1.0 / 6.0) / lambda
//     };

//     let s_2 = psi.powi(2) * c_2;

//     let u = s_2 * (t - t0) + body.mu * (c_4 - 3.0 * c_5) * psi.powi(5);
//     g * Matrix3::identity() - u * v * v0.transpose()
//         + Matrix3x2::new(r.x, v.x, r.y, v.y, r.z, v.z)
//             * Matrix2::new(-f_dot * s_2, (-g_dot - 1.0) * s_2, (f - 1.0) * s_2, g * s_2)
//             * Matrix2x3::new(r0.x, r0.y, r0.z, v0.x, v0.y, v0.z)
// }

// fn d_matrix(
//     t: f64,
//     t0: f64,
//     body: &Arc<Body>,
//     r: Vector3<f64>,
//     r0: Vector3<f64>,
//     v: Vector3<f64>,
//     v0: Vector3<f64>,
// ) -> Matrix3<f64> {
//     let (_f, _g, f_dot, g_dot, psi, alpha) =
//         StateVector::universal_variables(r0, v0, body, t - t0, 1e-7, 500);
//     let alpha = -alpha;

//     let lambda = alpha * psi.powi(2);
//     let c_2 = stumpff_c(lambda);
//     let c_3 = stumpff_s(lambda);
//     let c_4 = if lambda <= 1e-6 {
//         1.0 / 24.0
//     } else {
//         (c_2 - 1.0 / 2.0) / lambda
//     };
//     let c_5 = if lambda <= 1e-6 {
//         1.0 / 120.0
//     } else {
//         (c_3 - 1.0 / 6.0) / lambda
//     };
//     let c_1 = 1.0 + lambda * c_3;

//     let s_2 = psi.powi(2) * c_2;
//     let s_1 = psi * c_1;

//     let accel = -body.mu * r / r.norm().powi(3);

//     let u = s_2 * (t - t0) + body.mu * (c_4 - 3.0 * c_5) * psi.powi(5);
//     g_dot * Matrix3::identity() - u * accel * v0.transpose()
//         + Matrix3x2::new(r.x, v.x, r.y, v.y, r.z, v.z)
//             * Matrix2::new(
//                 -(f_dot * s_1 + (g_dot - 1.0) / r.norm()) / r.norm(),
//                 -((g_dot - 1.0) * s_1) / r.norm(),
//                 f_dot * s_2,
//                 (g_dot - 1.0) * s_2,
//             )
//             * Matrix2x3::new(r0.x, r0.y, r0.z, v0.x, v0.y, v0.z)
// }

#[allow(clippy::too_many_arguments)]
pub fn tlmcc_opt_1(
    soi_ut: UT,
    lat_pe: f64,
    lng_pe: f64,
    h_pe: f64,
    i: f64,
    sv_cur: &StateVector,
    central: &Arc<Body>,
    moon: &Arc<Body>,
) -> Option<Vector3<f64>> {
    let delta_t = soi_ut - sv_cur.time;

    let moon_sv_t1 = moon
        .ephem
        .sv_bci(central)
        .propagate(soi_ut - moon.ephem.epoch, 1e-7, 500)?;

    let (_v_tc, v_t1) = lambert::lambert(
        sv_cur.position,
        moon_sv_t1.position,
        delta_t.as_seconds_f64(),
        central.mu,
        1e-7,
        500,
    )
    .min_by_key(|(v0, _)| OrderedFloat((v0 - sv_cur.velocity).norm()))?;

    let r_pe = h_pe + moon.radius;

    let r_pe_bcbf = r_pe
        * Vector3::new(
            libm::cos(lat_pe) * libm::cos(lng_pe),
            libm::cos(lat_pe) * libm::sin(lng_pe),
            libm::sin(lat_pe),
        );
    let r_pe_bci = StateVector {
        body: moon.clone(),
        frame: ReferenceFrame::BodyCenteredBodyFixed,
        position: r_pe_bcbf,
        velocity: Vector3::new(f64::NAN, f64::NAN, f64::NAN),
        time: soi_ut,
    }
    .reframe(ReferenceFrame::BodyCenteredInertial)
    .position;

    let v_pe_pqw_dir = Vector3::new(0.0, 1.0, 0.0);

    // TODO: make this more efficient
    let mut lan = 0.0;
    while (lan - consts::TAU).abs() > 1e-8 {
        let nv_bci = Rotation3::new(Vector3::new(0.0, 0.0, lan)) * Vector3::new(1.0, 0.0, 0.0);
        let argpe = if r_pe_bci.z >= 0.0 {
            libm::acos(nv_bci.normalize().dot(&r_pe_bci.normalize()))
        } else {
            consts::TAU - libm::acos(nv_bci.normalize().dot(&r_pe_bci.normalize()))
        };
        let pqw_bci_matrix = Orbit::pqw_ijk_matrix1(lan, argpe, i);
        let v_pe_bci_dir = pqw_bci_matrix * v_pe_pqw_dir;

        if (libm::acos(r_pe_bci.normalize().dot(&v_pe_bci_dir.normalize())) - consts::FRAC_PI_2)
            .abs()
            <= 0.01f64.to_radians()
        {
            break;
        }
        lan += 0.001f64.to_radians();
    }
    let nv_bci = Rotation3::new(Vector3::new(0.0, 0.0, lan)) * Vector3::new(1.0, 0.0, 0.0);
    let argpe = if r_pe_bci.z >= 0.0 {
        libm::acos(nv_bci.normalize().dot(&r_pe_bci.normalize()))
    } else {
        consts::TAU - libm::acos(nv_bci.normalize().dot(&r_pe_bci.normalize()))
    };
    let pqw_bci_matrix = Orbit::pqw_ijk_matrix1(lan, argpe, i);
    let v_pe_bci_dir = pqw_bci_matrix * v_pe_pqw_dir;

    // let v_pe_bcbf_dir = Vector3::new(
    //     libm::cos(i) * libm::cos(lng_pe + consts::FRAC_PI_2),
    //     libm::cos(i) * libm::sin(lng_pe + consts::FRAC_PI_2),
    //     libm::sin(i),
    // );
    // let v_pe_bci_dir = StateVector {
    //     body: moon.clone(),
    //     frame: ReferenceFrame::BodyCenteredBodyFixed,
    //     position: v_pe_bcbf_dir,
    //     velocity: Vector3::new(f64::NAN, f64::NAN, f64::NAN),
    //     time: soi_ut,
    // }
    // .reframe(ReferenceFrame::BodyCenteredInertial)
    // .position;

    let h_bci_dir = r_pe_bci
        .normalize()
        .cross(&v_pe_bci_dir.normalize())
        .normalize();

    let v_rel = v_t1 - moon_sv_t1.velocity;
    let a_r = moon.mu / v_rel.norm().powi(3) * v_rel;

    let delta_t_p = 0.0;
    //let delta_t_p = 0.0;

    let frenet = frenet(sv_cur).try_inverse()?;

    // let b_matrix = b_matrix(
    //     soi_ut.into_duration().as_seconds_f64(),
    //     sv_cur.time.into_duration().as_seconds_f64(),
    //     central,
    //     moon_sv_t1.position,
    //     sv_cur.position,
    //     v_t1,
    //     sv_cur.velocity,
    // );
    // trace!("b_matrix={b_matrix}");
    // let b_matrix_inv = b_matrix.try_inverse()?;
    // // Δv1 is ignored -- perturbing force
    // let v_inf_1 = v_rel - a_r / delta_t.as_seconds_f64();
    // let a_1 = moon.mu / v_inf_1.norm().powi(3) * v_inf_1;
    // let b_1 = libm::sqrt(r_pe * (r_pe + 2.0 * a_1.norm()))
    //     * v_inf_1
    //         .normalize()
    //         .cross(&h_bci_dir.normalize())
    //         .normalize();
    // let e_1 = libm::sqrt(1.0 + b_1.norm_squared() / a_1.norm_squared());
    // trace!("a_1={a_1}");
    // trace!("b_1={b_1}");
    // trace!("e_1={e_1}");
    // // Δr1 is ignored -- perturbing force
    // let delta_v_c_1 = b_matrix_inv
    //     * (b_1
    //         - v_inf_1 * delta_t_p
    //         - a_1
    //             * (libm::log(
    //                 (2.0 * v_inf_1.norm() * delta_t.as_seconds_f64()) / (a_1.norm() * e_1),
    //             ) - 2.0));
    // trace!("delta_v_c_1={}", frenet * delta_v_c_1);

    // let d_matrix = d_matrix(
    //     soi_ut.into_duration().as_seconds_f64(),
    //     sv_cur.time.into_duration().as_seconds_f64(),
    //     central,
    //     moon_sv_t1.position,
    //     sv_cur.position,
    //     v_t1,
    //     sv_cur.velocity,
    // );
    // trace!("d_matrix={d_matrix}");

    // let v_inf_2 = v_rel - a_r / delta_t.as_seconds_f64() + d_matrix * delta_v_c_1;
    // let a_2 = moon.mu / v_inf_2.norm().powi(3) * v_inf_2;
    // let b_2 = libm::sqrt(r_pe * (r_pe + 2.0 * a_2.norm()))
    //     * v_inf_2.normalize().cross(&h_bci_dir.normalize());
    // trace!("b_2={b_2}");
    // let e_2 = libm::sqrt(1.0 + b_2.norm_squared() / a_2.norm_squared());
    // // Δr1 is ignored -- perturbing force
    // let delta_v_c_2 = b_matrix_inv
    //     * (b_2
    //         - v_inf_2 * delta_t_p
    //         - a_2
    //             * (libm::log(
    //                 (2.0 * v_inf_2.norm() * delta_t.as_seconds_f64()) / (a_2.norm() * e_2),
    //             ) - 2.0));
    // trace!("delta_v_c_1={}", frenet * delta_v_c_2);

    let v_inf_3 = v_rel - a_r / delta_t.as_seconds_f64(); //+ d_matrix * delta_v_c_2;
    let a_3 = moon.mu / v_inf_3.norm().powi(3) * v_inf_3;
    let b_3 = libm::sqrt(r_pe * (r_pe + 2.0 * a_3.norm()))
        * v_inf_3
            .normalize()
            .cross(&h_bci_dir.normalize())
            .normalize();
    let e_3 = libm::sqrt(1.0 + b_3.norm_squared() / a_3.norm_squared());
    let r_1 = moon_sv_t1.position
        + (b_3
            - v_inf_3 * delta_t_p
            - a_3
                * (libm::log(
                    (2.0 * v_inf_3.norm() * delta_t.as_seconds_f64()) / (a_3.norm() * e_3),
                ) - 2.0));

    let (v_tcp, _v_t1) = lambert::lambert(
        sv_cur.position,
        r_1,
        delta_t.as_seconds_f64(),
        central.mu,
        1e-7,
        500,
    )
    .min_by_key(|(v0, _)| OrderedFloat((v0 - sv_cur.velocity).norm()))?;

    Some(frenet * (v_tcp - sv_cur.velocity))

    // let r_nominal = sv_soi.position;
    // let r_geti = sv_cur.position;
    // let v_geti_init = sv_cur.velocity;

    // let (v0, _) = lambert::lambert(
    //     r_geti,
    //     r_nominal,
    //     delta_t.as_seconds_f64(),
    //     central.mu,
    //     1e-7,
    //     500,
    // )
    // .min_by_key(|(v0, _)| OrderedFloat((v0 - v_geti_init).norm()))?;

    // Some(frenet(sv_cur).try_inverse()? * (v0 - v_geti_init))
}
