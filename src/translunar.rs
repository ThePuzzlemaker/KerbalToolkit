//! Translunar trajectory generation and correction
#![allow(non_snake_case)]

use std::{f64::consts, ops::Range};

use argmin::{
    core::{CostFunction, Executor, Gradient},
    solver::{
        particleswarm::ParticleSwarm,
        simulatedannealing::{Anneal, SATempFunc, SimulatedAnnealing},
    },
};
use nalgebra::Vector3;
use ordered_float::OrderedFloat;
use rand::{
    distributions::{Slice, Uniform},
    thread_rng, Rng,
};
use time::Duration;
use tracing::trace;

use crate::{
    bodies::Body,
    kepler::{lambert, orbits::StateVector},
    time::UT,
};

#[allow(non_snake_case)]
pub struct TLISolver {
    cs: TLIConstraintSet,
    central: Body,
    moon: Body,
    moon_sv_t0: StateVector,
    opt_periapse: bool,
    allow_retrograde: bool,
}

#[allow(non_snake_case)]
pub struct TLIConstraintSet {
    pub central_sv: StateVector,
    // pub min_time: UT,
    pub flight_time: Range<Duration>,
    pub moon_periapse_radius: Range<f64>,
    // pub moon_inclination: f64,
    // pub moon_lan: f64,
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
        central: Body,
        moon: Body,
        t0: UT,
        opt_periapse: bool,
        allow_retrograde: bool,
    ) -> Self {
        let moon_sv_t0 = moon
            .ephem
            .sv_bci(&central)
            .propagate(t0 - moon.ephem.epoch, 1e-7, 35);
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
        let flight_time = params[0];
        let sv_init =
            self.cs
                .central_sv
                .clone()
                .propagate(Duration::seconds_f64(params[1]), 1e-7, 35);

        let moon_sv = self.moon_sv_t0.clone().propagate(
            Duration::seconds_f64(params[0] + params[1]),
            1e-7,
            35,
        );

        let r0 = sv_init.position;
        let (v0, _v2) = lambert::lambert(r0, moon_sv.position, params[0], self.central.mu, 1e-15, 35)
                .min_by_key(|x| OrderedFloat((x.0 - sv_init.velocity).norm()))
                // TODO: error?
                ?;

        let deltav = v0 - sv_init.velocity;

        let moon_sv_init =
            self.moon_sv_t0
                .clone()
                .propagate(Duration::seconds_f64(params[1]), 1e-7, 35);

        let deltav = if self.opt_periapse {
            let problem = TLIProblem2 {
                solver: self,
                sv_init: sv_init.clone(),
                moon_sv_init: moon_sv_init.clone(),
                dv_init: deltav,
                allow_retrograde: self.allow_retrograde,
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

            let deltav = Vector3::new(params[0], params[1], params[2]);

            let mut sv = sv_init.clone();
            sv.velocity += deltav;
            let sv = sv.intersect_soi_child(&moon_sv_init, &self.moon, 1e-7, 35)?;
            let obt = sv.clone().into_orbit(1e-8);
            if !self
                .cs
                .moon_periapse_radius
                .contains(&obt.periapsis_radius())
            {
                return None;
            }
            deltav
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
    dv_init: Vector3<f64>,
    allow_retrograde: bool,
}

impl<'a> Gradient for TLIProblem2<'a> {
    type Param = Vec<f64>;

    type Gradient = Vec<f64>;

    fn gradient(&self, param: &Self::Param) -> Result<Self::Gradient, argmin_math::Error> {
        let dvx = param[0];
        let dvy = param[1];
        let dvz = param[2];

        Ok(vec![2.0 * dvx, 2.0 * dvy, 2.0 * dvz])
    }
}

impl<'a> CostFunction for TLIProblem2<'a> {
    type Param = Vec<f64>;

    type Output = f64;

    fn cost(&self, param: &Self::Param) -> Result<Self::Output, argmin_math::Error> {
        let dvx = param[0];
        let dvy = param[1];
        let dvz = param[2];

        let mut sv = self.sv_init.clone();
        sv.velocity += Vector3::new(dvx, dvy, dvz);
        let Some(sv) = sv.intersect_soi_child(&self.moon_sv_init, &self.solver.moon, 1e-7, 35)
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
        let deltav_deviation = (Vector3::new(dvx, dvy, dvz) - self.dv_init).norm_squared();

        let retrograde_weight = if self.allow_retrograde { 0.0 } else { 100.0 };
        let inc = obt.i.rem_euclid(consts::TAU);
        let retrograde_factor = if (inc - consts::PI).abs() < 1e-6 {
            // Let's be reasonable.
            1e+6
        } else {
            // We take one over the retrograde factor as we want to have a high factor the smaller the difference is
            1.0 / (inc - consts::PI).abs()
        };

        Ok(100.0 * soi_factor
            + retrograde_weight * retrograde_factor
            + 50.0 * lo_periapsis_factor
            + 5.0 * hi_periapsis_factor
            + 1.0 * deltav_factor
            + 10.0 * deltav_deviation)
    }
}

impl<'a> Anneal for TLIProblem2<'a> {
    type Param = Vec<f64>;

    type Output = Vec<f64>;

    type Float = f64;

    fn anneal(&self, param: &Vec<f64>, temp: f64) -> Result<Self::Output, argmin_math::Error> {
        let mut param = param.clone();
        let distr = Uniform::from(0..param.len());
        let step = Slice::new(&[-0.01 / 1000.0, 0.01 / 1000.0]).unwrap();
        // TODO: use a faster rng
        let mut rng = thread_rng();

        for _ in 0..(temp.floor() as u64 + 1) {
            let idx = rng.sample(distr);

            // Step size of .01 m/s
            let val = rng.sample(step);

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
        let flight_time = Duration::seconds_f64(param[0]);
        let coast_time = Duration::seconds_f64(param[1]);

        let sv = self
            .solver
            .cs
            .central_sv
            .clone()
            .propagate(coast_time, 1e-7, 35);

        let moon_sv = self
            .solver
            .moon_sv_t0
            .clone()
            .propagate(flight_time + coast_time, 1e-7, 35);
        let Some((v0, _v2)) = lambert::lambert(
            sv.position,
            moon_sv.position,
            flight_time.as_seconds_f64(),
            self.solver.central.mu,
            1e-15,
            35,
        )
        .min_by_key(|x| OrderedFloat((x.0 - sv.velocity).norm())) else {
            return Ok(f64::MAX);
        };

        let deltav = (v0 - sv.velocity).norm();

        Ok(deltav)
    }
}
