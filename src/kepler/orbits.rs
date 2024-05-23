//! Keplerian orbits.

use std::f64::consts;

use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use time::Duration;

use crate::{bodies::Body, time::UT};

/// A Keplerian orbit.
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Orbit {
    /// Semi-latus rectum (km).
    pub p: f64,
    /// Eccentricity (dimensionless).
    pub e: f64,
    /// Inclination (radians).
    pub i: f64,
    /// Longitude of ascending node (radians).
    pub lan: f64,
    /// Argument of periapsis (radians).
    pub argpe: f64,
    /// The epoch at true anomaly (UT seconds).
    pub epoch: UT,
    /// True anomaly (radians).
    pub ta: f64,
}

impl Orbit {
    pub fn periapsis_radius(&self) -> f64 {
        self.p / (1.0 + self.e)
    }

    pub fn apoapsis_radius(&self) -> f64 {
        self.p / (1.0 - self.e)
    }

    /// Calculate the position and velocity in the perifocal
    /// coordinate system PQW at an orbit's current true anomaly.
    fn sv_pqw(&self, body: &Body) -> (Vector3<f64>, Vector3<f64>) {
        let r = self.p / (1.0 + self.e * libm::cos(self.ta));
        let rv = r * libm::cos(self.ta) * Vector3::new(1.0, 0.0, 0.0)
            + r * libm::sin(self.ta) * Vector3::new(0.0, 1.0, 0.0);
        let vv = libm::sqrt(body.mu / self.p)
            * (-libm::sin(self.ta) * Vector3::new(1.0, 0.0, 0.0)
                + (self.e + libm::cos(self.ta)) * Vector3::new(0.0, 1.0, 0.0));
        (rv, vv)
    }

    fn pqw_ijk_matrix(&self) -> Matrix3<f64> {
        let m11 = libm::cos(self.lan) * libm::cos(self.argpe)
            - libm::sin(self.lan) * libm::sin(self.argpe) * libm::cos(self.i);
        let m12 = -libm::cos(self.lan) * libm::sin(self.argpe)
            - libm::sin(self.lan) * libm::cos(self.argpe) * libm::cos(self.i);
        let m13 = libm::sin(self.lan) * libm::sin(self.i);
        let m21 = libm::sin(self.lan) * libm::cos(self.argpe)
            + libm::cos(self.lan) * libm::sin(self.argpe) * libm::cos(self.i);
        let m22 = -libm::sin(self.lan) * libm::sin(self.argpe)
            + libm::cos(self.lan) * libm::cos(self.argpe) * libm::cos(self.i);
        let m23 = -libm::cos(self.lan) * libm::sin(self.i);
        let m31 = libm::sin(self.argpe) * libm::sin(self.i);
        let m32 = libm::cos(self.argpe) * libm::sin(self.i);
        let m33 = libm::cos(self.i);

        Matrix3::new(m11, m12, m13, m21, m22, m23, m31, m32, m33)
    }

    /// Calculate the position and velocity in the body-centered
    /// equatorial coordinate system IJK at an orbit's current true
    /// anomaly.
    pub fn sv_bci(&self, body: &Body) -> StateVector {
        let (rv, vv) = self.sv_pqw(body);
        let mat = self.pqw_ijk_matrix();
        let rv = mat * rv;
        let vv = mat * vv;
        StateVector {
            body: body.clone(),
            frame: ReferenceFrame::BodyCenteredInertial,
            position: rv,
            velocity: vv,
            time: self.epoch,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ReferenceFrame {
    BodyCenteredInertial,
}

#[derive(Clone, Debug, PartialEq)]
pub struct StateVector {
    pub body: Body,
    pub frame: ReferenceFrame,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub time: UT,
}

impl StateVector {
    /// Convert this state vector into an [`Orbit`].
    ///
    /// Recommended tolerance (`tol`): `1e-8`.
    pub fn into_orbit(self, tol: f64) -> Orbit {
        assert_eq!(self.frame, ReferenceFrame::BodyCenteredInertial);
        let rv = self.position;
        let r = rv.norm();
        let vv = self.velocity;
        let v = vv.norm();
        let hv = rv.cross(&vv);
        let h = hv.norm();
        let nv = Vector3::new(0.0, 0.0, 1.0).cross(&hv);
        let ev = 1.0 / self.body.mu * ((v.powi(2) - self.body.mu / r) * rv - rv.dot(&vv) * vv);
        let p = h.powi(2) / self.body.mu;
        let e = ev.norm();
        let i = libm::acos(hv[2] / h);

        let circular = e < tol;
        let equatorial = i.abs() < tol;

        let (lan, argpe, ta) = if equatorial && !circular {
            (
                0.0,
                // Longitude of periapsis
                libm::atan2(ev[1], ev[0]) % (2.0 * consts::PI),
                libm::atan2(hv.dot(&ev.cross(&rv)) / h, rv.dot(&ev)),
            )
        } else if !equatorial && circular {
            (
                libm::atan2(nv[1], nv[0]) % (2.0 * consts::PI),
                0.0,
                // Argument of latitude
                libm::atan2(rv.dot(&hv.cross(&nv)) / h, rv.dot(&nv)),
            )
        } else if equatorial && circular {
            (
                0.0,
                0.0,
                // True longitude
                libm::atan2(rv[1], rv[0]) % (2.0 * consts::PI),
            )
        } else {
            let a = p / (1.0 - e.powi(2));
            let mua = self.body.mu * a;

            let ta = if a > 0.0 {
                let e_se = rv.dot(&vv) / libm::sqrt(mua);
                let e_ce = r * vv.dot(&vv) / self.body.mu - 1.0;
                e_to_ta(libm::atan2(e_se, e_ce), e)
            } else {
                let e_sh = rv.dot(&vv) / libm::sqrt(-mua);
                let e_ch = r * vv.norm_squared() / self.body.mu - 1.0;
                f_to_ta(libm::log((e_ch + e_sh) / (e_ch - e_sh)) / 2.0, e)
            };

            let lan = libm::atan2(nv[1], nv[0]) % (2.0 * consts::PI);
            let px = rv.dot(&nv);
            let py = (rv.dot(&hv.cross(&nv))) / h;
            let argpe = (libm::atan2(py, px) - ta) % (2.0 * consts::PI);

            (lan, argpe, ta)
        };

        let ta = (ta + consts::PI) % (2.0 * consts::PI) - consts::PI;
        Orbit {
            p,
            e,
            i,
            lan,
            argpe,
            epoch: self.time,
            ta,
        }
    }

    /// Propagate this orbit `delta_t` seconds.
    ///
    /// Recommended tolerance: `tol = 1e-7`, `maxiter = 35`.
    pub fn propagate(self, delta_t: Duration, tol: f64, maxiter: u64) -> StateVector {
        assert_eq!(self.frame, ReferenceFrame::BodyCenteredInertial);
        let delta_t = delta_t.as_seconds_f64();
        // let xi = self.velocity.norm_squared() / 2.0 - self.body.mu / self.position.norm(); // what is this used for??
        let alpha = -self.velocity.norm_squared() / self.body.mu + 2.0 / self.position.norm();

        let mut xn_new = if alpha > 1e-6 {
            if (alpha - 1.0).abs() < f64::EPSILON {
                panic!("StateVector::propagate({self:?}, {delta_t:?}, {tol}, {maxiter}): too close to converge");
            }
            libm::sqrt(self.body.mu) * delta_t * alpha
        } else if alpha.abs() < 1e-6 {
            let h = self.position.cross(&self.velocity);
            let p = h.norm_squared() / self.body.mu;
            let s = libm::atan2(1.0, 3.0 * delta_t * libm::sqrt(self.body.mu / p.powi(3)));
            let w = libm::atan(libm::cbrt(libm::tan(s)));
            libm::sqrt(p) * 2.0 * 1.0 / libm::tan(2.0 * w)
        } else if alpha < -1e-6 {
            let a = 1.0 / alpha;
            delta_t.signum()
                * libm::sqrt(-a)
                * libm::log(
                    (-2.0 * self.body.mu * alpha * delta_t)
                        / (self.position.dot(&self.velocity)
                            + delta_t.signum()
                                * libm::sqrt(-self.body.mu * a)
                                * (1.0 - self.position.norm() * alpha)),
                )
        } else {
            unreachable!("oops")
        };

        let mut xn = f64::NAN;
        let mut c2 = f64::NAN;
        let mut c3 = f64::NAN;
        let mut r = f64::NAN;
        let mut psi = f64::NAN;
        let dot_r0v0 = self.position.dot(&self.velocity);
        let norm_r0 = self.position.norm();
        let sqrt_mu = libm::sqrt(self.body.mu);
        let mut iter = 0;
        while iter < maxiter {
            xn = xn_new;
            psi = xn.powi(2) * alpha;
            (c2, c3) = calc_c2c3(psi);
            r = xn * xn * c2
                + dot_r0v0 / sqrt_mu * xn * (1.0 - psi * c3)
                + norm_r0 * (1.0 - psi * c2);
            xn_new = xn
                + (sqrt_mu * delta_t
                    - xn * xn * xn * c3
                    - dot_r0v0 / sqrt_mu * xn * xn * c2
                    - norm_r0 * xn * (1.0 - psi * c3))
                    / r;

            if (xn_new - xn).abs() < tol {
                break;
            }

            iter += 1;
        }
        if iter == maxiter {
            panic!(
                "StateVector::propagate({self:?}, {delta_t:?}, {tol}, {maxiter}): failed to converge"
            )
        }

        let f = 1.0 - xn.powi(2) / norm_r0 * c2;
        let g = delta_t - xn.powi(3) / sqrt_mu * c3;

        let gdot = 1.0 - xn.powi(2) / r * c2;
        let fdot = sqrt_mu / (r * norm_r0) * xn * (psi * c3 - 1.0);

        // assert!(((f*gdot + fdot * g).abs() - 1.0).abs() < 1e-6, "StateVector::propagate({self:?}, {delta_t:?}, {tol}, {maxiter}): converged solution was incorrect: {}", f*gdot + fdot*g); // Not sure if this is a me problem but this seems to trigger erroneously.

        let position = f * self.position + g * self.velocity;
        let velocity = fdot * self.position + gdot * self.velocity;

        StateVector {
            body: self.body,
            frame: self.frame,
            position,
            velocity,
            time: self.time + Duration::seconds_f64(delta_t),
        }
    }
}

fn e_to_ta(e: f64, ecc: f64) -> f64 {
    2.0 * libm::atan(libm::sqrt((1.0 + ecc) / (1.0 - ecc)) * libm::tan(e / 2.0))
}

fn f_to_ta(f: f64, ecc: f64) -> f64 {
    2.0 * libm::atan(libm::sqrt((ecc + 1.0) / (ecc - 1.0)) * libm::tanh(f / 2.0))
}

fn calc_c2c3(psi: f64) -> (f64, f64) {
    if psi > 1e-6 {
        let c2 = (1.0 - libm::cos(libm::sqrt(psi))) / psi;
        let c3 = (libm::sqrt(psi) - libm::sin(libm::sqrt(psi))) / (psi * libm::sqrt(psi));
        (c2, c3)
    } else if psi < -1e-6 {
        let c2 = (1.0 - libm::cosh(libm::sqrt(-psi))) / psi;
        let c3 = (libm::sinh(libm::sqrt(-psi)) - libm::sqrt(-psi)) / libm::sqrt((-psi).powi(3));
        (c2, c3)
    } else {
        (1.0 / 2.0, 1.0 / 6.0)
    }
}

pub fn ma_to_ta(ma: f64, e: f64, tol: f64, maxiter: u64) -> f64 {
    assert!(e >= 0.0);
    if e >= 1.0 {
        todo!("ma_to_ta: hyperbolic and parabolic");
    }

    let ea = ma_to_ea(ma, e, tol, maxiter);
    ea_to_ta(ea, e)
}

pub fn ea_to_ta(ea: f64, e: f64) -> f64 {
    assert!((0.0..1.0).contains(&e));
    let beta = e / (1.0 + libm::sqrt(1.0 - e.powi(2)));
    ea + 2.0 * libm::atan2(beta * libm::sin(ea), 1.0 - beta * libm::cos(ea))
}

pub fn ma_to_ea(ma: f64, e: f64, tol: f64, maxiter: u64) -> f64 {
    assert!((0.0..1.0).contains(&e));

    let mut ea_new = if -consts::PI < ma && ma < 0.0 || ma > consts::PI {
        ma - e
    } else {
        ma + e
    };

    let mut ea;
    let mut iter = 0;
    while iter < maxiter {
        ea = ea_new;
        ea_new = ea + (ma - ea + e * libm::sin(ea)) / (1.0 - e * libm::cos(ea));

        if (ea_new - ea).abs() < tol {
            return ea_new;
        }

        iter += 1;
    }
    panic!("ma_to_ea({ma}, {e}, {tol}, {maxiter}): failed to converge");
}
