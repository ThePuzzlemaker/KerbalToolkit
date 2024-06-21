//! A relatively quick and robust Lambert's problem solver.
//!
//! This solver is based on ["Revisiting Lambert's Problem" (Izzo
//! 2014)][1] and [poliastro's `poliastro.iod.izzo`][2].  (the latter
//! of which is licensed under MIT).
//!
//! [1]: https://arxiv.org/abs/1403.2705
//! [2]: https://github.com/poliastro/poliastro/blob/c7d12e9b715d3fd60f2be233af707d5b97617d39/src/poliastro/iod/izzo.py

use std::iter;

use nalgebra::Vector3;

use crate::math::hyp2f1;

fn calc_y(x: f64, lambda: f64) -> f64 {
    (1.0 - lambda.powi(2) * (1.0 - x.powi(2))).sqrt()
}

fn calc_psi(x: f64, y: f64, lambda: f64) -> f64 {
    if (-1.0..1.0).contains(&x) {
        // elliptic orbits
        (x * y + lambda * (1.0 - x.powi(2))).acos()
    } else if x > 1.0 {
        // hyperbolic orbits
        ((y - x * lambda) * (x.powi(2) - 1.0).sqrt()).asinh()
    } else {
        // parabolic orbits
        0.0
    }
}

fn tof(x: f64, y: f64, t0: f64, lambda: f64, m: i64) -> f64 {
    let t = if m == 0 && (0.6f64.sqrt()..1.4f64.sqrt()).contains(&x) {
        let eta = y - lambda * x;
        let s1 = 1.0 / 2.0 * (1.0 - lambda - x * eta);
        let q = 4.0 / 3.0 * hyp2f1(s1);
        1.0 / 2.0 * (eta.powi(3) * q + 4.0 * lambda * eta)
    } else {
        1.0 / (1.0 - x.powi(2))
            * ((calc_psi(x, y, lambda) + m as f64 * std::f64::consts::PI)
                / ((1.0 - x.powi(2)).abs()).sqrt()
                - x
                + lambda * y)
    };

    t - t0
}

fn dtof(x: f64, y: f64, t: f64, lambda: f64) -> f64 {
    (3.0 * t * x - 2.0 + 2.0 * lambda.powi(3) * x / y) / (1.0 - x.powi(2))
}

fn d2tof(x: f64, y: f64, t: f64, dt: f64, lambda: f64) -> f64 {
    (3.0 * t + 5.0 * x * dt + 2.0 * (1.0 - lambda.powi(2)) * lambda.powi(3) / y.powi(3))
        / (1.0 - x.powi(2))
}

fn d3tof(x: f64, y: f64, _: f64, dt: f64, ddt: f64, lambda: f64) -> f64 {
    (7.0 * x * ddt + 8.0 * dt - 6.0 * (1.0 - lambda.powi(2)) * lambda.powi(5) * x / y.powi(5))
        / (1.0 - x.powi(2))
}

fn calc_tmin(lambda: f64, m: i64, tol: f64, maxiter: u64) -> Option<f64> {
    if lambda == 1.0 {
        Some(tof(0.0, calc_y(0.0, lambda), 0.0, lambda, m))
    } else if m == 0 {
        Some(0.0)
    } else {
        // set x_i > 0 to avoid FP issues at λ = -1
        let x_i = 0.1;
        let t_i = tof(x_i, calc_y(x_i, lambda), 0.0, lambda, m);
        let x_tmin = halley(x_i, t_i, lambda, m, tol, maxiter)?;
        Some(tof(x_tmin, calc_y(x_tmin, lambda), 0.0, lambda, m))
    }
}

fn halley(mut x0: f64, t0: f64, lambda: f64, m: i64, tol: f64, maxiter: u64) -> Option<f64> {
    let mut iter = maxiter;
    while iter > 0 {
        let y = calc_y(x0, lambda);
        let f = tof(x0, y, t0, lambda, m);
        let t = f + t0;
        let df = dtof(x0, y, t, lambda);
        let d2f = d2tof(x0, y, t, df, lambda);
        if d2f == 0.0 {
            // TODO: don't panic
            return None;
            //panic!("halley({x0}, {t0}, {lambda}, {m}, {tol}, {maxiter}): derivative was zero");
        }
        let d3f = d3tof(x0, y, t, df, d2f, lambda);

        let x = x0 - 2.0 * df * d2f / (2.0 * d2f.powi(2) - df * d3f);

        if (x - x0).abs() < tol {
            return Some(x);
        }

        x0 = x;
        iter -= 1;
    }

    None
    //panic!("halley({x0}, {t0}, {lambda}, {m}, {tol}, {maxiter}): failed to converge")
}

fn householder(mut x0: f64, t0: f64, lambda: f64, m: i64, tol: f64, maxiter: u64) -> Option<f64> {
    let mut iter = maxiter;
    while iter > 0 {
        let y = calc_y(x0, lambda);
        let f = tof(x0, y, t0, lambda, m);
        let t = f + t0;
        let df = dtof(x0, y, t, lambda);
        let d2f = d2tof(x0, y, t, df, lambda);
        let d3f = d3tof(x0, y, t, df, d2f, lambda);

        let x = x0
            - f * ((df.powi(2) - f * d2f / 2.0)
                / (df * (df.powi(2) - f * d2f) + d3f * f.powi(2) / 6.0));

        if (x - x0).abs() < tol {
            return Some(x);
        }

        x0 = x;
        iter -= 1;
    }

    None
    //panic!("householder({x0}, {t0}, {lambda}, {m}, {tol}, {maxiter}): failed to converge");
}

fn findxy(lambda: f64, t: f64, tol: f64, maxiter: u64) -> Option<(Vec<f64>, Vec<f64>)> {
    if lambda.abs() >= 1.0 || t <= 0.0 {
        return None;
    }
    // assert!(lambda.abs() < 1.0);
    // assert!(t > 0.0);
    let pi = std::f64::consts::PI;
    let mut mmax = (t / pi).floor() as i64;
    let t00 = lambda.acos() + lambda * (1.0 - lambda.powi(2)).sqrt();
    if t < t00 + mmax as f64 * pi && mmax > 0 {
        let tmin = calc_tmin(lambda, mmax, tol, maxiter)?;
        if t < tmin {
            mmax -= 1;
        }
    }

    let t1 = 2.0 / 3.0 * (1.0 - lambda.powi(3));
    let x0 = if t >= t00 {
        (t00 / t).powf(2.0 / 3.0) - 1.0
    } else if t < t1 {
        5.0 / 2.0 * t1 / t * (t1 - t) / (1.0 - lambda.powi(5)) + 1.0
    } else {
        (2.0f64.ln() * (t / t00).ln() / (t1 / t00).ln()).exp() - 1.0
    };

    let mut xs = vec![];
    let mut ys = vec![];
    let x = householder(x0, t, lambda, 0, tol, maxiter)?;
    let y = calc_y(x, lambda);
    xs.push(x);
    ys.push(y);

    while mmax > 0 {
        let m = mmax;
        let x0l = (((m as f64 * pi + pi) / 8.0 * t).powf(2.0 / 3.0) - 1.0)
            / (((m as f64 * pi + pi) / 8.0 * t).powf(2.0 / 3.0) + 1.0);
        let x0r = ((8.0 * t / m as f64 * pi).powf(2.0 / 3.0) - 1.0)
            / ((8.0 * t / m as f64 * pi).powf(2.0 / 3.0) + 1.0);

        let xl = householder(x0l, t, lambda, m, tol, maxiter)?;
        let yl = calc_y(xl, lambda);

        let xr = householder(x0r, t, lambda, m, tol, maxiter)?;
        let yr = calc_y(xr, lambda);

        xs.push(xl);
        xs.push(xr);
        ys.push(yl);
        ys.push(yr);

        mmax -= 1;
    }

    Some((xs, ys))
}

/// Lambert's problem.
///
/// Given position vectors `r1v` and `r2v`, time-of-flight `tof`, and
/// gravitational parameter `mu`, calculate all possible velocity
/// vectors `v1` and `v2` for the corresponding orbits.
///
/// Note that this function is unit-agnostic, however the distance
/// units of the position vectors `r1`, `r2` and the time unit of the
/// time-of-flight `tof` must match with the respective distance and
/// time units of the gravitational parameter `mu`.
///
/// # Arguments
///
/// - `r1v`, `r2v`: position vectors.
/// - `tof`: time-of-flight.
/// - `mu`: standard gravitational parameter for the orbited body.
/// - `tol`: floating point tolerance. Values smaller than `1e-15`
///   will typically fail to converge. Recommended value: `1e-15`.
/// - `maxiter`: maximum iterations for convergence. This function
///   will panic if it could not converge within `maxiter`
///   iterations. Recommended value: `500`
pub fn lambert(
    r1v: Vector3<f64>,
    r2v: Vector3<f64>,
    tof: f64,
    mu: f64,
    tol: f64,
    maxiter: u64,
) -> Box<dyn Iterator<Item = (Vector3<f64>, Vector3<f64>)>> {
    let r1 = r1v.norm();
    let r2 = r2v.norm();
    let cv = r2v - r1v;
    let c = cv.norm();

    let s = 1.0 / 2.0 * (r1 + r2 + c);
    let ir1 = r1v / r1;
    let ir2 = r2v / r2;
    let ih = ir1.cross(&ir2);
    let ih = ih / ih.norm();

    let mut lambda = (1.0 - c / s).sqrt();
    let (it1, it2) = if (r1v[0] * r2v[1] - r1v[1] * r2v[0]) < 0.0 {
        lambda = -lambda;
        (ir1.cross(&ih), ir2.cross(&ih))
    } else {
        (ih.cross(&ir1), ih.cross(&ir2))
    };
    let t = tof * ((2.0 * mu) / (s.powi(3))).sqrt();
    // TODO: lazy findxy
    let Some((xs, ys)) = findxy(lambda, t, tol, maxiter) else {
        return Box::new(iter::empty());
    };

    let gamma = (mu * s / 2.0).sqrt();
    let rho = (r1 - r2) / c;
    let sigma = (1.0 - rho.powi(2)).sqrt();
    Box::new(xs.into_iter().zip(ys).map(move |(x, y)| {
        let vr1 = gamma * ((lambda * y - x) - rho * (lambda * y + x)) / r1;
        let vr2 = -gamma * ((lambda * y - x) + rho * (lambda * y + x)) / r2;
        let vt1 = gamma * sigma * (y + lambda * x) / r1;
        let vt2 = gamma * sigma * (y + lambda * x) / r2;
        let v1 = vr1 * (r1v / r1) + vt1 * it1;
        let v2 = vr2 * (r2v / r2) + vt2 * it2;
        (v1, v2)
    }))
}

#[test]
fn it_works() {
    let r0 = Vector3::new(15945.34, 0.0, 0.0);
    let r = Vector3::new(12214.83399, 10249.46731, 0.0);
    let tof = 76.0 * 60.0 /* min */;
    let mu = 3.986004418e5;

    let expected_v1 = Vector3::new(2.058912566174178, 2.9159645911539527, 0.0);
    let expected_v2 = Vector3::new(-3.4515665032801133, 0.9103135416619974, 0.0);

    let (v1, v2) = lambert(r0, r, tof, mu, f64::EPSILON, 500).next().unwrap();

    assert_eq!(v1, expected_v1);
    assert_eq!(v2, expected_v2);
}
