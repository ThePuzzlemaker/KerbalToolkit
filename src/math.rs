//! Math utilities.

/// The hypergeometric function `₂F₁(3, 1, 5/2, x)`.
pub fn hyp2f1(x: f64) -> f64 {
    if x >= 1.0 {
        f64::INFINITY
    } else {
        let mut res = 1.0;
        let mut term = 1.0;
        let mut i = 0;
        loop {
            let ii = i as f64;
            term = term * (3.0 + ii) * (1.0 + ii) / (5.0 / 2.0 + ii) * x / (ii + 1.0);
            let res_old = res;
            res += term;
            if res_old == res {
                return res;
            }
            i += 1;
        }
    }
}
