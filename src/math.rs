//! Math utilities.
use std::cmp;

use ordered_float::OrderedFloat;

use crate::misc::SortedList;

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

/// Cubic hermite spline, as implemented by Unity's `AnimationCurve`.
// TODO: rewrite without using SortedList as direct float comparison
// is... janky
#[derive(Clone, Debug)]
pub struct H1 {
    min_time: f64,
    max_time: f64,
    last_lo: i32,
    list: SortedList<OrderedFloat<f64>, HFrame>,
}

impl Default for H1 {
    fn default() -> Self {
        Self {
            min_time: f64::MAX,
            max_time: f64::MIN,
            last_lo: -1,
            list: SortedList::new(),
        }
    }
}

impl H1 {
    #[allow(clippy::too_many_arguments)]
    fn interpolant(
        &mut self,
        x1: f64,
        y1: f64,
        yp1: f64,
        x2: f64,
        y2: f64,
        yp2: f64,
        x: f64,
    ) -> f64 {
        let t = (x - x1) / (x2 - x1);
        let t2 = t * t;
        let t3 = t2 * t;
        let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        let h10 = t3 - 2.0 * t2 + t;
        let h01 = -2.0 * t3 + 3.0 * t2;
        let h11 = t3 - t2;
        h00 * y1 + h10 * (x2 - x1) * yp1 + h01 * y2 + h11 * (x2 - x1) * yp2
    }

    pub fn add(&mut self, time: f64, value: f64) {
        self.list.insert(
            OrderedFloat(time),
            HFrame {
                in_tangent: 0.0,
                out_tangent: 0.0,
                time,
                value,
                auto_tangent: true,
            },
        );
        self.min_time = cmp::min(OrderedFloat(self.min_time), OrderedFloat(time)).0;
        self.max_time = cmp::max(OrderedFloat(self.max_time), OrderedFloat(time)).0;
        self.recompute_tangents(self.list.find_first_position(&OrderedFloat(time)).unwrap() as i32);
        self.last_lo = -1;
    }

    pub fn add_with_tangents(&mut self, time: f64, value: f64, in_tangent: f64, out_tangent: f64) {
        self.list.insert(
            OrderedFloat(time),
            HFrame {
                in_tangent,
                out_tangent,
                time,
                value,
                auto_tangent: false,
            },
        );
        self.min_time = cmp::min(OrderedFloat(self.min_time), OrderedFloat(time)).0;
        self.max_time = cmp::max(OrderedFloat(self.max_time), OrderedFloat(time)).0;
        self.recompute_tangents(self.list.find_first_position(&OrderedFloat(time)).unwrap() as i32);
        self.last_lo = -1;
    }

    pub fn add_with_tangent(&mut self, time: f64, value: f64, tangent: f64) {
        if let Some(frame) = self.list.first_value_of_mut(&OrderedFloat(time)) {
            frame.value = value;
            frame.out_tangent = tangent;
        } else {
            self.add_with_tangents(time, value, tangent, tangent)
        }
    }

    fn find_index(&mut self, value: f64) -> i32 {
        assert!(self.list.len() > 1);
        assert!(value > self.min_time);
        assert!(value < self.max_time);

        if self.last_lo > 0 && value > *self.list.keys[self.last_lo as usize] {
            if value < *self.list.keys[(self.last_lo + 1) as usize] {
                return !(self.last_lo + 1);
            }

            if value == *self.list.keys[(self.last_lo + 1) as usize] {
                self.last_lo += 1;
                return self.last_lo;
            }

            if value > *self.list.keys[(self.last_lo + 1) as usize]
                && value < *self.list.keys[(self.last_lo + 2) as usize]
            {
                self.last_lo += 1;
                return !(self.last_lo + 1);
            }
        }

        let mut lo = 0;
        let mut hi = self.list.len() - 1;

        while lo <= hi {
            let i = lo + ((hi - lo) >> 1);
            let order = value.partial_cmp(&self.list.keys[i].0).unwrap();
            match order {
                cmp::Ordering::Less => {
                    hi = i - 1;
                }
                cmp::Ordering::Equal => {
                    self.last_lo = i as i32;
                    return i as i32;
                }
                cmp::Ordering::Greater => {
                    lo = i + 1;
                }
            }
        }

        self.last_lo = lo as i32 - 1;

        !(lo as i32)
    }

    fn recompute_tangents(&mut self, i: i32) {
        if self.list.len() == 1 {
            let temp = &mut self.list.values[0];
            if temp.auto_tangent {
                temp.in_tangent = 0.0;
                temp.out_tangent = 0.0;
            }
            return;
        }

        self.fix_tangent(i);

        if i != 0 {
            self.fix_tangent(i - 1);
        }
        if i != (self.list.len() - 1) as i32 {
            self.fix_tangent(i + 1);
        }
    }

    fn fix_tangent(&mut self, i: i32) {
        let mut current = self.list.values[i as usize];
        if !current.auto_tangent {
            return;
        }

        let mut slope1 = 0.0;

        if i < (self.list.len() - 1) as i32 {
            let right = self.list.values[(i + 1) as usize];

            slope1 = right.value - current.value;
            slope1 /= right.time - current.time;

            if i == 0 {
                current.in_tangent = slope1;
                current.out_tangent = slope1;
                self.list.values[i as usize] = current;
                return;
            }
        }

        let mut slope2 = 0.0;

        if i > 0 {
            let left = self.list.values[(i - 1) as usize];

            slope2 = current.value - left.value;
            slope2 /= current.time - left.time;

            if i == (self.list.len() - 1) as i32 {
                current.in_tangent = slope2;
                current.out_tangent = slope2;
                self.list.values[i as usize] = current;
                return;
            }
        }

        slope1 += slope2;
        slope1 /= 2.0;
        current.in_tangent = slope1;
        current.out_tangent = slope1;
        self.list.values[i as usize] = current;
    }

    pub fn evaluate(&mut self, t: f64) -> f64 {
        if self.list.is_empty() {
            return 0.0;
        }

        if t <= self.min_time {
            return self.list.values[0].value;
        }

        if t >= self.max_time {
            return self.list.values[self.list.len() - 1].value;
        }

        let hi = self.find_index(t);

        if hi >= 0 {
            return self.list.values[hi as usize].value;
        }

        let hi = !hi;

        let test_keyframe = self.list.values[(hi - 1) as usize];
        let test_keyframe2 = self.list.values[hi as usize];

        self.interpolant(
            test_keyframe.time,
            test_keyframe.value,
            test_keyframe.out_tangent,
            test_keyframe2.time,
            test_keyframe2.value,
            test_keyframe2.in_tangent,
            t,
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct HFrame {
    in_tangent: f64,
    out_tangent: f64,
    time: f64,
    value: f64,
    auto_tangent: bool,
}
