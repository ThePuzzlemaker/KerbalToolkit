use std::f64::consts;

use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use crate::{
    kepler::orbits::{Orbit, ReferenceFrame, StateVector},
    time::GET,
};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ManeuverKind {
    GeneralPurpose,
    TranslunarInjection,
    ManualInput,
    TranslunarMidcourse,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Maneuver {
    pub geti: GET,
    /// Delta-V vector in the Frenet frame.
    pub deltav: Vector3<f64>,
    pub tig_vector: StateVector,
    pub kind: ManeuverKind,
}

impl Maneuver {
    pub fn deltav_bci(&self) -> Vector3<f64> {
        frenet(&self.tig_vector) * self.deltav
    }

    /// Predict the resultant orbit from this maneuver.
    ///
    /// Recommended tolerance (`tol`): `1e-8`.
    pub fn predict_orbit(&self, tol: f64) -> Orbit {
        assert_eq!(self.tig_vector.frame, ReferenceFrame::BodyCenteredInertial);
        let mut sv = self.tig_vector.clone();
        sv.velocity += self.deltav_bci();
        sv.into_orbit(tol)
    }
}

/// Returns the Frenet frame to IJK conversion matrix for the given
/// state vector.
pub fn frenet(sv: &StateVector) -> Matrix3<f64> {
    let t = sv.velocity.normalize();
    let n = sv.position.cross(&sv.velocity).normalize();
    let b = t.cross(&n);
    Matrix3::from_columns(&[t, n, b])
}

/// Pitch and yaw of a maneuver in the Frenet frame.
pub fn pitch_yaw(deltav: Vector3<f64>) -> (f64, f64) {
    let theta = libm::acos(deltav[2] / deltav.norm());
    let psi = deltav[1].signum()
        * libm::acos(deltav[0] / libm::sqrt(deltav[0].powi(2) + deltav[1].powi(2)));
    let pitch = consts::FRAC_PI_2 - theta;
    (pitch, psi)
}

// #[test]
// pub fn foo() {
//     let orbit = Orbit {
//         p: 6571.0 * (1.0 - 0.0f64.powi(2)),
//         e: 0.0,
//         i: 0.0,
//         lan: 0.0,
//         argpe: 0.0,
//         epoch: UT::new_dhms(0, 0, 0, 0, 0),
//         ta: 0.0,
//         body: crate::bodies::sol::EARTH,
//     };
//     println!("{:#?}", orbit);
//     println!(
//         "ApA = {}, PeA = {}",
//         orbit.apoapsis_radius(),
//         orbit.periapsis_radius()
//     );
//     let sv = orbit.sv_bci();
//     let ra = 6371.0 + 5000.0;
//     let rp = 6371.0 + 200.0;
//     let a = (ra + rp) / 2.0;
//     let deltav = -libm::sqrt(sv.body.mu / sv.position.norm())
//         + libm::sqrt(sv.body.mu * (2.0 / rp - 1.0 / a));
//     let deltav = frenet(sv) * Vector3::new(deltav, 0.0, 0.0);
//     println!("deltav = {:#?}", deltav);
//     let maneuver = Maneuver {
//         geti: GET::new_dhms(0, 0, 0, 0, 0),
//         deltav,
//         tig_vector: sv,
//     };
//     let orbit2 = maneuver.predict_orbit(1e-8);
//     println!("{orbit2:#?}");
//     println!(
//         "ApA = {}, PeA = {}",
//         orbit2.apoapsis_radius(),
//         orbit2.periapsis_radius()
//     );

//     let half_period = consts::PI * libm::sqrt(a.powi(3) / sv.body.mu);
//     println!("{half_period}");
//     let sv2 = orbit2.sv_bci();
//     let sv2 = sv2.propagate(Duration::seconds_f64(half_period), 1e-7, 35);
//     println!("{:#?}", sv2);
//     println!("{:#?}", sv2.into_orbit(1e-8));

//     println!(
//         "{:#?}",
//         crate::time::GET::from_duration(sv2.time.into_duration())
//     );

//     panic!();
// }
