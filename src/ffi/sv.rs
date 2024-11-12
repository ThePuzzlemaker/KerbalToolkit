use std::{ptr, sync::Arc};

use color_eyre::eyre;
use jlrs::{
    data::managed::value::ValueRet,
    memory::target::frame::GcFrame,
    prelude::{IntoJlrsResult, Module, Tuple3, Value},
    weak_handle,
};
use kerbtk::{
    bodies::Body,
    kepler::orbits::{Orbit, ReferenceFrame, StateVector},
    time::UT,
};
use nalgebra::Vector3;
use num_enum::FromPrimitive;
use parking_lot::RwLock;
use time::Duration;

use crate::{ffi_defn, mission::Mission};

macro_rules! generate {
    ($($field:ident, $val:ident: $ty:ty, $retty:ty, $getname:ident, $setname:ident => $getter:expr, $setter:stmt);+$(;)?) => {
	$(
	    #[allow(dead_code)]
	    pub unsafe extern "C" fn $getname(sv: *const StateVector) -> $retty {
		assert!(!sv.is_null());
		let sv = &*sv;
		let $field = &sv.$field;
		$getter
	    }

	    #[allow(dead_code)]
	    pub unsafe extern "C" fn $setname(sv: *mut StateVector, $val: $ty) {
		assert!(!sv.is_null());
		let sv = &mut *sv;
		let $field = &mut sv.$field;
		$setter
	    }
	)+
    }
}

pub unsafe extern "C" fn ktk_sv_free(sv: *mut StateVector) {
    assert!(!sv.is_null());
    drop(Box::from_raw(sv))
}

pub unsafe extern "C" fn ktk_sv_new(
    body: *const Body,
    frame: u8,
    position: Tuple3<f64, f64, f64>,
    velocity: Tuple3<f64, f64, f64>,
    time: f64,
) -> *mut StateVector {
    Box::into_raw(Box::new(StateVector {
        body: {
            Arc::increment_strong_count(body);
            Arc::from_raw(body)
        },
        frame: ReferenceFrame::from_primitive(frame),
        position: Vector3::new(position.0, position.1, position.2),
        velocity: Vector3::new(velocity.0, velocity.1, velocity.2),
        time: UT::new_seconds(time),
    }))
}

pub unsafe extern "C" fn ktk_orbit_sv_bci(
    orbit: *const Orbit,
    body: *const Body,
) -> *mut StateVector {
    assert!(!orbit.is_null());
    assert!(!body.is_null());
    let orbit = &*orbit;
    Arc::increment_strong_count(body);
    let body = Arc::from_raw(body);
    Box::into_raw(Box::new(orbit.sv_bci(&body)))
}

pub unsafe extern "C" fn ktk_sv_to_orbit(sv: *const StateVector) -> *mut Orbit {
    assert!(!sv.is_null());
    let sv = &*sv;
    Box::into_raw(Box::new(sv.clone().into_orbit(1e-8)))
}

generate![
    body, val: *const Body, *const Body, ktk_sv_get_body, ktk_sv_set_body =>
    Arc::into_raw(body.clone()), {
        Arc::increment_strong_count(val);
        *body = Arc::from_raw(val)
    };

    frame, val: u8, u8, ktk_sv_get_frame, ktk_sv_set_frame =>
    (*frame).into(), *frame = ReferenceFrame::from_primitive(val);

    position, val: Tuple3<f64, f64, f64>, ValueRet, ktk_sv_get_position, ktk_sv_set_position =>
    match weak_handle!() {
        Ok(handle) => Value::eval_string(handle, format!("SVector{{3, Float64}}({},{},{})",position.x,position.y,position.z)).unwrap().leak(),
        Err(_) => panic!("Not called from Julia")
    }, *position = Vector3::new(val.0, val.1, val.2);

    velocity, val: Tuple3<f64, f64, f64>, ValueRet, ktk_sv_get_velocity, ktk_sv_set_velocity =>
    match weak_handle!() {
        Ok(handle) => Value::eval_string(handle, format!("SVector{{3, Float64}}({},{},{})",velocity.x,velocity.y,velocity.z)).unwrap().leak(),
        Err(_) => panic!("Not called from Julia")
    }, *velocity = Vector3::new(val.0, val.1, val.2);

    time, val: f64, f64, ktk_sv_get_time, ktk_sv_set_time =>
    time.into_duration().as_seconds_f64(), *time = UT::new_seconds(val);
];

pub unsafe extern "C" fn ktk_sv_propagate(
    mission: *const RwLock<Mission>,
    sv: *const StateVector,
    dt: f64,
    tol: f64,
    maxiter: u64,
) -> *mut StateVector {
    assert!(!sv.is_null());
    assert!(!mission.is_null());
    let sv = &*sv;
    let mission = &*mission;
    let mission = mission.read();
    let res =
        sv.clone()
            .propagate_with_soi(&mission.system, Duration::seconds_f64(dt), tol, maxiter);
    match res {
        Some(res) => Box::into_raw(Box::new(res)),
        None => ptr::null_mut(),
    }
}

pub unsafe extern "C" fn ktk_sv_propagate_no_soi(
    sv: *const StateVector,
    dt: f64,
    tol: f64,
    maxiter: u64,
) -> *mut StateVector {
    assert!(!sv.is_null());
    let sv = &*sv;
    let res = sv
        .clone()
        .propagate(Duration::seconds_f64(dt), tol, maxiter);
    match res {
        Some(res) => Box::into_raw(Box::new(res)),
        None => ptr::null_mut(),
    }
}

pub fn init_module<'a, 'b: 'a>(frame: &mut GcFrame<'a>, module: Module<'b>) -> eyre::Result<()> {
    ffi_defn!(ktk_sv_get_body @ frame, module);
    ffi_defn!(ktk_sv_get_frame @ frame, module);
    ffi_defn!(ktk_sv_get_position @ frame, module);
    ffi_defn!(ktk_sv_get_velocity @ frame, module);
    ffi_defn!(ktk_sv_get_time @ frame, module);

    ffi_defn!(ktk_sv_set_body @ frame, module);
    ffi_defn!(ktk_sv_set_frame @ frame, module);
    ffi_defn!(ktk_sv_set_position @ frame, module);
    ffi_defn!(ktk_sv_set_velocity @ frame, module);
    ffi_defn!(ktk_sv_set_time @ frame, module);

    ffi_defn!(ktk_sv_new @ frame, module);
    ffi_defn!(ktk_sv_free @ frame, module);

    ffi_defn!(ktk_orbit_sv_bci @ frame, module);
    ffi_defn!(ktk_sv_to_orbit @ frame, module);
    ffi_defn!(ktk_sv_propagate @ frame, module);
    ffi_defn!(ktk_sv_propagate_no_soi @ frame, module);

    Ok(())
}
