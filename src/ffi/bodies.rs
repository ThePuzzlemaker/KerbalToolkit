use std::sync::Arc;

use color_eyre::eyre;
use itertools::Itertools;
use jlrs::{
    data::managed::value::ValueRet,
    memory::target::frame::GcFrame,
    prelude::{IntoJlrsResult, JuliaString, Managed, Module, Value},
    weak_handle,
};
use kerbtk::{bodies::Body, kepler::orbits::Orbit};

macro_rules! generate {
    ($($field:ident : $ty:ty => $getname:ident @ $getter:expr);+) => {
	$(
	    #[allow(dead_code)]
	    pub unsafe extern "C" fn $getname(body: *const Body) -> $ty {
		assert!(!body.is_null());
		let body = &*body;
		let $field = &body.$field;
		$getter
	    }
	)+
    }
}

pub unsafe extern "C" fn ktk_body_free(body: *const Body) {
    assert!(!body.is_null());
    Arc::decrement_strong_count(body);
}

#[rustfmt::skip]
generate![
    mu: f64 => ktk_body_get_mu @ *mu;
    radius: f64 => ktk_body_get_radius @ *radius;
    ephem: *const Orbit => ktk_body_get_ephem @ ephem;
    rotperiod: f64 => ktk_body_get_rotperiod @ *rotperiod;
    rotini: f64 => ktk_body_get_rotini @ *rotini;
    satellites: ValueRet => ktk_body_get_satellites @
	match weak_handle!() {
            Ok(handle) => {
		let s = format!("[{}]", satellites.iter().map(|x| format!("{:#?}", x)).join(","));
		Value::eval_string(handle, s).unwrap().leak()
	    },
            Err(_) => panic!("Not running from Julia")
	};
    parent: ValueRet => ktk_body_get_parent @
	match weak_handle!() {
            Ok(handle) => match parent {
		Some(parent) => JuliaString::new(handle, parent).as_value().leak(),
		None => Value::nothing(&handle).leak(),
            }
            Err(_) => panic!("Not running from Julia")
	};
    name: ValueRet => ktk_body_get_name @
	match weak_handle!() {
            Ok(handle) => JuliaString::new(handle, name).as_value().leak(),
            Err(_) => panic!("Not running from Julia")
	};
    is_star: bool => ktk_body_get_is_star @ *is_star;
    soi: f64 => ktk_body_get_soi @ *soi;
    angvel: ValueRet => ktk_body_get_angvel @
	match weak_handle!() {
            Ok(handle) => {
		let s = format!("SVector{{3, Float64}}({},{},{})", angvel.x, angvel.y, angvel.z);
		Value::eval_string(handle, s).unwrap().leak()
            }
            Err(_) => panic!("Not running from Julia")
	}
];

pub fn init_module<'a, 'b: 'a>(frame: &mut GcFrame<'a>, module: Module<'b>) -> eyre::Result<()> {
    let ktk_body_get_mu = Value::new(&mut *frame, ktk_body_get_mu as *mut std::ffi::c_void);
    let ktk_body_get_radius = Value::new(&mut *frame, ktk_body_get_radius as *mut std::ffi::c_void);
    let ktk_body_get_ephem = Value::new(&mut *frame, ktk_body_get_ephem as *mut std::ffi::c_void);
    let ktk_body_get_rotperiod =
        Value::new(&mut *frame, ktk_body_get_rotperiod as *mut std::ffi::c_void);
    let ktk_body_get_rotini = Value::new(&mut *frame, ktk_body_get_rotini as *mut std::ffi::c_void);
    let ktk_body_get_satellites = Value::new(
        &mut *frame,
        ktk_body_get_satellites as *mut std::ffi::c_void,
    );
    let ktk_body_get_parent = Value::new(&mut *frame, ktk_body_get_parent as *mut std::ffi::c_void);
    let ktk_body_get_name = Value::new(&mut *frame, ktk_body_get_name as *mut std::ffi::c_void);
    let ktk_body_get_is_star =
        Value::new(&mut *frame, ktk_body_get_is_star as *mut std::ffi::c_void);
    let ktk_body_get_soi = Value::new(&mut *frame, ktk_body_get_soi as *mut std::ffi::c_void);
    let ktk_body_get_angvel = Value::new(&mut *frame, ktk_body_get_angvel as *mut std::ffi::c_void);

    let ktk_body_free = Value::new(&mut *frame, ktk_body_free as *mut std::ffi::c_void);

    unsafe {
        module
            .set_global(&mut *frame, "ktk_body_get_mu", ktk_body_get_mu)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_body_get_radius", ktk_body_get_radius)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_body_get_ephem", ktk_body_get_ephem)
            .into_jlrs_result()?;
        module
            .set_global(
                &mut *frame,
                "ktk_body_get_rotperiod",
                ktk_body_get_rotperiod,
            )
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_body_get_rotini", ktk_body_get_rotini)
            .into_jlrs_result()?;
        module
            .set_global(
                &mut *frame,
                "ktk_body_get_satellites",
                ktk_body_get_satellites,
            )
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_body_get_parent", ktk_body_get_parent)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_body_get_name", ktk_body_get_name)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_body_get_is_star", ktk_body_get_is_star)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_body_get_soi", ktk_body_get_soi)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_body_get_angvel", ktk_body_get_angvel)
            .into_jlrs_result()?;

        module
            .set_global(&mut *frame, "ktk_body_free", ktk_body_free)
            .into_jlrs_result()?;
    }

    Ok(())
}
