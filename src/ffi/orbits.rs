use color_eyre::eyre;
use jlrs::{
    memory::target::frame::LocalGcFrame,
    prelude::{IntoJlrsResult, Module, Value},
};
use kerbtk::{kepler::orbits::Orbit, time::UT};

macro_rules! generate {
    ($($field:ident, $val:ident @ $getname:ident, $setname:ident => $getter:expr, $setter:stmt);+) => {
	$(
	    #[allow(dead_code)]
	    pub unsafe extern "C" fn $getname(obt: *const Orbit) -> f64 {
		assert!(!obt.is_null());
		let obt = &*obt;
		let $field = obt.$field;
		$getter
	    }

	    #[allow(dead_code)]
	    pub unsafe extern "C" fn $setname(obt: *mut Orbit, $val: f64) {
		assert!(!obt.is_null());
		let obt = &mut *obt;
		let $field = &mut obt.$field;
		$setter
	    }
	)+
    }
}

pub unsafe extern "C" fn ktk_orbit_free(obt: *mut Orbit) {
    assert!(!obt.is_null());
    drop(Box::from_raw(obt))
}

pub unsafe extern "C" fn ktk_orbit_new(
    p: f64,
    e: f64,
    i: f64,
    lan: f64,
    argpe: f64,
    epoch: f64,
    ta: f64,
) -> *mut Orbit {
    Box::into_raw(Box::new(Orbit {
        p,
        e,
        i,
        lan,
        argpe,
        epoch: UT::new_seconds(epoch),
        ta,
    }))
}

generate![
    p, val @ ktk_orbit_get_p, ktk_orbit_set_p =>
    p, *p = val;

    e, val @ ktk_orbit_get_e, ktk_orbit_set_e =>
    e, *e = val;

    i, val @ ktk_orbit_get_i, ktk_orbit_set_i =>
    i, *i = val;

    lan, val @ ktk_orbit_get_lan, ktk_orbit_set_lan =>
    lan, *lan = val;

    argpe, val @ ktk_orbit_get_argpe, ktk_orbit_set_argpe =>
    argpe, *argpe = val;

    epoch, val @ ktk_orbit_get_epoch, ktk_orbit_set_epoch =>
    epoch.into_duration().as_seconds_f64(), *epoch = UT::new_seconds(val);

    ta, val @ ktk_orbit_get_ta, ktk_orbit_set_ta =>
    ta, *ta = val
];

pub fn init_module<'a, 'b: 'a, const N: usize>(
    frame: &mut LocalGcFrame<'a, N>,
    module: Module<'b>,
) -> eyre::Result<()> {
    let ktk_orbit_get_p = Value::new(&mut *frame, ktk_orbit_get_p as *mut std::ffi::c_void);
    let ktk_orbit_get_e = Value::new(&mut *frame, ktk_orbit_get_e as *mut std::ffi::c_void);
    let ktk_orbit_get_i = Value::new(&mut *frame, ktk_orbit_get_i as *mut std::ffi::c_void);
    let ktk_orbit_get_lan = Value::new(&mut *frame, ktk_orbit_get_lan as *mut std::ffi::c_void);
    let ktk_orbit_get_argpe = Value::new(&mut *frame, ktk_orbit_get_argpe as *mut std::ffi::c_void);
    let ktk_orbit_get_epoch = Value::new(&mut *frame, ktk_orbit_get_epoch as *mut std::ffi::c_void);
    let ktk_orbit_get_ta = Value::new(&mut *frame, ktk_orbit_get_ta as *mut std::ffi::c_void);

    let ktk_orbit_set_p = Value::new(&mut *frame, ktk_orbit_set_p as *mut std::ffi::c_void);
    let ktk_orbit_set_e = Value::new(&mut *frame, ktk_orbit_set_e as *mut std::ffi::c_void);
    let ktk_orbit_set_i = Value::new(&mut *frame, ktk_orbit_set_i as *mut std::ffi::c_void);
    let ktk_orbit_set_lan = Value::new(&mut *frame, ktk_orbit_set_lan as *mut std::ffi::c_void);
    let ktk_orbit_set_argpe = Value::new(&mut *frame, ktk_orbit_set_argpe as *mut std::ffi::c_void);
    let ktk_orbit_set_epoch = Value::new(&mut *frame, ktk_orbit_set_epoch as *mut std::ffi::c_void);
    let ktk_orbit_set_ta = Value::new(&mut *frame, ktk_orbit_set_ta as *mut std::ffi::c_void);

    let ktk_orbit_free = Value::new(&mut *frame, ktk_orbit_free as *mut std::ffi::c_void);
    let ktk_orbit_new = Value::new(&mut *frame, ktk_orbit_new as *mut std::ffi::c_void);

    unsafe {
        module
            .set_global(&mut *frame, "ktk_orbit_get_p", ktk_orbit_get_p)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_get_e", ktk_orbit_get_e)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_get_i", ktk_orbit_get_i)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_get_lan", ktk_orbit_get_lan)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_get_argpe", ktk_orbit_get_argpe)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_get_epoch", ktk_orbit_get_epoch)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_get_ta", ktk_orbit_get_ta)
            .into_jlrs_result()?;

        module
            .set_global(&mut *frame, "ktk_orbit_set_p", ktk_orbit_set_p)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_set_e", ktk_orbit_set_e)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_set_i", ktk_orbit_set_i)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_set_lan", ktk_orbit_set_lan)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_set_argpe", ktk_orbit_set_argpe)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_set_epoch", ktk_orbit_set_epoch)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_set_ta", ktk_orbit_set_ta)
            .into_jlrs_result()?;

        module
            .set_global(&mut *frame, "ktk_orbit_free", ktk_orbit_free)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_orbit_new", ktk_orbit_new)
            .into_jlrs_result()?;
    }

    Ok(())
}
