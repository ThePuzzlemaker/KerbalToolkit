use color_eyre::eyre;
use jlrs::{
    memory::target::frame::GcFrame,
    prelude::{IntoJlrsResult, Module, Value},
};
use kerbtk::{
    kepler::orbits::{time_of_flight, Orbit},
    time::UT,
};

use crate::ffi_defn;

macro_rules! generate {
    ($($field:ident, $val:ident @ $getname:ident, $setname:ident => $getter:expr, $setter:stmt);+$(;)?) => {
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

pub unsafe extern "C" fn ktk_orbit_tof(r0: f64, r: f64, ta0: f64, ta: f64, p: f64, mu: f64) -> f64 {
    time_of_flight(r0, r, ta0, ta, p, mu)
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
    ta, *ta = val;
];

pub fn init_module<'a, 'b: 'a>(frame: &mut GcFrame<'a>, module: Module<'b>) -> eyre::Result<()> {
    ffi_defn!(ktk_orbit_get_p @ frame, module);
    ffi_defn!(ktk_orbit_get_e @ frame, module);
    ffi_defn!(ktk_orbit_get_i @ frame, module);
    ffi_defn!(ktk_orbit_get_lan @ frame, module);
    ffi_defn!(ktk_orbit_get_argpe @ frame, module);
    ffi_defn!(ktk_orbit_get_epoch @ frame, module);
    ffi_defn!(ktk_orbit_get_ta @ frame, module);

    ffi_defn!(ktk_orbit_set_p @ frame, module);
    ffi_defn!(ktk_orbit_set_e @ frame, module);
    ffi_defn!(ktk_orbit_set_i @ frame, module);
    ffi_defn!(ktk_orbit_set_lan @ frame, module);
    ffi_defn!(ktk_orbit_set_argpe @ frame, module);
    ffi_defn!(ktk_orbit_set_epoch @ frame, module);
    ffi_defn!(ktk_orbit_set_ta @ frame, module);

    ffi_defn!(ktk_orbit_new @ frame, module);
    ffi_defn!(ktk_orbit_free @ frame, module);

    ffi_defn!(ktk_orbit_tof @ frame, module);

    Ok(())
}
