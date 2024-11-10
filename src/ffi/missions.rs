#![allow(dead_code)]
use color_eyre::eyre;
use jlrs::{
    memory::target::frame::GcFrame,
    prelude::{IntoJlrsResult, Module, Value},
};
use kerbtk::bodies::SolarSystem;
use parking_lot::{RawRwLock, RwLock};

use crate::mission::Mission;

pub unsafe extern "C" fn ktk_missions_get_data_ptr(lck: *const RwLock<Mission>) -> *const Mission {
    assert!(!lck.is_null());
    let lck = &*lck;
    lck.data_ptr()
}

pub unsafe extern "C" fn ktk_missions_get_lock_ptr(
    lck: *const RwLock<Mission>,
) -> *const RawRwLock {
    assert!(!lck.is_null());
    let lck = &*lck;
    lck.raw()
}

pub unsafe extern "C" fn ktk_missions_get_system(mission: *const Mission) -> *const SolarSystem {
    assert!(!mission.is_null());
    let mission = &*mission;
    &*mission.system
}

pub fn init_module<'a, 'b: 'a>(frame: &mut GcFrame<'a>, module: Module<'b>) -> eyre::Result<()> {
    let ktk_missions_get_data_ptr = Value::new(
        &mut *frame,
        ktk_missions_get_data_ptr as *mut std::ffi::c_void,
    );
    let ktk_missions_get_lock_ptr = Value::new(
        &mut *frame,
        ktk_missions_get_lock_ptr as *mut std::ffi::c_void,
    );
    let ktk_missions_get_system = Value::new(
        &mut *frame,
        ktk_missions_get_system as *mut std::ffi::c_void,
    );

    unsafe {
        module
            .set_global(
                &mut *frame,
                "ktk_missions_get_data_ptr",
                ktk_missions_get_data_ptr,
            )
            .into_jlrs_result()?;
        module
            .set_global(
                &mut *frame,
                "ktk_missions_get_lock_ptr",
                ktk_missions_get_lock_ptr,
            )
            .into_jlrs_result()?;
        module
            .set_global(
                &mut *frame,
                "ktk_missions_get_system",
                ktk_missions_get_system,
            )
            .into_jlrs_result()?;
    }

    Ok(())
}
