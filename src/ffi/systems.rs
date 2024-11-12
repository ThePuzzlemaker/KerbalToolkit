use std::{ptr, sync::Arc};

use color_eyre::eyre;
use itertools::Itertools;
use jlrs::{
    data::managed::value::ValueRet,
    memory::target::frame::GcFrame,
    prelude::{IntoJlrsResult, JuliaString, Module, Value},
    weak_handle,
};
use kerbtk::bodies::{Body, SolarSystem};

pub unsafe extern "C" fn ktk_systems_get_size(system: *const SolarSystem) -> u64 {
    assert!(!system.is_null());
    let system = &*system;
    system.bodies.len() as u64
}

pub unsafe extern "C" fn ktk_systems_get_names(system: *const SolarSystem) -> ValueRet {
    assert!(!system.is_null());
    let system = &*system;
    match weak_handle!() {
        Ok(handle) => {
            let s = format!(
                "[{}]",
                system.bodies.keys().map(|x| format!("{:#?}", x)).join(",")
            );
            Value::eval_string(handle, s).unwrap().leak()
        }
        Err(_) => panic!("Not running from Julia"),
    }
}

pub unsafe extern "C" fn ktk_systems_get_bodies(system: *const SolarSystem) -> ValueRet {
    assert!(!system.is_null());
    let system = &*system;
    match weak_handle!() {
        Ok(handle) => {
            let s = format!(
                "[{}]",
                system
                    .bodies
                    .values()
                    .cloned()
                    .map(|x| format!("Ptr{{KerbTk.Bodies.Opaque}}({:#p})", Arc::into_raw(x)))
                    .join(",")
            );
            Value::eval_string(handle, s).unwrap().leak()
        }
        Err(_) => panic!("Not running from Julia"),
    }
}

pub unsafe extern "C" fn ktk_systems_to_vec(system: *const SolarSystem) -> ValueRet {
    assert!(!system.is_null());
    let system = &*system;
    match weak_handle!() {
        Ok(handle) => {
            let s = format!(
                "[{}]",
                system
                    .bodies
                    .iter()
                    .map(|(k, v)| format!(
                        "({:#?},Ptr{{KerbTk.Bodies.Opaque}}({:#p}))",
                        k,
                        Arc::into_raw(v.clone())
                    ))
                    .join(",")
            );
            Value::eval_string(handle, s).unwrap().leak()
        }
        Err(_) => panic!("Not running from Julia"),
    }
}

pub unsafe extern "C" fn ktk_systems_has_name(
    system: *const SolarSystem,
    name: JuliaString,
) -> bool {
    assert!(!system.is_null());
    let system = &*system;
    if let Ok(name) = name.as_str() {
        system.bodies.contains_key(name)
    } else {
        false
    }
}

pub unsafe extern "C" fn ktk_systems_get(
    system: *const SolarSystem,
    name: JuliaString,
) -> *const Body {
    assert!(!system.is_null());
    let system = &*system;
    let Ok(name) = name.as_str() else {
        return ptr::null();
    };
    let Some(body) = system.bodies.get(name) else {
        return ptr::null();
    };
    Arc::into_raw(body.clone())
}

pub unsafe extern "C" fn ktk_systems_free(system: *const SolarSystem) {
    drop(Arc::from_raw(system))
}

pub fn init_module<'a, 'b: 'a>(frame: &mut GcFrame<'a>, module: Module<'b>) -> eyre::Result<()> {
    let ktk_systems_get_names =
        Value::new(&mut *frame, ktk_systems_get_names as *mut std::ffi::c_void);
    let ktk_systems_get_bodies =
        Value::new(&mut *frame, ktk_systems_get_bodies as *mut std::ffi::c_void);
    let ktk_systems_has_name =
        Value::new(&mut *frame, ktk_systems_has_name as *mut std::ffi::c_void);
    let ktk_systems_get = Value::new(&mut *frame, ktk_systems_get as *mut std::ffi::c_void);
    let ktk_systems_to_vec = Value::new(&mut *frame, ktk_systems_to_vec as *mut std::ffi::c_void);
    let ktk_systems_get_size =
        Value::new(&mut *frame, ktk_systems_get_size as *mut std::ffi::c_void);
    let ktk_systems_free = Value::new(&mut *frame, ktk_systems_free as *mut std::ffi::c_void);

    unsafe {
        module
            .set_global(&mut *frame, "ktk_systems_get_names", ktk_systems_get_names)
            .into_jlrs_result()?;
        module
            .set_global(
                &mut *frame,
                "ktk_systems_get_bodies",
                ktk_systems_get_bodies,
            )
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_systems_has_name", ktk_systems_has_name)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_systems_get", ktk_systems_get)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_systems_to_vec", ktk_systems_to_vec)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_systems_get_size", ktk_systems_get_size)
            .into_jlrs_result()?;
        module
            .set_global(&mut *frame, "ktk_systems_free", ktk_systems_free)
            .into_jlrs_result()?;
    }

    Ok(())
}
