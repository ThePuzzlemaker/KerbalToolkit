#![allow(dead_code)]
use std::sync::{Arc, LazyLock, OnceLock};

use color_eyre::eyre;
use jlrs::{
    data::managed::value::ValueRet,
    memory::target::frame::GcFrame,
    prelude::{IntoJlrsResult, JuliaString, Managed, Module, Value},
    weak_handle,
};
use parking_lot::{Mutex, RwLock};

use crate::{ffi_defn, mission::Mission};

static ERROR: LazyLock<Arc<Mutex<Option<String>>>> = LazyLock::new(|| Arc::new(Mutex::new(None)));
pub static MISSION: OnceLock<Arc<RwLock<Mission>>> = OnceLock::new();

pub fn replace_error(s: String) {
    ERROR.lock().replace(s);
}

pub unsafe extern "C" fn ktk_support_take_error() -> ValueRet {
    match weak_handle!() {
        Ok(handle) => match ERROR.lock().take() {
            Some(error) => JuliaString::new(handle, error).as_value().leak(),
            None => Value::nothing(&handle).leak(),
        },
        Err(_) => panic!("Not called from Julia"),
    }
}

pub fn init_module<'a, 'b: 'a>(frame: &mut GcFrame<'a>, module: Module<'b>) -> eyre::Result<()> {
    ffi_defn!(ktk_support_take_error @ frame, module);
    Ok(())
}
