#![allow(dead_code)]
use std::sync::Arc;

use color_eyre::eyre;
use jlrs::{
    memory::target::frame::GcFrame,
    prelude::{IntoJlrsResult, JuliaString, Module, Value},
};
use kerbtk::bodies::SolarSystem;
use parking_lot::RwLock;

use crate::{ffi, ffi::support::replace_error, ffi_defn, mission::Mission};

pub unsafe extern "C" fn ktk_missions_get_system(
    mission: *const RwLock<Mission>,
) -> *const SolarSystem {
    assert!(!mission.is_null());
    let mission = &*mission;
    Arc::into_raw(mission.read().system.clone())
}

ffi! {
    #[fallible]
    fn ktk_missions_load(mission: *const RwLock<Mission>, path: JuliaString) -> () {
    assert!(!mission.is_null());
    let mission = &*mission;
    let path = path.as_str()?;
    let mut new_mission: Mission = ron::from_str(&std::fs::read_to_string(path)?)?;
    new_mission.was_replaced = true;
    *mission.write() = new_mission;
    Ok(())
    }
}

ffi! {
    #[fallible]
    fn ktk_missions_save(mission: *const RwLock<Mission>, path: JuliaString) -> () {
        assert!(!mission.is_null());
        let mission = &*mission;
        let path = path.as_str()?;
        std::fs::write(
            path,
            ron::ser::to_string_pretty(
                &mission,
                ron::ser::PrettyConfig::default()
                    .struct_names(true)
                    .enumerate_arrays(true),
            )
            .expect("oops"),
        )?;
        Ok(())
    }
}

pub fn init_module<'a, 'b: 'a>(frame: &mut GcFrame<'a>, module: Module<'b>) -> eyre::Result<()> {
    ffi_defn!(ktk_missions_get_system @ frame, module);
    ffi_defn!(ktk_missions_load @ frame, module);
    ffi_defn!(ktk_missions_save @ frame, module);

    Ok(())
}
