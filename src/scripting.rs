use std::{sync::Arc, thread::JoinHandle};

use color_eyre::eyre;
use jlrs::prelude::{AsyncHandle, Builder, IntoJlrsResult, Module, Tokio, Value};

use parking_lot::RwLock;

use crate::{ffi, i18n, mission::Mission, KtkDisplay};

pub struct ScriptingContext {
    pub jl: AsyncHandle,
    pub thread: JoinHandle<()>,
    pub mission: Arc<RwLock<Mission>>,
}

impl ScriptingContext {
    pub fn into_thread(self) -> JoinHandle<()> {
        self.thread
    }

    pub fn new(mission: &Arc<RwLock<Mission>>) -> eyre::Result<Self> {
        let (jl, thread) = Builder::new()
            .async_runtime(Tokio::<3>::new(true))
            .spawn()?;
        let this = Self {
            jl,
            thread,
            mission: mission.clone(),
        };

        let mission1 = mission.clone();
        this.jl
            .blocking_task(|mut frame| -> eyre::Result<()> {
                unsafe {
                    Value::eval_string(
                        &mut frame,
                        r#"
import Pkg
Pkg.activate("scripting/")
using KerbTk
using StaticArrays
"#,
                    )
                    .into_jlrs_result()?
                };
                let main = Module::main(&frame);

                let ktk = main.submodule(&mut frame, "KerbTk")?;
                let orbits = ktk.submodule(&mut frame, "Orbits")?;
                let bodies = ktk.submodule(&mut frame, "Bodies")?;
                let support = ktk.submodule(&mut frame, "Support")?;
                let missions = ktk.submodule(&mut frame, "Missions")?;
                let systems = ktk.submodule(&mut frame, "SolarSystems")?;

                ffi::orbits::init_module(&mut frame, orbits)?;
                ffi::bodies::init_module(&mut frame, bodies)?;
                ffi::support::init_module(&mut frame, support)?;
                ffi::missions::init_module(&mut frame, missions)?;
                ffi::systems::init_module(&mut frame, systems)?;

                unsafe {
                    let ktk_mission_ptr =
                        Value::new(&mut frame, Arc::into_raw(mission1) as *mut std::ffi::c_void);
                    missions
                        .set_global(&mut frame, "ktk_mission_ptr", ktk_mission_ptr)
                        .into_jlrs_result()?;
                }

                Ok(())
            })
            .try_dispatch()
            .unwrap()
            .blocking_recv()??;

        Ok(this)
    }
}

pub struct TerminalDisplay {}

impl TerminalDisplay {
    pub fn new() -> Self {
        Self {}
    }
}

impl KtkDisplay for TerminalDisplay {
    fn show(
        &mut self,
        _mission: &crate::mission::Mission,
        _toasts: &mut egui_notify::Toasts,
        _backend: &mut crate::Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut crate::Displays,
    ) {
        let (char_width, char_height) = {
            let style = ctx.style();
            let monospace = style.text_styles.get(&egui::TextStyle::Monospace).unwrap();
            let char_width = ctx.fonts(|fonts| fonts.glyph_width(&monospace, 'x'));
            let char_height = monospace.size;
            (char_width, char_height)
        };
        egui::Window::new(i18n!("scripting-title"))
            .open(&mut open.script)
            .default_size([80.0 * char_width, 25.0 * char_height])
            .show(ctx, |_ui| {});
    }

    fn handle_rx(
        &mut self,
        _res: eyre::Result<crate::backend::HRes>,
        _mission: &crate::mission::Mission,
        _toasts: &mut egui_notify::Toasts,
        _backend: &mut crate::Backend,
        _ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()> {
        todo!()
    }
}
