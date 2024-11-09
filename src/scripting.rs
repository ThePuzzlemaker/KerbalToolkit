use std::sync::Arc;

use color_eyre::eyre;
use jlrs::{
    prelude::{Builder, IntoJlrsResult, LocalScope, Module, Value},
    runtime::handle::local_handle::LocalHandle,
};
use kerbtk::{bodies::Body, kepler::orbits::Orbit, time::UT};
use nalgebra::Vector3;
use parking_lot::RwLock;

use crate::{ffi, i18n, mission::Mission, KtkDisplay};

pub struct ScriptingContext {
    pub jl: LocalHandle,
    pub mission: Arc<RwLock<Mission>>,
}

impl ScriptingContext {
    pub fn new(mission: &Arc<RwLock<Mission>>) -> eyre::Result<Self> {
        let this = Self {
            jl: Builder::new().start_local()?,
            mission: mission.clone(),
        };

        this.jl
            .local_scope::<_, 100>(|mut frame| -> eyre::Result<()> {
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

                let body = Arc::new(Body {
                    mu: 65.1383975207807,
                    radius: 200.0,
                    ephem: Orbit {
                        p: 12000.0,
                        e: 0.0,
                        i: 0.0,
                        lan: 0.0,
                        argpe: 0.0,
                        epoch: UT::new_seconds(0.0),
                        ta: 1.70000004768372,
                    },
                    rotperiod: 138984.37657447575,
                    rotini: 4.014257279586958,
                    satellites: Arc::new([]),
                    parent: Some(Arc::from("Kerbin")),
                    name: Arc::from("Mun"),
                    is_star: false,
                    soi: 2429.5591165647475,
                    angvel: Vector3::new(-0.0, 0.0, -0.000045207853300062813),
                });
                unsafe {
                    let ktk_test_body =
                        Value::new(&mut frame, Arc::into_raw(body) as *mut std::ffi::c_void);
                    bodies
                        .set_global(&mut frame, "ktk_test_body", ktk_test_body)
                        .into_jlrs_result()?;
                }

                ffi::orbits::init_module(&mut frame, orbits)?;
                ffi::bodies::init_module(&mut frame, bodies)?;

                Ok(())
            })?;

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
