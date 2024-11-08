use color_eyre::eyre;
use jlrs::{
    prelude::{Builder, IntoJlrsResult, LocalScope, Module, Value},
    runtime::handle::local_handle::LocalHandle,
};

use crate::{i18n, KtkDisplay};

pub struct ScriptingContext {
    pub jl: LocalHandle,
}

impl ScriptingContext {
    pub fn new() -> eyre::Result<Self> {
        let this = Self {
            jl: Builder::new().start_local()?,
        };

        this.jl
            .local_scope::<_, 7>(|mut frame| -> eyre::Result<()> {
                unsafe {
                    Value::eval_string(&mut frame, include_str!("../scripting/src/KerbTk.jl"))
                        .into_jlrs_result()?
                };
                let main = Module::main(&frame);

                let _ktk = unsafe { main.submodule(&frame, "KerbTk")?.as_managed() };

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
