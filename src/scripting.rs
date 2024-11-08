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
    pub fn new(// _pty: Pty, inbuf_r: PipeReader, outbuf_w: PipeWriter
    ) -> eyre::Result<Self> {
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
                // let inbuf = Value::new(&mut frame, inbuf_r.into_raw_fd());
                // let outbuf = Value::new(&mut frame, outbuf_w.into_raw_fd());

                // unsafe {
                //     ktk.set_global(&frame, "ktk_jlrepl_inbuf", inbuf)
                //         .map_err(|_| eyre!("oops"))?;
                //     ktk.set_global(&frame, "ktk_jlrepl_outbuf", outbuf)
                //         .map_err(|_| eyre!("oops"))?;
                // };

                Ok(())
            })?;

        Ok(this)
    }
}

pub struct TerminalDisplay {
    // pub _terminal_backend: TerminalBackend,
    // pub _pty_proxy_receiver: Receiver<(u64, egui_term::PtyEvent)>,
}

// #[derive(Clone)]
// pub struct Pty {
//     pub window_size: Arc<RwLock<WindowSize>>,
//     pub inbuf: ArcPipeWriter,
//     pub outbuf: ArcPipeReader,
//     pub poller: Arc<RwLock<Option<Arc<Poller>>>>,
//     pub event: Arc<RwLock<Option<Event>>>,
//     pub mode: Arc<RwLock<Option<PollMode>>>,
// }

// #[derive(Clone)]
// pub struct ArcPipeReader {
//     pub inner: Arc<RwLock<PipeReader>>,
// }

// #[derive(Clone)]
// pub struct ArcPipeWriter {
//     pub inner: Arc<RwLock<PipeWriter>>,
// }

// impl Read for ArcPipeReader {
//     fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
//         (&mut *self.inner.write()).read(buf)
//     }
// }

// impl Write for ArcPipeWriter {
//     fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
//         (&mut *self.inner.write()).write(buf)
//     }

//     fn flush(&mut self) -> std::io::Result<()> {
//         (&mut *self.inner.write()).flush()
//     }
// }

// impl EventedReadWrite for Pty {
//     type Reader = ArcPipeReader;

//     type Writer = ArcPipeWriter;

//     unsafe fn register(
//         &mut self,
//         poll: &Arc<Poller>,
//         mut event: Event,
//         mode: PollMode,
//     ) -> std::io::Result<()> {
//         event.key = 0;
//         unsafe {
//             poll.add_with_mode(&(&*self.outbuf.inner.read()).as_fd(), event, mode)?;
//             poll.add_with_mode(&(&*self.inbuf.inner.read()).as_fd(), event, mode)?;
//         }

//         *self.poller.write() = Some(poll.clone());
//         *self.event.write() = Some(event);
//         *self.mode.write() = Some(mode);
//         Ok(())
//     }

//     fn reregister(
//         &mut self,
//         poll: &Arc<Poller>,
//         mut event: Event,
//         mode: PollMode,
//     ) -> std::io::Result<()> {
//         event.key = 0;
//         poll.modify_with_mode((&*self.outbuf.inner.read()).as_fd(), event, mode)?;
//         poll.modify_with_mode((&*self.inbuf.inner.read()).as_fd(), event, mode)?;

//         *self.poller.write() = Some(poll.clone());
//         *self.event.write() = Some(event);
//         *self.mode.write() = Some(mode);
//         Ok(())
//     }

//     fn deregister(&mut self, poll: &std::sync::Arc<Poller>) -> std::io::Result<()> {
//         poll.delete((&*self.outbuf.inner.read()).as_fd())?;
//         poll.delete((&*self.inbuf.inner.read()).as_fd())?;
//         *self.poller.write() = None;
//         *self.event.write() = None;
//         *self.mode.write() = None;
//         Ok(())
//     }

//     fn reader(&mut self) -> &mut Self::Reader {
//         &mut self.outbuf
//     }

//     fn writer(&mut self) -> &mut Self::Writer {
//         &mut self.inbuf
//     }
// }

// impl EventedPty for Pty {
//     fn next_child_event(&mut self) -> Option<alacritty_terminal::tty::ChildEvent> {
//         None
//     }
// }

// impl OnResize for Pty {
//     fn on_resize(&mut self, window_size: WindowSize) {
//         *self.window_size.write() = window_size;
//     }
// }

impl TerminalDisplay {
    pub fn new() -> Self {
        // let (pty_proxy_sender, pty_proxy_receiver) = std::sync::mpsc::channel();

        // let terminal_backend =
        //     TerminalBackend::new(0, ctx.clone(), pty_proxy_sender.clone(), pty.clone()).unwrap();

        Self {
            // _terminal_backend: terminal_backend,
            // _pty_proxy_receiver: pty_proxy_receiver,
        }
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
            .show(ctx, |_ui| {
                // let terminal = TerminalView::new(ui, &mut self.terminal_backend)
                //     .set_focus(true)
                //     .set_size(egui::Vec2::new(ui.available_width(), ui.available_height()));

                // ui.add(terminal);
            });
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
