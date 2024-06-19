use std::{collections::HashMap, mem, sync::Arc, time::Instant};

use color_eyre::eyre;
use egui_notify::Toasts;
use itertools::Itertools;
use kerbtk::{bodies::Body, time::GET, vessel::VesselRef};
use time::Duration;
use tracing::{error, info};

use crate::{
    backend::{HReq, HRes},
    handle, i18n, i18n_args,
    mission::Mission,
    widgets::{icon, TimeDisplayBtn, TimeDisplayKind, TimeInput1, TimeInputKind2},
    Backend, DisplaySelect, Displays, KtkDisplay, UTorGET,
};

pub struct SystemConfiguration {
    _ui_id: egui::Id,
    loading: bool,
}

impl Default for SystemConfiguration {
    fn default() -> Self {
        Self {
            _ui_id: egui::Id::new(Instant::now()),
            loading: false,
        }
    }
}

impl KtkDisplay for SystemConfiguration {
    fn show(
        &mut self,
        mission: &Mission,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    ) {
        egui::Window::new(i18n!("syscfg-title"))
            .open(&mut open.syscfg)
            .default_width(148.0)
            .show(ctx, |ui| {
                handle(toasts, |toasts| {
                    ui.horizontal(|ui| {
                        handle(toasts, |_| {
                            if ui.button(i18n!("syscfg-load-krpc")).clicked() {
                                backend.tx(DisplaySelect::SysCfg, HReq::LoadSystem)?;
                                self.loading = true;
                            }
                            if self.loading {
                                ui.spinner();
                            }
                            Ok(())
                        });
                    });

                    ui.label(i18n_args!(
                        "syscfg-bodies-loaded",
                        "bodies",
                        mission.system.bodies.len()
                    ));
                    let mut openall = None;
                    ui.horizontal(|ui| {
                        if ui.button(i18n!("expand-all")).clicked() {
                            openall = Some(true);
                        }
                        if ui.button(i18n!("collapse-all")).clicked() {
                            openall = Some(false);
                        }
                    });
                    egui::ScrollArea::vertical()
                        //.max_height(256.0)
                        .auto_shrink(false)
                        .show(ui, |ui| {
                            ui.with_layout(
                                egui::Layout::top_down(egui::Align::LEFT).with_cross_justify(true),
                                |ui| {
                                    let bodies = &mission.system.bodies;
                                    let stars = bodies.iter().filter(|(_, b)| b.is_star);
                                    for (star, body) in stars {
                                        Self::show_body(bodies, ui, star, body, &mut openall);
                                    }
                                },
                            );
                        });
                    Ok(())
                });
            });
    }

    fn handle_rx(
        &mut self,
        res: eyre::Result<HRes>,
        _mission: &Mission,
        _toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()> {
        self.loading = false;
        if let Ok(HRes::LoadedSystem(system)) = res {
            backend.effect(|mission| {
                mission.system = Arc::new(system);
                Ok(())
            });
            ctx.request_repaint();
            Ok(())
        } else {
            res.map(|_| ())
        }
    }
}

impl SystemConfiguration {
    fn show_body(
        bodies: &HashMap<Arc<str>, Arc<Body>>,
        ui: &mut egui::Ui,
        name: &str,
        body: &Body,
        openall: &mut Option<bool>,
    ) {
        if body.satellites.is_empty() {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(format!(" ▪ {}", name)));
            });
        } else {
            egui::CollapsingHeader::new(egui::RichText::new(name).strong())
                .open(*openall)
                .show(ui, |ui| {
                    ui.vertical(|ui| {
                        for (name, body) in body.satellites.iter().map(|x| (x, &bodies[x])) {
                            Self::show_body(bodies, ui, name, body, openall);
                        }
                    })
                });
        }
    }
}

#[derive(Debug)]
pub struct KRPCConfig {
    _ui_id: egui::Id,
    ip: String,
    rpc_port: String,
    stream_port: String,
    status: String,
    loading: bool,
}

impl Default for KRPCConfig {
    fn default() -> Self {
        Self {
            _ui_id: egui::Id::new(Instant::now()),
            ip: "127.0.0.1".into(),
            rpc_port: "50000".into(),
            stream_port: "50001".into(),
            status: i18n!("krpc-status-noconn"),
            loading: false,
        }
    }
}

impl KtkDisplay for KRPCConfig {
    fn show(
        &mut self,
        _mission: &Mission,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    ) {
        egui::Window::new(i18n!("krpc-title"))
            .open(&mut open.krpc)
            .auto_sized()
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label(i18n!("krpc-host"));
                    ui.add(egui::TextEdit::singleline(&mut self.ip).desired_width(128.0));
                    ui.label(i18n!("krpc-rpc-port"));
                    ui.add(egui::TextEdit::singleline(&mut self.rpc_port).desired_width(48.0));
                    ui.label(i18n!("krpc-stream-port"));
                    ui.add(egui::TextEdit::singleline(&mut self.stream_port).desired_width(48.0));
                });
                ui.horizontal(|ui| {
                    handle(toasts, |_| {
                        if ui.button(i18n!("connect")).clicked() {
                            backend.tx(
                                DisplaySelect::Krpc,
                                HReq::RPCConnect(
                                    self.ip.clone(),
                                    self.rpc_port.clone(),
                                    self.stream_port.clone(),
                                ),
                            )?;
                            self.loading = true;
                        }
                        if ui.button(i18n!("disconnect")).clicked() {
                            backend.tx(DisplaySelect::Krpc, HReq::RPCDisconnect)?;
                            self.loading = true;
                        }
                        if self.loading {
                            ui.spinner();
                        }
                        Ok(())
                    });
                });
                ui.label(&self.status);
            });
    }

    fn handle_rx(
        &mut self,
        res: eyre::Result<HRes>,
        _mission: &Mission,
        toasts: &mut Toasts,
        _backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()> {
        self.loading = false;
        match res {
            Ok(HRes::ConnectionFailure(e)) | Err(e) => {
                self.status = i18n_args!("krpc-status-error", "error", e.to_string());
                error!("{}", i18n_args!("error-krpc-conn", "error", e.to_string()));
                toasts.error(i18n_args!("error-krpc-conn", "error", e.to_string()));
                self.loading = false;
                ctx.request_repaint();
                Ok(())
            }
            Ok(HRes::Connected(version)) => {
                self.status = i18n_args!("krpc-status-success", "version", &version);
                info!("{}", i18n_args!("krpc-log-success", "version", &version));
                toasts.info(i18n_args!("krpc-log-success", "version", version));
                self.loading = false;
                ctx.request_repaint();
                Ok(())
            }
            Ok(HRes::Disconnected) => {
                self.status = i18n!("krpc-status-noconn");
                info!("{}", i18n!("krpc-log-disconnected"));
                self.loading = false;
                ctx.request_repaint();
                Ok(())
            }
            _ => unreachable!(),
        }
    }
}

pub struct TimeUtils {
    ui_id: egui::Id,
    t1_buf: String,
    t1_disp: TimeDisplayKind,
    t1_input: TimeInputKind2,
    t1: Option<UTorGET>,
    vessel: Option<VesselRef>,
    t2_buf: String,
    t2_disp: TimeDisplayKind,
    t2: Option<UTorGET>,
    sub: bool,
    t3_disp: TimeDisplayKind,
    t3_kind: TimeInputKind2,
    t3_buf: String,
}

impl Default for TimeUtils {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
            t1_buf: "".into(),
            t1_disp: TimeDisplayKind::Dhms,
            t1_input: TimeInputKind2::UT,
            t1: None,
            vessel: None,
            t2_buf: "".into(),
            t2_disp: TimeDisplayKind::Dhms,
            t2: None,
            sub: false,
            t3_buf: "".into(),
            t3_disp: TimeDisplayKind::Dhms,
            t3_kind: TimeInputKind2::UT,
        }
    }
}

impl KtkDisplay for TimeUtils {
    fn show(
        &mut self,
        mission: &Mission,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    ) {
        egui::Window::new(i18n!("time-utils-title"))
            .open(&mut open.time_utils)
            .default_size([384.0, 384.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label(i18n!("time-utils-vessel"));
                    egui::ComboBox::from_id_source(self.ui_id.with("VesselSelector"))
                        .selected_text(
                            self.vessel
                                .clone()
                                .map(|x| x.0.read().name.clone())
                                .unwrap_or_else(|| i18n!("vc-no-vessel")),
                        )
                        .show_ui(ui, |ui| {
                            for iter_vessel in mission
                                .vessels
                                .iter()
                                .map(|(_, x)| x)
                                .sorted_by_key(|x| x.read().name.clone())
                            {
                                ui.selectable_value(
                                    &mut self.vessel,
                                    Some(VesselRef(iter_vessel.clone())),
                                    &iter_vessel.read().name,
                                );
                            }
                        });
                });

                ui.separator();

                ui.vertical_centered(|ui| {
                    ui.horizontal(|ui| {
                        ui.add(TimeInput1::new(
                            &mut self.t1_buf,
                            &mut self.t1,
                            Some(128.0),
                            self.t1_input,
                            self.t1_disp,
                            true,
                            true,
                        ));
                        ui.add(TimeDisplayBtn(&mut self.t1_disp));
                        ui.radio_value(
                            &mut self.t1_input,
                            TimeInputKind2::UT,
                            i18n!("time-utils-ut"),
                        );
                        ui.radio_value(
                            &mut self.t1_input,
                            TimeInputKind2::GET,
                            i18n!("time-utils-get"),
                        );
                    });

                    ui.horizontal(|ui| {
                        if ui.button(icon("\u{e8d5}")).clicked() {
                            mem::swap(&mut self.t1_buf, &mut self.t2_buf);
                            mem::swap(&mut self.t1_disp, &mut self.t2_disp);
                            mem::swap(&mut self.t1, &mut self.t2);
                        }
                        ui.selectable_value(&mut self.sub, false, icon("\u{e145}"));
                        ui.selectable_value(&mut self.sub, true, icon("\u{e15b}"));
                    });

                    ui.horizontal(|ui| {
                        ui.add(TimeInput1::new(
                            &mut self.t2_buf,
                            &mut self.t2,
                            Some(128.0),
                            TimeInputKind2::UT,
                            self.t2_disp,
                            true,
                            true,
                        ));
                        ui.add(TimeDisplayBtn(&mut self.t2_disp));
                    });

                    ui.horizontal(|ui| {
                        let t1 = match self.t1 {
                            Some(UTorGET::UT(ut)) => ut.into_duration(),
                            Some(UTorGET::GET(get)) => {
                                let get_base = self
                                    .vessel
                                    .as_ref()
                                    .map(|x| x.0.read().get_base.into_duration())
                                    .unwrap_or_default();
                                get_base + get.into_duration()
                            }
                            None => Duration::new(0, 0),
                        };
                        let t2 = match self.t2 {
                            Some(UTorGET::UT(ut)) => ut.into_duration(),
                            Some(UTorGET::GET(get)) => {
                                let get_base = self
                                    .vessel
                                    .as_ref()
                                    .map(|x| x.0.read().get_base.into_duration())
                                    .unwrap_or_default();
                                get_base + get.into_duration()
                            }
                            None => Duration::new(0, 0),
                        };
                        let t3 = if self.sub { t1 - t2 } else { t1 + t2 };
                        let mut t3 = Some(self.t3_kind.with_duration(t3));
                        if let Some(UTorGET::GET(get)) = t3 {
                            let get_base = self
                                .vessel
                                .as_ref()
                                .map(|x| x.0.read().get_base.into_duration())
                                .unwrap_or_default();
                            let dur = get.into_duration() - get_base;
                            t3 = Some(UTorGET::GET(GET::from_duration(dur)));
                        }

                        ui.add(TimeInput1::new(
                            &mut self.t3_buf,
                            &mut t3,
                            Some(128.0),
                            self.t3_kind,
                            self.t3_disp,
                            false,
                            true,
                        ));
                        ui.add(TimeDisplayBtn(&mut self.t3_disp));
                        ui.radio_value(
                            &mut self.t3_kind,
                            TimeInputKind2::UT,
                            i18n!("time-utils-ut"),
                        );
                        ui.radio_value(
                            &mut self.t3_kind,
                            TimeInputKind2::GET,
                            i18n!("time-utils-get"),
                        );
                    });
                });
            });
    }

    fn handle_rx(
        &mut self,
        res: eyre::Result<HRes>,
        _mission: &Mission,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
        _ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()> {
        res.map(|_| ())
    }
}
