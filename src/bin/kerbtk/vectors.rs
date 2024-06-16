use std::{sync::Arc, time::Instant};

use color_eyre::eyre::{self, bail, OptionExt};
use egui_extras::Column;
use egui_notify::Toasts;
use itertools::Itertools;
use kerbtk::{
    kepler::orbits::{Orbit, StateVector},
    time::UT,
    vessel::VesselRef,
};
use parking_lot::RwLock;

use crate::{
    backend::{HReq, HRes},
    handle, i18n, i18n_args, icon_label,
    mission::Mission,
    Backend, DisplaySelect, KtkDisplay, TimeInput, UTorGET,
};

#[derive(Debug)]
pub struct VectorComparison {
    pub comparison_time_input: TimeInput,
    pub comparison_time_unparsed: String,
    pub comparison_time: Option<UTorGET>,
    pub v1: Option<VesselRef>,
    pub cached_v1: Option<StateVector>,
    pub cached_v1_time: Option<UT>,
    pub cached_v1_err: bool,
    pub cached_v1_obt: Option<Orbit>,
    pub v1_slot: String,
    pub v2: Option<VesselRef>,
    pub cached_v2: Option<StateVector>,
    pub cached_v2_time: Option<UT>,
    pub cached_v2_err: bool,
    pub cached_v2_obt: Option<Orbit>,
    pub v2_slot: String,
    pub v3: Option<VesselRef>,
    pub cached_v3: Option<StateVector>,
    pub cached_v3_time: Option<UT>,
    pub cached_v3_err: bool,
    pub cached_v3_obt: Option<Orbit>,
    pub v3_slot: String,
    pub v4: Option<VesselRef>,
    pub cached_v4: Option<StateVector>,
    pub cached_v4_time: Option<UT>,
    pub cached_v4_err: bool,
    pub cached_v4_obt: Option<Orbit>,
    pub v4_slot: String,
    pub ui_id: egui::Id,
}

impl Default for VectorComparison {
    fn default() -> Self {
        Self {
            comparison_time_input: Default::default(),
            comparison_time_unparsed: Default::default(),
            comparison_time: Default::default(),
            v1: None,
            cached_v1: None,
            cached_v1_time: None,
            cached_v1_err: false,
            cached_v1_obt: None,
            v1_slot: "".into(),
            v2: None,
            cached_v2: None,
            cached_v2_time: None,
            cached_v2_err: false,
            cached_v2_obt: None,
            v2_slot: "".into(),
            v3: None,
            cached_v3: None,
            cached_v3_time: None,
            cached_v3_err: false,
            cached_v3_obt: None,
            v3_slot: "".into(),
            v4: None,
            cached_v4: None,
            cached_v4_time: None,
            cached_v4_err: false,
            cached_v4_obt: None,
            v4_slot: "".into(),
            ui_id: egui::Id::new(Instant::now()),
        }
    }
}

pub struct VectorSelector<'a> {
    ui_id: egui::Id,
    vessel: &'a mut Option<VesselRef>,
    slot: &'a mut String,
    mission: Arc<RwLock<Mission>>,
    label: egui::WidgetText,
}

impl<'a> VectorSelector<'a> {
    pub fn new(
        ui_id: egui::Id,
        label: impl Into<egui::WidgetText>,
        mission: &Arc<RwLock<Mission>>,
        vessel: &'a mut Option<VesselRef>,
        slot: &'a mut String,
    ) -> Self {
        Self {
            ui_id,
            vessel,
            slot,
            mission: mission.clone(),
            label: label.into(),
        }
    }
}

impl<'a> egui::Widget for VectorSelector<'a> {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        let mut dirty = false;
        let mut res = ui.scope(|ui| {
            ui.label(self.label);
            egui::ComboBox::from_id_source(self.ui_id.with("VesselSelector"))
                .selected_text(
                    self.vessel
                        .clone()
                        .map(|x| x.0.read().name.clone())
                        .unwrap_or_else(|| i18n!("vc-no-vessel")),
                )
                .show_ui(ui, |ui| {
                    for iter_vessel in self
                        .mission
                        .read()
                        .vessels
                        .iter()
                        .sorted_by_key(|x| x.read().name.clone())
                    {
                        if ui
                            .selectable_value(
                                self.vessel,
                                Some(VesselRef(iter_vessel.clone())),
                                &iter_vessel.read().name,
                            )
                            .clicked()
                        {
                            dirty = true;
                        }
                    }
                });
            ui.label(i18n!("vector-select-slot"));

            if egui::TextEdit::singleline(self.slot)
                .char_limit(16)
                .desired_width(32.0)
                .show(ui)
                .response
                .changed()
            {
                dirty = true;
            };
        });

        if dirty {
            res.response.mark_changed();
        }
        res.response
    }
}

impl KtkDisplay for VectorComparison {
    fn show(
        &mut self,
        mission: &Arc<RwLock<Mission>>,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut bool,
    ) {
        // TODO?: ApT, PeT, SOI-T
        egui::Window::new(i18n!("vc-title"))
            .open(open)
            .default_size([384.0, 480.0])
            .show(ctx, |ui| {
                let mut dirty = false;
                ui.horizontal(|ui| {
                    ui.label(i18n!("vc-comparison-time"));
                    egui::ComboBox::from_id_source(self.ui_id.with("ComparisonTime"))
                        .selected_text(format!("{}", self.comparison_time_input))
                        .wrap(false)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self.comparison_time_input,
                                TimeInput::UTSeconds,
                                TimeInput::UTSeconds.to_string(),
                            );
                            ui.selectable_value(
                                &mut self.comparison_time_input,
                                TimeInput::UTDHMS,
                                TimeInput::UTDHMS.to_string(),
                            );
                            ui.selectable_value(
                                &mut self.comparison_time_input,
                                TimeInput::GETDHMS,
                                TimeInput::GETDHMS.to_string(),
                            );
                        });
                    if self.comparison_time.is_none()
                        && !self.comparison_time_unparsed.trim().is_empty()
                    {
                        ui.visuals_mut().selection.stroke.color =
                            egui::Color32::from_rgb(255, 0, 0);
                    }
                    if egui::TextEdit::singleline(&mut self.comparison_time_unparsed)
                        .font(egui::TextStyle::Monospace)
                        .desired_width(128.0)
                        .show(ui)
                        .response
                        .changed()
                    {
                        self.comparison_time = self
                            .comparison_time_input
                            .parse(self.comparison_time_unparsed.trim());
                        if self.comparison_time.is_some() {
                            dirty = true;
                        }
                    }
                });

                let comparison_ut = self.comparison_time.map(|x| match x {
                    UTorGET::UT(ut) => ut,
                    UTorGET::GET(_) => todo!(),
                });

                ui.horizontal(|ui| {
                    ui.add(VectorSelector::new(
                        self.ui_id.with("V1Sel"),
                        i18n_args!("vc-vec", "n" => 1),
                        mission,
                        &mut self.v1,
                        &mut self.v1_slot,
                    ));
                    ui.add(VectorSelector::new(
                        self.ui_id.with("V3Sel"),
                        i18n_args!("vc-vec", "n" => 3),
                        mission,
                        &mut self.v3,
                        &mut self.v3_slot,
                    ));
                });
                ui.horizontal(|ui| {
                    ui.add(VectorSelector::new(
                        self.ui_id.with("V2Sel"),
                        i18n_args!("vc-vec", "n" => 2),
                        mission,
                        &mut self.v2,
                        &mut self.v2_slot,
                    ));
                    ui.add(VectorSelector::new(
                        self.ui_id.with("V4Sel"),
                        i18n_args!("vc-vec", "n" => 4),
                        mission,
                        &mut self.v4,
                        &mut self.v4_slot,
                    ));

                    // TODO: offload this to the handler thread
                    if ui
                        .button(icon_label("\u{e5d5}", &i18n!("vc-calculate")))
                        .clicked()
                    {
                        self.cached_v1 = self
                            .v1
                            .as_ref()
                            .and_then(|x| x.0.read().svs.get(&self.v1_slot).cloned());
                        self.cached_v1_time = self.cached_v1.as_ref().map(|x| x.time);
                        self.cached_v1_err = false;
                        self.cached_v2 = self
                            .v2
                            .as_ref()
                            .and_then(|x| x.0.read().svs.get(&self.v2_slot).cloned());
                        self.cached_v2_time = self.cached_v2.as_ref().map(|x| x.time);
                        self.cached_v2_err = false;
                        self.cached_v3 = self
                            .v3
                            .as_ref()
                            .and_then(|x| x.0.read().svs.get(&self.v3_slot).cloned());
                        self.cached_v3_time = self.cached_v3.as_ref().map(|x| x.time);
                        self.cached_v3_err = false;
                        self.cached_v4 = self
                            .v4
                            .as_ref()
                            .and_then(|x| x.0.read().svs.get(&self.v4_slot).cloned());
                        self.cached_v4_time = self.cached_v4.as_ref().map(|x| x.time);
                        self.cached_v4_err = false;

                        if let Some(ut) = comparison_ut {
                            if let Some(sv) = self.cached_v1.clone() {
                                let delta_t = ut - sv.time;
                                if delta_t.as_seconds_f64() < -1e-3 {
                                    self.cached_v1_err = true;
                                } else if delta_t.as_seconds_f64() > 1e-3 {
                                    self.cached_v1 = Some(sv.propagate_with_soi(
                                        &mission.read().system,
                                        delta_t,
                                        1e-7,
                                        35,
                                    ));
                                }
                            }
                            if let Some(sv) = self.cached_v2.clone() {
                                let delta_t = ut - sv.time;
                                if delta_t.as_seconds_f64() < -1e-3 {
                                    self.cached_v2_err = true;
                                } else if delta_t.as_seconds_f64() > 1e-3 {
                                    self.cached_v2 = Some(sv.propagate_with_soi(
                                        &mission.read().system,
                                        delta_t,
                                        1e-7,
                                        35,
                                    ));
                                }
                            }

                            if let Some(sv) = self.cached_v3.clone() {
                                let delta_t = ut - sv.time;
                                if delta_t.as_seconds_f64() < -1e-3 {
                                    self.cached_v3_err = true;
                                } else if delta_t.as_seconds_f64() > 1e-3 {
                                    self.cached_v3 = Some(sv.propagate_with_soi(
                                        &mission.read().system,
                                        delta_t,
                                        1e-7,
                                        35,
                                    ));
                                }
                            }

                            if let Some(sv) = self.cached_v4.clone() {
                                let delta_t = ut - sv.time;
                                if delta_t.as_seconds_f64() < -1e-3 {
                                    self.cached_v4_err = true;
                                } else if delta_t.as_seconds_f64() > 1e-3 {
                                    self.cached_v4 = Some(sv.propagate_with_soi(
                                        &mission.read().system,
                                        delta_t,
                                        1e-7,
                                        35,
                                    ));
                                }
                            }
                        }

                        self.cached_v1_obt = self.cached_v1.clone().map(|x| x.into_orbit(1e-8));
                        self.cached_v2_obt = self.cached_v2.clone().map(|x| x.into_orbit(1e-8));
                        self.cached_v3_obt = self.cached_v3.clone().map(|x| x.into_orbit(1e-8));
                        self.cached_v4_obt = self.cached_v4.clone().map(|x| x.into_orbit(1e-8));
                    }
                });

                let sv1 = &self.cached_v1;
                let sv2 = &self.cached_v2;
                let sv3 = &self.cached_v3;
                let sv4 = &self.cached_v4;

                let sv1_time = self.cached_v1_time;
                let sv2_time = self.cached_v2_time;
                let sv3_time = self.cached_v3_time;
                let sv4_time = self.cached_v4_time;

                let ob1 = &self.cached_v1_obt;
                let ob2 = &self.cached_v2_obt;
                let ob3 = &self.cached_v3_obt;
                let ob4 = &self.cached_v4_obt;

                let err_sv1 = self.cached_v1_err;
                let err_sv2 = self.cached_v2_err;
                let err_sv3 = self.cached_v3_err;
                let err_sv4 = self.cached_v4_err;

                ui.separator();

                egui_extras::TableBuilder::new(ui)
                    .column(Column::auto_with_initial_suggestion(24.0).resizable(false))
                    .column(Column::auto_with_initial_suggestion(108.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(108.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(108.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(108.0).resizable(true))
                    .cell_layout(
                        egui::Layout::default()
                            .with_cross_align(egui::Align::RIGHT)
                            .with_main_align(egui::Align::Center)
                            .with_main_wrap(false)
                            .with_cross_justify(true)
                            .with_main_justify(true),
                    )
                    .striped(true)
                    .header(16.0, |mut header| {
                        header.col(|_ui| {});
                        header.col(|ui| {
                            ui.with_layout(
                                egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                                |ui| {
                                    ui.label(i18n_args!("vc-vec", "n" => 1));
                                },
                            );
                        });
                        header.col(|ui| {
                            ui.with_layout(
                                egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                                |ui| {
                                    ui.label(i18n_args!("vc-vec", "n" => 2));
                                },
                            );
                        });
                        header.col(|ui| {
                            ui.with_layout(
                                egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                                |ui| {
                                    ui.label(i18n_args!("vc-vec", "n" => 3));
                                },
                            );
                        });
                        header.col(|ui| {
                            ui.with_layout(
                                egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                                |ui| {
                                    ui.label(i18n_args!("vc-vec", "n" => 4));
                                },
                            );
                        });
                    })
                    .body(|mut body| {
                        body.ui_mut().style_mut().override_text_style =
                            Some(egui::TextStyle::Monospace);

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label(i18n!("vc-tag"));
                            });
                            row.col(|ui| {
                                if let Some(sv1_time) = &sv1_time {
                                    let (d, h, m, s, ms) = (
                                        sv1_time.days(),
                                        sv1_time.hours(),
                                        sv1_time.minutes(),
                                        sv1_time.seconds(),
                                        sv1_time.millis(),
                                    );
                                    ui.add(
                                        egui::Label::new(
                                            egui::RichText::new(format!(
                                                "{}:{:02}:{:02}:{:02}.{:03}",
                                                d, h, m, s, ms
                                            ))
                                            .color(
                                                if err_sv1 {
                                                    egui::Color32::from_rgb(255, 0, 0)
                                                } else {
                                                    ui.visuals().text_color()
                                                },
                                            ),
                                        )
                                        .wrap(false),
                                    );
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv2_time) = &sv2_time {
                                    let (d, h, m, s, ms) = (
                                        sv2_time.days(),
                                        sv2_time.hours(),
                                        sv2_time.minutes(),
                                        sv2_time.seconds(),
                                        sv2_time.millis(),
                                    );
                                    ui.add(
                                        egui::Label::new(
                                            egui::RichText::new(format!(
                                                "{}:{:02}:{:02}:{:02}.{:03}",
                                                d, h, m, s, ms
                                            ))
                                            .color(
                                                if err_sv2 {
                                                    egui::Color32::from_rgb(255, 0, 0)
                                                } else {
                                                    ui.visuals().text_color()
                                                },
                                            ),
                                        )
                                        .wrap(false),
                                    );
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv3_time) = &sv3_time {
                                    let (d, h, m, s, ms) = (
                                        sv3_time.days(),
                                        sv3_time.hours(),
                                        sv3_time.minutes(),
                                        sv3_time.seconds(),
                                        sv3_time.millis(),
                                    );
                                    ui.add(
                                        egui::Label::new(
                                            egui::RichText::new(format!(
                                                "{}:{:02}:{:02}:{:02}.{:03}",
                                                d, h, m, s, ms
                                            ))
                                            .color(
                                                if err_sv3 {
                                                    egui::Color32::from_rgb(255, 0, 0)
                                                } else {
                                                    ui.visuals().text_color()
                                                },
                                            ),
                                        )
                                        .wrap(false),
                                    );
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv4_time) = &sv4_time {
                                    let (d, h, m, s, ms) = (
                                        sv4_time.days(),
                                        sv4_time.hours(),
                                        sv4_time.minutes(),
                                        sv4_time.seconds(),
                                        sv4_time.millis(),
                                    );
                                    ui.add(
                                        egui::Label::new(
                                            egui::RichText::new(format!(
                                                "{}:{:02}:{:02}:{:02}.{:03}",
                                                d, h, m, s, ms
                                            ))
                                            .color(
                                                if err_sv4 {
                                                    egui::Color32::from_rgb(255, 0, 0)
                                                } else {
                                                    ui.visuals().text_color()
                                                },
                                            ),
                                        )
                                        .wrap(false),
                                    );
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("Ra");
                            });
                            row.col(|ui| {
                                if let Some(ob1) = ob1 {
                                    ui.label(format!("{:.4}", ob1.apoapsis_radius()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob2) = ob2 {
                                    ui.label(format!("{:.4}", ob2.apoapsis_radius()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob3) = ob3 {
                                    ui.label(format!("{:.4}", ob3.apoapsis_radius()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob4) = ob4 {
                                    ui.label(format!("{:.4}", ob4.apoapsis_radius()));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("Rp");
                            });
                            row.col(|ui| {
                                if let Some(ob1) = ob1 {
                                    ui.label(format!("{:.4}", ob1.periapsis_radius()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob2) = ob2 {
                                    ui.label(format!("{:.4}", ob2.periapsis_radius()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob3) = ob3 {
                                    ui.label(format!("{:.4}", ob3.periapsis_radius()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob4) = ob4 {
                                    ui.label(format!("{:.4}", ob4.periapsis_radius()));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("V");
                            });
                            row.col(|ui| {
                                if let Some(sv1) = &sv1 {
                                    ui.label(format!("{:.4}", sv1.velocity.norm()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv2) = &sv2 {
                                    ui.label(format!("{:.4}", sv2.velocity.norm()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv3) = &sv3 {
                                    ui.label(format!("{:.4}", sv3.velocity.norm()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv4) = &sv4 {
                                    ui.label(format!("{:.4}", sv4.velocity.norm()));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("a");
                            });
                            row.col(|ui| {
                                if let Some(ob1) = ob1 {
                                    ui.label(format!("{:.4}", ob1.semimajor_axis()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob2) = ob2 {
                                    ui.label(format!("{:.4}", ob2.semimajor_axis()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob3) = ob3 {
                                    ui.label(format!("{:.4}", ob3.semimajor_axis()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob4) = ob4 {
                                    ui.label(format!("{:.4}", ob4.semimajor_axis()));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("e");
                            });
                            row.col(|ui| {
                                if let Some(ob1) = ob1 {
                                    ui.label(format!("{:.6}", ob1.e));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob2) = ob2 {
                                    ui.label(format!("{:.6}", ob2.e));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob3) = ob3 {
                                    ui.label(format!("{:.6}", ob3.e));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob4) = ob4 {
                                    ui.label(format!("{:.6}", ob4.e));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("i");
                            });
                            row.col(|ui| {
                                if let Some(ob1) = ob1 {
                                    ui.label(format!("{:.6}", ob1.i.to_degrees()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob2) = ob2 {
                                    ui.label(format!("{:.6}", ob2.i.to_degrees()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob3) = ob3 {
                                    ui.label(format!("{:.6}", ob3.i.to_degrees()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob4) = ob4 {
                                    ui.label(format!("{:.6}", ob4.i.to_degrees()));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("ω");
                            });
                            row.col(|ui| {
                                if let Some(ob1) = ob1 {
                                    ui.label(format!("{:.6}", ob1.argpe.to_degrees()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob2) = ob2 {
                                    ui.label(format!("{:.6}", ob2.argpe.to_degrees()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob3) = ob3 {
                                    ui.label(format!("{:.6}", ob3.argpe.to_degrees()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob4) = ob4 {
                                    ui.label(format!("{:.6}", ob4.argpe.to_degrees()));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("Ω");
                            });
                            row.col(|ui| {
                                if let Some(ob1) = ob1 {
                                    ui.label(format!("{:.6}", ob1.lan.to_degrees()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob2) = ob2 {
                                    ui.label(format!("{:.6}", ob2.lan.to_degrees()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob3) = ob3 {
                                    ui.label(format!("{:.6}", ob3.lan.to_degrees()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob4) = ob4 {
                                    ui.label(format!("{:.6}", ob4.lan.to_degrees()));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("ν");
                            });
                            row.col(|ui| {
                                if let Some(ob1) = ob1 {
                                    ui.label(format!("{:.6}", ob1.ta.to_degrees()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob2) = ob2 {
                                    ui.label(format!("{:.6}", ob2.ta.to_degrees()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob3) = ob3 {
                                    ui.label(format!("{:.6}", ob3.ta.to_degrees()));
                                }
                            });
                            row.col(|ui| {
                                if let Some(ob4) = ob4 {
                                    ui.label(format!("{:.6}", ob4.ta.to_degrees()));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("Rx");
                            });
                            row.col(|ui| {
                                if let Some(sv1) = &sv1 {
                                    ui.label(format!("{:.4}", sv1.position.x));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv2) = &sv2 {
                                    ui.label(format!("{:.4}", sv2.position.x));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv3) = &sv3 {
                                    ui.label(format!("{:.4}", sv3.position.x));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv4) = &sv4 {
                                    ui.label(format!("{:.4}", sv4.position.x));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("Ry");
                            });
                            row.col(|ui| {
                                if let Some(sv1) = &sv1 {
                                    ui.label(format!("{:.4}", sv1.position.y));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv2) = &sv2 {
                                    ui.label(format!("{:.4}", sv2.position.y));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv3) = &sv3 {
                                    ui.label(format!("{:.4}", sv3.position.y));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv4) = &sv4 {
                                    ui.label(format!("{:.4}", sv4.position.y));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("Rz");
                            });
                            row.col(|ui| {
                                if let Some(sv1) = &sv1 {
                                    ui.label(format!("{:.4}", sv1.position.z));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv2) = &sv2 {
                                    ui.label(format!("{:.4}", sv2.position.z));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv3) = &sv3 {
                                    ui.label(format!("{:.4}", sv3.position.z));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv4) = &sv4 {
                                    ui.label(format!("{:.4}", sv4.position.z));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("Vx");
                            });
                            row.col(|ui| {
                                if let Some(sv1) = &sv1 {
                                    ui.label(format!("{:.4}", sv1.velocity.x));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv2) = &sv2 {
                                    ui.label(format!("{:.4}", sv2.velocity.x));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv3) = &sv3 {
                                    ui.label(format!("{:.4}", sv3.velocity.x));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv4) = &sv4 {
                                    ui.label(format!("{:.4}", sv4.velocity.x));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("Vy");
                            });
                            row.col(|ui| {
                                if let Some(sv1) = &sv1 {
                                    ui.label(format!("{:.4}", sv1.velocity.y));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv2) = &sv2 {
                                    ui.label(format!("{:.4}", sv2.velocity.y));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv3) = &sv3 {
                                    ui.label(format!("{:.4}", sv3.velocity.y));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv4) = &sv4 {
                                    ui.label(format!("{:.4}", sv4.velocity.y));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("Vz");
                            });
                            row.col(|ui| {
                                if let Some(sv1) = &sv1 {
                                    ui.label(format!("{:.4}", sv1.velocity.z));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv2) = &sv2 {
                                    ui.label(format!("{:.4}", sv2.velocity.z));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv3) = &sv3 {
                                    ui.label(format!("{:.4}", sv3.velocity.z));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv4) = &sv4 {
                                    ui.label(format!("{:.4}", sv4.velocity.z));
                                }
                            });
                        });

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("SOI");
                            });
                            row.col(|ui| {
                                if let Some(sv1) = &sv1 {
                                    ui.label(format!("{}", sv1.body.name));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv2) = &sv2 {
                                    ui.label(format!("{}", sv2.body.name));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv3) = &sv3 {
                                    ui.label(format!("{}", sv3.body.name));
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv4) = &sv4 {
                                    ui.label(format!("{}", sv4.body.name));
                                }
                            });
                        });
                    });
            });
    }

    fn handle_rx(
        &mut self,
        _res: eyre::Result<HRes>,
        _mission: &Arc<RwLock<Mission>>,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
        _ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()> {
        Ok(())
    }
}

#[derive(Debug)]
pub struct VectorPanelSummary {
    pub ui_id: egui::Id,
    pub vessel: Option<VesselRef>,
    pub slot: String,
    pub loading: bool,
}

impl Default for VectorPanelSummary {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
            vessel: None,
            slot: String::new(),
            loading: false,
        }
    }
}

impl KtkDisplay for VectorPanelSummary {
    fn show(
        &mut self,
        mission: &Arc<RwLock<Mission>>,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut bool,
    ) {
        egui::Window::new(i18n!("vps-title"))
            .open(open)
            .default_width(128.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.add(VectorSelector::new(
                        self.ui_id.with("SVSel"),
                        i18n!("vps-slot"),
                        mission,
                        &mut self.vessel,
                        &mut self.slot,
                    ));
                });
                ui.horizontal(|ui| {
                    if ui
                        .button(icon_label("\u{e255}", &i18n!("vps-load-ksp")))
                        .clicked()
                    {
                        handle(toasts, |_| {
                            backend.tx(
                                DisplaySelect::VPS,
                                HReq::LoadStateVector(
                                    self.vessel
                                        .clone()
                                        .ok_or_eyre(i18n!("vps-error-no-vessel"))?
                                        .0
                                        .read()
                                        .link
                                        .ok_or_eyre(i18n!("vps-error-no-link"))?,
                                ),
                            )?;
                            self.loading = true;
                            Ok(())
                        });
                    }
                    if self.loading {
                        ui.spinner();
                    }
                });
                if let Some(vessel) = self.vessel.clone() {
                    if let Some(sv) = vessel.0.read().svs.get(&self.slot) {
                        ui.horizontal(|ui| {
                            ui.vertical(|ui| {
                                let time = sv.time.into_duration().as_seconds_f64();
                                let len = std::cmp::max(
                                    format!("{:.4}", sv.position.x).len(),
                                    std::cmp::max(
                                        format!("{:.4}", sv.position.y).len(),
                                        std::cmp::max(
                                            format!("{:.4}", sv.position.z).len(),
                                            format!("{:.2}", time).len(),
                                        ),
                                    ),
                                );
                                ui.monospace(format!("Rx: {: >len$.4}", sv.position.x, len = len));
                                ui.monospace(format!("Ry: {: >len$.4}", sv.position.y, len = len));
                                ui.monospace(format!("Rz: {: >len$.4}", sv.position.z, len = len));
                                ui.monospace(format!("T:  {: >len$.2}", time, len = len));
                            });
                            ui.separator();
                            ui.vertical(|ui| {
                                let len = std::cmp::max(
                                    format!("{:.4}", sv.velocity.x).len(),
                                    std::cmp::max(
                                        format!("{:.4}", sv.velocity.y).len(),
                                        std::cmp::max(
                                            format!("{:.4}", sv.velocity.z).len(),
                                            sv.body.name.len(),
                                        ),
                                    ),
                                );
                                ui.monospace(format!("Vx: {: >len$.4}", sv.velocity.x, len = len));
                                ui.monospace(format!("Vy: {: >len$.4}", sv.velocity.y, len = len));
                                ui.monospace(format!("Vz: {: >len$.4}", sv.velocity.z, len = len));
                                ui.monospace(format!("CB: {: >len$}", sv.body.name, len = len));
                            });
                        });
                    } else {
                        ui.label(i18n!("vps-no-sv"));
                    }
                }
            });
    }

    fn handle_rx(
        &mut self,
        res: eyre::Result<HRes>,
        _mission: &Arc<RwLock<Mission>>,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
        _ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()> {
        self.loading = false;
        if let Ok(HRes::LoadedStateVector(sv)) = res {
            if self.slot.trim().is_empty() {
                bail!("{}", i18n!("vps-error-empty-name"));
            }
            self.vessel
                .clone()
                .ok_or_eyre(i18n!("vps-error-no-vessel"))?
                .0
                .write()
                .svs
                .insert(self.slot.clone(), sv);
            Ok(())
        } else {
            res.map(|_| ())
        }
    }
}
