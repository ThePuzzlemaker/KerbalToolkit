use std::{collections::HashMap, sync::Arc, time::Instant};

use color_eyre::eyre::{self, bail, OptionExt};
use egui_extras::Column;
use egui_notify::Toasts;
use itertools::Itertools;
use kerbtk::vessel::VesselRef;
use parking_lot::RwLock;

use crate::{
    backend::{HReq, HRes},
    handle, icon_label,
    mission::Mission,
    Backend, DisplaySelect, KtkDisplay, TimeInput, UTorGET,
};

#[derive(Debug)]
pub struct VectorComparison {
    pub comparison_time_input: TimeInput,
    pub comparison_time_unparsed: String,
    pub comparison_time: Option<UTorGET>,
    pub v1: Option<VesselRef>,
    pub v1_slot: String,
    pub v2: Option<VesselRef>,
    pub v2_slot: String,
    pub v3: Option<VesselRef>,
    pub v3_slot: String,
    pub v4: Option<VesselRef>,
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
            v1_slot: "".into(),
            v2: None,
            v2_slot: "".into(),
            v3: None,
            v3_slot: "".into(),
            v4: None,
            v4_slot: "".into(),
            ui_id: egui::Id::new(Instant::now()),
        }
    }
}

impl VectorComparison {
    fn selector(
        ui_id: egui::Id,
        text: &str,
        mission: &Arc<RwLock<Mission>>,
        vessel: &mut Option<VesselRef>,
        slot: &mut String,
        ui: &mut egui::Ui,
    ) {
        ui.horizontal(|ui| {
            ui.label(text);
            egui::ComboBox::from_id_source(ui_id.with(text).with("VesselSelector"))
                .selected_text(
                    vessel
                        .clone()
                        .map(|x| x.0.read().name.clone())
                        .unwrap_or("N/A".into()),
                )
                .show_ui(ui, |ui| {
                    for iter_vessel in mission
                        .read()
                        .vessels
                        .iter()
                        .sorted_by_key(|x| x.read().name.clone())
                    {
                        ui.selectable_value(
                            vessel,
                            Some(VesselRef(iter_vessel.clone())),
                            &iter_vessel.read().name,
                        );
                    }
                });
            egui::TextEdit::singleline(slot)
                .char_limit(16)
                .desired_width(32.0)
                .show(ui);
        });
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
        egui::Window::new("Vector Comparison")
            .open(open)
            .default_size([384.0, 480.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Comparison Time");
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
                    if egui::TextEdit::singleline(&mut self.comparison_time_unparsed)
                        .font(egui::TextStyle::Monospace)
                        .desired_width(128.0)
                        .show(ui)
                        .response
                        .changed()
                    {
                        self.comparison_time = self
                            .comparison_time_input
                            .parse(&self.comparison_time_unparsed);
                    }
                });

                let sv1 = self
                    .v1
                    .as_ref()
                    .and_then(|x| x.0.read().svs.get(&self.v1_slot).cloned());
                let ob1 = sv1.clone().map(|x| x.into_orbit(1e-8));
                let sv2 = self
                    .v2
                    .as_ref()
                    .and_then(|x| x.0.read().svs.get(&self.v2_slot).cloned());
                let ob2 = sv2.clone().map(|x| x.into_orbit(1e-8));
                let sv3 = self
                    .v3
                    .as_ref()
                    .and_then(|x| x.0.read().svs.get(&self.v3_slot).cloned());
                let ob3 = sv3.clone().map(|x| x.into_orbit(1e-8));
                let sv4 = self
                    .v4
                    .as_ref()
                    .and_then(|x| x.0.read().svs.get(&self.v4_slot).cloned());
                let ob4 = sv4.clone().map(|x| x.into_orbit(1e-8));

                ui.horizontal(|ui| {
                    Self::selector(
                        self.ui_id,
                        "V1",
                        mission,
                        &mut self.v1,
                        &mut self.v1_slot,
                        ui,
                    );
                    Self::selector(
                        self.ui_id,
                        "V3",
                        mission,
                        &mut self.v3,
                        &mut self.v3_slot,
                        ui,
                    );
                });
                ui.horizontal(|ui| {
                    Self::selector(
                        self.ui_id,
                        "V2",
                        mission,
                        &mut self.v2,
                        &mut self.v2_slot,
                        ui,
                    );
                    Self::selector(
                        self.ui_id,
                        "V4",
                        mission,
                        &mut self.v4,
                        &mut self.v4_slot,
                        ui,
                    );
                });

                ui.separator();

                egui_extras::TableBuilder::new(ui)
                    .column(Column::auto_with_initial_suggestion(24.0).resizable(false))
                    .column(Column::auto_with_initial_suggestion(96.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(96.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(96.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(96.0).resizable(true))
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
                                    ui.label("V1");
                                },
                            );
                        });
                        header.col(|ui| {
                            ui.with_layout(
                                egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                                |ui| {
                                    ui.label("V2");
                                },
                            );
                        });
                        header.col(|ui| {
                            ui.with_layout(
                                egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                                |ui| {
                                    ui.label("V3");
                                },
                            );
                        });
                        header.col(|ui| {
                            ui.with_layout(
                                egui::Layout::centered_and_justified(egui::Direction::LeftToRight),
                                |ui| {
                                    ui.label("V4");
                                },
                            );
                        });
                    })
                    .body(|mut body| {
                        body.ui_mut().style_mut().override_text_style =
                            Some(egui::TextStyle::Monospace);

                        body.row(16.0, |mut row| {
                            row.col(|ui| {
                                ui.label("Tag");
                            });
                            row.col(|ui| {
                                if let Some(sv1) = &sv1 {
                                    let (d, h, m, s, ms) = (
                                        sv1.time.days(),
                                        sv1.time.hours(),
                                        sv1.time.minutes(),
                                        sv1.time.seconds(),
                                        sv1.time.millis(),
                                    );
                                    ui.add(
                                        egui::Label::new(format!(
                                            "{}:{:02}:{:02}:{:02}.{:03}",
                                            d, h, m, s, ms
                                        ))
                                        .wrap(false),
                                    );
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv2) = &sv2 {
                                    let (d, h, m, s, ms) = (
                                        sv2.time.days(),
                                        sv2.time.hours(),
                                        sv2.time.minutes(),
                                        sv2.time.seconds(),
                                        sv2.time.millis(),
                                    );
                                    ui.add(
                                        egui::Label::new(format!(
                                            "{}:{:02}:{:02}:{:02}.{:03}",
                                            d, h, m, s, ms
                                        ))
                                        .wrap(false),
                                    );
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv3) = &sv3 {
                                    let (d, h, m, s, ms) = (
                                        sv3.time.days(),
                                        sv3.time.hours(),
                                        sv3.time.minutes(),
                                        sv3.time.seconds(),
                                        sv3.time.millis(),
                                    );
                                    ui.add(
                                        egui::Label::new(format!(
                                            "{}:{:02}:{:02}:{:02}.{:03}",
                                            d, h, m, s, ms
                                        ))
                                        .wrap(false),
                                    );
                                }
                            });
                            row.col(|ui| {
                                if let Some(sv4) = &sv4 {
                                    let (d, h, m, s, ms) = (
                                        sv4.time.days(),
                                        sv4.time.hours(),
                                        sv4.time.minutes(),
                                        sv4.time.seconds(),
                                        sv4.time.millis(),
                                    );
                                    ui.add(
                                        egui::Label::new(format!(
                                            "{}:{:02}:{:02}:{:02}.{:03}",
                                            d, h, m, s, ms
                                        ))
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
        egui::Window::new("Vector Panel Summary")
            .open(open)
            .default_width(128.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Vessel");
                    egui::ComboBox::from_id_source(self.ui_id.with("Vessel"))
                        .selected_text(
                            self.vessel
                                .clone()
                                .map(|x| x.0.read().name.clone())
                                .unwrap_or("N/A".into()),
                        )
                        .show_ui(ui, |ui| {
                            for vessel in mission
                                .read()
                                .vessels
                                .iter()
                                .sorted_by_key(|x| x.read().name.clone())
                            {
                                ui.selectable_value(
                                    &mut self.vessel,
                                    Some(VesselRef(vessel.clone())),
                                    &vessel.read().name,
                                );
                            }
                        });
                    ui.label("Slot");
                    egui::TextEdit::singleline(&mut self.slot)
                        .char_limit(16)
                        .show(ui);
                });
                ui.horizontal(|ui| {
                    if ui.button(icon_label("\u{e255}", "Load from KSP")).clicked() {
                        handle(toasts, |_| {
                            backend.tx(
                                DisplaySelect::VPS,
                                HReq::LoadStateVector(
                                    self.vessel
                                        .clone()
                                        .ok_or_eyre("No vessel selected")?
                                        .0
                                        .read()
                                        .link
                                        .ok_or_eyre("Vessel was not linked to a KSP vessel")?,
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
                        ui.label("No state vector in slot.");
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
                bail!("State vector slot name cannot be empty");
            }
            self.vessel
                .clone()
                .ok_or_eyre("No vessel selected")?
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
