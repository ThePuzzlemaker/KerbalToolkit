use std::{collections::HashMap, sync::Arc, time::Instant};

use color_eyre::eyre;
use egui_notify::Toasts;
use itertools::Itertools;
use kerbtk::kepler::orbits::StateVector;
use parking_lot::RwLock;

use crate::{backend::HRes, mission::Mission, Backend, KtkDisplay, TimeInput, UTorGET};

#[derive(Debug)]
pub struct VectorComparison {
    pub open: bool,
    pub vectors: HashMap<String, StateVector>,
    pub comparison_time_input: TimeInput,
    pub comparison_time_unparsed: String,
    pub comparison_time: Option<UTorGET>,
    pub v1: String,
    pub v2: String,
    pub v3: String,
    pub v4: String,
    pub ui_id: egui::Id,
}

impl Default for VectorComparison {
    fn default() -> Self {
        Self {
            open: Default::default(),
            vectors: Default::default(),
            comparison_time_input: Default::default(),
            comparison_time_unparsed: Default::default(),
            comparison_time: Default::default(),
            v1: "N/A".to_string(),
            v2: "N/A".to_string(),
            v3: "N/A".to_string(),
            v4: "N/A".to_string(),
            ui_id: egui::Id::new(Instant::now()),
        }
    }
}

impl KtkDisplay for VectorComparison {
    fn show(
        &mut self,
        _mission: &Arc<RwLock<Mission>>,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut bool,
    ) {
        egui::Window::new("Vector Comparison")
            .open(open)
            .default_size([384.0, 256.0])
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
                ui.horizontal(|ui| {
                    ui.label("V1");
                    egui::ComboBox::from_id_source(self.ui_id.with("V1"))
                        .selected_text(self.v1.clone())
                        .wrap(false)
                        .show_ui(ui, |ui| {
                            for key in self.vectors.keys().sorted() {
                                ui.selectable_value(&mut self.v1, key.to_string(), key);
                            }
                            if self.vectors.is_empty() {
                                ui.selectable_value(&mut self.v1, "N/A".to_string(), "N/A");
                            }
                        });

                    ui.label("V2");
                    egui::ComboBox::from_id_source(self.ui_id.with("V2"))
                        .selected_text(self.v1.clone())
                        .wrap(false)
                        .show_ui(ui, |ui| {
                            for key in self.vectors.keys().sorted() {
                                ui.selectable_value(&mut self.v2, key.to_string(), key);
                            }
                            if self.vectors.is_empty() {
                                ui.selectable_value(&mut self.v2, "N/A".to_string(), "N/A");
                            }
                        });

                    ui.label("V3");
                    egui::ComboBox::from_id_source(self.ui_id.with("V3"))
                        .selected_text(self.v3.clone())
                        .wrap(false)
                        .show_ui(ui, |ui| {
                            for key in self.vectors.keys().sorted() {
                                ui.selectable_value(&mut self.v3, key.to_string(), key);
                            }
                            ui.selectable_value(&mut self.v3, "N/A".to_string(), "N/A");
                        });

                    ui.label("V4");
                    egui::ComboBox::from_id_source(self.ui_id.with("V4"))
                        .selected_text(self.v1.clone())
                        .wrap(false)
                        .show_ui(ui, |ui| {
                            for key in self.vectors.keys().sorted() {
                                ui.selectable_value(&mut self.v4, key.to_string(), key);
                            }
                            if self.vectors.is_empty() {
                                ui.selectable_value(&mut self.v4, "N/A".to_string(), "N/A");
                            }
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
}

impl Default for VectorPanelSummary {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
        }
    }
}

impl KtkDisplay for VectorPanelSummary {
    fn show(
        &mut self,
        _mission: &Arc<RwLock<Mission>>,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut bool,
    ) {
        egui::Window::new("Vector Panel Summary")
            .open(open)
            .default_width(128.0)
            .show(ctx, |_ui| {});
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
