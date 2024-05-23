use std::{collections::HashMap, time::Instant};

use itertools::Itertools;
use kerbtk::kepler::orbits::StateVector;

use crate::{App, TimeInput, UTorGET};

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

impl VectorComparison {
    pub fn show(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::Window::new("Vector Comparison")
            .open(&mut self.open)
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
}

#[derive(Debug)]
pub struct VectorPanelSummary {
    pub open: bool,
    pub ui_id: egui::Id,
}

impl Default for VectorPanelSummary {
    fn default() -> Self {
        Self {
            open: Default::default(),
            ui_id: egui::Id::new(Instant::now()),
        }
    }
}

impl VectorPanelSummary {
    pub fn show(app: &mut App, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::Window::new("Vector Panel Summary")
            .open(&mut app.vps.open)
            .default_width(128.0)
            .show(ctx, |_ui| {});
    }
}
