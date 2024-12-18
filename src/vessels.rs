use std::{
    collections::{HashMap, HashSet},
    fmt,
    time::Instant,
};

use color_eyre::eyre::{self, OptionExt};
use egui::TextBuffer;
use egui_extras::Size;
use egui_grid::GridBuilder;
use egui_modal::Modal;
use egui_notify::Toasts;
use itertools::Itertools;
use kerbtk::{
    arena::Arena,
    krpc,
    vessel::{self, Decouplers, PartId, Vessel, VesselClass, VesselClassId, VesselId},
};

use crate::{
    backend::{HReq, HRes},
    handle, i18n, i18n_args, icon, icon_label,
    mission::Mission,
    widgets::{TimeDisplayBtn, TimeDisplayKind, TimeInput1, TimeInputKind2},
    Backend, DisplaySelect, Displays, KtkDisplay, UTorGET,
};

#[derive(Debug)]
pub struct Classes {
    ui_id: egui::Id,
    search: String,
    current_class: Option<VesselClassId>,
    renaming: bool,
    just_clicked_rename: bool,
    classes_filtered: Vec<VesselClassId>,
    pub force_refilter: bool,
    loading: bool,
    checkboxes: HashMap<PartId, bool>,
    fairings: HashMap<PartId, bool>,
    rocheckboxes: HashMap<PartId, (bool, bool)>,
    subvessels: Vec<HashSet<PartId>>,
    subvessel_options: Vec<SubvesselOption>,
    subvessel_names: Vec<String>,
}

impl Default for Classes {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
            search: String::new(),
            current_class: None,
            renaming: false,
            just_clicked_rename: false,
            classes_filtered: vec![],
            force_refilter: false,
            loading: false,
            checkboxes: HashMap::default(),
            fairings: HashMap::default(),
            rocheckboxes: HashMap::default(),
            subvessels: vec![],
            subvessel_options: vec![],
            subvessel_names: vec![],
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SubvesselOption {
    Keep,
    Discard,
}

impl fmt::Display for SubvesselOption {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SubvesselOption::Keep => write!(f, "{}", i18n!("classes-keep")),
            SubvesselOption::Discard => write!(f, "{}", i18n!("classes-discard")),
        }
    }
}

impl Classes {
    fn refilter(&mut self, mission: &Mission) {
        self.classes_filtered = mission
            .classes
            .iter()
            .sorted_by_key(|(_, x)| x.name.clone())
            .filter_map(|(id, x)| {
                (self.search.is_empty() || x.name.trim().starts_with(self.search.trim()))
                    .then_some(id)
            })
            .collect();
    }

    fn search_box(&mut self, ui: &mut egui::Ui, mission: &Mission, backend: &mut Backend) {
        egui::Frame::group(ui.style()).show(ui, |ui| {
            egui::ScrollArea::vertical()
                .id_salt(self.ui_id.with("Classes"))
                .auto_shrink(false)
                .show(ui, |ui| {
                    let search = egui::TextEdit::singleline(&mut self.search)
                        .hint_text(i18n!("classes-searchbox-hint"))
                        .frame(true)
                        .show(ui);

                    if search.response.changed() {
                        self.refilter(mission);
                    }

                    if self.force_refilter {
                        self.refilter(mission);
                    }

                    let already_exists = mission
                        .classes
                        .iter()
                        .find_map(|(id, x)| (x.name.trim() == self.search.trim()).then_some(id));

                    if already_exists.is_none()
                        && (search.response.lost_focus()
                            && ui.input(|i| i.key_pressed(egui::Key::Enter))
                            && !self.search.trim().is_empty())
                        || (already_exists.is_none()
                            && !self.search.trim().is_empty()
                            && ui
                                .button(i18n_args!("classes-create", "class", self.search.trim()))
                                .clicked())
                    {
                        let class = VesselClass {
                            name: self.search.take(),
                            ..VesselClass::default()
                        };
                        backend.effect(move |mission, state| {
                            let class_id = mission.classes.push(class);
                            state.classes.current_class = Some(class_id);
                            state.classes.refilter(mission);
                            Ok(())
                        });
                        self.search.clear();
                    }

                    if let Some(class) = already_exists {
                        if search.response.lost_focus()
                            && ui.input(|i| i.key_pressed(egui::Key::Enter))
                            && !self.search.trim().is_empty()
                        {
                            self.current_class = Some(class);
                        }
                    }

                    for class in &self.classes_filtered {
                        let checked = self.current_class.as_ref().is_some_and(|x| class == x);
                        if ui
                            .selectable_label(checked, mission.classes[*class].name.trim())
                            .clicked()
                        {
                            self.checkboxes.clear();
                            self.rocheckboxes.clear();
                            self.current_class = Some(*class);
                        };
                    }
                });
        });
    }

    fn modal(
        &mut self,
        ctx: &egui::Context,
        ui: &mut egui::Ui,
        modal: &Modal,
        class_id: VesselClassId,
        backend: &mut Backend,
        mission: &Mission,
    ) {
        let class = &mission.classes[class_id];
        egui::ScrollArea::vertical().show(ui, |ui| {
            for (ix, subvessel) in self.subvessels.iter().enumerate() {
                let mut state = egui::collapsing_header::CollapsingState::load_with_default_open(
                    ctx,
                    self.ui_id.with("Subvessel").with(ix),
                    false,
                );
                let header_res = ui.horizontal(|ui| {
                    state.show_toggle_button(ui, egui::collapsing_header::paint_default_icon);
                    ui.label(
                        egui::RichText::new(i18n_args!("classes-subvessel", "n", ix + 1)).strong(),
                    );
                    egui::ComboBox::from_id_salt(self.ui_id.with("SubvesselComboBox").with(ix))
                        .selected_text(self.subvessel_options[ix].to_string())
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self.subvessel_options[ix],
                                SubvesselOption::Keep,
                                i18n!("classes-keep"),
                            );
                            ui.selectable_value(
                                &mut self.subvessel_options[ix],
                                SubvesselOption::Discard,
                                i18n!("classes-discard"),
                            );
                        });
                    if self.subvessel_options[ix] == SubvesselOption::Keep {
                        ui.text_edit_singleline(&mut self.subvessel_names[ix]);
                    }
                });
                state.show_body_indented(&header_res.response, ui, |ui| {
                    ui.vertical(|ui| {
                        for part in subvessel.iter().sorted_by_key(|&part| {
                            (&class.parts[*part].title, &class.parts[*part].tag)
                        }) {
                            let fairing = self.fairings.get(part).copied().unwrap_or(false);
                            let has_fairing = class.parts[*part]
                                .mass_modifiers
                                .iter()
                                .any(|x| x.module_name == "ModuleProceduralFairing");

                            let fairing = has_fairing.then_some(fairing);
                            let tag = class.parts[*part].tag.trim();
                            let tag = (!tag.is_empty()).then_some(tag);
                            let title = &class.parts[*part].title;

                            let text = match (fairing, tag) {
                                (None, None) => i18n_args!("classes-part", "part", title),
                                (None, Some(tag)) => {
                                    i18n_args!("classes-part-tag", "part", title, "tag", tag)
                                }
                                (Some(true), None) => {
                                    i18n_args!("classes-part-staged-fairing", "part", title)
                                }
                                (Some(false), None) => {
                                    i18n_args!("classes-part-unstaged-fairing", "part", title)
                                }
                                (Some(true), Some(tag)) => i18n_args!(
                                    "classes-part-staged-fairing-tag",
                                    "part",
                                    title,
                                    "tag",
                                    tag
                                ),
                                (Some(false), Some(tag)) => i18n_args!(
                                    "classes-part-unstaged-fairing-tag",
                                    "part",
                                    title,
                                    "tag",
                                    tag
                                ),
                            };

                            ui.label(egui::RichText::new(format!(" ▪ {text}")));
                        }
                    })
                });
            }
            ui.horizontal(|ui| {
                if ui.button(i18n!("classes-cancel")).clicked() {
                    modal.close();
                }
                if ui.button(i18n!("classes-finish")).clicked() {
                    modal.close();
                    self.force_refilter = true;
                    for ((subvessel, option), name) in self
                        .subvessels
                        .iter()
                        .zip(self.subvessel_options.iter().copied())
                        .zip(self.subvessel_names.iter())
                    {
                        let mut parts = Arena::new();
                        for partid in subvessel {
                            let mut part = class.parts[*partid].clone();

                            if self.fairings.get(partid).copied().unwrap_or(false) {
                                if let Some(fairing) = part
                                    .mass_modifiers
                                    .iter_mut()
                                    .find(|x| x.module_name == "ModuleProceduralFairing")
                                {
                                    fairing.current_mass = fairing.staged_mass;
                                }
                            }

                            if part
                                .parent
                                .is_some_and(|parent| !subvessel.contains(&parent))
                            {
                                part.parent = None;
                            }
                            part.children.retain(|x| subvessel.contains(x));

                            match part.decouplers.as_mut() {
                                Some(Decouplers::Single(decoupler)) => {
                                    if decoupler
                                        .attached_part
                                        .is_some_and(|part| !subvessel.contains(&part))
                                    {
                                        decoupler.attached_part = None;
                                    };
                                }
                                Some(Decouplers::RODecoupler { top, bot }) => {
                                    if top
                                        .attached_part
                                        .is_some_and(|part| !subvessel.contains(&part))
                                    {
                                        top.attached_part = None;
                                    };
                                    if bot
                                        .attached_part
                                        .is_some_and(|part| !subvessel.contains(&part))
                                    {
                                        bot.attached_part = None;
                                    };
                                }
                                _ => {}
                            }
                            parts.insert(*partid, part);
                        }
                        let mut root = None;
                        if let Some(mut part) = subvessel.iter().next().copied() {
                            while let Some(new_part) = parts[part].parent {
                                part = new_part;
                            }
                            root = Some(part);
                        }

                        let mut persistent_id_map = HashMap::new();
                        for (id, part) in parts.iter() {
                            persistent_id_map.insert(part.tracked_id, id);
                        }

                        if option == SubvesselOption::Keep {
                            let vessel = VesselClass {
                                name: name.clone(),
                                description: String::new(),
                                shortcode: String::new(),
                                parts,
                                root,
                                tracked_id_map: persistent_id_map,
                            };
                            backend.effect(|mission, _| {
                                mission.classes.push(vessel);
                                Ok(())
                            });
                        }

                        self.fairings.clear();
                    }
                }
            });
        });
    }
}

impl KtkDisplay for Classes {
    fn show(
        &mut self,
        mission: &Mission,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    ) {
        egui::Window::new(i18n!("classes-title"))
            .open(&mut open.classes)
            .default_size([384.0, 512.0])
            .show(ctx, |ui| {
                GridBuilder::new()
                    .new_row(Size::initial(384.0))
                    .cell(Size::relative(1.0 / 3.0))
                    .cell(Size::remainder())
                    .show(ui, |mut grid| {
                        grid.cell(|ui| {
                            self.search_box(ui, mission, backend);
                        });
                        grid.cell(|ui| {
                            egui::Frame::none().inner_margin(6.0).show(ui, |ui| {
                                egui::ScrollArea::vertical()
                                    .id_salt(self.ui_id.with("Parts"))
                                    .show(ui, |ui| {
                                        if let Some(class_id) = self.current_class {
                                            if self.renaming {
						let mut name = mission.classes[class_id].name.clone();
                                                let text =
                                                    egui::TextEdit::singleline(&mut name)
                                                        .font(egui::TextStyle::Heading)
                                                        .show(ui);

                                                if text.response.changed() {
						    backend.effect(move |mission, state| {
							mission.classes[class_id].name = name;
							state.classes.force_refilter = true;
							Ok(())
						    });
                                                }

                                                if self.just_clicked_rename {
                                                    self.just_clicked_rename = false;
                                                    text.response.request_focus();
                                                }

                                                if text.response.lost_focus()
                                                    && ui.input(|i| i.key_pressed(egui::Key::Enter))
                                                {
                                                    self.renaming = false;
                                                }
                                            } else if !self.renaming {
                                                ui.heading(mission.classes[class_id].name.trim());
                                            }

                                            ui.horizontal(|ui| {
                                                if self.renaming
                                                    && ui
                                                        .button(icon("\u{e161}"))
                                                        .on_hover_text(i18n!("classes-save"))
                                                        .clicked()
                                                {
                                                    self.renaming = false;
                                                } else if !self.renaming
                                                    && ui
                                                        .button(icon("\u{e3c9}"))
                                                        .on_hover_text(i18n!("classes-rename"))
                                                        .clicked()
                                                {
                                                    self.renaming = true;
                                                    self.just_clicked_rename = true;
                                                    self.force_refilter = true;
                                                }

                                                // TODO: confirm delete
                                                if ui
                                                    .button(icon("\u{e872}"))
                                                    .on_hover_text(i18n!("classes-delete"))
                                                    .clicked()
                                                {
                                                    self.renaming = false;
                                                    let pos = self
                                                        .classes_filtered
                                                        .iter()
                                                        .enumerate()
                                                        .find_map(|(i, x)| {
                                                            (*x == class_id).then_some(i)
                                                        })
                                                        .expect("oops");
						    backend.effect(move |mission, _| {
							mission.classes.retain(|i, _| i != class_id);
							Ok(())
						    });
                                                    self.classes_filtered.remove(pos);
                                                    self.current_class =
                                                        self.classes_filtered.get(pos).copied().or(
                                                            self.classes_filtered
                                                                .get(pos.saturating_sub(1))
                                                                .copied(),
                                                        );
                                                    self.force_refilter = true;
                                                }
                                            });

                                            ui.horizontal(|ui| {
                                                handle(toasts, |_| {
                                                    let load_btn = ui
                                                        .button(i18n!("classes-load-editor"))
                                                        .on_hover_text(i18n!(
                                                            "classes-load-editor-explainer"
                                                        ));
                                                    if load_btn.clicked() {
                                                        backend.tx(
                                                            DisplaySelect::Classes,
                                                            HReq::LoadVesselPartsFromEditor,
                                                        )?;
                                                        self.loading = true;
                                                    }
                                                    let load_btn = ui
                                                        .button(i18n!("classes-load-flight"))
                                                        .on_hover_text(i18n!(
                                                            "classes-load-flight-explainer"
                                                        ));
                                                    if load_btn.clicked() {
                                                        backend.tx(
                                                            DisplaySelect::Classes,
                                                            HReq::LoadVesselClassFromFlight,
                                                        )?;
                                                        self.loading = true;
                                                    }
                                                    Ok(())
                                                });
                                                if self.loading {
                                                    ui.spinner();
                                                }
                                            });

                                            ui.label(i18n!("classes-description"));
					    let mut description = mission.classes[class_id].description.clone();
                                            if egui::TextEdit::multiline(&mut description)
                                                .show(ui).response.changed() {
						    backend.effect(move |mission, _| {
							mission.classes[class_id].description = description;
							Ok(())
						    });
						};
                                            ui.horizontal(|ui| {
                                                ui.label(i18n!("classes-shortcode"));
						let mut shortcode = mission.classes[class_id].shortcode.clone();
                                                let shortcode_edit = egui::TextEdit::singleline(
                                                    &mut shortcode,
                                                )
                                                .char_limit(5)
                                                    .show(ui);
						if shortcode_edit.response.changed() {
						    backend.effect(move |mission, _| {
							mission.classes[class_id].shortcode = shortcode;
							Ok(())
						    });
						}
                                                shortcode_edit.response.on_hover_ui(|ui| {
                                                    ui.horizontal_wrapped(|ui| {
                                                        ui.label(i18n!(
                                                            "classes-shortcode-explainer-1"
                                                        ));
                                                        ui.label(
                                                            egui::RichText::new(i18n!(
                                                                "classes-shortcode-explainer-2"
                                                            ))
                                                            .strong(),
                                                        );
                                                    });
                                                });
                                            });
                                            ui.heading(i18n!("classes-decouplers"));
                                            ui.label(i18n!("classes-calcsep-explainer"));
                                            for (partid, part) in mission.classes[class_id]
                                                .parts
                                                .iter()
                                                .filter(|x| {
                                                    x.1.decouplers.is_some()
                                                        || !x.1.mass_modifiers.is_empty()
                                                })
                                                .sorted_by_key(|(_, x)| (&x.title, &x.tag))
                                            {
                                                ui.add_space(0.2);

                                                if part.mass_modifiers.iter().any(|x| {
                                                    x.module_name == "ModuleProceduralFairing"
                                                        && (x.current_mass - x.unstaged_mass).abs() <= f64::EPSILON
                                                }) {
                                                    if part.tag.trim().is_empty() {
                                                        ui.checkbox(
                                                            self.fairings
                                                                .entry(partid)
                                                                .or_insert(false),
                                                            i18n_args!("classes-decoupler-fairing", "part", &part.title)
                                                        );
                                                    } else {
                                                        ui.checkbox(
                                                            self.fairings
                                                                .entry(partid)
                                                                .or_insert(false),
                                                            i18n_args!("classes-decoupler-fairing-tag", "part", &part.title, "tag", &part.tag)
                                                        );
                                                    }
                                                }

                                                match part.decouplers.as_ref() {
                                                    Some(Decouplers::Single(_) | Decouplers::ProceduralFairing(_)) => {
                                                        if part.tag.trim().is_empty() {
                                                            ui.checkbox(
                                                                self.checkboxes
                                                                    .entry(partid)
                                                                    .or_insert(false),
                                                                &part.title,
                                                            );
                                                        } else {
                                                            ui.checkbox(
                                                                self.checkboxes
                                                                    .entry(partid)
                                                                    .or_insert(false),
                                                                i18n_args!("classes-decoupler-tag", "part", &part.title, "tag", &part.tag)
                                                            );
                                                        }
                                                    }
                                                    Some(Decouplers::RODecoupler { .. }) => {
                                                        let &mut (ref mut top, ref mut bot) = self
                                                            .rocheckboxes
                                                            .entry(partid)
                                                            .or_insert((false, false));
                                                        ui.horizontal_wrapped(|ui| {
                                                            if part.tag.trim().is_empty() {
                                                                ui.checkbox(top, i18n!("classes-decoupler-top"));
                                                                ui.checkbox(
                                                                    bot,
                                                                    i18n_args!("classes-decoupler-bottom", "part", &part.title),
                                                                );
                                                            } else {
                                                                ui.checkbox(top, i18n!("classes-decoupler-top"));
                                                                ui.checkbox(
                                                                    bot,
                                                                    i18n_args!("classes-decoupler-bottom-tag", "part", &part.title, "tag", &part.tag),
                                                                );
                                                            }
                                                        });
                                                    }
                                                    None => {}
                                                }
                                            }

                                            let modal = Modal::new(ctx, "SeparationModal");

                                            modal.show(|ui| {
                                                self.modal(ctx, ui, &modal, class_id, backend, mission);
                                            });

                                            if ui.button(i18n!("classes-calcsep")).clicked() {
                                                let parts = self
                                                    .checkboxes
                                                    .iter()
                                                    .filter_map(|(id, checked)| {
                                                        checked.then_some(*id)
                                                    })
                                                    .collect::<Vec<_>>();
                                                let subvessels = vessel::decoupled_vessels(
                                                    &mission.classes[class_id],
                                                    &parts,
                                                    &self.rocheckboxes,
                                                );

                                                self.subvessel_options =
                                                    std::iter::repeat(SubvesselOption::Keep)
                                                        .take(subvessels.len())
                                                        .collect();
                                                self.subvessel_names = subvessels
                                                    .iter()
                                                    .enumerate()
                                                    .map(|(i, _)| {
                                                        i18n_args!(
							    "classes-calcsep-default-name",
							    "class",
							    &mission.classes[class_id].name,
							    "n",
							    i + 1
							)
                                                    })
                                                    .collect();
                                                self.subvessels = subvessels;

                                                modal.open();
                                                self.checkboxes.clear();
                                                self.rocheckboxes.clear();
                                            }

                                            ui.heading(i18n!("classes-engines"));

                                        } else {
                                            ui.heading(i18n!("classes-no-class"));
                                        }
                                    });
                            });
                        });
                    });
            });
    }

    fn handle_rx(
        &mut self,
        res: Result<HRes, eyre::Error>,
        _mission: &Mission,
        _toasts: &mut Toasts,
        backend: &mut Backend,
        _ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()> {
        self.loading = false;
        if let Ok(HRes::LoadedVesselClass(parts, root, tracked_id_map)) = res {
            if let Some(class_id) = self.current_class {
                backend.effect(move |mission, _| {
                    let class = &mut mission.classes[class_id];
                    class.parts = parts;
                    class.root = root;
                    class.tracked_id_map = tracked_id_map;
                    Ok(())
                });
            }
            Ok(())
        } else {
            res.map(|_| ())
        }
    }
}

#[derive(Debug)]
#[allow(clippy::struct_excessive_bools)]
pub struct Vessels {
    ui_id: egui::Id,
    search: String,
    current_vessel: Option<VesselId>,
    renaming: bool,
    just_clicked_rename: bool,
    vessels_filtered: Vec<VesselId>,
    pub force_refilter: bool,
    loading: u64,
    in_game_vessels: Vec<(String, krpc::Vessel)>,
    get_base: Option<UTorGET>,
    get_base_unparsed: String,
    get_base_disp: TimeDisplayKind,
    editing_get_base: bool,
    just_clicked_edit_get_base: bool,
}

impl Default for Vessels {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
            search: String::new(),
            current_vessel: None,
            renaming: false,
            just_clicked_rename: false,
            vessels_filtered: vec![],
            force_refilter: false,
            loading: 0,
            in_game_vessels: vec![],
            get_base: None,
            get_base_unparsed: String::new(),
            get_base_disp: TimeDisplayKind::Dhms,
            editing_get_base: false,
            just_clicked_edit_get_base: false,
        }
    }
}

impl Vessels {
    fn refilter(&mut self, mission: &Mission) {
        self.vessels_filtered = mission
            .vessels
            .iter()
            .sorted_by_key(|(_, x)| x.name.clone())
            .filter_map(|(id, x)| {
                (self.search.is_empty() || x.name.trim().starts_with(self.search.trim()))
                    .then_some(id)
            })
            .collect();
    }

    fn search_box(&mut self, ui: &mut egui::Ui, mission: &Mission, backend: &mut Backend) {
        egui::Frame::group(ui.style()).show(ui, |ui| {
            egui::ScrollArea::vertical()
                .id_salt(self.ui_id.with("Vessels"))
                .auto_shrink(false)
                .show(ui, |ui| {
                    let search = egui::TextEdit::singleline(&mut self.search)
                        .hint_text(i18n!("vessels-searchbox-hint"))
                        .frame(true)
                        .show(ui);

                    if search.response.changed() {
                        self.refilter(mission);
                    }

                    if self.force_refilter {
                        self.refilter(mission);
                    }

                    let already_exists = mission
                        .vessels
                        .iter()
                        .find_map(|(id, x)| (x.name.trim() == self.search.trim()).then_some(id));

                    if already_exists.is_none()
                        && (search.response.lost_focus()
                            && ui.input(|i| i.key_pressed(egui::Key::Enter))
                            && !self.search.trim().is_empty())
                        || (already_exists.is_none()
                            && !self.search.trim().is_empty()
                            && ui
                                .button(i18n_args!("vessels-create", "vessel", &self.search))
                                .clicked())
                    {
                        let vessel = Vessel {
                            name: self.search.take(),
                            ..Vessel::default()
                        };
                        backend.effect(move |mission, state| {
                            let vessel_id = mission.vessels.push(vessel);
                            state.vessels.current_vessel = Some(vessel_id);
                            state.vessels.refilter(mission);
                            Ok(())
                        });

                        self.search.clear();
                    }

                    if let Some(vessel) = already_exists {
                        if search.response.lost_focus()
                            && ui.input(|i| i.key_pressed(egui::Key::Enter))
                            && !self.search.trim().is_empty()
                        {
                            self.current_vessel = Some(vessel);
                        }
                    }

                    for vessel in &self.vessels_filtered {
                        let checked = self.current_vessel.as_ref().is_some_and(|x| vessel == x);
                        if ui
                            .selectable_label(checked, mission.vessels[*vessel].name.trim())
                            .clicked()
                        {
                            self.current_vessel = Some(*vessel);
                        };
                    }
                });
        });
    }
}

impl KtkDisplay for Vessels {
    fn show(
        &mut self,
        mission: &Mission,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    ) {
        egui::Window::new(i18n!("vessels-title"))
            .open(&mut open.vessels)
            .default_size([416.0, 512.0])
            .show(ctx, |ui| {
                GridBuilder::new()
                    .new_row(Size::initial(384.0))
                    .cell(Size::relative(1.0 / 3.0))
                    .cell(Size::remainder())
                    .show(ui, |mut grid| {
                        grid.cell(|ui| {
                            self.search_box(ui, mission, backend);
                        });
                        grid.cell(|ui| {
                            egui::Frame::none().inner_margin(6.0).show(ui, |ui| {
                                egui::ScrollArea::vertical()
                                    .id_salt(self.ui_id.with("Parts"))
                                    .show(ui, |ui| {
                                        if let Some(vessel_id) = self.current_vessel {
                                            if self.renaming {
                                                let mut name =
                                                    mission.vessels[vessel_id].name.clone();
                                                let text = egui::TextEdit::singleline(&mut name)
                                                    .font(egui::TextStyle::Heading)
                                                    .show(ui);

                                                if text.response.changed() {
                                                    self.force_refilter = true;
                                                    backend.effect(move |mission, _| {
                                                        mission.vessels[vessel_id].name = name;
                                                        Ok(())
                                                    });
                                                }

                                                if self.just_clicked_rename {
                                                    self.just_clicked_rename = false;
                                                    text.response.request_focus();
                                                }

                                                if text.response.lost_focus()
                                                    && ui.input(|i| i.key_pressed(egui::Key::Enter))
                                                {
                                                    self.renaming = false;
                                                }
                                            } else if !self.renaming {
                                                ui.heading(mission.vessels[vessel_id].name.trim());
                                            }

                                            ui.horizontal(|ui| {
                                                if self.renaming
                                                    && ui
                                                        .button(icon("\u{e161}"))
                                                        .on_hover_text(i18n!("vessels-save"))
                                                        .clicked()
                                                {
                                                    self.renaming = false;
                                                } else if !self.renaming
                                                    && ui
                                                        .button(icon("\u{e3c9}"))
                                                        .on_hover_text(i18n!("vessels-rename"))
                                                        .clicked()
                                                {
                                                    self.renaming = true;
                                                    self.just_clicked_rename = true;
                                                    self.force_refilter = true;
                                                }

                                                // TODO: confirm delete
                                                if ui
                                                    .button(icon("\u{e872}"))
                                                    .on_hover_text(i18n!("vessels-delete"))
                                                    .clicked()
                                                {
                                                    self.renaming = false;
                                                    let pos = self
                                                        .vessels_filtered
                                                        .iter()
                                                        .enumerate()
                                                        .find_map(|(i, x)| {
                                                            (*x == vessel_id).then_some(i)
                                                        })
                                                        .expect("oops");
                                                    backend.effect(move |mission, _| {
                                                        mission
                                                            .vessels
                                                            .retain(|i, _| vessel_id != i);
                                                        Ok(())
                                                    });
                                                    self.vessels_filtered.remove(pos);
                                                    self.current_vessel =
                                                        self.vessels_filtered.get(pos).copied().or(
                                                            self.vessels_filtered
                                                                .get(pos.saturating_sub(1))
                                                                .copied(),
                                                        );
                                                    self.force_refilter = true;
                                                }
                                            });

                                            ui.label(i18n!("vessels-description"));
                                            let mut description =
                                                mission.vessels[vessel_id].description.clone();
                                            if egui::TextEdit::multiline(&mut description)
                                                .show(ui)
                                                .response
                                                .changed()
                                            {
                                                backend.effect(move |mission, _| {
                                                    mission.vessels[vessel_id].description =
                                                        description;
                                                    Ok(())
                                                });
                                            };
                                            ui.horizontal(|ui| {
                                                ui.label(i18n!("vessels-class"));
                                                {
                                                    let mut class =
                                                        mission.vessels[vessel_id].class;
                                                    egui::ComboBox::from_id_salt(
                                                        self.ui_id.with("Class"),
                                                    )
                                                    .selected_text(
                                                        class
                                                            .map(|x| {
                                                                mission.classes[x].name.clone()
                                                            })
                                                            .unwrap_or(i18n!("vessels-no-class")),
                                                    )
                                                    .show_ui(ui, |ui| {
                                                        for (id, iter_class) in
                                                            mission.classes.iter()
                                                        {
                                                            if ui
                                                                .selectable_value(
                                                                    &mut class,
                                                                    Some(id),
                                                                    &iter_class.name,
                                                                )
                                                                .clicked()
                                                            {
                                                                backend.effect(
                                                                    move |mission, _| {
                                                                        mission.vessels
                                                                            [vessel_id]
                                                                            .class = Some(id);
                                                                        Ok(())
                                                                    },
                                                                );
                                                            };
                                                        }
                                                    });
                                                }
                                            });

                                            ui.vertical(|ui| {
                                                // TODO: instead of this mess, try and do something like DragValue on click
                                                ui.label(i18n!("vessels-get-base"));
                                                ui.horizontal(|ui| {
                                                    if !self.editing_get_base {
                                                        self.get_base = Some(UTorGET::UT(
                                                            mission.vessels[vessel_id].get_base,
                                                        ));
                                                    }
                                                    let res = ui.add(TimeInput1::new(
                                                        &mut self.get_base_unparsed,
                                                        &mut self.get_base,
                                                        Some(128.0),
                                                        TimeInputKind2::UT,
                                                        self.get_base_disp,
                                                        self.editing_get_base,
                                                        false,
                                                    ));
                                                    if self.just_clicked_edit_get_base {
                                                        res.request_focus();
                                                        self.just_clicked_edit_get_base = false;
                                                    }
                                                    if res.lost_focus() {
                                                        if let Some(UTorGET::UT(ut)) = self.get_base
                                                        {
                                                            backend.effect(move |mission, _| {
                                                                mission.vessels[vessel_id]
                                                                    .get_base = ut;
                                                                Ok(())
                                                            });
                                                        }
                                                        if ui.input(|i| {
                                                            i.key_pressed(egui::Key::Enter)
                                                        }) {
                                                            self.editing_get_base = false;
                                                        }
                                                    }
                                                    ui.add(TimeDisplayBtn(&mut self.get_base_disp));

                                                    if self.editing_get_base
                                                        && ui
                                                            .button(icon("\u{e161}"))
                                                            .on_hover_text(i18n!("vessels-save"))
                                                            .clicked()
                                                    {
                                                        self.editing_get_base = false;
                                                    } else if !self.editing_get_base
                                                        && ui
                                                            .button(icon("\u{e3c9}"))
                                                            .on_hover_text(i18n!(
                                                                "vessels-edit-get-base"
                                                            ))
                                                            .clicked()
                                                    {
                                                        self.editing_get_base = true;
                                                        self.just_clicked_edit_get_base = true;
                                                    }
                                                });
                                            });

                                            ui.heading(i18n!("vessels-link"));
                                            ui.label(i18n!("vessels-link-explainer"));

                                            ui.collapsing(i18n!("vessels-link-utilities"), |ui| {
                                                if self.loading == 2 {
                                                    ui.spinner();
                                                }
                                                if ui
                                                    .button(i18n!("vessels-link-getbase"))
                                                    .clicked()
                                                {
                                                    handle(toasts, |_| {
                                                        backend.tx(
                                                            DisplaySelect::Vessels,
                                                            HReq::LoadVesselGETBase(
                                                                mission.vessels[vessel_id]
                                                                    .link
                                                                    .ok_or_eyre(i18n!(
                                                                        "vessels-error-no-link"
                                                                    ))?,
                                                            ),
                                                        )?;
                                                        self.loading = 2;
                                                        Ok(())
                                                    });
                                                }

                                                if ui
                                                    .button(i18n!("vessels-link-resources"))
                                                    .clicked()
                                                {
                                                    handle(toasts, |_| {
                                                        backend.tx(
                                                            DisplaySelect::Vessels,
                                                            HReq::LoadVesselResources(
                                                                mission.vessels[vessel_id]
                                                                    .link
                                                                    .ok_or_eyre(i18n!(
                                                                        "vessels-error-no-link"
                                                                    ))?,
                                                                vessel_id,
                                                                mission.vessels[vessel_id]
                                                                    .class
                                                                    .ok_or_eyre(i18n!(
                                                                        "vessels-error-no-class"
                                                                    ))?,
                                                            ),
                                                        )?;
                                                        self.loading = 2;
                                                        Ok(())
                                                    });
                                                }
                                            });

                                            ui.collapsing(i18n!("vessels-resources"), |ui| {
                                                ui.monospace(format!(
                                                    "{:#?}",
                                                    mission.vessels[vessel_id].resources
                                                ));
                                            });

                                            ui.horizontal(|ui| {
                                                if ui
                                                    .button(icon_label(
                                                        "\u{e5d5}",
                                                        &i18n!("vessels-refresh-list"),
                                                    ))
                                                    .clicked()
                                                {
                                                    handle(toasts, |_| {
                                                        backend.tx(
                                                            DisplaySelect::Vessels,
                                                            HReq::LoadVesselsList,
                                                        )?;
                                                        self.loading = 1;
                                                        Ok(())
                                                    });
                                                }
                                                if self.loading == 1 {
                                                    ui.spinner();
                                                }
                                            });

                                            let mut link = mission.vessels[vessel_id].link;
                                            for (name, in_game_vessel) in
                                                self.in_game_vessels.iter().map(|(n, x)| (n, *x))
                                            {
                                                if ui
                                                    .selectable_value(
                                                        &mut link,
                                                        Some(in_game_vessel),
                                                        name,
                                                    )
                                                    .clicked()
                                                {
                                                    handle(toasts, |_| {
                                                        backend.tx(
                                                            DisplaySelect::Vessels,
                                                            HReq::TrackVessel(
                                                                in_game_vessel,
                                                                vessel_id,
                                                            ),
                                                        )?;
                                                        self.loading = 1;
                                                        backend.effect(move |mission, _| {
                                                            mission.vessels[vessel_id].link =
                                                                Some(in_game_vessel);
                                                            Ok(())
                                                        });
                                                        Ok(())
                                                    });
                                                };
                                            }
                                        } else {
                                            ui.heading(i18n!("vessels-no-vessel"));
                                        }
                                    });
                            });
                        });
                    });
            });
    }

    fn handle_rx(
        &mut self,
        res: Result<HRes, eyre::Error>,
        _mission: &Mission,
        _toasts: &mut Toasts,
        backend: &mut Backend,
        _ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()> {
        self.loading = 0;
        if let Ok(HRes::LoadedVesselsList(list)) = res {
            self.in_game_vessels = list
                .into_iter()
                .sorted_by_key(|x| x.0.clone())
                .collect::<Vec<_>>();
            Ok(())
        } else if let Ok(HRes::LoadedGETBase(ut)) = res {
            if let Some(vessel_id) = self.current_vessel {
                backend.effect(move |mission, _| {
                    mission.vessels[vessel_id].get_base = ut;
                    Ok(())
                });
                self.get_base = Some(UTorGET::UT(ut));
                self.editing_get_base = false;
                self.just_clicked_edit_get_base = false;
            }
            Ok(())
        } else if let Ok(HRes::LoadedVesselResources(resources)) = res {
            if let Some(vessel_id) = self.current_vessel {
                backend.effect(move |mission, _| {
                    mission.vessels[vessel_id].resources = resources;
                    Ok(())
                });
            }
            Ok(())
        } else {
            res.map(|_| ())
        }
    }
}
