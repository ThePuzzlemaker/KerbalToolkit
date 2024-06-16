use std::{
    collections::{HashMap, HashSet},
    fmt,
    sync::Arc,
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
    ffs::{Conditions, FuelFlowSimulation, FuelStats, SimPart, SimVessel},
    krpc,
    vessel::{self, Decouplers, PartId, Vessel, VesselClass, VesselClassRef},
};
use nalgebra::Vector3;
use parking_lot::RwLock;

use crate::{
    backend::{HReq, HRes},
    handle, i18n, i18n_args, icon, icon_label,
    mission::Mission,
    Backend, DisplaySelect, KtkDisplay, TimeInput, UTorGET,
};

#[derive(Debug)]
pub struct Classes {
    ui_id: egui::Id,
    search: String,
    current_class: Option<Arc<RwLock<VesselClass>>>,
    renaming: bool,
    just_clicked_rename: bool,
    classes_filtered: Vec<Arc<RwLock<VesselClass>>>,
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
            search: "".into(),
            current_class: None,
            renaming: false,
            just_clicked_rename: false,
            classes_filtered: vec![],
            force_refilter: false,
            loading: false,
            checkboxes: Default::default(),
            fairings: Default::default(),
            rocheckboxes: Default::default(),
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
            SubvesselOption::Keep => write!(f, "Keep"),
            SubvesselOption::Discard => write!(f, "Discard"),
        }
    }
}

impl Classes {
    fn refilter(&mut self, mission: &Arc<RwLock<Mission>>) {
        self.classes_filtered = mission
            .read()
            .classes
            .iter()
            .sorted_by_key(|x| x.read().name.clone())
            .filter(|x| {
                self.search.is_empty() || x.read().name.trim().starts_with(self.search.trim())
            })
            .cloned()
            .collect();
    }

    fn search_box(&mut self, ui: &mut egui::Ui, mission: &Arc<RwLock<Mission>>) {
        egui::Frame::group(ui.style()).show(ui, |ui| {
            egui::ScrollArea::vertical()
                .id_source(self.ui_id.with("Classes"))
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
                        .read()
                        .classes
                        .iter()
                        .find(|x| x.read().name.trim() == self.search.trim())
                        .cloned();

                    if already_exists.is_none()
                        && (search.response.lost_focus()
                            && ui.input(|i| i.key_pressed(egui::Key::Enter))
                            && !self.search.trim().is_empty())
                        || (already_exists.is_none()
                            && !self.search.trim().is_empty()
                            && ui
                                .button(i18n_args!("classes-create", "class" => self.search.trim()))
                                .clicked())
                    {
                        let class = Arc::new(RwLock::new(VesselClass {
                            name: self.search.take(),
                            ..VesselClass::default()
                        }));
                        mission.write().classes.push(class.clone());
                        self.current_class = Some(class);
                        self.search.clear();

                        self.refilter(mission);
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
                        let checked = self
                            .current_class
                            .as_ref()
                            .map(|x| Arc::ptr_eq(class, x))
                            .unwrap_or_default();
                        if ui
                            .selectable_label(checked, class.read().name.trim())
                            .clicked()
                        {
                            self.checkboxes.clear();
                            self.rocheckboxes.clear();
                            self.current_class = Some(class.clone());
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
        class: &mut VesselClass,
        mission: &Arc<RwLock<Mission>>,
    ) {
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
                        egui::RichText::new(i18n_args!("classes-subvessel", "n" => ix + 1))
                            .strong(),
                    );
                    egui::ComboBox::from_id_source(self.ui_id.with("SubvesselComboBox").with(ix))
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
                                (None, None) => i18n_args!("classes-part", "part" => title),
                                (None, Some(tag)) => i18n_args!("classes-part-tag", "part" => title, "tag" => tag),
                                (Some(true), None) => i18n_args!("classes-part-staged-fairing", "part" => title),
				(Some(false), None) => i18n_args!("classes-part-unstaged-fairing", "part" => title),
                                (Some(true), Some(tag)) => i18n_args!("classes-part-staged-fairing-tag", "part" => title, "tag" => tag),
				(Some(false), Some(tag)) => i18n_args!("classes-part-unstaged-fairing-tag", "part" => title, "tag" => tag),
                            };

                            ui.label(egui::RichText::new(format!(" ▪ {}", text)));
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
                                .map(|parent| !subvessel.contains(&parent))
                                .unwrap_or_default()
                            {
                                part.parent = None;
                            }
                            part.children.retain(|x| subvessel.contains(x));

                            match part.decouplers.as_mut() {
                                Some(Decouplers::Single(decoupler)) => {
                                    if decoupler
                                        .attached_part
                                        .map(|part| !subvessel.contains(&part))
                                        .unwrap_or_default()
                                    {
                                        decoupler.attached_part = None
                                    };
                                }
                                Some(Decouplers::RODecoupler { top, bot }) => {
                                    if top
                                        .attached_part
                                        .map(|part| !subvessel.contains(&part))
                                        .unwrap_or_default()
                                    {
                                        top.attached_part = None
                                    };
                                    if bot
                                        .attached_part
                                        .map(|part| !subvessel.contains(&part))
                                        .unwrap_or_default()
                                    {
                                        bot.attached_part = None
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
			    persistent_id_map.insert(part.persistent_id, id);
			}

                        if option == SubvesselOption::Keep {
                            let vessel = Arc::new(RwLock::new(VesselClass {
                                name: name.clone(),
                                description: String::new(),
                                shortcode: String::new(),
                                parts,
                                root,
				persistent_id_map,
                            }));
                            mission.write().classes.push(vessel);
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
        mission: &Arc<RwLock<Mission>>,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut bool,
    ) {
        egui::Window::new(i18n!("classes-title"))
            .open(open)
            .default_size([384.0, 512.0])
            .show(ctx, |ui| {
                GridBuilder::new()
                    .new_row(Size::initial(384.0))
                    .cell(Size::relative(1.0 / 3.0))
                    .cell(Size::remainder())
                    .show(ui, |mut grid| {
                        grid.cell(|ui| {
                            self.search_box(ui, mission);
                        });
                        grid.cell(|ui| {
                            egui::Frame::none().inner_margin(6.0).show(ui, |ui| {
                                egui::ScrollArea::vertical()
                                    .id_source(self.ui_id.with("Parts"))
                                    .show(ui, |ui| {
                                        if let Some(class_rc) = self.current_class.clone() {
                                            let mut class = class_rc.write();
                                            if self.renaming {
                                                let text =
                                                    egui::TextEdit::singleline(&mut class.name)
                                                        .font(egui::TextStyle::Heading)
                                                        .show(ui);

                                                if text.response.changed() {
                                                    self.force_refilter = true;
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
                                                ui.heading(class.name.trim());
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
                                                            Arc::ptr_eq(&class_rc, x).then_some(i)
                                                        })
                                                        .expect("oops");
                                                    mission
                                                        .write()
                                                        .classes
                                                        .retain(|x| !Arc::ptr_eq(&class_rc, x));
                                                    self.classes_filtered.remove(pos);
                                                    self.current_class =
                                                        self.classes_filtered.get(pos).cloned().or(
                                                            self.classes_filtered
                                                                .get(pos.saturating_sub(1))
                                                                .cloned(),
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
                                            egui::TextEdit::multiline(&mut class.description)
                                                .show(ui);
                                            ui.horizontal(|ui| {
                                                ui.label(i18n!("classes-shortcode"));
                                                let shortcode = egui::TextEdit::singleline(
                                                    &mut class.shortcode,
                                                )
                                                .char_limit(5)
                                                .show(ui);
                                                shortcode.response.on_hover_ui(|ui| {
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
                                            for (partid, part) in class
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
                                                        && x.current_mass == x.unstaged_mass
                                                }) {
                                                    if part.tag.trim().is_empty() {
                                                        ui.checkbox(
                                                            self.fairings
                                                                .entry(partid)
                                                                .or_insert(false),
                                                            i18n_args!("classes-decoupler-fairing", "part" => &part.title)
                                                        );
                                                    } else {
                                                        ui.checkbox(
                                                            self.fairings
                                                                .entry(partid)
                                                                .or_insert(false),
                                                            i18n_args!("classes-decoupler-fairing-tag", "part" => &part.title, "tag" => &part.tag)
                                                        );
                                                    }
                                                }

                                                match part.decouplers.as_ref() {
                                                    Some(Decouplers::Single(_))
                                                    | Some(Decouplers::ProceduralFairing(_)) => {
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
                                                                i18n_args!("classes-decoupler-tag", "part" => &part.title, "tag" => &part.tag)
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
                                                                    i18n_args!("classes-decoupler-bottom", "part" => &part.title),
                                                                );
                                                            } else {
                                                                ui.checkbox(top, i18n!("classes-decoupler-top"));
                                                                ui.checkbox(
                                                                    bot,
                                                                    i18n_args!("classes-decoupler-bottom-tag", "part" => &part.title, "tag" => &part.tag),
                                                                );
                                                            }
                                                        });
                                                    }
                                                    None => {}
                                                }
                                            }

                                            let modal = Modal::new(ctx, "SeparationModal");

                                            modal.show(|ui| {
                                                self.modal(ctx, ui, &modal, &mut class, mission)
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
                                                    &class,
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
                                                        i18n_args!("classes-calcsep-default-name", "class" => &class.name, "n" => i + 1)
                                                    })
                                                    .collect();
                                                self.subvessels = subvessels;

                                                modal.open();
                                                self.checkboxes.clear();
                                                self.rocheckboxes.clear();
                                            }

                                            ui.heading(i18n!("classes-engines"));

                                            let mut ffs = FuelFlowSimulation {
                                                segments: vec![],
                                                current_segment: FuelStats::default(),
                                                time: 0.0,
                                                dv_linear_thrust: false,
                                                parts_with_resource_drains: HashSet::new(),
                                                sources: vec![],
                                            };

                                            let mut vessel = SimVessel {
                                                parts: Arena::new(),
                                                active_engines: vec![],
                                                mass: 0.0,
                                                thrust_current: Vector3::zeros(),
                                                thrust_magnitude: 0.0,
                                                thrust_no_cos_loss: 0.0,
                                                spoolup_current: 0.0,
                                                conditions: Conditions {
                                                    atm_pressure: 0.0,
                                                    atm_density: 0.0,
                                                    mach_number: 0.0,
                                                    main_throttle: 1.0,
                                                },
                                            };

                                            // TODO: merge SimPart and Part for simplicity?
                                            let mut map = HashMap::new();
                                            for (pid, part) in class.parts.iter() {
                                                let mut modules_current_mass = 0.0;
                                                if !part.mass_modifiers.is_empty() {
                                                    ui.label(&part.name);
                                                    for modifier in &part.mass_modifiers {
                                                        ui.monospace(format!("{:#?}", modifier));
                                                        modules_current_mass +=
                                                            modifier.current_mass;
                                                    }
                                                }
                                                ui.add_space(1.0);

                                                let sid = vessel.parts.push(SimPart {
                                                    crossfeed_part_set: vec![],
                                                    resources: part.resources.clone(),
                                                    resource_drains: HashMap::new(),
                                                    resource_priority: part.resource_priority,
                                                    resource_request_remaining_threshold: part
                                                        .resource_request_remaining_threshold,
                                                    mass: part.mass,
                                                    dry_mass: part.dry_mass,
                                                    crew_mass: part.crew_mass,
                                                    modules_current_mass,
                                                    disabled_resource_mass: part
                                                        .disabled_resource_mass,
                                                    is_launch_clamp: part.is_launch_clamp,
                                                });
                                                vessel.active_engines.extend(
                                                    part.engines.iter().cloned().map(|mut x| {
                                                        x.part = sid;
                                                        x
                                                    }),
                                                );
                                                map.insert(pid, sid);
                                            }
                                            for (pid, sid) in &map {
                                                vessel.parts[*sid].crossfeed_part_set = class.parts
                                                    [*pid]
                                                    .crossfeed_part_set
                                                    .iter()
                                                    .flat_map(|x| map.get(x).copied())
                                                    .collect();
                                            }

                                            ffs.run(&mut vessel);

                                            ui.monospace(format!("{:#?}", ffs));
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
        _mission: &Arc<RwLock<Mission>>,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
        _ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()> {
        self.loading = false;
        if let Ok(HRes::LoadedVesselClass(parts, root, persistent_id_map)) = res {
            if let Some(class) = self.current_class.as_ref() {
                let mut class = class.write();
                class.parts = parts;
                class.root = root;
                class.persistent_id_map = persistent_id_map;
            }
            Ok(())
        } else {
            res.map(|_| ())
        }
    }
}

#[derive(Debug)]
pub struct Vessels {
    ui_id: egui::Id,
    search: String,
    current_vessel: Option<Arc<RwLock<Vessel>>>,
    renaming: bool,
    just_clicked_rename: bool,
    vessels_filtered: Vec<Arc<RwLock<Vessel>>>,
    pub force_refilter: bool,
    loading: u64,
    in_game_vessels: Vec<(String, krpc::Vessel)>,
    get_base_input: TimeInput,
    get_base: Option<UTorGET>,
    get_base_unparsed: String,
}

impl Default for Vessels {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
            search: "".into(),
            current_vessel: None,
            renaming: false,
            just_clicked_rename: false,
            vessels_filtered: vec![],
            force_refilter: false,
            loading: 0,
            in_game_vessels: vec![],
            get_base_input: TimeInput::UTDHMS,
            get_base: None,
            get_base_unparsed: "".into(),
        }
    }
}

impl Vessels {
    fn refilter(&mut self, mission: &Arc<RwLock<Mission>>) {
        self.vessels_filtered = mission
            .read()
            .vessels
            .iter()
            .sorted_by_key(|x| x.read().name.clone())
            .filter(|x| {
                self.search.is_empty() || x.read().name.trim().starts_with(self.search.trim())
            })
            .cloned()
            .collect();
    }

    fn search_box(&mut self, ui: &mut egui::Ui, mission: &Arc<RwLock<Mission>>) {
        egui::Frame::group(ui.style()).show(ui, |ui| {
            egui::ScrollArea::vertical()
                .id_source(self.ui_id.with("Vessels"))
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
                        .read()
                        .vessels
                        .iter()
                        .find(|x| x.read().name.trim() == self.search.trim())
                        .cloned();

                    if already_exists.is_none()
                        && (search.response.lost_focus()
                            && ui.input(|i| i.key_pressed(egui::Key::Enter))
                            && !self.search.trim().is_empty())
                        || (already_exists.is_none()
                            && !self.search.trim().is_empty()
                            && ui
                                .button(i18n_args!("vessels-create", "vessel" => &self.search))
                                .clicked())
                    {
                        let vessel = Arc::new(RwLock::new(Vessel {
                            name: self.search.take(),
                            ..Vessel::default()
                        }));
                        mission.write().vessels.push(vessel.clone());
                        self.current_vessel = Some(vessel);
                        self.search.clear();

                        self.refilter(mission);
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
                        let checked = self
                            .current_vessel
                            .as_ref()
                            .map(|x| Arc::ptr_eq(vessel, x))
                            .unwrap_or_default();
                        if ui
                            .selectable_label(checked, vessel.read().name.trim())
                            .clicked()
                        {
                            self.current_vessel = Some(vessel.clone());
                        };
                    }
                });
        });
    }
}

impl KtkDisplay for Vessels {
    fn show(
        &mut self,
        mission: &Arc<RwLock<Mission>>,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut bool,
    ) {
        egui::Window::new(i18n!("vessels-title"))
            .open(open)
            .default_size([416.0, 512.0])
            .show(ctx, |ui| {
                GridBuilder::new()
                    .new_row(Size::initial(384.0))
                    .cell(Size::relative(1.0 / 3.0))
                    .cell(Size::remainder())
                    .show(ui, |mut grid| {
                        grid.cell(|ui| {
                            self.search_box(ui, mission);
                        });
                        grid.cell(|ui| {
                            egui::Frame::none().inner_margin(6.0).show(ui, |ui| {
                                egui::ScrollArea::vertical()
                                    .id_source(self.ui_id.with("Parts"))
                                    .show(ui, |ui| {
                                        if let Some(vessel_rc) = self.current_vessel.clone() {
                                            if self.renaming {
                                                let text = egui::TextEdit::singleline(
                                                    &mut vessel_rc.write().name,
                                                )
                                                .font(egui::TextStyle::Heading)
                                                .show(ui);

                                                if text.response.changed() {
                                                    self.force_refilter = true;
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
                                                ui.heading(vessel_rc.read().name.trim());
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
                                                            Arc::ptr_eq(&vessel_rc, x).then_some(i)
                                                        })
                                                        .expect("oops");
                                                    mission
                                                        .write()
                                                        .vessels
                                                        .retain(|x| !Arc::ptr_eq(&vessel_rc, x));
                                                    self.vessels_filtered.remove(pos);
                                                    self.current_vessel =
                                                        self.vessels_filtered.get(pos).cloned().or(
                                                            self.vessels_filtered
                                                                .get(pos.saturating_sub(1))
                                                                .cloned(),
                                                        );
                                                    self.force_refilter = true;
                                                }
                                            });

                                            ui.label(i18n!("vessels-description"));
                                            egui::TextEdit::multiline(
                                                &mut vessel_rc.write().description,
                                            )
                                            .show(ui);
                                            ui.horizontal(|ui| {
                                                ui.label(i18n!("vessels-class"));
                                                {
                                                    let mut vessel_rc = vessel_rc.write();
                                                    egui::ComboBox::from_id_source(
                                                        self.ui_id.with("Class"),
                                                    )
                                                    .selected_text(
                                                        vessel_rc
                                                            .class
                                                            .clone()
                                                            .map(|x| x.0.read().name.to_string())
                                                            .unwrap_or(i18n!("vessels-no-class")),
                                                    )
                                                    .show_ui(ui, |ui| {
                                                        for class in &mission.read().classes {
                                                            ui.selectable_value(
                                                                &mut vessel_rc.class,
                                                                Some(VesselClassRef(class.clone())),
                                                                &class.read().name,
                                                            );
                                                        }
                                                    });
                                                }
                                            });

                                            ui.vertical(|ui| {
                                                // TODO: maybe use a system similar to the rename on vessels/classes?
                                                // that way this is easier to sync
                                                // ultimately though, I need to separate all these time selectors into their own widget
                                                ui.label(i18n!("vessels-get-base"));
                                                ui.horizontal(|ui| {
                                                    egui::ComboBox::from_id_source(
                                                        self.ui_id.with("GETBase"),
                                                    )
                                                    .selected_text(format!(
                                                        "{}",
                                                        self.get_base_input
                                                    ))
                                                    .wrap(false)
                                                    .show_ui(ui, |ui| {
                                                        if ui
                                                            .selectable_value(
                                                                &mut self.get_base_input,
                                                                TimeInput::UTSeconds,
                                                                TimeInput::UTSeconds.to_string(),
                                                            )
                                                            .clicked()
                                                        {
                                                            if let Some(UTorGET::UT(get_base)) =
                                                                self.get_base
                                                            {
                                                                self.get_base_unparsed =
                                                                    TimeInput::UTSeconds
                                                                        .format(get_base, None);
                                                            }
                                                        };
                                                        if ui
                                                            .selectable_value(
                                                                &mut self.get_base_input,
                                                                TimeInput::UTDHMS,
                                                                TimeInput::UTDHMS.to_string(),
                                                            )
                                                            .clicked()
                                                        {
                                                            if let Some(UTorGET::UT(get_base)) =
                                                                self.get_base
                                                            {
                                                                self.get_base_unparsed =
                                                                    TimeInput::UTDHMS
                                                                        .format(get_base, None);
                                                            }
                                                        };
                                                    });
                                                    if self.get_base.is_none()
                                                        && !self.get_base_unparsed.trim().is_empty()
                                                    {
                                                        ui.visuals_mut().selection.stroke.color =
                                                            egui::Color32::from_rgb(255, 0, 0);
                                                    }
                                                    if egui::TextEdit::singleline(
                                                        &mut self.get_base_unparsed,
                                                    )
                                                    .font(egui::TextStyle::Monospace)
                                                    .desired_width(128.0)
                                                    .show(ui)
                                                    .response
                                                    .changed()
                                                    {
                                                        self.get_base = self
                                                            .get_base_input
                                                            .parse(self.get_base_unparsed.trim());
                                                        if let Some(UTorGET::UT(ut)) = self.get_base
                                                        {
                                                            vessel_rc.write().get_base = ut;
                                                        }
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
                                                                vessel_rc.read().link.ok_or_eyre(
                                                                    i18n!("vessels-error-no-link"),
                                                                )?,
                                                            ),
                                                        )?;
                                                        self.loading = 2;
                                                        Ok(())
                                                    });
                                                }

                                                // TODO: fix the deadlock here
                                                if ui
                                                    .button(i18n!("vessels-link-resources"))
                                                    .clicked()
                                                {
                                                    handle(toasts, |_| {
                                                        backend.tx(
                                                            DisplaySelect::Vessels,
                                                            HReq::LoadVesselResources(
                                                                vessel_rc.read().link.ok_or_eyre(
                                                                    i18n!("vessels-error-no-link"),
                                                                )?,
                                                                vessel_rc
                                                                    .read()
                                                                    .class
                                                                    .clone()
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
                                                    vessel_rc.read().resources
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

                                            for (name, in_game_vessel) in &self.in_game_vessels {
                                                ui.selectable_value(
                                                    &mut vessel_rc.write().link,
                                                    Some(*in_game_vessel),
                                                    name,
                                                );
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
        _mission: &Arc<RwLock<Mission>>,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
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
            if let Some(vessel) = &self.current_vessel {
                vessel.write().get_base = ut;
                self.get_base = Some(UTorGET::UT(ut));
                self.get_base_unparsed = self.get_base_input.format(ut, None);
            }
            Ok(())
        } else if let Ok(HRes::LoadedVesselResources(resources)) = res {
            if let Some(vessel) = &self.current_vessel {
                vessel.write().resources = resources;
            }
            Ok(())
        } else {
            res.map(|_| ())
        }
    }
}
