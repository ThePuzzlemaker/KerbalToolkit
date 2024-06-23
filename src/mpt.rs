use std::{cmp::Ordering, collections::HashMap, time::Instant};

use crate::{
    backend::HRes, find_sv, find_sv_mpt, handle, i18n, i18n_args, Backend, DisplaySelect, Displays,
    KtkDisplay, UTorGET,
};
use color_eyre::eyre::{self, bail, OptionExt};
use egui_extras::Column;
use itertools::Itertools;
use nalgebra::Vector3;
//use tokio::runtime::{self, Runtime};

use crate::mission::{Mission, MissionPlan, PlannedManeuver};
use crate::widgets::{icon, DVInput, TimeDisplayBtn, TimeDisplayKind, TimeInput1, TimeInputKind2};
use egui_notify::Toasts;
use kerbtk::{
    arena::Arena,
    ffs::{self, Conditions, FuelFlowSimulation, Resource, ResourceId, SimPart, SimVessel},
    maneuver::{Maneuver, ManeuverKind},
    time::UT,
    vessel::{PartId, VesselClass, VesselId},
};

#[derive(Debug)]
pub struct MissionPlanTable {
    ui_id: egui::Id,
    vessel: Option<VesselId>,
    pub reinit: bool,
}

impl MissionPlanTable {
    fn run_ffs(
        mnv: PlannedManeuver,
        prev_resources: Option<HashMap<u32, HashMap<ResourceId, Resource>>>,
        class: &VesselClass,
        sim_vessel: &SimVessel,
    ) -> PlannedManeuver {
        let mut sim_vessel = sim_vessel.clone();

        if let Some(prev_resources) = prev_resources {
            for (pid, resources) in prev_resources {
                if let Some((_, part)) = sim_vessel
                    .parts
                    .iter_mut()
                    .find(|x| x.1.persistent_id == pid)
                {
                    part.resources = resources;
                }
            }
        }

        for (pid, part) in class.parts.iter() {
            if mnv.engines.contains(&pid) {
                let sid = sim_vessel
                    .parts
                    .iter()
                    .find_map(|(i, x)| (x.persistent_id == part.persistent_id).then_some(i))
                    .expect("oops");
                sim_vessel
                    .active_engines
                    .extend(part.engines.iter().cloned().map(|mut x| {
                        x.part = sid;
                        x
                    }));
            }
        }

        let mut ffs = FuelFlowSimulation::default();

        ffs.run(
            &mut sim_vessel,
            Some(mnv.inner.deltav.norm() * 1000.0),
            false,
        );

        let resources = sim_vessel
            .parts
            .iter()
            .map(|(_, x)| (x.persistent_id, x.resources.clone()))
            .collect::<HashMap<_, _>>();

        tracing::trace!("{ffs:#?}");
        ffs.run(&mut sim_vessel, None, true);
        tracing::trace!("{ffs:#?}");
        //ffs.run(&mut sim_vessel, None);

        let mut deltav = mnv.inner.deltav.norm() * 1000.0;
        let mut bt = 0.0;
        let start_mass_tons = ffs.segments[0].start_mass;
        let mut end_mass = ffs.segments[0].start_mass * 1000.0;
        for segment in &ffs.segments {
            match segment.deltav.total_cmp(&deltav) {
                Ordering::Less | Ordering::Equal => {
                    bt += segment.delta_time;
                    deltav -= segment.deltav;
                    end_mass = segment.end_mass * 1000.0;
                }
                Ordering::Greater => {
                    // Calculate end mass with Tsiolkovsky rocket equation

                    let start_mass = end_mass;
                    let exhvel = segment.isp * ffs::G0;
                    let alpha = libm::exp(-deltav / exhvel);
                    end_mass = start_mass * alpha;

                    bt += (start_mass * exhvel) / (1000.0 * segment.thrust) * (1.0 - alpha);
                    deltav = 0.0;
                    break;
                }
            }
        }

        let dvrem = if deltav > 1e-6 {
            -deltav
        } else {
            ffs.segments
                .iter()
                .fold(-mnv.inner.deltav.norm() * 1000.0, |acc, x| acc + x.deltav)
        };

        PlannedManeuver {
            inner: mnv.inner,
            engines: mnv.engines,
            dvrem,
            bt,
            start_mass: start_mass_tons,
            end_mass: end_mass / 1000.0,
            resources,
            source: mnv.source,
        }
    }
}

impl Default for MissionPlanTable {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
            vessel: None,
            reinit: false,
        }
    }
}

impl KtkDisplay for MissionPlanTable {
    fn show(
        &mut self,
        mission: &Mission,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    ) {
        egui::Window::new(i18n!("mpt-title"))
            .open(&mut open.mpt)
            .default_size([256.0, 256.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label(i18n!("mpt-vessel"));
                    egui::ComboBox::from_id_source(self.ui_id.with("VesselSelector"))
                        .selected_text(self.vessel.map_or_else(
                            || i18n!("vc-no-vessel"),
                            |x| mission.vessels[x].name.clone(),
                        ))
                        .show_ui(ui, |ui| {
                            for (id, iter_vessel) in
                                mission.vessels.iter().sorted_by_key(|(_, x)| &x.name)
                            {
                                ui.selectable_value(&mut self.vessel, Some(id), &iter_vessel.name);
                            }
                        });
                    'slot: {
                        let Some(vessel_id) = self.vessel else {
                            break 'slot;
                        };

                        let Some(plan) = mission.plan.get(&vessel_id) else {
                            backend.effect(move |mission, _| {
                                mission.plan.insert(vessel_id, MissionPlan::default());
                                Ok(())
                            });
                            break 'slot;
                        };

                        let mut av_slot = plan.anchor_vector_slot.clone();

                        ui.label(i18n!("mpt-av-slot"));
                        if egui::TextEdit::singleline(&mut av_slot)
                            .char_limit(16)
                            .desired_width(32.0)
                            .show(ui)
                            .response
                            .changed()
                        {
                            backend.effect(move |mission, _| {
                                mission
                                    .plan
                                    .get_mut(&vessel_id)
                                    .expect("plan deleted")
                                    .anchor_vector_slot = av_slot;
                                Ok(())
                            });
                        };

                        let vessel = &mission.vessels[vessel_id];
                        let Some(class_id) = vessel.class else {
                            handle(toasts, |_| -> eyre::Result<()> {
                                bail!("{}", i18n!("error-mpt-noclass"))
                            });
                            break 'slot;
                        };
                        let class = &mission.classes[class_id];
                        if ui
                            .button(if plan.sim_vessel.is_some() {
                                i18n!("mpt-reinit")
                            } else {
                                i18n!("mpt-init")
                            })
                            .clicked()
                            || self.reinit
                        {
                            self.reinit = false;
                            let mut sim_vessel = SimVessel {
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

                            let mut map = HashMap::new();

                            for (pid, part) in class.parts.iter() {
                                let mut modules_current_mass = 0.0;
                                if !part.mass_modifiers.is_empty() {
                                    for modifier in &part.mass_modifiers {
                                        modules_current_mass += modifier.current_mass;
                                    }
                                }
                                let resources = vessel
                                    .resources
                                    .iter()
                                    .filter_map(|x| {
                                        (x.0 .0 == pid).then_some((x.0 .1, x.1.clone()))
                                    })
                                    .collect::<HashMap<_, _>>();
                                let disabled_resource_mass =
                                    resources.iter().fold(0.0, |acc, x| {
                                        if x.1.enabled {
                                            acc
                                        } else {
                                            acc + x.1.density * x.1.amount
                                        }
                                    });
                                let sid = sim_vessel.parts.push(SimPart {
                                    persistent_id: part.persistent_id,
                                    crossfeed_part_set: vec![],
                                    resources,
                                    resource_drains: HashMap::new(),
                                    resource_priority: part.resource_priority,
                                    resource_request_remaining_threshold: part
                                        .resource_request_remaining_threshold,
                                    mass: part.mass,
                                    dry_mass: part.dry_mass,
                                    crew_mass: part.crew_mass,
                                    modules_current_mass,
                                    disabled_resource_mass,
                                    is_launch_clamp: part.is_launch_clamp,
                                });
                                map.insert(pid, sid);
                            }
                            for (pid, sid) in &map {
                                sim_vessel.parts[*sid].crossfeed_part_set = class.parts[*pid]
                                    .crossfeed_part_set
                                    .iter()
                                    .filter_map(|x| map.get(x).copied())
                                    .collect();
                            }
                            let mut prev_resources = None;

                            let Some(av) =
                                handle(toasts, |_| find_sv(Some(vessel), &plan.anchor_vector_slot))
                            else {
                                break 'slot;
                            };
                            let mut prev_vector = av;
                            let mut prev_deltav = Vector3::zeros();
                            let maneuvers = plan
                                .maneuvers
                                .iter()
                                .cloned()
                                .map(|mut mnv| {
                                    let mnv_ut = UT::from_duration(
                                        mnv.inner.geti.into_duration()
                                            + vessel.get_base.into_duration(),
                                    );
                                    if mnv_ut > prev_vector.time {
                                        let mut sv = prev_vector.clone();
                                        sv.velocity += prev_deltav;
                                        mnv.inner.tig_vector = sv.propagate_with_soi(
                                            &mission.system,
                                            mnv_ut - prev_vector.time,
                                            1e-7,
                                            30000,
                                        );
                                        prev_vector = mnv.inner.tig_vector.clone();
                                    } else if (mnv_ut.into_duration().as_seconds_f64()
                                        - prev_vector.time.into_duration().as_seconds_f64())
                                    .abs()
                                        < 1e-3
                                    {
                                        mnv.inner.tig_vector = prev_vector.clone();
                                    }
                                    prev_deltav = mnv.inner.deltav_bci();

                                    let mnv = Self::run_ffs(
                                        mnv,
                                        prev_resources.clone(),
                                        class,
                                        &sim_vessel,
                                    );
                                    prev_resources = Some(mnv.resources.clone());
                                    mnv
                                })
                                .collect();

                            backend.effect(move |mission, _| {
                                let plan = mission.plan.get_mut(&vessel_id).expect("plan deleted");
                                plan.sim_vessel = Some(sim_vessel);
                                plan.maneuvers = maneuvers;
                                Ok(())
                            });
                        }
                    }
                });
                ui.horizontal(|ui| 'status: {
                    ui.label(i18n!("mpt-status"));
                    let Some(vessel_id) = self.vessel else {
                        ui.strong(i18n!("mpt-status-no-vessel"));
                        break 'status;
                    };
                    let Some(plan) = mission.plan.get(&vessel_id) else {
                        ui.strong(i18n!("mpt-status-no-vessel"));
                        break 'status;
                    };
                    let Ok(sv) =
                        find_sv(Some(&mission.vessels[vessel_id]), &plan.anchor_vector_slot)
                    else {
                        ui.strong(i18n!("mpt-status-missing-av"));
                        break 'status;
                    };

                    let get_base = mission.vessels[vessel_id].get_base;
                    let partially_active = plan.maneuvers.iter().any(|mnv| {
                        sv.time.into_duration()
                            > (mnv.inner.geti.into_duration() + get_base.into_duration())
                    });
                    if partially_active {
                        ui.strong(i18n!("mpt-status-partial-av"));
                        break 'status;
                    }
                    if plan.sim_vessel.is_some() {
                        ui.strong(i18n!("mpt-status-active"));
                    } else {
                        ui.strong(i18n!("mpt-status-active-noinit"));
                    }
                });
                ui.separator();

                ui.style_mut().override_text_style = Some(egui::TextStyle::Monospace);
                egui_extras::TableBuilder::new(ui)
                    .column(Column::auto_with_initial_suggestion(32.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(48.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(48.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(64.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(48.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(48.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(96.0).resizable(true))
                    .cell_layout(
                        egui::Layout::left_to_right(egui::Align::Center)
                            .with_cross_align(egui::Align::RIGHT)
                            .with_main_wrap(false)
                            .with_cross_justify(true)
                            .with_main_justify(true),
                    )
                    .header(16.0, |mut header| {
                        let layout = egui::Layout::default().with_cross_align(egui::Align::Center);
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("mpt-geti")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("mpt-delta-t")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("mpt-delta-v")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("mpt-delta-v-rem")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("mpt-ha")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("mpt-hp")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("mpt-code")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });
                    })
                    .body(|mut body| 'display: {
                        let Some(vessel_id) = self.vessel else {
                            break 'display;
                        };
                        let vessel = &mission.vessels[vessel_id];

                        let Some(plan) = mission.plan.get(&vessel_id) else {
                            break 'display;
                        };
                        let Ok(sv) = find_sv(Some(vessel), &plan.anchor_vector_slot) else {
                            break 'display;
                        };

                        let maneuvers = &plan.maneuvers;
                        for (ix, maneuver) in maneuvers.iter().enumerate() {
                            let deltav_bci = maneuver.inner.deltav_bci();
                            let mut post_sv = maneuver.inner.tig_vector.clone();
                            post_sv.velocity += deltav_bci;
                            let r = post_sv.body.radius;
                            let post_obt = post_sv.into_orbit(1e-8);

                            body.row(16.0, |mut row| {
                                row.col(|ui| {
                                    let geti = maneuver.inner.geti;
                                    let (d, h, m, s, ms) = (
                                        geti.days().abs(),
                                        geti.hours(),
                                        geti.minutes(),
                                        geti.seconds(),
                                        geti.millis(),
                                    );
                                    let ut = UT::from_duration(
                                        maneuver.inner.geti.into_duration()
                                            + vessel.get_base.into_duration(),
                                    );
                                    let n = if geti.is_negative() { "-" } else { "" };
                                    ui.add(
                                        egui::Label::new(
                                            egui::RichText::new(format!(
                                                "{n}{d:03}:{h:02}:{m:02}:{s:02}.{ms:03}",
                                            ))
                                            .color(
                                                if sv.time > ut {
                                                    egui::Color32::from_rgb(238, 210, 2)
                                                } else {
                                                    ui.visuals().text_color()
                                                },
                                            ),
                                        )
                                        .wrap(false),
                                    );
                                });
                                row.col(|ui| {
                                    if ix != 0 {
                                        let prev_mnv = &maneuvers[ix - 1];
                                        let deltat = maneuver.inner.geti - prev_mnv.inner.geti;
                                        let (h, m) = (
                                            deltat.whole_hours(),
                                            deltat.whole_minutes().unsigned_abs() % 60,
                                        );

                                        ui.add(egui::Label::new(format!("{h}:{m:02}")).wrap(false));
                                    }
                                });
                                row.col(|ui| {
                                    ui.add(
                                        egui::Label::new(format!(
                                            "{:>+08.2}",
                                            maneuver.inner.deltav.norm() * 1000.0
                                        ))
                                        .wrap(false),
                                    );
                                });
                                row.col(|ui| {
                                    ui.add(
                                        egui::Label::new(format!("{:.2}", maneuver.dvrem))
                                            .wrap(false),
                                    );
                                });
                                // TODO
                                let ra = post_obt.apoapsis_radius();
                                let rp = post_obt.periapsis_radius();
                                row.col(|ui| {
                                    ui.add(egui::Label::new(format!("{:.1}", ra - r)).wrap(false));
                                });
                                row.col(|ui| {
                                    ui.add(egui::Label::new(format!("{:.1}", rp - r)).wrap(false));
                                });
                                row.col(|ui| {
                                    ui.horizontal(|ui| {
                                        ui.add(
                                            egui::Label::new(format!(
                                                "{:04}X/{:02}",
                                                maneuver.source as u16,
                                                ix + 1
                                            ))
                                            .wrap(false),
                                        );

                                        if ui
                                            .button(icon("\u{e872}"))
                                            .on_hover_text(i18n!("mpt-delete"))
                                            .clicked()
                                        {
                                            backend.effect(move |mission, state| {
                                                mission
                                                    .plan
                                                    .get_mut(&vessel_id)
                                                    .expect("plan deleted")
                                                    .maneuvers
                                                    .remove(ix);
                                                state.mpt.reinit = true;
                                                Ok(())
                                            });
                                        }
                                    });
                                });
                            });
                        }
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

pub struct MPTTransfer {
    ui_id: egui::Id,
    vessel: Option<VesselId>,
    mnv: Option<(Maneuver, u64)>,
    vessel_engines: HashMap<PartId, bool>,
    planned: Option<PlannedManeuver>,
    mnv_ctr: u64,
    maninp_mode: bool,
    maninp_time_unparsed: String,
    maninp_time_parsed: Option<UTorGET>,
    maninp_time_disp: TimeDisplayKind,
    maninp_dvx: String,
    maninp_dvy: String,
    maninp_dvz: String,
    maninp_dvx_val: Option<f64>,
    maninp_dvy_val: Option<f64>,
    maninp_dvz_val: Option<f64>,
    source: DisplaySelect,
}

impl Default for MPTTransfer {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
            vessel: None,
            mnv: None,
            vessel_engines: HashMap::new(),
            planned: None,
            maninp_mode: false,
            maninp_time_unparsed: String::new(),
            maninp_time_parsed: None,
            maninp_time_disp: TimeDisplayKind::Dhms,
            maninp_dvx: "0.0".into(),
            maninp_dvy: "0.0".into(),
            maninp_dvz: "0.0".into(),
            maninp_dvx_val: Some(0.0),
            maninp_dvy_val: Some(0.0),
            maninp_dvz_val: Some(0.0),
            mnv_ctr: 1,
            source: DisplaySelect::TLIProcessor,
        }
    }
}

impl MPTTransfer {
    fn run_ffs(
        &mut self,
        class: &VesselClass,
        plan: &MissionPlan,
        mnv: Maneuver,
    ) -> eyre::Result<PlannedManeuver> {
        // TODO: shunt this to the handler thread
        let mut ffs = FuelFlowSimulation::default();

        let maneuvers = &plan.maneuvers;
        let ix = maneuvers.binary_search_by_key(&mnv.geti.into_duration(), |mnv| {
            mnv.inner.geti.into_duration()
        });
        let ix = match ix {
            Ok(x) | Err(x) => x,
        };
        let mut sim_vessel = plan
            .sim_vessel
            .clone()
            .ok_or_eyre(i18n!("error-mpt-no-init"))?;

        if ix > 0 {
            let mnv_prev = &maneuvers[ix - 1];
            for (pid, resources) in &mnv_prev.resources {
                if let Some((_, part)) = sim_vessel
                    .parts
                    .iter_mut()
                    .find(|x| x.1.persistent_id == *pid)
                {
                    part.resources.clone_from(resources);
                }
            }
        }

        for (pid, part) in class.parts.iter() {
            if self.vessel_engines.get(&pid).copied().unwrap_or(false) {
                let sid = sim_vessel
                    .parts
                    .iter()
                    .find_map(|(i, x)| (x.persistent_id == part.persistent_id).then_some(i))
                    .expect("oops");
                sim_vessel
                    .active_engines
                    .extend(part.engines.iter().cloned().map(|mut x| {
                        x.part = sid;
                        x
                    }));
            }
        }

        ffs.run(&mut sim_vessel, Some(mnv.deltav.norm() * 1000.0), false);

        let resources = sim_vessel
            .parts
            .iter()
            .map(|(_, x)| (x.persistent_id, x.resources.clone()))
            .collect::<HashMap<_, _>>();

        tracing::trace!("{ffs:#?}");
        ffs.run(&mut sim_vessel, None, true);
        tracing::trace!("{ffs:#?}");
        //ffs.run(&mut sim_vessel, None);

        let mut deltav = mnv.deltav.norm() * 1000.0;
        let mut bt = 0.0;
        let start_mass_tons = ffs.segments[0].start_mass;
        let mut end_mass = ffs.segments[0].start_mass * 1000.0;
        for segment in &ffs.segments {
            match segment.deltav.total_cmp(&deltav) {
                Ordering::Less | Ordering::Equal => {
                    bt += segment.delta_time;
                    deltav -= segment.deltav;
                    end_mass = segment.end_mass * 1000.0;
                }
                Ordering::Greater => {
                    // Calculate end mass with Tsiolkovsky rocket equation

                    let start_mass = end_mass;
                    let exhvel = segment.isp * ffs::G0;
                    let alpha = libm::exp(-deltav / exhvel);
                    end_mass = start_mass * alpha;

                    bt += (start_mass * exhvel) / (1000.0 * segment.thrust) * (1.0 - alpha);
                    deltav = 0.0;
                    break;
                }
            }
        }

        let dvrem = if deltav > 1e-6 {
            -deltav
        } else {
            ffs.segments
                .iter()
                .fold(-mnv.deltav.norm() * 1000.0, |acc, x| acc + x.deltav)
        };

        Ok(PlannedManeuver {
            inner: mnv,
            engines: self
                .vessel_engines
                .iter()
                .filter_map(|(i, x)| x.then_some(*i))
                .collect(),
            dvrem,
            bt,
            start_mass: start_mass_tons,
            end_mass: end_mass / 1000.0,
            resources,
            source: self.source,
        })
    }
}

impl KtkDisplay for MPTTransfer {
    fn show(
        &mut self,
        mission: &Mission,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    ) {
        egui::Window::new(i18n!("mpt-trfr-title"))
            .open(&mut open.mpt_trfr)
            .default_size([256.0, 512.0])
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    ui.horizontal(|ui| {
                        ui.label(i18n!("mpt-trfr-vessel"));
                        egui::ComboBox::from_id_source(self.ui_id.with("VesselSelector"))
                            .selected_text(self.vessel.map_or_else(
                                || i18n!("mpt-trfr-no-vessel"),
                                |x| mission.vessels[x].name.clone(),
                            ))
                            .show_ui(ui, |ui| {
                                for (id, iter_vessel) in mission
                                    .vessels
                                    .iter()
                                    .sorted_by_key(|(_, x)| x.name.clone())
                                {
                                    if ui
                                        .selectable_value(
                                            &mut self.vessel,
                                            Some(id),
                                            &iter_vessel.name,
                                        )
                                        .clicked()
                                    {
                                        self.vessel_engines.clear();
                                    };
                                }
                            });
                    });
                    if ui.button(i18n!("mpt-trfr-maninp")).clicked() {
                        self.maninp_mode = true;
                        self.mnv.take();
                    }
                    ui.separator();
                    ui.heading(i18n!("mpt-trfr-mnvinfo"));
                    if self.maninp_mode && self.mnv.is_none() {
                        ui.horizontal(|ui| {
                            ui.vertical(|ui| {
                                let spacing = ui.spacing().interact_size.y
                                    - ui.text_style_height(&egui::TextStyle::Monospace);

                                ui.monospace(i18n!("mpt-trfr-code"));
                                ui.monospace(i18n!("mpt-trfr-geti"));
                                ui.add_space(spacing);
                                ui.monospace(i18n!("mpt-trfr-dv-prograde"));
                                ui.add_space(spacing);
                                ui.monospace(i18n!("mpt-trfr-dv-normal"));
                                ui.add_space(spacing);
                                ui.monospace(i18n!("mpt-trfr-dv-radial"));
                                ui.add_space(spacing);
                                ui.monospace(i18n!("mpt-trfr-dv-total"));
                            });
                            ui.vertical(|ui| {
                                ui.monospace(format!("0101C/{:02}", self.mnv_ctr));
                                ui.horizontal(|ui| {
                                    ui.add(TimeInput1::new(
                                        &mut self.maninp_time_unparsed,
                                        &mut self.maninp_time_parsed,
                                        Some(128.0),
                                        TimeInputKind2::GET,
                                        self.maninp_time_disp,
                                        true,
                                        false,
                                    ));
                                    ui.add(TimeDisplayBtn(&mut self.maninp_time_disp));
                                });
                                ui.add(DVInput::new(
                                    &mut self.maninp_dvx,
                                    &mut self.maninp_dvx_val,
                                    Some(64.0),
                                    egui::Color32::from_rgb(0, 214, 0),
                                    true,
                                ));

                                ui.add(DVInput::new(
                                    &mut self.maninp_dvy,
                                    &mut self.maninp_dvy_val,
                                    Some(64.0),
                                    egui::Color32::from_rgb(214, 0, 214),
                                    true,
                                ));
                                ui.add(DVInput::new(
                                    &mut self.maninp_dvz,
                                    &mut self.maninp_dvz_val,
                                    Some(64.0),
                                    egui::Color32::from_rgb(0, 214, 214),
                                    true,
                                ));
                                ui.monospace(format!(
                                    "{:>+08.2}",
                                    Vector3::new(
                                        self.maninp_dvx_val.unwrap_or(0.0),
                                        self.maninp_dvy_val.unwrap_or(0.0),
                                        self.maninp_dvz_val.unwrap_or(0.0)
                                    )
                                    .norm()
                                ));
                            });
                        });
                        if ui.button(i18n!("mpt-trfr-maninp-conf")).clicked() {
                            handle(toasts, |_| {
                                let geti = self
                                    .maninp_time_parsed
                                    .ok_or_eyre(i18n!("mpt-trfr-error-no-geti"))?;
                                let UTorGET::GET(geti) = geti else {
                                    bail!("{}", i18n!("mpt-trfr-error-no-geti"));
                                };
                                let sv =
                                    find_sv_mpt(self.vessel, self.maninp_time_parsed, mission)?;

                                self.mnv = Some((
                                    Maneuver {
                                        geti,
                                        deltav: Vector3::new(
                                            self.maninp_dvx_val.unwrap_or(0.0),
                                            self.maninp_dvy_val.unwrap_or(0.0),
                                            self.maninp_dvz_val.unwrap_or(0.0),
                                        ) / 1000.0,
                                        tig_vector: sv,
                                        kind: ManeuverKind::ManualInput,
                                    },
                                    self.mnv_ctr,
                                ));
                                self.source = DisplaySelect::MPTTransfer;
                                self.mnv_ctr += 1;
                                Ok(())
                            });
                        }
                    }
                    if let Some((mnv, code)) = &self.mnv {
                        ui.horizontal(|ui| {
                            ui.vertical(|ui| {
                                ui.monospace(i18n!("mpt-trfr-code"));
                                ui.monospace(i18n!("mpt-trfr-geti"));
                                ui.monospace(i18n!("mpt-trfr-dv-prograde"));
                                ui.monospace(i18n!("mpt-trfr-dv-normal"));
                                ui.monospace(i18n!("mpt-trfr-dv-radial"));
                                ui.monospace(i18n!("mpt-trfr-dv-total"));
                            });

                            ui.vertical(|ui| {
                                ui.monospace(format!("{:04}C/{:02}", self.source as u16, code));
                                let geti = mnv.geti;
                                let (d, h, m, s, ms) = (
                                    geti.days(),
                                    geti.hours(),
                                    geti.minutes(),
                                    geti.seconds(),
                                    geti.millis(),
                                );
                                ui.monospace(format!("{d:03}:{h:02}:{m:02}:{s:02}.{ms:03}"));
                                ui.add(egui::Label::new(
                                    egui::RichText::new(format!(
                                        "{:>+08.2}",
                                        mnv.deltav.x * 1000.0
                                    ))
                                    .color(egui::Color32::from_rgb(0, 214, 0))
                                    .monospace(),
                                ));
                                ui.add(egui::Label::new(
                                    egui::RichText::new(format!(
                                        "{:>+08.2}",
                                        mnv.deltav.y * 1000.0
                                    ))
                                    .color(egui::Color32::from_rgb(214, 0, 214))
                                    .monospace(),
                                ));
                                ui.add(egui::Label::new(
                                    egui::RichText::new(format!(
                                        "{:>+08.2}",
                                        mnv.deltav.z * 1000.0
                                    ))
                                    .color(egui::Color32::from_rgb(0, 214, 214))
                                    .monospace(),
                                ));
                                ui.vertical(|ui| {
                                    ui.monospace(format!("{:>+08.2}", mnv.deltav.norm() * 1000.0));
                                });
                            });
                        });
                    }

                    ui.heading(i18n!("mpt-trfr-vessel-cfg"));
                    'engines: {
                        let Some(vessel_id) = self.vessel else {
                            break 'engines;
                        };
                        let vessel = &mission.vessels[vessel_id];
                        let Some(class_id) = vessel.class else {
                            break 'engines;
                        };
                        let class = &mission.classes[class_id];
                        let Some((mnv, code)) = self.mnv.clone() else {
                            break 'engines;
                        };

                        // TODO: multiple engines per part
                        for (partid, part) in class
                            .parts
                            .iter()
                            .filter(|x| !x.1.engines.is_empty())
                            .sorted_by_key(|x| (&x.1.title, &x.1.tag))
                        {
                            if part.tag.trim().is_empty() {
                                ui.checkbox(
                                    self.vessel_engines.entry(partid).or_insert(false),
                                    &*part.title,
                                );
                            } else {
                                ui.checkbox(
                                    self.vessel_engines.entry(partid).or_insert(false),
                                    i18n_args!("part-tag", "part", &part.title, "tag", &part.tag),
                                );
                            }
                        }

                        if ui.button(i18n!("mpt-trfr-calc-fuel")).clicked() {
                            handle(toasts, |_| {
                                let plan = mission
                                    .plan
                                    .get(&vessel_id)
                                    .ok_or_eyre(i18n!("error-mpt-no-init"))?;
                                self.planned = Some(self.run_ffs(class, plan, mnv.clone())?);
                                Ok(())
                            });
                        }
                        ui.horizontal(|ui| {
                            ui.vertical(|ui| {
                                ui.monospace(i18n!("mpt-trfr-dvrem"));
                                ui.monospace(i18n!("mpt-trfr-startmass"));
                                ui.monospace(i18n!("mpt-trfr-endmass"));
                                ui.monospace(i18n!("mpt-trfr-burntime"));
                            });
                            ui.vertical(|ui| {
                                let (deltav, start_mass_tons, end_mass_tons, bt) =
                                    if let Some(planned) = &self.planned {
                                        (
                                            planned.dvrem,
                                            planned.start_mass,
                                            planned.end_mass,
                                            planned.bt,
                                        )
                                    } else {
                                        (0.0, 0.0, 0.0, 0.0)
                                    };
                                ui.monospace(format!("{deltav:.2}"));
                                ui.monospace(format!("{start_mass_tons:.3}"));
                                ui.monospace(format!("{end_mass_tons:.3}"));
                                ui.monospace(format!("{bt:.3}"));
                            });
                        });
                        if ui.button(i18n!("mpt-trfr-trfr")).clicked() {
                            handle(toasts, |_| {
                                let plan = mission
                                    .plan
                                    .get(&vessel_id)
                                    .ok_or_eyre(i18n!("error-mpt-no-init"))?;

                                let mnv = self.planned.clone().map_or_else(
                                    || self.run_ffs(class, plan, mnv.clone()),
                                    Result::Ok,
                                )?;
                                let ix = plan
                                    .maneuvers
                                    .binary_search_by_key(&mnv.inner.geti.into_duration(), |mnv| {
                                        mnv.inner.geti.into_duration()
                                    });
                                let ix = match ix {
                                    Ok(ix) => {
                                        bail!(
                                            "{}",
                                            i18n_args!(
                                                "error-mpt-uniqueness",
                                                "candidate",
                                                format!("{:04}C/{:02}", self.source as u16, code),
                                                "executed",
                                                format!(
                                                    "{:04}X/{:02}",
                                                    plan.maneuvers[ix].source as u16,
                                                    ix + 1
                                                )
                                            )
                                        );
                                    }
                                    Err(ix) => ix,
                                };

                                backend.effect(move |mission, state| {
                                    let plan = mission
                                        .plan
                                        .get_mut(&vessel_id)
                                        .ok_or_eyre(i18n!("error-mpt-no-init"))?;
                                    plan.maneuvers.insert(ix, mnv);
                                    state.mpt.reinit = true;
                                    Ok(())
                                });
                                Ok(())
                            });
                        }
                    }
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
        if let Ok(HRes::MPTTransfer(mnv, source, code)) = res {
            self.mnv = Some((mnv, code));
            self.source = source;
            self.maninp_mode = false;
            self.planned = None;
            Ok(())
        } else {
            res.map(|_| ())
        }
    }
}
