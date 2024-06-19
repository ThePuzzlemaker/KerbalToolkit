#![warn(clippy::unwrap_used)]
use std::{
    cmp::Ordering,
    collections::{HashMap, HashSet, VecDeque},
    f64::consts,
    fmt,
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc,
    },
    thread,
    time::Instant,
};

use backend::{handler_thread, HReq, HRes, TLIInputs};
use color_eyre::eyre::{self, OptionExt};
use egui::TextBuffer;
use egui_extras::Column;
use itertools::Itertools;
use nalgebra::Vector3;
use parking_lot::RwLock;
//use tokio::runtime::{self, Runtime};
use utils::{KRPCConfig, SystemConfiguration, TimeUtils};

use egui_notify::Toasts;
use kerbtk::{
    arena::Arena,
    ffs::{self, Conditions, FuelFlowSimulation, FuelStats, SimPart, SimVessel},
    kepler::orbits::StateVector,
    maneuver::Maneuver,
    time::{GET, UT},
    translunar::TLIConstraintSet,
    vessel::{PartId, Vessel, VesselClass, VesselClassRef, VesselRef},
};
use mission::{Mission, MissionPlan, PlannedManeuver};
use num_enum::FromPrimitive;
use time::Duration;
use tracing::error;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};
use unic_langid::LanguageIdentifier;
use vectors::{VectorComparison, VectorPanelSummary, VectorSelector};
use vessels::{Classes, Vessels};
use widgets::{icon, icon_label, DurationInput};

mod backend;
mod mission;
mod utils;
mod vectors;
mod vessels;
mod widgets;

fluent_templates::static_loader! {
    static LOCALES = {
        locales: "src/locales",
        fallback_language: "en-US",
    };
}

const US_ENGLISH: LanguageIdentifier = unic_langid::langid!("en-US");

#[macro_export]
macro_rules! i18n {
    ($v:expr) => {{
        use ::fluent_templates::Loader;
        $crate::LOCALES.lookup(&$crate::US_ENGLISH, $v)
    }};
}

#[macro_export]
macro_rules! i18n_args {
    ($v:expr, $($arg:expr, $val:expr),*) => {{
	use ::fluent_templates::Loader;
	let mut args = ::std::collections::HashMap::new();
	$(
	    args.insert(::std::string::String::from($arg), ::fluent::FluentValue::from($val));
	)*
	$crate::LOCALES.lookup_with_args(&$crate::US_ENGLISH, $v, &args)
    }}
}

fn handle<T>(toasts: &mut Toasts, f: impl FnOnce(&mut Toasts) -> eyre::Result<T>) -> Option<T> {
    match f(toasts) {
        Ok(v) => Some(v),
        Err(e) => {
            toasts.error(format!("{}", e));
            error!("{:#}", e);
            None
        }
    }
}

fn main() -> eyre::Result<()> {
    color_eyre::install()?;
    tracing_subscriber::registry()
        .with(tracing_subscriber::fmt::layer())
        .with(EnvFilter::from_default_env())
        .init();
    let native_options = eframe::NativeOptions::default();
    let (main_tx, handler_rx) = mpsc::channel();
    let (handler_tx, main_rx) = mpsc::channel();

    let main_tx_loopback = handler_tx.clone();
    let _ = thread::spawn(|| handler_thread(handler_rx, handler_tx));
    eframe::run_native(
        &i18n!("title"),
        native_options,
        Box::new(|cc| {
            Box::new(NewApp::new(
                cc,
                Backend {
                    tx: main_tx,
                    rx: main_rx,
                    tx_loopback: main_tx_loopback,
                    txc: 0,
                    txq: HashMap::new(),
                    ctx: cc.egui_ctx.clone(),
                    stq: VecDeque::new(),
                    // rt: runtime::Builder::new_multi_thread()
                    //     .enable_all()
                    //     .build()
                    //     .expect("oops"),
                },
            ))
        }),
    )
    .expect(&i18n!("error-start-failed"));
    std::process::exit(0)
}

struct NewApp {
    mission: Mission,
    state: State,
    backend: Backend,
}

impl NewApp {
    fn new(cc: &eframe::CreationContext, backend: Backend) -> Self {
        cc.egui_ctx
            .style_mut(|style| style.explanation_tooltips = true);
        let mut fonts = egui::FontDefinitions::default();

        fonts.font_data.insert(
            "mtl-icons".to_owned(),
            egui::FontData::from_static(include_bytes!("assets/MaterialSymbolsOutlined.ttf")),
        );
        fonts.families.insert(
            egui::FontFamily::Name("mtl-icons".into()),
            vec!["mtl-icons".into(), "Hack".into()],
        );

        cc.egui_ctx.set_fonts(fonts);

        Self {
            mission: Mission::default(),
            state: State::default(),
            backend,
        }
    }

    fn open_window(&mut self, selector: &str) {
        match selector.parse::<u16>().unwrap_or(u16::MAX).into() {
            DisplaySelect::SysCfg => self.state.dis.syscfg = true,
            DisplaySelect::Krpc => self.state.dis.krpc = true,
            DisplaySelect::Logs => self.state.dis.logs = true,
            DisplaySelect::TimeUtils => self.state.dis.time_utils = true,
            DisplaySelect::MPT => self.state.dis.mpt = true,
            DisplaySelect::MPTTransfer => self.state.dis.mpt_trfr = true,
            DisplaySelect::VC => self.state.dis.vc = true,
            DisplaySelect::VPS => self.state.dis.vps = true,
            DisplaySelect::Classes => self.state.dis.classes = true,
            DisplaySelect::Vessels => self.state.dis.vessels = true,
            DisplaySelect::TLIProcessor => self.state.dis.tliproc = true,
            DisplaySelect::Unknown => {}
        }
    }
}

macro_rules! show_display {
    ($self:ident, $display:ident, $mission:ident, $ctx:ident, $frame:ident) => {
        $self.state.$display.show(
            &$mission,
            &mut $self.state.toasts,
            &mut $self.backend,
            $ctx,
            $frame,
            &mut $self.state.dis,
        );
    };
}

macro_rules! handle_rx {
    ($res:ident, $self:ident, $display:ident, $mission:ident, $ctx:ident, $frame:ident) => {
        $self.state.$display.handle_rx(
            $res,
            &$mission,
            &mut $self.state.toasts,
            &mut $self.backend,
            $ctx,
            $frame,
        )
    };
}

impl eframe::App for NewApp {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        let mission = &self.mission;

        while let Ok((txi, res)) = self
            .backend
            .rx
            .recv_timeout(std::time::Duration::from_millis(10))
        {
            use DisplaySelect::*;
            // TODO
            let res = match self.backend.txq.remove(&txi) {
                Some(Krpc) => handle_rx!(res, self, krpc, mission, ctx, frame),
                Some(SysCfg) => handle_rx!(res, self, syscfg, mission, ctx, frame),
                Some(VC) => handle_rx!(res, self, vc, mission, ctx, frame),
                Some(VPS) => handle_rx!(res, self, vps, mission, ctx, frame),
                Some(TLIProcessor) => handle_rx!(res, self, tliproc, mission, ctx, frame),
                Some(MPTTransfer) => handle_rx!(res, self, mpt_trfr, mission, ctx, frame),
                Some(MPT) => handle_rx!(res, self, mpt, mission, ctx, frame),
                _ => Ok(()),
            };
            handle(&mut self.state.toasts, |_| res);
        }

        show_display!(self, krpc, mission, ctx, frame);
        show_display!(self, logs, mission, ctx, frame);
        show_display!(self, syscfg, mission, ctx, frame);
        show_display!(self, tliproc, mission, ctx, frame);
        show_display!(self, vc, mission, ctx, frame);
        show_display!(self, vps, mission, ctx, frame);
        show_display!(self, time_utils, mission, ctx, frame);
        show_display!(self, classes, mission, ctx, frame);
        show_display!(self, vessels, mission, ctx, frame);
        show_display!(self, mpt_trfr, mission, ctx, frame);
        show_display!(self, mpt, mission, ctx, frame);

        egui::Window::new(i18n!("menu-title"))
            .default_width(196.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    handle(&mut self.state.toasts, |_| {
                        let file = rfd::FileDialog::new().set_file_name("mission.ktk");

                        if ui
                            .button(icon_label("\u{e2c8}", &i18n!("menu-load-mission")))
                            .clicked()
                        {
                            if let Some(path) = file.clone().pick_file() {
                                let new_mission = ron::from_str(&std::fs::read_to_string(path)?)?;
                                self.backend.effect(|mission| {
                                    *mission = new_mission;
                                    Ok(())
                                });
                                //self.classes.force_refilter = true;
                                //self.vessels.force_refilter = true;
                                ctx.request_repaint();
                            }
                        }
                        if ui
                            .button(icon_label("\u{e161}", &i18n!("menu-save-mission")))
                            .clicked()
                        {
                            if let Some(path) = file.save_file() {
                                std::fs::write(path, ron::to_string(&self.mission).expect("oops"))?;
                            }
                        }
                        Ok(())
                    });
                });
                ui.horizontal(|ui| {
                    ui.add(egui::Label::new(i18n!("menu-display-select")));
                    let response = ui.add(
                        egui::TextEdit::singleline(&mut self.state.menu.selector)
                            .char_limit(4)
                            .font(egui::TextStyle::Monospace)
                            .desired_width(32.0),
                    );
                    if (response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)))
                        || ui.button(i18n!("menu-open")).clicked()
                    {
                        let selector = self.state.menu.selector.take();
                        self.open_window(&selector);
                    }
                });
                if ui
                    .add(egui::Button::new(i18n!("menu-organize-windows")).wrap(false))
                    .clicked()
                {
                    ui.ctx().memory_mut(|mem| mem.reset_areas());
                }
                ui.separator();
                let mut openall = None;
                ui.horizontal(|ui| {
                    if ui.button(i18n!("expand-all")).clicked() {
                        openall = Some(true);
                    }
                    if ui.button(i18n!("collapse-all")).clicked() {
                        openall = Some(false);
                    }
                });
                if ui.button(i18n!("menu-close-all-windows")).clicked() {
                    self.state.dis = Default::default();
                }
                ui.vertical(|ui| {
                    egui::CollapsingHeader::new(i18n!("menu-display-config"))
                        .open(openall)
                        .show(ui, |ui| {
                            ui.checkbox(&mut self.state.dis.syscfg, i18n!("menu-display-syscfg"));
                            ui.checkbox(&mut self.state.dis.krpc, i18n!("menu-display-krpc"));
                            ui.checkbox(&mut self.state.dis.logs, i18n!("menu-display-logs"));
                            ui.checkbox(&mut self.state.dis.time_utils, i18n!("menu-display-time"));
                        });
                    egui::CollapsingHeader::new(i18n!("menu-display-mpt"))
                        .open(openall)
                        .show(ui, |ui| {
                            ui.checkbox(&mut self.state.dis.mpt, i18n!("menu-display-open-mpt"));
                            ui.checkbox(
                                &mut self.state.dis.mpt_trfr,
                                i18n!("menu-display-mpt-trfr"),
                            );
                        });
                    egui::CollapsingHeader::new(i18n!("menu-display-sv"))
                        .open(openall)
                        .show(ui, |ui| {
                            ui.checkbox(&mut self.state.dis.vc, i18n!("menu-display-sv-comp"));
                            ui.checkbox(&mut self.state.dis.vps, i18n!("menu-display-sv-vps"));
                        });
                    egui::CollapsingHeader::new(i18n!("menu-display-vesselsclasses"))
                        .open(openall)
                        .show(ui, |ui| {
                            ui.checkbox(&mut self.state.dis.classes, i18n!("menu-display-classes"));
                            ui.checkbox(&mut self.state.dis.vessels, i18n!("menu-display-vessels"));
                        });
                    egui::CollapsingHeader::new(i18n!("menu-display-target"))
                        .open(openall)
                        .show(ui, |ui| {
                            ui.checkbox(&mut self.state.dis.tliproc, i18n!("menu-display-tliproc"));
                        });
                });
            });

        self.state.toasts.show(ctx);

        for st in self.backend.stq.drain(..) {
            handle(&mut self.state.toasts, |_| {
                (st)(&mut self.mission)?;
                Ok(())
            });
        }
    }
}

#[derive(Default)]
struct State {
    toasts: Toasts,
    dis: Displays,
    menu: Menu,
    krpc: KRPCConfig,
    logs: Logs,
    time_utils: TimeUtils,
    syscfg: SystemConfiguration,
    vc: VectorComparison,
    vps: VectorPanelSummary,
    tliproc: TLIProcessor,
    classes: Classes,
    vessels: Vessels,
    mpt_trfr: MPTTransfer,
    mpt: MissionPlanTable,
}

struct Backend {
    ctx: egui::Context,
    tx: Sender<(usize, egui::Context, HReq)>,
    rx: Receiver<(usize, eyre::Result<HRes>)>,
    tx_loopback: Sender<(usize, eyre::Result<HRes>)>,
    txc: usize,
    txq: HashMap<usize, DisplaySelect>,
    stq: VecDeque<StateUpdater>,
    //pub rt: Runtime,
}

type StateUpdater = Box<dyn FnOnce(&mut Mission) -> eyre::Result<()> + Send + 'static>;

impl Backend {
    fn tx(&mut self, src: DisplaySelect, req: HReq) -> eyre::Result<()> {
        self.tx.send((self.txc, self.ctx.clone(), req))?;
        self.txq.insert(self.txc, src);
        self.txc += 1;
        Ok(())
    }

    fn tx_loopback(&mut self, src: DisplaySelect, res: eyre::Result<HRes>) -> eyre::Result<()> {
        self.tx_loopback.send((self.txc, res))?;
        self.txq.insert(self.txc, src);
        self.txc += 1;
        Ok(())
    }

    fn effect(&mut self, f: impl FnOnce(&mut Mission) -> eyre::Result<()> + Send + 'static) {
        self.stq.push_back(Box::new(f));
    }
}

trait KtkDisplay {
    fn show(
        &mut self,
        mission: &Mission,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    );

    fn handle_rx(
        &mut self,
        res: eyre::Result<HRes>,
        mission: &Mission,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()>;
}

#[derive(Default, Debug)]
struct Menu {
    selector: String,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TimeInputKind {
    UTSeconds,
    UTDHMS,
    GETDHMS,
}

mod parse {
    use nom::branch::alt;
    use nom::bytes::complete::tag;
    use nom::character::complete::{char, space0, u64};
    use nom::combinator::{eof, opt};
    use nom::sequence::{pair, terminated};
    use nom::IResult;
    use time::Duration;

    fn dot_u64(input: &str) -> IResult<&str, u64> {
        let (input, _) = char('.')(input)?;
        let (input, num) = u64(input)?;
        Ok((input, num))
    }

    fn colon_u64(input: &str) -> IResult<&str, u64> {
        let (input, _) = char(':')(input)?;
        let (input, num) = u64(input)?;
        Ok((input, num))
    }

    pub fn parse_dhms_duration(input: &str, allow_neg: bool) -> IResult<&str, Duration> {
        if input.is_empty() {
            return Err(nom::Err::Failure(nom::error::make_error(
                input,
                nom::error::ErrorKind::Eof,
            )));
        }

        let (input, neg) = if allow_neg {
            opt(tag("-"))(input)?
        } else {
            (input, None)
        };
        let (input, days) = opt(terminated(u64, pair(tag("d"), space0)))(input)?;
        let (input, hours) = opt(terminated(u64, pair(tag("h"), space0)))(input)?;
        let (input, mins) = opt(terminated(u64, pair(alt((tag("min"), tag("m"))), space0)))(input)?;
        let (input, seconds) = opt(pair(
            opt(terminated(u64, tag("."))),
            terminated(u64, pair(tag("s"), space0)),
        ))(input)?;
        let (input, _) = eof(input)?;

        let days = days.unwrap_or(0);
        let hours = hours.unwrap_or(0);
        let mins = mins.unwrap_or(0);
        let (n1, n2) = seconds.unwrap_or((None, 0));
        let (seconds, millis) = if let Some(n1) = n1 { (n1, n2) } else { (n2, 0) };

        let seconds = neg.map(|_| -1.0).unwrap_or(1.0)
            * (days as f64 * 60.0 * 60.0 * 24.0
                + hours as f64 * 60.0 * 60.0
                + mins as f64 * 60.0
                + seconds as f64
                + millis as f64 / 1000.0);
        Ok((input, Duration::seconds_f64(seconds)))
    }

    pub fn parse_sec_time(input: &str, allow_neg: bool) -> IResult<&str, Duration> {
        if input.is_empty() {
            return Err(nom::Err::Failure(nom::error::make_error(
                input,
                nom::error::ErrorKind::Eof,
            )));
        }

        let (input, neg) = if allow_neg {
            opt(tag("-"))(input)?
        } else {
            (input, None)
        };
        let (input, sec) = opt(u64)(input)?;
        let (input, millis) = opt(dot_u64)(input)?;
        let (input, _) = eof(input)?;

        let seconds = neg.map(|_| -1.0).unwrap_or(1.0)
            * (millis.map(|millis| millis as f64 / 1000.0).unwrap_or(0.0)
                + sec.unwrap_or(0) as f64);

        Ok((input, Duration::seconds_f64(seconds)))
    }

    pub fn parse_dhms_time(input: &str, allow_neg: bool) -> IResult<&str, Duration> {
        let (input, neg) = if allow_neg {
            opt(tag("-"))(input)?
        } else {
            (input, None)
        };
        let (input, n1) = u64(input)?;
        let (input, _) = char(':')(input)?;
        let (input, n2) = u64(input)?;
        let (input, _) = char(':')(input)?;
        let (input, n3) = u64(input)?;
        let (input, n4) = opt(colon_u64)(input)?;
        let (input, millis) = opt(dot_u64)(input)?;

        let (days, hours, minutes, seconds) = if let Some(n4) = n4 {
            (Some(n1), n2, n3, n4)
        } else {
            (None, n1, n2, n3)
        };

        let seconds = neg.map(|_| -1.0).unwrap_or(1.0)
            * (millis.map(|millis| millis as f64 / 1000.0).unwrap_or(0.0)
                + seconds as f64
                + minutes as f64 * 60.0
                + hours as f64 * 60.0 * 60.0
                + days
                    .map(|days| days as f64 * 60.0 * 60.0 * 24.0)
                    .unwrap_or(0.0));
        Ok((input, Duration::seconds_f64(seconds)))
    }
}

impl TimeInputKind {
    pub fn parse(self, s: &str) -> Option<UTorGET> {
        match self {
            TimeInputKind::UTSeconds => s
                .parse::<f64>()
                .ok()
                .map(|x| UTorGET::UT(UT::from_duration(Duration::seconds_f64(x)))),
            TimeInputKind::UTDHMS => parse::parse_dhms_time(s, false)
                .ok()
                .map(|x| UTorGET::UT(UT::from_duration(x.1))),
            TimeInputKind::GETDHMS => parse::parse_dhms_time(s, false)
                .ok()
                .map(|x| UTorGET::GET(GET::from_duration(x.1))),
        }
    }

    pub fn format(self, t: UT, _get_base: Option<UT>) -> String {
        match self {
            TimeInputKind::UTSeconds => format!("{}", t.into_duration().as_seconds_f64()),
            TimeInputKind::UTDHMS => {
                format!(
                    "{}:{:02}:{:02}:{:02}.{:03}",
                    t.days(),
                    t.hours(),
                    t.minutes(),
                    t.seconds(),
                    t.millis()
                )
            }
            TimeInputKind::GETDHMS => todo!(),
        }
    }
}

impl Default for TimeInputKind {
    fn default() -> Self {
        Self::UTDHMS
    }
}

impl fmt::Display for TimeInputKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}",
            match self {
                TimeInputKind::UTSeconds => i18n!("time-ut"),
                TimeInputKind::UTDHMS => i18n!("time-ut-dhms"),
                TimeInputKind::GETDHMS => i18n!("time-get-dhms"),
            }
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum UTorGET {
    UT(UT),
    GET(GET),
}

#[derive(Debug)]
struct MissionPlanTable {
    ui_id: egui::Id,
    vessel: Option<VesselRef>,
}

impl Default for MissionPlanTable {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
            vessel: None,
        }
    }
}

impl KtkDisplay for MissionPlanTable {
    fn show(
        &mut self,
        mission: &Mission,
        _toasts: &mut Toasts,
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
                    'slot: {
                        let Some(vessel) = self.vessel.clone() else {
                            break 'slot;
                        };
                        let Some(vessel_id) = mission
                            .vessels
                            .iter()
                            .find_map(|(id, x)| Arc::ptr_eq(x, &vessel.0).then_some(id))
                        else {
                            break 'slot;
                        };

                        let Some(plan) = mission.plan.get(&vessel_id) else {
                            let vessel1 = vessel.clone();
                            backend.effect(move |mission| {
                                mission.plan.insert(
                                    vessel_id,
                                    MissionPlan {
                                        maneuvers: HashMap::new(),
                                        anchor_vector_slot: "".into(),
                                        vessel: vessel1,
                                    },
                                );
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
                            backend.effect(move |mission| {
                                mission
                                    .plan
                                    .get_mut(&vessel_id)
                                    .expect("plan deleted")
                                    .anchor_vector_slot = av_slot;
                                Ok(())
                            });
                        };
                    }
                });
                ui.horizontal(|ui| 'status: {
                    ui.label(i18n!("mpt-status"));
                    let Some(vessel) = self.vessel.clone() else {
                        ui.strong(i18n!("mpt-status-no-vessel"));
                        break 'status;
                    };
                    let Some(vessel_id) = mission
                        .vessels
                        .iter()
                        .find_map(|(id, x)| Arc::ptr_eq(x, &vessel.0).then_some(id))
                    else {
                        ui.strong(i18n!("mpt-status-no-vessel"));
                        break 'status;
                    };
                    let Some(plan) = mission.plan.get(&vessel_id) else {
                        ui.strong(i18n!("mpt-status-no-vessel"));
                        break 'status;
                    };
                    let Ok(_sv) = find_sv(self.vessel.as_ref(), &plan.anchor_vector_slot) else {
                        ui.strong(i18n!("mpt-status-missing-av"));
                        break 'status;
                    };
                    // TODO: partial active
                    ui.strong(i18n!("mpt-status-active"));
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
                        egui::Layout::default()
                            .with_cross_align(egui::Align::RIGHT)
                            .with_main_align(egui::Align::Center)
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
                        let Some(vessel) = self.vessel.clone() else {
                            break 'display;
                        };
                        let Some(vessel_id) = mission
                            .vessels
                            .iter()
                            .find_map(|(id, x)| Arc::ptr_eq(x, &vessel.0).then_some(id))
                        else {
                            break 'display;
                        };

                        let Some(plan) = mission.plan.get(&vessel_id) else {
                            break 'display;
                        };
                        let Ok(_sv) = find_sv(self.vessel.as_ref(), &plan.anchor_vector_slot)
                        else {
                            break 'display;
                        };

                        let maneuvers: Vec<(_, _)> = plan
                            .maneuvers
                            .iter()
                            .sorted_by_key(|x| x.1.inner.geti.into_duration())
                            .collect();
                        for (ix, &(code, maneuver)) in maneuvers.iter().enumerate() {
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
                                    let n = if geti.is_negative() { "-" } else { "" };
                                    ui.add(
                                        egui::Label::new(format!(
                                            "{n}{d:03}:{h:02}:{m:02}:{s:02}.{ms:03}",
                                        ))
                                        .wrap(false),
                                    );
                                });
                                row.col(|ui| {
                                    if ix != 0 {
                                        let (_, prev_mnv) = maneuvers[ix - 1];
                                        let deltat = maneuver.inner.geti - prev_mnv.inner.geti;
                                        let (h, m) = (
                                            deltat.whole_hours(),
                                            deltat.whole_minutes().unsigned_abs() % 60,
                                        );

                                        ui.add(
                                            egui::Label::new(format!("{}:{:02}", h, m)).wrap(false),
                                        );
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
                                row.col(|_ui| {
                                    // ui.add(
                                    //     egui::Label::new(format!("{:.1}", maneuver.ha)).wrap(false),
                                    // );
                                });
                                row.col(|_ui| {
                                    // ui.add(
                                    //     egui::Label::new(format!("{:.1}", maneuver.hp)).wrap(false),
                                    // );
                                });
                                row.col(|ui| {
                                    ui.add(egui::Label::new(code).wrap(false));
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

#[derive(Default)]
struct Displays {
    syscfg: bool,
    krpc: bool,
    logs: bool,
    mpt: bool,
    vc: bool,
    vps: bool,
    classes: bool,
    vessels: bool,
    tliproc: bool,
    mpt_trfr: bool,
    time_utils: bool,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, FromPrimitive)]
#[repr(u16)]
pub enum DisplaySelect {
    SysCfg = 0,
    Krpc = 1,
    Logs = 2,
    TimeUtils = 3,
    MPT = 100,
    MPTTransfer = 101,
    VC = 200,
    VPS = 201,
    Classes = 300,
    Vessels = 301,
    TLIProcessor = 400,
    #[default]
    Unknown = u16::MAX,
}

#[derive(Default)]
struct Logs {}

impl KtkDisplay for Logs {
    fn show(
        &mut self,
        _mission: &Mission,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    ) {
        egui::Window::new(i18n!("logs-title"))
            .open(&mut open.logs)
            .show(ctx, |_ui| {});
    }

    fn handle_rx(
        &mut self,
        _res: eyre::Result<HRes>,
        _mission: &Mission,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
        _ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()> {
        Ok(())
    }
}

pub struct MPTTransfer {
    ui_id: egui::Id,
    vessel: Option<VesselRef>,
    mnv: Option<(Maneuver, String)>,
    vessel_engines: HashMap<PartId, bool>,
    fuel_stats: (f64, f64, f64, f64),
}

impl Default for MPTTransfer {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
            vessel: None,
            mnv: None,
            vessel_engines: HashMap::new(),
            fuel_stats: (0.0, 0.0, 0.0, 0.0),
        }
    }
}

impl MPTTransfer {
    fn run_ffs(
        &mut self,
        class: &Arc<RwLock<VesselClass>>,
        vessel: &Arc<RwLock<Vessel>>,
        mnv: Maneuver,
    ) {
        // TODO: shunt this to the handler thread
        // TODO: impl Default for FuelFlowSimulation
        let mut ffs = FuelFlowSimulation {
            segments: vec![],
            current_segment: FuelStats::default(),
            time: 0.0,
            dv_linear_thrust: false,
            parts_with_resource_drains: HashSet::new(),
            sources: vec![],
        };

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

        for (pid, part) in class.read().parts.iter() {
            let mut modules_current_mass = 0.0;
            if !part.mass_modifiers.is_empty() {
                for modifier in &part.mass_modifiers {
                    modules_current_mass += modifier.current_mass;
                }
            }
            let resources = vessel
                .read()
                .resources
                .iter()
                .filter_map(|x| (x.0 .0 == pid).then_some((x.0 .1, x.1.clone())))
                .collect::<HashMap<_, _>>();
            let disabled_resource_mass = resources.iter().fold(0.0, |acc, x| {
                if !x.1.enabled {
                    acc + x.1.density * x.1.amount
                } else {
                    acc
                }
            });
            let sid = sim_vessel.parts.push(SimPart {
                crossfeed_part_set: vec![],
                resources,
                resource_drains: HashMap::new(),
                resource_priority: part.resource_priority,
                resource_request_remaining_threshold: part.resource_request_remaining_threshold,
                mass: part.mass,
                dry_mass: part.dry_mass,
                crew_mass: part.crew_mass,
                modules_current_mass,
                disabled_resource_mass,
                is_launch_clamp: part.is_launch_clamp,
            });
            if self.vessel_engines.get(&pid).copied().unwrap_or(false) {
                sim_vessel
                    .active_engines
                    .extend(part.engines.iter().cloned().map(|mut x| {
                        x.part = sid;
                        x
                    }));
            }
            map.insert(pid, sid);
        }
        for (pid, sid) in &map {
            sim_vessel.parts[*sid].crossfeed_part_set = class.read().parts[*pid]
                .crossfeed_part_set
                .iter()
                .flat_map(|x| map.get(x).copied())
                .collect();
        }

        ffs.run(&mut sim_vessel);

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

        self.fuel_stats = (dvrem, start_mass_tons, end_mass / 1000.0, bt);
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
                            .selected_text(
                                self.vessel
                                    .clone()
                                    .map(|x| x.0.read().name.clone())
                                    .unwrap_or_else(|| i18n!("mpt-trfr-no-vessel")),
                            )
                            .show_ui(ui, |ui| {
                                for iter_vessel in mission
                                    .vessels
                                    .iter()
                                    .map(|(_, x)| x)
                                    .sorted_by_key(|x| x.read().name.clone())
                                {
                                    if ui
                                        .selectable_value(
                                            &mut self.vessel,
                                            Some(VesselRef(iter_vessel.clone())),
                                            &iter_vessel.read().name,
                                        )
                                        .clicked()
                                    {
                                        self.vessel_engines.clear();
                                    };
                                }
                            });
                    });
                    ui.separator();
                    ui.heading(i18n!("mpt-trfr-mnvinfo"));
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
                                ui.monospace(code);
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
                        let Some(VesselRef(vessel)) = self.vessel.clone() else {
                            break 'engines;
                        };
                        let Some(VesselClassRef(class)) = vessel.read().class.clone() else {
                            break 'engines;
                        };
                        let Some((mnv, code)) = self.mnv.clone() else {
                            break 'engines;
                        };

                        // TODO: multiple engines per part
                        for (partid, part) in class
                            .read()
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

                        let engines = class
                            .read()
                            .parts
                            .iter()
                            .filter_map(|(x, _)| {
                                self.vessel_engines
                                    .get(&x)
                                    .copied()
                                    .unwrap_or(false)
                                    .then_some(x)
                            })
                            .collect::<Vec<_>>();

                        if ui.button(i18n!("mpt-trfr-calc-fuel")).clicked() {
                            self.run_ffs(&class, &vessel, mnv.clone());
                        }
                        ui.horizontal(|ui| {
                            ui.vertical(|ui| {
                                ui.monospace(i18n!("mpt-trfr-dvrem"));
                                ui.monospace(i18n!("mpt-trfr-startmass"));
                                ui.monospace(i18n!("mpt-trfr-endmass"));
                                ui.monospace(i18n!("mpt-trfr-burntime"));
                            });
                            ui.vertical(|ui| {
                                let (deltav, start_mass_tons, end_mass_tons, bt) = self.fuel_stats;
                                ui.monospace(format!("{deltav:.2}"));
                                ui.monospace(format!("{start_mass_tons:.3}"));
                                ui.monospace(format!("{end_mass_tons:.3}"));
                                ui.monospace(format!("{bt:.3}"));
                            });
                        });
                        if ui.button(i18n!("mpt-trfr-trfr")).clicked() {
                            if self.fuel_stats == (0.0, 0.0, 0.0, 0.0) {
                                self.run_ffs(&class, &vessel, mnv.clone());
                            }

                            handle(toasts, |_| {
                                let vessel_id = mission
                                    .vessels
                                    .iter()
                                    .find_map(|(id, x)| Arc::ptr_eq(x, &vessel).then_some(id))
                                    .expect("vessel not in mission");
                                let fuel_stats = self.fuel_stats;
                                backend.effect(move |mission| {
                                    let plan = mission
                                        .plan
                                        .get_mut(&vessel_id)
                                        .ok_or_eyre(i18n!("error-mpt-no-init"))?;
                                    plan.maneuvers.insert(
                                        code,
                                        PlannedManeuver {
                                            inner: mnv.clone(),
                                            engines,
                                            dvrem: fuel_stats.0,
                                        },
                                    );
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
        if let Ok(HRes::MPTTransfer(mnv, code)) = res {
            self.mnv = Some((mnv, code));
            Ok(())
        } else {
            res.map(|_| ())
        }
    }
}

pub struct TLIProcessor {
    ui_id: egui::Id,
    loading: u8,
    sv_vessel: Option<VesselRef>,
    sv_slot: String,
    moon: String,
    maxiter: u64,
    temp: f64,
    ft_min: String,
    ft_min_p: Option<Duration>,
    ft_max: String,
    ft_max_p: Option<Duration>,
    ct_min: String,
    ct_min_p: Option<Duration>,
    ct_max: String,
    ct_max_p: Option<Duration>,
    pe_min: f64,
    pe_max: f64,
    opt_periapse: bool,
    mnvs: Vec<Option<(u64, Maneuver)>>,
    mnv_ctr: u64,
    allow_retrograde: bool,
}

impl Default for TLIProcessor {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
            loading: 0,
            sv_vessel: None,
            sv_slot: "".into(),
            moon: i18n!("tliproc-no-moon"),
            maxiter: 100_000,
            temp: 100.0,
            ft_min: "".into(),
            ft_min_p: None,
            ft_max: "".into(),
            ft_max_p: None,
            ct_min: "".into(),
            ct_min_p: None,
            ct_max: "".into(),
            ct_max_p: None,
            pe_min: 0.0,
            pe_max: 0.0,
            opt_periapse: true,
            mnvs: vec![],
            mnv_ctr: 1,
            allow_retrograde: false,
        }
    }
}

pub fn find_sv(sv_vessel: Option<&VesselRef>, sv_slot: &str) -> eyre::Result<StateVector> {
    let vessel = sv_vessel.ok_or_eyre(i18n!("error-no-vessel"))?;
    let vessel = vessel.0.read();
    vessel
        .svs
        .get(sv_slot)
        .ok_or_eyre(i18n!("error-no-sv-in-slot"))
        .cloned()
}

impl KtkDisplay for TLIProcessor {
    fn show(
        &mut self,
        mission: &Mission,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    ) {
        egui::Window::new(i18n!("tliproc-title"))
            .open(&mut open.tliproc)
            .default_size([512.0, 512.0])
            .show(ctx, |ui| {
                let mut moon_soi = None;
                let mut moon_radius = None;
                ui.horizontal(|ui| {
                    ui.add(VectorSelector::new(
                        self.ui_id.with("SVSel"),
                        i18n!("tliproc-parking-sv"),
                        mission,
                        &mut self.sv_vessel,
                        &mut self.sv_slot,
                    ));
                });
                if let Ok(sv) = find_sv(self.sv_vessel.as_ref(), &self.sv_slot) {
                    ui.horizontal(|ui| {
                        ui.label(i18n!("tliproc-central-body"));
                        ui.strong(&*sv.body.name);
                        ui.separator();
                        ui.label(i18n!("tliproc-moon-body"));
                        egui::ComboBox::from_id_source(self.ui_id.with("Moon"))
                            .selected_text(&self.moon)
                            .show_ui(ui, |ui| {
                                for moon in &*sv.body.satellites {
                                    ui.selectable_value(&mut self.moon, moon.to_string(), &**moon);
                                }
                            });
                    });
                }
                handle(toasts, |_| {
                    if let Ok(sv) = find_sv(self.sv_vessel.as_ref(), &self.sv_slot) {
                        if let Some(moon) = mission.system.bodies.get(&*self.moon) {
                            moon_soi = Some(moon.soi);
                            moon_radius = Some(moon.radius);
                            let r2 = moon.ephem.apoapsis_radius();
                            let obt = sv.clone().into_orbit(1e-8);
                            let r1 = obt.periapsis_radius();

                            let at = 0.5 * (r1 + r2);
                            let t12 = consts::PI * libm::sqrt(at.powi(3) / sv.body.mu);

                            ui.horizontal(|ui| {
                                ui.label(i18n!("tliproc-hohmann-tof"));
                                let t = UT::new_seconds(t12);
                                let (d, h, m, s, ms) = (
                                    t.days().abs(),
                                    t.hours(),
                                    t.minutes(),
                                    t.seconds(),
                                    t.millis(),
                                );
                                let n = if t.is_negative() { "-" } else { "" };
                                ui.strong(format!("{n}{d}d {h:>02}h {m:>02}m {s:>02}.{ms:>03}s"));
                            });
                        }
                    }
                    Ok(())
                });
                ui.separator();

                ui.horizontal(|ui| {
                    ui.with_layout(
                        egui::Layout::top_down(egui::Align::LEFT)
                            .with_main_justify(true)
                            .with_main_align(egui::Align::Center),
                        |ui| {
                            let spacing = ui.spacing().interact_size.y
                                - ui.text_style_height(&egui::TextStyle::Body);
                            ui.spacing_mut().item_spacing.y += spacing;

                            ui.label(i18n!("tliproc-iter-fac"));
                            ui.label(i18n!("tliproc-iter-max"));
                            ui.label(i18n!("tliproc-flight-time"));
                            ui.label(i18n!("tliproc-coast-time"));
                            ui.label(i18n!("tliproc-periapse"));
                        },
                    );

                    ui.vertical(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.temp)
                                .clamp_range(0.1..=1000.0)
                                .max_decimals(2),
                        );
                        ui.add(
                            egui::DragValue::new(&mut self.maxiter).clamp_range(10_000..=1_000_000),
                        );

                        ui.horizontal(|ui| {
                            ui.add(DurationInput::new(
                                &mut self.ft_min,
                                &mut self.ft_min_p,
                                Some(128.0),
                                true,
                                false,
                            ));
                            ui.label(i18n!("tliproc-to"));
                            ui.add(DurationInput::new(
                                &mut self.ft_max,
                                &mut self.ft_max_p,
                                Some(128.0),
                                true,
                                false,
                            ));
                        });
                        ui.horizontal(|ui| {
                            ui.add(DurationInput::new(
                                &mut self.ct_min,
                                &mut self.ct_min_p,
                                Some(128.0),
                                true,
                                false,
                            ));
                            ui.label(i18n!("tliproc-to"));
                            ui.add(DurationInput::new(
                                &mut self.ct_max,
                                &mut self.ct_max_p,
                                Some(128.0),
                                true,
                                false,
                            ));
                        });

                        ui.horizontal(|ui| {
                            ui.add(
                                egui::DragValue::new(&mut self.pe_min)
                                    .clamp_range(
                                        -moon_radius.unwrap_or(0.0)
                                            ..=moon_soi.unwrap_or(0.0).floor(),
                                    )
                                    .suffix("km")
                                    .max_decimals(3),
                            );
                            ui.label(i18n!("tliproc-to"));
                            ui.add(
                                egui::DragValue::new(&mut self.pe_max)
                                    .clamp_range(
                                        -moon_radius.unwrap_or(0.0)
                                            ..=moon_soi.unwrap_or(0.0).floor(),
                                    )
                                    .suffix("km")
                                    .max_decimals(3),
                            );
                            ui.label(i18n!("tliproc-opt-periapse"));
                            ui.add(egui::Checkbox::without_text(&mut self.opt_periapse));
                            ui.label(i18n!("tliproc-allow-retro"));
                            ui.add(egui::Checkbox::without_text(&mut self.allow_retrograde));
                        });
                    });
                });
                ui.horizontal(|ui| {
                    if ui.button(i18n!("tliproc-calc")).clicked() {
                        handle(toasts, |_| {
                            let sv = find_sv(self.sv_vessel.as_ref(), &self.sv_slot)?;
                            let vessel = self
                                .sv_vessel
                                .clone()
                                .ok_or_eyre(i18n!("error-no-vessel"))?;
                            let moon = mission
                                .system
                                .bodies
                                .get(&*self.moon)
                                .ok_or_eyre(i18n!("error-body-no-load"))?
                                .clone();

                            let central = sv.body.clone();
                            backend.tx(
                                DisplaySelect::TLIProcessor,
                                HReq::CalculateTLI(Box::new(TLIInputs {
                                    cs: TLIConstraintSet {
                                        central_sv: sv,
                                        flight_time: self.ft_min_p.unwrap_or_default()
                                            ..self.ft_max_p.unwrap_or_default(),
                                        moon_periapse_radius: (moon.radius + self.pe_min)
                                            ..(moon.radius + self.pe_max),
                                        coast_time: self.ct_min_p.unwrap_or_default()
                                            ..self.ct_max_p.unwrap_or_default(),
                                    },
                                    central,
                                    moon,
                                    get_base: vessel.0.read().get_base,
                                    maxiter: self.maxiter,
                                    temp: self.temp,
                                    opt_periapse: self.opt_periapse,
                                    allow_retrograde: self.allow_retrograde,
                                })),
                            )?;
                            self.loading = 1;
                            Ok(())
                        });
                    }

                    if self.loading == 1 {
                        ui.spinner();
                    }
                });

                ui.separator();

                self.mnvs.retain(|x| x.is_some());
                egui_extras::TableBuilder::new(ui)
                    .striped(true)
                    .column(Column::auto_with_initial_suggestion(96.0).resizable(true))
                    .column(Column::auto_with_initial_suggestion(128.0).resizable(true))
                    .columns(
                        Column::auto_with_initial_suggestion(72.0).resizable(true),
                        4,
                    )
                    .cell_layout(
                        egui::Layout::default()
                            .with_cross_align(egui::Align::RIGHT)
                            .with_main_align(egui::Align::Center)
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
                                        egui::RichText::new(i18n!("tliproc-code")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });

                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-geti")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });

                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-dv-prograde")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-dv-normal")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-dv-radial")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-dv-total")).heading(),
                                    )
                                    .wrap(false),
                                );
                            });
                        });
                    })
                    .body(|body| {
                        body.rows(24.0, self.mnvs.len(), |mut row| {
                            let ix = row.index();
                            if let Some((code, mnv)) = self.mnvs[ix].clone() {
                                row.col(|ui| {
                                    ui.horizontal(|ui| {
                                        ui.monospace(format!("0400C/{code:02}"));

                                        if ui
                                            .button(icon("\u{f506}"))
                                            .on_hover_text(i18n!("transfer-to-mpt"))
                                            .clicked()
                                        {
                                            open.mpt_trfr = true;
                                            handle(toasts, |_| {
                                                backend.tx_loopback(
                                                    DisplaySelect::MPTTransfer,
                                                    Ok(HRes::MPTTransfer(
                                                        mnv.clone(),
                                                        format!("0400C/{code:02}"),
                                                    )),
                                                )?;
                                                Ok(())
                                            });
                                        };

                                        if ui
                                            .button(icon("\u{e872}"))
                                            .on_hover_text(i18n!("tliproc-delete"))
                                            .clicked()
                                        {
                                            self.mnvs[ix] = None;
                                        }
                                    });
                                });
                                row.col(|ui| {
                                    let geti = mnv.geti;
                                    let (d, h, m, s, ms) = (
                                        geti.days(),
                                        geti.hours(),
                                        geti.minutes(),
                                        geti.seconds(),
                                        geti.millis(),
                                    );
                                    ui.monospace(format!("{d:03}:{h:02}:{m:02}:{s:02}.{ms:03}"));
                                });
                                row.col(|ui| {
                                    ui.add(egui::Label::new(
                                        egui::RichText::new(format!(
                                            "{:>+08.2}",
                                            mnv.deltav.x * 1000.0
                                        ))
                                        .color(egui::Color32::from_rgb(0, 214, 0))
                                        .monospace(),
                                    ));
                                });
                                row.col(|ui| {
                                    ui.add(egui::Label::new(
                                        egui::RichText::new(format!(
                                            "{:>+08.2}",
                                            mnv.deltav.y * 1000.0
                                        ))
                                        .color(egui::Color32::from_rgb(214, 0, 214))
                                        .monospace(),
                                    ));
                                });
                                row.col(|ui| {
                                    ui.add(egui::Label::new(
                                        egui::RichText::new(format!(
                                            "{:>+08.2}",
                                            mnv.deltav.z * 1000.0
                                        ))
                                        .color(egui::Color32::from_rgb(0, 214, 214))
                                        .monospace(),
                                    ));
                                });
                                row.col(|ui| {
                                    ui.monospace(format!("{:>+08.2}", mnv.deltav.norm() * 1000.0));
                                });
                            }
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
        self.loading = 0;
        if let Ok(HRes::CalculatedManeuver(mnv)) = res {
            self.mnvs.push(Some((self.mnv_ctr, mnv)));
            self.mnv_ctr += 1;
            Ok(())
        } else {
            res.map(|_| ())
        }
    }
}
