#![warn(clippy::unwrap_used, clippy::pedantic)]
#![allow(
    clippy::cast_lossless,
    clippy::cast_possible_truncation,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::must_use_candidate,
    clippy::many_single_char_names,
    clippy::module_name_repetitions,
    clippy::too_many_lines,
    clippy::similar_names,
    clippy::doc_markdown,
    clippy::struct_field_names,
    clippy::struct_excessive_bools
)]
use std::{
    collections::{HashMap, VecDeque},
    f64::consts,
    fmt,
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc,
    },
    thread,
    time::Instant,
};

use backend::{handler_thread, HReq, HRes, TLIInputs, TLMCCInputs};
use color_eyre::eyre::{self, bail, OptionExt};
use egui::TextBuffer;
use egui_extras::Column;
use mpt::{MPTSeparation, MPTTransfer, MissionPlanTable};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
//use tokio::runtime::{self, Runtime};
use egui_notify::Toasts;
use kerbtk::{
    kepler::orbits::{self, Apsis, StateVector},
    maneuver::{gpm, Maneuver},
    time::{GET, UT},
    translunar::TLIConstraintSet,
    vessel::{Vessel, VesselId},
};
use mission::{Mission, MissionRef, NodalTargets, PlannedEvent};
use num_enum::FromPrimitive;
use time::Duration;
use tracing::error;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};
use unic_langid::LanguageIdentifier;
use utils::{KRPCConfig, SystemConfiguration, TimeUtils};
use vectors::{MPTVectorSelector, VectorComparison, VectorPanelSummary};
use vessels::{Classes, Vessels};
use widgets::{
    icon, icon_label, DurationInput, TimeDisplayBtn, TimeDisplayKind, TimeInput1, TimeInputKind2,
};

mod backend;
mod mission;
mod mpt;
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
            toasts.error(format!("{e}"));
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

    let mission = Arc::new(RwLock::new(Mission::default()));
    let handler_mission = MissionRef::new(mission.clone());
    let main_tx_loopback = handler_tx.clone();
    let _ = thread::spawn(|| handler_thread(handler_rx, handler_tx, handler_mission));
    eframe::run_native(
        &i18n!("title"),
        native_options,
        Box::new(|cc| {
            Ok(Box::new(NewApp::new(
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
                mission,
            )))
        }),
    )
    .expect(&i18n!("error-start-failed"));
    std::process::exit(0)
}

struct NewApp {
    mission: Arc<RwLock<Mission>>,
    state: State,
    backend: Backend,
}

impl NewApp {
    fn new(cc: &eframe::CreationContext, backend: Backend, mission: Arc<RwLock<Mission>>) -> Self {
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

        // TODO: toggle
        cc.egui_ctx.set_visuals(egui::Visuals::dark());
        cc.egui_ctx.set_fonts(fonts);

        Self {
            mission,
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
            DisplaySelect::MPTSep => self.state.dis.mpt_sep = true,
            DisplaySelect::VC => self.state.dis.vc = true,
            DisplaySelect::VPS => self.state.dis.vps = true,
            DisplaySelect::Classes => self.state.dis.classes = true,
            DisplaySelect::Vessels => self.state.dis.vessels = true,
            DisplaySelect::TLIProcessor => self.state.dis.tliproc = true,
            DisplaySelect::TLMCCProcessor => self.state.dis.tlmcc = true,
            DisplaySelect::GPM => self.state.dis.gpm = true,
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

#[allow(clippy::enum_glob_use)]
impl eframe::App for NewApp {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        let mission = self.mission.clone();
        let mission = mission.read();

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
                Some(Classes) => handle_rx!(res, self, classes, mission, ctx, frame),
                Some(Vessels) => handle_rx!(res, self, vessels, mission, ctx, frame),
                Some(TLMCCProcessor) => handle_rx!(res, self, tlmcc, mission, ctx, frame),
                Some(MPTSep) => handle_rx!(res, self, mpt_sep, mission, ctx, frame),
                Some(GPM) => handle_rx!(res, self, gpm, mission, ctx, frame),
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
        show_display!(self, tlmcc, mission, ctx, frame);
        show_display!(self, mpt_sep, mission, ctx, frame);
        show_display!(self, gpm, mission, ctx, frame);

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
                                self.backend.effect(|mission, state| {
                                    *mission = new_mission;
                                    state.classes.force_refilter = true;
                                    state.vessels.force_refilter = true;
                                    Ok(())
                                });
                                ctx.request_repaint();
                            }
                        }
                        if ui
                            .button(icon_label("\u{e161}", &i18n!("menu-save-mission")))
                            .clicked()
                        {
                            if let Some(path) = file.save_file() {
                                std::fs::write(
                                    path,
                                    ron::ser::to_string_pretty(
                                        &self.mission,
                                        ron::ser::PrettyConfig::default()
                                            .struct_names(true)
                                            .enumerate_arrays(true),
                                    )
                                    .expect("oops"),
                                )?;
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
                    .add(egui::Button::new(i18n!("menu-organize-windows")).truncate())
                    .clicked()
                {
                    ui.ctx().memory_mut(egui::Memory::reset_areas);
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
                    self.state.dis = Displays::default();
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
                            ui.checkbox(&mut self.state.dis.mpt_sep, i18n!("menu-display-mpt-sep"));
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
                            ui.checkbox(&mut self.state.dis.tlmcc, i18n!("menu-display-tlmcc"));
                            ui.checkbox(&mut self.state.dis.gpm, i18n!("menu-display-gpm"));
                        });
                });
            });

        self.state.toasts.show(ctx);

        drop(mission);
        let mission = self.mission.clone();
        let mut mission = mission.write();
        for st in self.backend.stq.drain(..) {
            let res = (st)(&mut mission, &mut self.state);
            handle(&mut self.state.toasts, move |_| res);
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
    tlmcc: TLMCCProcessor,
    mpt_sep: MPTSeparation,
    gpm: GPM,
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

type StateUpdater = Box<dyn FnOnce(&mut Mission, &mut State) -> eyre::Result<()> + Send + 'static>;

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

    fn effect(
        &mut self,
        f: impl FnOnce(&mut Mission, &mut State) -> eyre::Result<()> + Send + 'static,
    ) {
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

#[allow(clippy::cast_precision_loss)]
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

        let seconds = neg.map_or(1.0, |_| -1.0)
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

        let seconds = neg.map_or(1.0, |_| -1.0)
            * (millis.map_or(0.0, |millis| millis as f64 / 1000.0) + sec.unwrap_or(0) as f64);

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

        let seconds = neg.map_or(1.0, |_| -1.0)
            * (millis.map_or(0.0, |millis| millis as f64 / 1000.0)
                + seconds as f64
                + minutes as f64 * 60.0
                + hours as f64 * 60.0 * 60.0
                + days.map_or(0.0, |days| days as f64 * 60.0 * 60.0 * 24.0));
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

#[derive(Default)]
struct Displays {
    syscfg: bool,
    krpc: bool,
    logs: bool,
    mpt: bool,
    mpt_sep: bool,
    vc: bool,
    vps: bool,
    classes: bool,
    vessels: bool,
    tliproc: bool,
    mpt_trfr: bool,
    time_utils: bool,
    tlmcc: bool,
    gpm: bool,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, FromPrimitive, Serialize, Deserialize)]
#[repr(u16)]
pub enum DisplaySelect {
    SysCfg = 0,
    Krpc = 1,
    Logs = 2,
    TimeUtils = 3,
    MPT = 100,
    MPTTransfer = 101,
    MPTSep = 102,
    VC = 200,
    VPS = 201,
    Classes = 300,
    Vessels = 301,
    TLIProcessor = 400,
    TLMCCProcessor = 401,
    GPM = 402,
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

pub fn find_sv(sv_vessel: Option<&Vessel>, sv_slot: &str) -> eyre::Result<StateVector> {
    let vessel = sv_vessel.ok_or_eyre(i18n!("error-no-vessel"))?;

    vessel
        .svs
        .get(sv_slot)
        .ok_or_eyre(i18n!("error-no-sv-in-slot"))
        .cloned()
}

pub fn find_sv_mpt(
    sv_vessel_id: Option<VesselId>,
    sv_time: Option<UTorGET>,
    mission: &Mission,
) -> eyre::Result<StateVector> {
    let vessel_id = sv_vessel_id.ok_or_eyre(i18n!("error-no-vessel"))?;
    let sv_vessel = &mission.vessels[vessel_id];
    let plan = mission
        .plan
        .get(&vessel_id)
        .ok_or_eyre(i18n!("error-mpt-no-init"))?;

    let av = sv_vessel
        .svs
        .get(&plan.anchor_vector_slot)
        .ok_or_eyre(i18n!("error-no-sv-in-slot"))
        .cloned()?;
    let sv_time = sv_time.ok_or_eyre(i18n!("error-no-anchor-time"))?;
    let mut time = match sv_time {
        UTorGET::UT(ut) => ut,
        UTorGET::GET(get) => {
            UT::from_duration(get.into_duration() + sv_vessel.get_base.into_duration())
        }
    };

    if time < av.time {
        time = av.time;
    }

    if time == av.time {
        return Ok(av);
    }

    let maneuvers = &plan.events;
    let ix = maneuvers.binary_search_by_key(&time.into_duration(), |mnv| {
        mnv.get().into_duration() + sv_vessel.get_base.into_duration()
    });
    match ix {
        Ok(ix) if matches!(&maneuvers[ix], PlannedEvent::Maneuver(_)) => {
            let PlannedEvent::Maneuver(mnv) = &maneuvers[ix] else {
                unreachable!();
            };
            let mut sv = mnv.inner.tig_vector.clone();
            sv.velocity += mnv.inner.deltav_bci();
            Ok(sv)
        }
        Ok(ix) | Err(ix) => {
            if let Some(mnv) = maneuvers[..ix].iter().rev().find_map(PlannedEvent::mnv_ref) {
                let tig_ut = mnv.inner.tig_vector.time;
                let mut sv = mnv.inner.tig_vector.clone();
                sv.velocity += mnv.inner.deltav_bci();
                Ok(sv
                    .propagate_with_soi(&mission.system, time - tig_ut, 1e-7, 30000)
                    .ok_or_eyre(i18n!("error-calc-general"))?)
            } else {
                let av_time = av.time;
                Ok(av
                    .propagate_with_soi(&mission.system, time - av_time, 1e-7, 30000)
                    .ok_or_eyre(i18n!("error-calc-general"))?)
            }
        }
    }
}

pub struct TLMCCProcessor {
    ui_id: egui::Id,
    loading: u8,
    sv: MPTVectorSelector,
    moon: String,
    mnvs: Vec<Option<(u64, Maneuver)>>,
    mnv_ctr: u64,
    opt: TLMCCOption,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TLMCCOption {
    NodalTargeting,
    FixedEllipseLOI,
}

impl fmt::Display for TLMCCOption {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TLMCCOption::NodalTargeting => write!(f, "{}", i18n!("tlmcc-opt-1")),
            TLMCCOption::FixedEllipseLOI => write!(f, "{}", i18n!("tlmcc-opt-2")),
        }
    }
}

impl Default for TLMCCProcessor {
    fn default() -> Self {
        let ui_id = egui::Id::new(Instant::now());
        Self {
            ui_id,
            loading: 0,
            sv: MPTVectorSelector::new(ui_id, TimeInputKind2::GET, TimeDisplayKind::Dhms),
            moon: i18n!("tliproc-no-moon"),
            mnvs: vec![],
            mnv_ctr: 1,
            opt: TLMCCOption::FixedEllipseLOI,
        }
    }
}

impl KtkDisplay for TLMCCProcessor {
    fn show(
        &mut self,
        mission: &Mission,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    ) {
        egui::Window::new(i18n!("tlmcc-title"))
            .open(&mut open.tlmcc)
            .default_size([512.0, 512.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    self.sv.show(ui, i18n!("tlmcc-sv"), mission);
                });
                let vessel = self.sv.vessel.map(|x| (x, &mission.vessels[x]));
                if let Ok(sv) = find_sv_mpt(self.sv.vessel, self.sv.time_parsed, mission) {
                    ui.horizontal(|ui| {
                        ui.label(i18n!("tlmcc-central-body"));
                        ui.strong(&*sv.body.name);
                        ui.separator();
                        ui.label(i18n!("tlmcc-moon-body"));
                        egui::ComboBox::from_id_salt(self.ui_id.with("Moon"))
                            .selected_text(&self.moon)
                            .show_ui(ui, |ui| {
                                for moon in &*sv.body.satellites {
                                    ui.selectable_value(&mut self.moon, moon.to_string(), &**moon);
                                }
                            });
                    });
                }
                // if let Ok(sv) = find_sv_mpt(self.sv_vessel, self.sv_time_parsed, mission) {

                // }
                ui.separator();
                ui.horizontal(|ui| {
                    ui.with_layout(
                        egui::Layout::top_down(egui::Align::LEFT)
                            .with_main_justify(true)
                            .with_main_align(egui::Align::Center),
                        |ui| {
                            let spacing = ui.spacing().interact_size.y
                                - ui.text_style_height(&egui::TextStyle::Body);
                            //ui.spacing_mut().item_spacing.y += spacing;

                            ui.label(i18n!("tlmcc-opt"));
                            match self.opt {
                                TLMCCOption::NodalTargeting => {
                                    ui.add_space(spacing);
                                    ui.label(i18n!("tlmcc-nt-refmnv"));
                                }
                                _ => {}
                            }
                        },
                    );

                    ui.vertical(|ui| {
                        let spacing = ui.spacing().interact_size.y
                            - ui.text_style_height(&egui::TextStyle::Monospace);
                        let _spacing_body = ui.spacing().interact_size.y
                            - ui.text_style_height(&egui::TextStyle::Body);
                        //ui.spacing_mut().item_spacing.y += spacing;

                        egui::ComboBox::from_id_salt(self.ui_id.with("Option"))
                            .selected_text(self.opt.to_string())
                            .show_ui(ui, |ui| {
                                ui.selectable_value(
                                    &mut self.opt,
                                    TLMCCOption::NodalTargeting,
                                    i18n!("tlmcc-opt-1"),
                                );
                                ui.selectable_value(
                                    &mut self.opt,
                                    TLMCCOption::FixedEllipseLOI,
                                    i18n!("tlmcc-opt-2"),
                                );
                            });

                        match self.opt {
                            TLMCCOption::NodalTargeting => 'nt: {
                                ui.add_space(spacing);
                                let Some(vessel_id) = self.sv.vessel else {
                                    ui.monospace(i18n!("tlmcc-nt-nomnv"));
                                    ui.add_space(spacing);
                                    break 'nt;
                                };
                                let Some(plan) = mission.plan.get(&vessel_id) else {
                                    ui.monospace(i18n!("tlmcc-nt-nomnv"));
                                    ui.add_space(spacing);
                                    break 'nt;
                                };
                                let Some(NodalTargets::Translunar { mnv_base_code, .. }) =
                                    &plan.nodal_targets
                                else {
                                    ui.monospace(i18n!("tlmcc-nt-nomnv"));
                                    ui.add_space(spacing);
                                    break 'nt;
                                };
                                ui.monospace(mnv_base_code);
                                ui.add_space(spacing);
                            }
                            _ => {}
                        }
                    });
                });
                ui.horizontal(|ui| {
                    if ui.button(i18n!("tlmcc-calc")).clicked() {
                        handle(toasts, |_| {
                            let sv = find_sv_mpt(self.sv.vessel, self.sv.time_parsed, mission)?;
                            let (vessel_id, vessel) =
                                vessel.ok_or_eyre(i18n!("error-no-vessel"))?;
                            let moon = mission
                                .system
                                .bodies
                                .get(&*self.moon)
                                .ok_or_eyre(i18n!("error-body-no-load"))?
                                .clone();

                            let central = sv.body.clone();
                            match self.opt {
                                TLMCCOption::NodalTargeting => {
                                    let _plan = mission
                                        .plan
                                        .get(&vessel_id)
                                        .ok_or_eyre(i18n!("error-mpt-no-init"))?;
                                    // let Some(NodalTargets::Translunar {
                                    //     soi_ut,
                                    //     lat_pe,
                                    //     lng_pe,
                                    //     h_pe,
                                    //     i,
                                    //     ..
                                    // }) = &plan.nodal_targets
                                    // else {
                                    //     bail!("{}", i18n!("error-tlmcc-no-targets"));
                                    // };
                                    backend.tx(
                                        DisplaySelect::TLMCCProcessor,
                                        HReq::CalculateTLMCC(Box::new(
                                            TLMCCInputs::NodalTargeting {
                                                sv_cur: sv,
                                                central,
                                                get_base: vessel.get_base,
                                                moon,
                                                soi_ut: UT::new_seconds(0.0), //*soi_ut,
                                                lat_pe: 0.0,                  //*lat_pe,
                                                lng_pe: 0.0,                  //*lng_pe,
                                                h_pe: 0.0,                    //*h_pe,
                                                i: 0.0,                       //*i,
                                            },
                                        )),
                                    )?;
                                }
                                TLMCCOption::FixedEllipseLOI => todo!(),
                            }
                            self.loading = 1;
                            Ok(())
                        });
                    }

                    if self.loading == 1 {
                        ui.spinner();
                    }
                });

                ui.separator();

                self.mnvs.retain(Option::is_some);
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
                                    .extend(),
                                );
                            });
                        });

                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-geti")).heading(),
                                    )
                                    .extend(),
                                );
                            });
                        });

                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-dv-prograde")).heading(),
                                    )
                                    .extend(),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-dv-normal")).heading(),
                                    )
                                    .extend(),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-dv-radial")).heading(),
                                    )
                                    .extend(),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-dv-total")).heading(),
                                    )
                                    .extend(),
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
                                        ui.monospace(format!("0401C/{code:02}"));

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
                                                        DisplaySelect::TLMCCProcessor,
                                                        code,
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

pub struct TLIProcessor {
    ui_id: egui::Id,
    loading: u8,
    sv: MPTVectorSelector,
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
}

impl Default for TLIProcessor {
    fn default() -> Self {
        let ui_id = egui::Id::new(Instant::now());
        Self {
            ui_id,
            loading: 0,
            sv: MPTVectorSelector::new(
                ui_id.with("SVSel"),
                TimeInputKind2::GET,
                TimeDisplayKind::Dhms,
            ),
            moon: i18n!("tliproc-no-moon"),
            maxiter: 100_000,
            temp: 100.0,
            ft_min: String::new(),
            ft_min_p: None,
            ft_max: String::new(),
            ft_max_p: None,
            ct_min: String::new(),
            ct_min_p: None,
            ct_max: String::new(),
            ct_max_p: None,
            pe_min: 0.0,
            pe_max: 0.0,
            opt_periapse: true,
            mnvs: vec![],
            mnv_ctr: 1,
        }
    }
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
                    self.sv.show(ui, i18n!("tliproc-parking-sv"), mission);
                });
                let vessel = self.sv.vessel.map(|x| &mission.vessels[x]);
                if let Ok(sv) = find_sv_mpt(self.sv.vessel, self.sv.time_parsed, mission) {
                    ui.horizontal(|ui| {
                        ui.label(i18n!("tliproc-central-body"));
                        ui.strong(&*sv.body.name);
                        ui.separator();
                        ui.label(i18n!("tliproc-moon-body"));
                        egui::ComboBox::from_id_salt(self.ui_id.with("Moon"))
                            .selected_text(&self.moon)
                            .show_ui(ui, |ui| {
                                for moon in &*sv.body.satellites {
                                    ui.selectable_value(&mut self.moon, moon.to_string(), &**moon);
                                }
                            });
                    });
                }
                handle(toasts, |_| {
                    if let Ok(sv) = find_sv_mpt(self.sv.vessel, self.sv.time_parsed, mission) {
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
                                .range(0.1..=1000.0)
                                .max_decimals(2),
                        );
                        ui.add(egui::DragValue::new(&mut self.maxiter).range(10_000..=1_000_000));

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
                                    .range(
                                        -moon_radius.unwrap_or(0.0)
                                            ..=moon_soi.unwrap_or(0.0).floor(),
                                    )
                                    .suffix("km")
                                    .max_decimals(3),
                            );
                            ui.label(i18n!("tliproc-to"));
                            ui.add(
                                egui::DragValue::new(&mut self.pe_max)
                                    .range(
                                        -moon_radius.unwrap_or(0.0)
                                            ..=moon_soi.unwrap_or(0.0).floor(),
                                    )
                                    .suffix("km")
                                    .max_decimals(3),
                            );
                            ui.label(i18n!("tliproc-opt-periapse"));
                            ui.add(egui::Checkbox::without_text(&mut self.opt_periapse));
                        });
                    });
                });
                ui.horizontal(|ui| {
                    if ui.button(i18n!("tliproc-calc")).clicked() {
                        handle(toasts, |_| {
                            let sv = find_sv_mpt(self.sv.vessel, self.sv.time_parsed, mission)?;
                            let vessel = vessel.ok_or_eyre(i18n!("error-no-vessel"))?;
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
                                    get_base: vessel.get_base,
                                    maxiter: self.maxiter,
                                    opt_periapse: self.opt_periapse,
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

                self.mnvs.retain(Option::is_some);
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
                                    .extend(),
                                );
                            });
                        });

                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-geti")).heading(),
                                    )
                                    .extend(),
                                );
                            });
                        });

                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-dv-prograde")).heading(),
                                    )
                                    .extend(),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-dv-normal")).heading(),
                                    )
                                    .extend(),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-dv-radial")).heading(),
                                    )
                                    .extend(),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(
                                        egui::RichText::new(i18n!("tliproc-dv-total")).heading(),
                                    )
                                    .extend(),
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
                                                        DisplaySelect::TLIProcessor,
                                                        code,
                                                    )),
                                                )?;
                                                Ok(())
                                            });
                                        };

                                        if ui
                                            .button(icon("\u{f70c}"))
                                            .on_hover_text(i18n!("transfer-to-tlmcc"))
                                            .clicked()
                                        {
                                            handle(toasts, |_| {
                                                let vessel_id = self
                                                    .sv
                                                    .vessel
                                                    .ok_or_eyre(i18n!("error-no-vessel"))?;

                                                let moon = mission
                                                    .system
                                                    .bodies
                                                    .get(&*self.moon)
                                                    .ok_or_eyre(i18n!("error-body-no-load"))?
                                                    .clone();

                                                let mut sv = mnv.tig_vector.clone();
                                                sv.velocity += mnv.deltav_bci();
                                                let moon_sv = moon
                                                    .ephem
                                                    .sv_bci(&sv.body)
                                                    .propagate(
                                                        sv.time - moon.ephem.epoch,
                                                        1e-7,
                                                        500,
                                                    )
                                                    .ok_or_eyre(i18n!("error-calc-general"))?;
                                                let sv_soi = sv
                                                    .intersect_soi_child(
                                                        &moon_sv, &moon, 1e-7, 30000,
                                                    )
                                                    .ok_or_eyre(i18n!("error-calc-general"))?;

                                                let sv_obt = sv_soi.clone().into_orbit(1e-8);
                                                let h_pe =
                                                    sv_obt.periapsis_radius() - sv_soi.body.radius;
                                                let mut sv_pe = sv_obt;
                                                let tof = orbits::time_of_flight(
                                                    sv_soi.position.norm(),
                                                    sv_pe.periapsis_radius(),
                                                    sv_pe.ta - consts::TAU,
                                                    0.0,
                                                    sv_pe.p,
                                                    //sv_pe.i,
                                                    moon.mu,
                                                );
                                                sv_pe.ta = 0.0;
                                                sv_pe.epoch =
                                                    sv_pe.epoch + Duration::seconds_f64(tof);
                                                let i = sv_pe.i;
                                                let sv_pe = sv_pe.sv_bci(&sv_soi.body);

                                                let (lat_pe, lng_pe) = sv_pe.latlng();
                                                let nodal_targets =
                                                    Some(NodalTargets::Translunar {
                                                        soi_ut: sv_soi.time,
                                                        mnv_base_code: format!("0400C/{code:02}"),
                                                        lat_pe,
                                                        lng_pe,
                                                        h_pe,
                                                        i,
                                                    });
                                                backend.effect(move |mission, _| {
                                                    let plan = mission
                                                        .plan
                                                        .get_mut(&vessel_id)
                                                        .expect("plan deleted");
                                                    plan.nodal_targets = nodal_targets;
                                                    Ok(())
                                                });
                                                Ok(())
                                            });
                                        }

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

pub struct GPM {
    ui_id: egui::Id,
    loading: u8,
    sv: MPTVectorSelector,
    mnv_ctr: u64,
    mode: GPMMode,

    circ_mode: CircMode,
    altitude: f64,
    circ_deltatime_unparsed: String,
    circ_deltatime_parsed: Option<Duration>,
    circ_time_unparsed: String,
    circ_time_parsed: Option<UTorGET>,
    circ_time_input: TimeInputKind2,
    circ_time_disp: TimeDisplayKind,

    apsis_mode: ApsisChangeMode,

    tt: TradeoffTable,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GPMMode {
    Circ,
    ChangeApoapsis,
    ChangePeriapsis,
}

impl fmt::Display for GPMMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GPMMode::Circ => write!(f, "{}", i18n!("gpm-mode-circ")),
            GPMMode::ChangeApoapsis => write!(f, "{}", i18n!("gpm-mode-apo")),
            GPMMode::ChangePeriapsis => write!(f, "{}", i18n!("gpm-mode-peri")),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CircMode {
    Apoapsis,
    Periapsis,
    FixedAltitude,
    FixedDeltaT,
    FixedTime,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ApsisChangeMode {
    AtApoapsis,
    AtPeriapsis,
    FixedDeltaT,
    FixedTime,
    Optimum,
}

impl fmt::Display for ApsisChangeMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ApsisChangeMode::AtApoapsis => write!(f, "{}", i18n!("gpm-circ-ap")),
            ApsisChangeMode::AtPeriapsis => write!(f, "{}", i18n!("gpm-circ-pe")),
            ApsisChangeMode::FixedDeltaT => write!(f, "{}", i18n!("gpm-circ-deltat")),
            ApsisChangeMode::FixedTime => write!(f, "{}", i18n!("gpm-circ-fixtime")),
            ApsisChangeMode::Optimum => write!(f, "{}", i18n!("gpm-optimum")),
        }
    }
}

impl fmt::Display for CircMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CircMode::Apoapsis => write!(f, "{}", i18n!("gpm-circ-ap")),
            CircMode::Periapsis => write!(f, "{}", i18n!("gpm-circ-pe")),
            CircMode::FixedAltitude => write!(f, "{}", i18n!("gpm-circ-alt")),
            CircMode::FixedDeltaT => write!(f, "{}", i18n!("gpm-circ-deltat")),
            CircMode::FixedTime => write!(f, "{}", i18n!("gpm-circ-fixtime")),
        }
    }
}

impl Default for GPM {
    fn default() -> Self {
        let ui_id = egui::Id::new(Instant::now());
        Self {
            ui_id,
            loading: 0,
            sv: MPTVectorSelector::new(
                ui_id.with("SVSel"),
                TimeInputKind2::GET,
                TimeDisplayKind::Dhms,
            ),
            mnv_ctr: 1,
            mode: GPMMode::Circ,
            circ_mode: CircMode::FixedAltitude,
            altitude: 0.0,
            circ_deltatime_unparsed: String::from("0s"),
            circ_deltatime_parsed: None,
            circ_time_unparsed: String::new(),
            circ_time_parsed: None,
            circ_time_input: TimeInputKind2::GET,
            circ_time_disp: TimeDisplayKind::Dhms,
            apsis_mode: ApsisChangeMode::Optimum,
            tt: TradeoffTable {
                mnvs: vec![],
                display: DisplaySelect::GPM,
            },
        }
    }
}

impl KtkDisplay for GPM {
    fn show(
        &mut self,
        mission: &Mission,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    ) {
        egui::Window::new(i18n!("gpm-title"))
            .open(&mut open.gpm)
            .default_size([512.0, 512.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    self.sv.show(ui, i18n!("gpm-sv"), mission);
                });
                let vessel = self.sv.vessel.map(|x| (x, &mission.vessels[x]));
                ui.separator();
                ui.horizontal(|ui| {
                    ui.with_layout(
                        egui::Layout::top_down(egui::Align::LEFT)
                            .with_main_justify(true)
                            .with_main_align(egui::Align::Center),
                        |ui| {
                            let spacing = ui.spacing().interact_size.y
                                - ui.text_style_height(&egui::TextStyle::Body);

                            ui.label(i18n!("gpm-mode"));
                            match self.mode {
                                GPMMode::Circ => {
                                    ui.add_space(spacing * 3.0 / 2.0);
                                    ui.label(i18n!("gpm-circ-when"));
                                }
                                GPMMode::ChangeApoapsis | GPMMode::ChangePeriapsis => {
                                    ui.add_space(spacing * 3.0 / 2.0);
                                    ui.label(i18n!("altitude"));
                                    ui.add_space(spacing * 3.0 / 2.0);
                                    ui.label(i18n!("gpm-circ-when"));
                                }
                            }
                        },
                    );

                    ui.vertical(|ui| {
                        let spacing = ui.spacing().interact_size.y
                            - ui.text_style_height(&egui::TextStyle::Monospace);
                        let _spacing_body = ui.spacing().interact_size.y
                            - ui.text_style_height(&egui::TextStyle::Body);

                        egui::ComboBox::from_id_salt(self.ui_id.with("Option"))
                            .selected_text(self.mode.to_string())
                            .show_ui(ui, |ui| {
                                ui.selectable_value(
                                    &mut self.mode,
                                    GPMMode::Circ,
                                    i18n!("gpm-mode-circ"),
                                );
                                ui.selectable_value(
                                    &mut self.mode,
                                    GPMMode::ChangeApoapsis,
                                    GPMMode::ChangeApoapsis.to_string(),
                                );
                                ui.selectable_value(
                                    &mut self.mode,
                                    GPMMode::ChangePeriapsis,
                                    GPMMode::ChangePeriapsis.to_string(),
                                );
                            });

                        match self.mode {
                            GPMMode::Circ => {
                                ui.add_space(spacing);
                                ui.horizontal(|ui| {
                                    egui::ComboBox::from_id_salt(self.ui_id.with("CircMode"))
                                        .selected_text(self.circ_mode.to_string())
                                        .show_ui(ui, |ui| {
                                            ui.selectable_value(
                                                &mut self.circ_mode,
                                                CircMode::FixedDeltaT,
                                                CircMode::FixedDeltaT.to_string(),
                                            );
                                            ui.selectable_value(
                                                &mut self.circ_mode,
                                                CircMode::FixedTime,
                                                CircMode::FixedTime.to_string(),
                                            );
                                            ui.selectable_value(
                                                &mut self.circ_mode,
                                                CircMode::FixedAltitude,
                                                CircMode::FixedAltitude.to_string(),
                                            );
                                            ui.selectable_value(
                                                &mut self.circ_mode,
                                                CircMode::Apoapsis,
                                                CircMode::Apoapsis.to_string(),
                                            );
                                            ui.selectable_value(
                                                &mut self.circ_mode,
                                                CircMode::Periapsis,
                                                CircMode::Periapsis.to_string(),
                                            );
                                        });
                                    match self.circ_mode {
                                        CircMode::Apoapsis | CircMode::Periapsis => {}
                                        CircMode::FixedAltitude => {
                                            let (periapsis, apoapsis) = {
                                                if let Ok(sv) = find_sv_mpt(
                                                    self.sv.vessel,
                                                    self.sv.time_parsed,
                                                    mission,
                                                ) {
                                                    let obt = sv.clone().into_orbit(1e-8);
                                                    (
                                                        obt.periapsis_radius() - sv.body.radius,
                                                        obt.apoapsis_radius() - sv.body.radius,
                                                    )
                                                } else {
                                                    (0.0, 0.0)
                                                }
                                            };

                                            ui.add(
                                                egui::DragValue::new(&mut self.altitude)
                                                    .range(periapsis..=apoapsis)
                                                    .max_decimals(2)
                                                    .suffix("km"),
                                            );
                                        }
                                        CircMode::FixedDeltaT => {
                                            ui.add(DurationInput::new(
                                                &mut self.circ_deltatime_unparsed,
                                                &mut self.circ_deltatime_parsed,
                                                Some(128.0),
                                                true,
                                                false,
                                            ));
                                        }
                                        CircMode::FixedTime => {
                                            ui.add(TimeInput1::new(
                                                &mut self.circ_time_unparsed,
                                                &mut self.circ_time_parsed,
                                                Some(128.0),
                                                self.circ_time_input,
                                                self.circ_time_disp,
                                                true,
                                                false,
                                            ));
                                            ui.add(TimeDisplayBtn(&mut self.circ_time_disp));
                                            ui.radio_value(
                                                &mut self.circ_time_input,
                                                TimeInputKind2::UT,
                                                i18n!("time-utils-ut"),
                                            );
                                            ui.radio_value(
                                                &mut self.circ_time_input,
                                                TimeInputKind2::GET,
                                                i18n!("time-utils-get"),
                                            );
                                        }
                                    }
                                });
                            }
                            GPMMode::ChangeApoapsis | GPMMode::ChangePeriapsis => {
                                ui.add_space(spacing);
                                ui.horizontal(|ui| {
                                    let (lo, hi) = {
                                        if let Ok(sv) = find_sv_mpt(
                                            self.sv.vessel,
                                            self.sv.time_parsed,
                                            mission,
                                        ) {
                                            if self.mode == GPMMode::ChangeApoapsis {
                                                (
                                                    sv.clone().into_orbit(1e-8).periapsis_radius()
                                                        - sv.body.radius,
                                                    sv.body.soi - sv.body.radius,
                                                )
                                            } else {
                                                (
                                                    -sv.body.radius,
                                                    sv.clone().into_orbit(1e-8).apoapsis_radius()
                                                        - sv.body.radius,
                                                )
                                            }
                                        } else {
                                            (0.0, 0.0)
                                        }
                                    };
                                    ui.add(
                                        egui::DragValue::new(&mut self.altitude)
                                            .range(lo..=hi)
                                            .max_decimals(2)
                                            .suffix("km"),
                                    );
                                });
                                ui.add_space(spacing);
                                ui.horizontal(|ui| {
                                    egui::ComboBox::from_id_salt(
                                        self.ui_id.with("ApsisChangeMode"),
                                    )
                                    .selected_text(self.apsis_mode.to_string())
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(
                                            &mut self.apsis_mode,
                                            ApsisChangeMode::FixedDeltaT,
                                            ApsisChangeMode::FixedDeltaT.to_string(),
                                        );
                                        ui.selectable_value(
                                            &mut self.apsis_mode,
                                            ApsisChangeMode::FixedTime,
                                            ApsisChangeMode::FixedTime.to_string(),
                                        );
                                        ui.selectable_value(
                                            &mut self.apsis_mode,
                                            ApsisChangeMode::Optimum,
                                            ApsisChangeMode::Optimum.to_string(),
                                        );
                                        ui.selectable_value(
                                            &mut self.apsis_mode,
                                            ApsisChangeMode::AtApoapsis,
                                            ApsisChangeMode::AtApoapsis.to_string(),
                                        );
                                        ui.selectable_value(
                                            &mut self.apsis_mode,
                                            ApsisChangeMode::AtPeriapsis,
                                            ApsisChangeMode::AtPeriapsis.to_string(),
                                        );
                                    });
                                    match self.apsis_mode {
                                        ApsisChangeMode::FixedDeltaT => {
                                            ui.add(DurationInput::new(
                                                &mut self.circ_deltatime_unparsed,
                                                &mut self.circ_deltatime_parsed,
                                                Some(128.0),
                                                true,
                                                false,
                                            ));
                                        }
                                        ApsisChangeMode::FixedTime => {
                                            ui.add(TimeInput1::new(
                                                &mut self.circ_time_unparsed,
                                                &mut self.circ_time_parsed,
                                                Some(128.0),
                                                self.circ_time_input,
                                                self.circ_time_disp,
                                                true,
                                                false,
                                            ));
                                            ui.add(TimeDisplayBtn(&mut self.circ_time_disp));
                                            ui.radio_value(
                                                &mut self.circ_time_input,
                                                TimeInputKind2::UT,
                                                i18n!("time-utils-ut"),
                                            );
                                            ui.radio_value(
                                                &mut self.circ_time_input,
                                                TimeInputKind2::GET,
                                                i18n!("time-utils-get"),
                                            );
                                        }
                                        _ => {}
                                    }
                                });
                            }
                        }
                    });
                });
                ui.add_space(2.0);
                ui.horizontal(|ui| {
                    if ui.button(i18n!("tlmcc-calc")).clicked() {
                        handle(toasts, |_| {
                            let sv = find_sv_mpt(self.sv.vessel, self.sv.time_parsed, mission)?;
                            let (_vessel_id, vessel) =
                                vessel.ok_or_eyre(i18n!("error-no-vessel"))?;

                            match self.mode {
                                GPMMode::Circ => {
                                    // TODO: move to backend
                                    let circ_mode = match self.circ_mode {
                                        CircMode::Apoapsis => gpm::CircMode::Apoapsis,
                                        CircMode::Periapsis => gpm::CircMode::Periapsis,
                                        CircMode::FixedAltitude => {
                                            gpm::CircMode::FixedAltitude(self.altitude)
                                        }
                                        CircMode::FixedDeltaT => gpm::CircMode::FixedDeltaT(
                                            self.circ_deltatime_parsed.unwrap_or(Duration::ZERO),
                                        ),
                                        CircMode::FixedTime => {
                                            let Some(time) = self.circ_time_parsed else {
                                                bail!(i18n!("error-no-mnv-time"));
                                            };
                                            let time = match time {
                                                UTorGET::GET(get) => {
                                                    vessel.get_base + get.into_duration()
                                                }
                                                UTorGET::UT(ut) => ut,
                                            };
                                            let deltatime = time - sv.time;
                                            if deltatime.is_negative() {
                                                bail!(i18n!("error-mnv-negative-time"));
                                            }
                                            gpm::CircMode::FixedDeltaT(deltatime)
                                        }
                                    };
                                    self.tt.mnvs.push(Some((
                                        self.mnv_ctr,
                                        gpm::circ(&mission.system, sv, circ_mode, vessel.get_base)
                                            .ok_or_eyre(i18n!("error-calc-general"))?,
                                    )));
                                    self.mnv_ctr += 1;
                                }
                                GPMMode::ChangeApoapsis | GPMMode::ChangePeriapsis => {
                                    let mode = match self.apsis_mode {
                                        ApsisChangeMode::AtApoapsis => {
                                            gpm::ApsisChangeMode::AtApoapsis
                                        }
                                        ApsisChangeMode::AtPeriapsis => {
                                            gpm::ApsisChangeMode::AtPeriapsis
                                        }
                                        ApsisChangeMode::FixedDeltaT => {
                                            gpm::ApsisChangeMode::FixedDeltaT(
                                                self.circ_deltatime_parsed
                                                    .unwrap_or(Duration::ZERO),
                                            )
                                        }
                                        ApsisChangeMode::FixedTime => {
                                            let Some(time) = self.circ_time_parsed else {
                                                bail!(i18n!("error-no-mnv-time"));
                                            };
                                            let time = match time {
                                                UTorGET::GET(get) => {
                                                    vessel.get_base + get.into_duration()
                                                }
                                                UTorGET::UT(ut) => ut,
                                            };
                                            let deltatime = time - sv.time;
                                            if deltatime.is_negative() {
                                                bail!(i18n!("error-mnv-negative-time"));
                                            }
                                            gpm::ApsisChangeMode::FixedDeltaT(deltatime)
                                        }
                                        ApsisChangeMode::Optimum => gpm::ApsisChangeMode::Optimum,
                                    };
                                    // TODO: move to backend
                                    self.tt.mnvs.push(Some((
                                        self.mnv_ctr,
                                        gpm::change_apsis(
                                            &mission.system,
                                            sv.clone(),
                                            if self.mode == GPMMode::ChangeApoapsis {
                                                Apsis::Apoapsis
                                            } else {
                                                Apsis::Periapsis
                                            },
                                            self.altitude + sv.body.radius,
                                            mode,
                                            vessel.get_base,
                                        )
                                        .ok_or_eyre(i18n!("error-calc-general"))?,
                                    )));
                                    self.mnv_ctr += 1;
                                }
                            }
                            //self.loading = 1;
                            Ok(())
                        });
                    }

                    if self.loading == 1 {
                        ui.spinner();
                    }
                });

                ui.separator();

                self.tt.show(ui, backend, toasts, &mut open.mpt_trfr)
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
            self.tt.mnvs.push(Some((self.mnv_ctr, mnv)));
            self.mnv_ctr += 1;
            Ok(())
        } else {
            res.map(|_| ())
        }
    }
}

#[derive(Debug)]
struct TradeoffTable {
    pub mnvs: Vec<Option<(u64, Maneuver)>>,
    pub display: DisplaySelect,
}

impl TradeoffTable {
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        backend: &mut Backend,
        toasts: &mut Toasts,
        mpt_trfr_open: &mut bool,
    ) {
        self.mnvs.retain(Option::is_some);
        egui_extras::TableBuilder::new(ui)
            .striped(true)
            .column(Column::auto_with_initial_suggestion(96.0).resizable(true))
            .column(Column::auto_with_initial_suggestion(128.0).resizable(true))
            .columns(
                Column::auto_with_initial_suggestion(72.0).resizable(true),
                4,
            )
            .cell_layout(
                egui::Layout::left_to_right(egui::Align::Center)
                    .with_main_align(egui::Align::RIGHT)
                    .with_main_wrap(false)
                    .with_cross_justify(true)
                    .with_main_justify(true),
            )
            .header(16.0, |mut header| {
                let layout = egui::Layout::default().with_cross_align(egui::Align::Center);
                header.col(|ui| {
                    ui.with_layout(layout, |ui| {
                        ui.add(
                            egui::Label::new(egui::RichText::new(i18n!("tliproc-code")).heading())
                                .extend(),
                        );
                    });
                });

                header.col(|ui| {
                    ui.with_layout(layout, |ui| {
                        ui.add(
                            egui::Label::new(egui::RichText::new(i18n!("tliproc-geti")).heading())
                                .extend(),
                        );
                    });
                });

                header.col(|ui| {
                    ui.with_layout(layout, |ui| {
                        ui.add(
                            egui::Label::new(
                                egui::RichText::new(i18n!("tliproc-dv-prograde")).heading(),
                            )
                            .extend(),
                        );
                    });
                });
                header.col(|ui| {
                    ui.with_layout(layout, |ui| {
                        ui.add(
                            egui::Label::new(
                                egui::RichText::new(i18n!("tliproc-dv-normal")).heading(),
                            )
                            .extend(),
                        );
                    });
                });
                header.col(|ui| {
                    ui.with_layout(layout, |ui| {
                        ui.add(
                            egui::Label::new(
                                egui::RichText::new(i18n!("tliproc-dv-radial")).heading(),
                            )
                            .extend(),
                        );
                    });
                });
                header.col(|ui| {
                    ui.with_layout(layout, |ui| {
                        ui.add(
                            egui::Label::new(
                                egui::RichText::new(i18n!("tliproc-dv-total")).heading(),
                            )
                            .extend(),
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
                                let display = self.display as u16;
                                ui.monospace(format!("{display:04}C/{code:02}"));

                                if ui
                                    .button(icon("\u{f506}"))
                                    .on_hover_text(i18n!("transfer-to-mpt"))
                                    .clicked()
                                {
                                    *mpt_trfr_open = true;
                                    handle(toasts, |_| {
                                        backend.tx_loopback(
                                            DisplaySelect::MPTTransfer,
                                            Ok(HRes::MPTTransfer(mnv.clone(), self.display, code)),
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
                                egui::RichText::new(format!("{:>+08.2}", mnv.deltav.x * 1000.0))
                                    .color(egui::Color32::from_rgb(0, 214, 0))
                                    .monospace(),
                            ));
                        });
                        row.col(|ui| {
                            ui.add(egui::Label::new(
                                egui::RichText::new(format!("{:>+08.2}", mnv.deltav.y * 1000.0))
                                    .color(egui::Color32::from_rgb(214, 0, 214))
                                    .monospace(),
                            ));
                        });
                        row.col(|ui| {
                            ui.add(egui::Label::new(
                                egui::RichText::new(format!("{:>+08.2}", mnv.deltav.z * 1000.0))
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
    }
}
