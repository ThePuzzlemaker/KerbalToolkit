#![warn(clippy::unwrap_used)]
use std::{
    cmp::Ordering,
    collections::{HashMap, HashSet},
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

use egui_notify::Toasts;
use itertools::Itertools;
use kerbtk::{
    arena::Arena,
    bodies::Body,
    ffs::{self, Conditions, FuelFlowSimulation, FuelStats, SimPart, SimVessel},
    kepler::orbits::StateVector,
    maneuver::Maneuver,
    time::{GET, UT},
    translunar::TLIConstraintSet,
    vessel::{PartId, VesselClassRef, VesselRef},
};
use mission::Mission;
use nalgebra::Vector3;
use num_enum::FromPrimitive;
use parking_lot::RwLock;
use time::Duration;
use tracing::{error, info};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};
use unic_langid::LanguageIdentifier;
use vectors::{VectorComparison, VectorPanelSummary, VectorSelector};
use vessels::{Classes, Vessels};

// TODO: organize this properly
#[path = "kerbtk/backend.rs"]
mod backend;
#[path = "kerbtk/mission.rs"]
mod mission;
#[path = "kerbtk/vectors.rs"]
mod vectors;
#[path = "kerbtk/vessels.rs"]
mod vessels;

fluent_templates::static_loader! {
    static LOCALES = {
        locales: "src/bin/kerbtk/locales",
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
    ($v:expr, $($arg:expr => $val:expr),*) => { $crate::i18n_args!($v, $($arg, $val),*) };
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
    let mission = Arc::new(RwLock::new(Default::default()));
    let mission1 = mission.clone();
    let main_tx_loopback = handler_tx.clone();
    let _ = thread::spawn(|| handler_thread(handler_rx, handler_tx, mission1));
    eframe::run_native(
        &i18n!("title"),
        native_options,
        Box::new(|cc| Box::new(App::new(main_rx, main_tx, main_tx_loopback, cc, mission))),
    )
    .expect(&i18n!("error-start-failed"));
    std::process::exit(0)
}

struct App {
    mission: Arc<RwLock<Mission>>,
    menu: Menu,
    dis: Displays,
    mpt: MissionPlanTable,
    vc: VectorComparison,
    syscfg: SystemConfiguration,
    krpc: KRPCConfig,
    vps: VectorPanelSummary,
    classes: Classes,
    vessels: Vessels,
    toasts: Toasts,
    logs: Logs,
    backend: Backend,
    tliproc: TLIProcessor,
    mpt_trfr: MPTTransfer,
}

struct Backend {
    tx: Sender<(usize, HReq)>,
    rx: Receiver<(usize, eyre::Result<HRes>)>,
    tx_loopback: Sender<(usize, eyre::Result<HRes>)>,
    txc: usize,
    txq: HashMap<usize, DisplaySelect>,
}

impl Backend {
    fn tx(&mut self, src: DisplaySelect, req: HReq) -> eyre::Result<()> {
        self.tx.send((self.txc, req))?;
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
}

fn geti(h: u64, m: u8, s: u8, ms: u16) -> Duration {
    Duration::new(
        h as i64 * 60 * 60 + m as i64 * 60 + s as i64,
        ms as i32 * 1_000_000,
    )
}

fn geti_hms(geti: Duration) -> (u64, u8, u8, u16) {
    let ms = geti.whole_milliseconds() % 1000;
    let s = geti.whole_seconds() % 60;
    let m = geti.whole_minutes() % 60;
    let h = geti.whole_hours();
    (
        h.unsigned_abs(),
        m.unsigned_abs() as u8,
        s.unsigned_abs() as u8,
        ms.unsigned_abs() as u16,
    )
}

trait KtkDisplay {
    fn show(
        &mut self,
        mission: &Arc<RwLock<Mission>>,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    );

    fn handle_rx(
        &mut self,
        res: eyre::Result<HRes>,
        mission: &Arc<RwLock<Mission>>,
        toasts: &mut Toasts,
        backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()>;
}

impl App {
    fn new(
        rx: Receiver<(usize, eyre::Result<HRes>)>,
        tx: Sender<(usize, HReq)>,
        tx_loopback: Sender<(usize, eyre::Result<HRes>)>,
        cc: &eframe::CreationContext<'_>,
        mission: Arc<RwLock<Mission>>,
    ) -> Self {
        cc.egui_ctx
            .style_mut(|style| style.explanation_tooltips = true);
        let mut fonts = egui::FontDefinitions::default();

        fonts.font_data.insert(
            "mtl-icons".to_owned(),
            egui::FontData::from_static(include_bytes!(
                "kerbtk/assets/MaterialSymbolsOutlined.ttf"
            )),
        );
        fonts.families.insert(
            egui::FontFamily::Name("mtl-icons".into()),
            vec!["mtl-icons".into(), "Hack".into()],
        );

        cc.egui_ctx.set_fonts(fonts);

        let app = Self {
            mission,
            dis: Default::default(),
            menu: Default::default(),
            mpt: Default::default(),
            vc: Default::default(),
            syscfg: Default::default(),
            krpc: Default::default(),
            vps: Default::default(),
            classes: Default::default(),
            vessels: Default::default(),
            toasts: Default::default(),
            logs: Default::default(),
            tliproc: Default::default(),
            mpt_trfr: Default::default(),
            backend: Backend {
                tx,
                rx,
                tx_loopback,
                txc: 1,
                txq: HashMap::new(),
            },
        };
        let mut this = app;
        this.mpt.maneuvers = vec![
            MPTManeuver {
                geti: geti(26, 44, 31, 400),
                deltat: geti(0, 0, 0, 0),
                deltav: 21.5,
                dvrem: 5440.0,
                ha: 99999.9,
                hp: 99999.9,
                code: "ECCSX01MC".into(),
            },
            MPTManeuver {
                geti: geti(75, 56, 19, 200),
                deltat: geti(49, 12, 0, 0),
                deltav: 0.0,
                dvrem: 118.0,
                ha: 0.0,
                hp: 60.2,
                code: "CCRX02FC".into(),
            },
            MPTManeuver {
                geti: geti(75, 58, 19, 200),
                deltat: geti(0, 2, 0, 0),
                deltav: 2923.7,
                dvrem: 2517.0,
                ha: 169.4,
                hp: 60.0,
                code: "CCSX03LO".into(),
            },
            MPTManeuver {
                geti: geti(80, 20, 44, 900),
                deltat: geti(4, 22, 0, 0),
                deltav: 137.2,
                dvrem: 2380.0,
                ha: 60.0,
                hp: 60.0,
                code: "CCSX04CI".into(),
            },
            MPTManeuver {
                geti: geti(101, 44, 10, 0),
                deltat: geti(21, 23, 0, 0),
                deltav: 71.8,
                dvrem: 7417.0,
                ha: 60.0,
                hp: 8.2,
                code: "LLDX01DI".into(),
            },
            MPTManeuver {
                geti: geti(102, 41, 18, 200),
                deltat: geti(0, 57, 0, 0),
                deltav: 6888.4,
                dvrem: 499.0,
                ha: -0.0,
                hp: -937.0,
                code: "LLDX02PD".into(),
            },
            MPTManeuver {
                geti: geti(106, 5, 0, 0),
                deltat: geti(3, 24, 0, 0),
                deltav: 15.0,
                dvrem: 2365.0,
                ha: 60.1,
                hp: 60.0,
                code: "CCSX05FC".into(),
            },
        ];
        this
    }
}

#[derive(Default, Debug)]
struct Menu {
    selector: String,
}

#[derive(Default, Debug)]
struct MissionPlanTable {
    maneuvers: Vec<MPTManeuver>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TimeInput {
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

    pub fn parse_dhms_duration(input: &str) -> IResult<&str, Duration> {
        if input.is_empty() {
            return Err(nom::Err::Failure(nom::error::make_error(
                input,
                nom::error::ErrorKind::Eof,
            )));
        }

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

        let seconds = days as f64 * 60.0 * 60.0 * 24.0
            + hours as f64 * 60.0 * 60.0
            + mins as f64 * 60.0
            + seconds as f64
            + millis as f64 / 1000.0;
        Ok((input, Duration::seconds_f64(seconds)))
    }

    pub fn parse_dhms_time(input: &str) -> IResult<&str, Duration> {
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

        let seconds = millis.map(|millis| millis as f64 / 1000.0).unwrap_or(0.0)
            + seconds as f64
            + minutes as f64 * 60.0
            + hours as f64 * 60.0 * 60.0
            + days
                .map(|days| days as f64 * 60.0 * 60.0 * 24.0)
                .unwrap_or(0.0);
        Ok((input, Duration::seconds_f64(seconds)))
    }
}

impl TimeInput {
    pub fn parse(self, s: &str) -> Option<UTorGET> {
        match self {
            TimeInput::UTSeconds => s
                .parse::<f64>()
                .ok()
                .map(|x| UTorGET::UT(UT::from_duration(Duration::seconds_f64(x)))),
            TimeInput::UTDHMS => parse::parse_dhms_time(s)
                .ok()
                .map(|x| UTorGET::UT(UT::from_duration(x.1))),
            TimeInput::GETDHMS => parse::parse_dhms_time(s)
                .ok()
                .map(|x| UTorGET::GET(GET::from_duration(x.1))),
        }
    }

    pub fn format(self, t: UT, _get_base: Option<UT>) -> String {
        match self {
            TimeInput::UTSeconds => format!("{}", t.into_duration().as_seconds_f64()),
            TimeInput::UTDHMS => {
                format!(
                    "{}:{:02}:{:02}:{:02}.{:03}",
                    t.days(),
                    t.hours(),
                    t.minutes(),
                    t.seconds(),
                    t.millis()
                )
            }
            TimeInput::GETDHMS => todo!(),
        }
    }
}

impl Default for TimeInput {
    fn default() -> Self {
        Self::UTDHMS
    }
}

impl fmt::Display for TimeInput {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}",
            match self {
                TimeInput::UTSeconds => i18n!("time-ut"),
                TimeInput::UTDHMS => i18n!("time-ut-dhms"),
                TimeInput::GETDHMS => i18n!("time-get-dhms"),
            }
        )
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum UTorGET {
    UT(UT),
    GET(GET),
}

struct SystemConfiguration {
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
        mission: &Arc<RwLock<Mission>>,
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

                    ui.label(i18n_args!("syscfg-bodies-loaded", "bodies" => mission.read().system.bodies.len()));
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
                                    let bodies = &mission.read().system.bodies;
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
        mission: &Arc<RwLock<Mission>>,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()> {
        self.loading = false;
        if let Ok(HRes::LoadedSystem(system)) = res {
            mission.write().system = system;
            ctx.request_repaint();
            Ok(())
        } else {
            res.map(|_| ())
        }
    }
}

impl SystemConfiguration {
    fn show_body(
        bodies: &HashMap<Arc<str>, Body>,
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
struct KRPCConfig {
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
        _mission: &Arc<RwLock<Mission>>,
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
        _mission: &Arc<RwLock<Mission>>,
        toasts: &mut Toasts,
        _backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
    ) -> eyre::Result<()> {
        self.loading = false;
        match res {
            Ok(HRes::ConnectionFailure(e)) | Err(e) => {
                self.status = i18n_args!("krpc-status-error", "error" => e.to_string());
                error!(
                    "{}",
                    i18n_args!("error-krpc-conn", "error" => e.to_string())
                );
                toasts.error(i18n_args!("error-krpc-conn", "error" => e.to_string()));
                self.loading = false;
                ctx.request_repaint();
                Ok(())
            }
            Ok(HRes::Connected(version)) => {
                self.status = i18n_args!("krpc-status-success", "version" => &version);
                info!("{}", i18n_args!("krpc-log-success", "version" => &version));
                toasts.info(i18n_args!("krpc-log-success", "version" => version));
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

#[derive(Debug, Clone)]
pub struct MPTManeuver {
    pub geti: Duration,
    pub deltat: Duration,
    pub deltav: f64,
    pub dvrem: f64,
    pub ha: f64,
    pub hp: f64,
    pub code: String,
}

impl KtkDisplay for MissionPlanTable {
    fn show(
        &mut self,
        _mission: &Arc<RwLock<Mission>>,
        _toasts: &mut Toasts,
        _backend: &mut Backend,
        ctx: &egui::Context,
        _frame: &mut eframe::Frame,
        open: &mut Displays,
    ) {
        egui::Window::new(i18n!("mpt-title"))
            .open(&mut open.mpt)
            .default_size([256.0, 256.0])
            .show(ctx, |ui| {
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
                    .body(|mut body| {
                        for maneuver in &self.maneuvers {
                            body.row(16.0, |mut row| {
                                row.col(|ui| {
                                    let (h, m, s, ms) = geti_hms(maneuver.geti);
                                    ui.add(
                                        egui::Label::new(format!(
                                            "{}:{:02}:{:02}.{}",
                                            h,
                                            m,
                                            s,
                                            ms / 100
                                        ))
                                        .wrap(false),
                                    );
                                });
                                row.col(|ui| {
                                    if !maneuver.deltat.is_zero() {
                                        let (h, m, _, _) = geti_hms(maneuver.deltat);

                                        ui.add(
                                            egui::Label::new(format!("{}:{:02}", h, m)).wrap(false),
                                        );
                                    }
                                });
                                row.col(|ui| {
                                    ui.add(
                                        egui::Label::new(format!("{:.1}", maneuver.deltav))
                                            .wrap(false),
                                    );
                                });
                                row.col(|ui| {
                                    ui.add(
                                        egui::Label::new(format!("{:.0}", maneuver.dvrem))
                                            .wrap(false),
                                    );
                                });
                                row.col(|ui| {
                                    ui.add(
                                        egui::Label::new(format!("{:.1}", maneuver.ha)).wrap(false),
                                    );
                                });
                                row.col(|ui| {
                                    ui.add(
                                        egui::Label::new(format!("{:.1}", maneuver.hp)).wrap(false),
                                    );
                                });
                                row.col(|ui| {
                                    ui.add(egui::Label::new(&maneuver.code).wrap(false));
                                });
                            });
                        }
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

impl App {
    fn open_window(&mut self, selector: &str) {
        match selector.parse::<u16>().unwrap_or(u16::MAX).into() {
            DisplaySelect::SysCfg => self.dis.syscfg = true,
            DisplaySelect::Krpc => self.dis.krpc = true,
            DisplaySelect::Logs => self.dis.logs = true,
            DisplaySelect::MPT => self.dis.mpt = true,
            DisplaySelect::MPTTransfer => self.dis.mpt_trfr = true,
            DisplaySelect::VC => self.dis.vc = true,
            DisplaySelect::VPS => self.dis.vps = true,
            DisplaySelect::Classes => self.dis.classes = true,
            DisplaySelect::Vessels => self.dis.vessels = true,
            DisplaySelect::TLIProcessor => self.dis.tliproc = true,
            DisplaySelect::Unknown => {}
        }
    }
}

pub fn icon_label(icon: &str, label: &str) -> egui::text::LayoutJob {
    let mut job = egui::text::LayoutJob::default();

    egui::RichText::new(icon)
        .family(egui::FontFamily::Name("mtl-icons".into()))
        .append_to(
            &mut job,
            &egui::Style::default(),
            egui::FontSelection::Default,
            egui::Align::LEFT,
        );
    egui::RichText::new(format!(" {}", label)).append_to(
        &mut job,
        &egui::Style::default(),
        egui::FontSelection::Default,
        egui::Align::LEFT,
    );
    job
}

pub fn icon(icon: &str) -> egui::RichText {
    egui::RichText::new(icon).family(egui::FontFamily::Name("mtl-icons".into()))
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        if let Ok((txi, res)) = self
            .backend
            .rx
            .recv_timeout(std::time::Duration::from_millis(10))
        {
            let res = match self.backend.txq.remove(&txi) {
                Some(DisplaySelect::Krpc) => self.krpc.handle_rx(
                    res,
                    &self.mission,
                    &mut self.toasts,
                    &mut self.backend,
                    ctx,
                    frame,
                ),
                Some(DisplaySelect::SysCfg) => self.syscfg.handle_rx(
                    res,
                    &self.mission,
                    &mut self.toasts,
                    &mut self.backend,
                    ctx,
                    frame,
                ),
                Some(DisplaySelect::Classes) => self.classes.handle_rx(
                    res,
                    &self.mission,
                    &mut self.toasts,
                    &mut self.backend,
                    ctx,
                    frame,
                ),
                Some(DisplaySelect::Vessels) => self.vessels.handle_rx(
                    res,
                    &self.mission,
                    &mut self.toasts,
                    &mut self.backend,
                    ctx,
                    frame,
                ),
                Some(DisplaySelect::VPS) => self.vps.handle_rx(
                    res,
                    &self.mission,
                    &mut self.toasts,
                    &mut self.backend,
                    ctx,
                    frame,
                ),
                Some(DisplaySelect::TLIProcessor) => self.tliproc.handle_rx(
                    res,
                    &self.mission,
                    &mut self.toasts,
                    &mut self.backend,
                    ctx,
                    frame,
                ),
                Some(DisplaySelect::MPTTransfer) => self.mpt_trfr.handle_rx(
                    res,
                    &self.mission,
                    &mut self.toasts,
                    &mut self.backend,
                    ctx,
                    frame,
                ),
                _ => Ok(()),
            };
            handle(&mut self.toasts, |_| res);
        }

        egui::Window::new(i18n!("menu-title"))
            .default_width(196.0)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    handle(&mut self.toasts, |_| {
                        let file = rfd::FileDialog::new().set_file_name("mission.ktk");

                        if ui
                            .button(icon_label("\u{e2c8}", &i18n!("menu-load-mission")))
                            .clicked()
                        {
                            if let Some(path) = file.clone().pick_file() {
                                *self.mission.write() =
                                    ron::from_str(&std::fs::read_to_string(path)?)?;
                                self.classes.force_refilter = true;
                                self.vessels.force_refilter = true;
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
                        egui::TextEdit::singleline(&mut self.menu.selector)
                            .char_limit(4)
                            .font(egui::TextStyle::Monospace)
                            .desired_width(32.0),
                    );
                    if (response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)))
                        || ui.button(i18n!("menu-open")).clicked()
                    {
                        let selector = self.menu.selector.take();
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
                    self.dis = Default::default();
                }
                ui.vertical(|ui| {
                    egui::CollapsingHeader::new(i18n!("menu-display-config"))
                        .open(openall)
                        .show(ui, |ui| {
                            ui.checkbox(&mut self.dis.syscfg, i18n!("menu-display-syscfg"));
                            ui.checkbox(&mut self.dis.krpc, i18n!("menu-display-krpc"));
                            ui.checkbox(&mut self.dis.logs, i18n!("menu-display-logs"));
                        });
                    egui::CollapsingHeader::new(i18n!("menu-display-mpt"))
                        .open(openall)
                        .show(ui, |ui| {
                            ui.checkbox(&mut self.dis.mpt, i18n!("menu-display-open-mpt"));
                            ui.checkbox(&mut self.dis.mpt_trfr, i18n!("menu-display-mpt-trfr"));
                        });
                    egui::CollapsingHeader::new(i18n!("menu-display-sv"))
                        .open(openall)
                        .show(ui, |ui| {
                            ui.checkbox(&mut self.dis.vc, i18n!("menu-display-sv-comp"));
                            ui.checkbox(&mut self.dis.vps, i18n!("menu-display-sv-vps"));
                        });
                    egui::CollapsingHeader::new(i18n!("menu-display-vesselsclasses"))
                        .open(openall)
                        .show(ui, |ui| {
                            ui.checkbox(&mut self.dis.classes, i18n!("menu-display-classes"));
                            ui.checkbox(&mut self.dis.vessels, i18n!("menu-display-vessels"));
                        });
                    egui::CollapsingHeader::new(i18n!("menu-display-target"))
                        .open(openall)
                        .show(ui, |ui| {
                            ui.checkbox(&mut self.dis.tliproc, i18n!("menu-display-tliproc"));
                        });
                });
            });

        self.syscfg.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis,
        );
        self.krpc.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis,
        );
        self.mpt.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis,
        );
        self.vc.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis,
        );
        self.vps.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis,
        );
        self.classes.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis,
        );
        self.logs.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis,
        );
        self.vessels.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis,
        );
        self.tliproc.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis,
        );
        self.mpt_trfr.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis,
        );

        self.toasts.show(ctx);
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
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, FromPrimitive)]
#[repr(u16)]
pub enum DisplaySelect {
    SysCfg = 0,
    Krpc = 1,
    Logs = 2,
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
        _mission: &Arc<RwLock<Mission>>,
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
        _mission: &Arc<RwLock<Mission>>,
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

impl KtkDisplay for MPTTransfer {
    fn show(
        &mut self,
        mission: &Arc<RwLock<Mission>>,
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
                                    .read()
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
                        let Some((mnv, _)) = &self.mnv else {
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

                        if ui.button(i18n!("mpt-trfr-calc-fuel")).clicked() {
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
                                    .filter_map(|x| {
                                        (x.0 .0 == pid).then_some((x.0 .1, x.1.clone()))
                                    })
                                    .collect::<HashMap<_, _>>();
                                let disabled_resource_mass =
                                    resources.iter().fold(0.0, |acc, x| {
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
                                    resource_request_remaining_threshold: part
                                        .resource_request_remaining_threshold,
                                    mass: part.mass,
                                    dry_mass: part.dry_mass,
                                    crew_mass: part.crew_mass,
                                    modules_current_mass,
                                    disabled_resource_mass,
                                    is_launch_clamp: part.is_launch_clamp,
                                });
                                if self.vessel_engines.get(&pid).copied().unwrap_or(false) {
                                    sim_vessel.active_engines.extend(
                                        part.engines.iter().cloned().map(|mut x| {
                                            x.part = sid;
                                            x
                                        }),
                                    );
                                }
                                map.insert(pid, sid);
                            }
                            for (pid, sid) in &map {
                                sim_vessel.parts[*sid].crossfeed_part_set = class.read().parts
                                    [*pid]
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

                                        bt += (start_mass * exhvel) / (1000.0 * segment.thrust)
                                            * (1.0 - alpha);
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
                    }
                });
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
        mission: &Arc<RwLock<Mission>>,
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
                        if let Some(moon) = mission.read().system.bodies.get(&*self.moon) {
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
                                let (d, h, m, s, ms) =
                                    (t.days(), t.hours(), t.minutes(), t.seconds(), t.millis());
                                ui.strong(format!("{d}d {h:>02}h {m:>02}m {s:>02}.{ms:>03}s"));
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
                            ));
                            ui.label(i18n!("tliproc-to"));
                            ui.add(DurationInput::new(
                                &mut self.ft_max,
                                &mut self.ft_max_p,
                                Some(128.0),
                            ));
                        });
                        ui.horizontal(|ui| {
                            ui.add(DurationInput::new(
                                &mut self.ct_min,
                                &mut self.ct_min_p,
                                Some(128.0),
                            ));
                            ui.label(i18n!("tliproc-to"));
                            ui.add(DurationInput::new(
                                &mut self.ct_max,
                                &mut self.ct_max_p,
                                Some(128.0),
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
                                .read()
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
        _mission: &Arc<RwLock<Mission>>,
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

pub struct DurationInput<'a> {
    buf: &'a mut String,
    parsed: &'a mut Option<Duration>,
    desired_width: Option<f32>,
}

impl<'a> DurationInput<'a> {
    pub fn new(
        buf: &'a mut String,
        parsed: &'a mut Option<Duration>,
        desired_width: Option<f32>,
    ) -> Self {
        Self {
            parsed,
            buf,
            desired_width,
        }
    }
}

impl egui::Widget for DurationInput<'_> {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        let mut dirty = false;
        let mut res = ui
            .scope(|ui| {
                if let Ok((_, parsed)) = parse::parse_dhms_duration(self.buf) {
                    *self.parsed = Some(parsed);
                } else {
                    *self.parsed = None;
                    ui.visuals_mut().selection.stroke.color = egui::Color32::from_rgb(255, 0, 0);
                }
                let edit = egui::TextEdit::singleline(self.buf);
                let edit = if let Some(desired_width) = self.desired_width {
                    edit.desired_width(desired_width)
                } else {
                    edit
                };
                let output = edit.show(ui);
                if output.response.changed() {
                    dirty = true;
                }
                if output.response.lost_focus() {
                    if let Some(parsed) = *self.parsed {
                        let t = UT::from_duration(parsed);
                        let (d, h, m, s, ms) =
                            (t.days(), t.hours(), t.minutes(), t.seconds(), t.millis());
                        *self.buf = format!("{d}d {h:>02}h {m:>02}m {s:>02}.{ms:>03}s");
                    }
                }
            })
            .response;
        if dirty {
            res.mark_changed();
        }
        res
    }
}
