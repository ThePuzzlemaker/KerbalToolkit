#![warn(clippy::unwrap_used)]
use std::{
    collections::{HashMap, HashSet},
    fmt,
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc,
    },
    thread,
    time::Instant,
};

use backend::{handler_thread, HReq, HRes};
use color_eyre::eyre::{self};
use egui::TextBuffer;
use egui_extras::{Column, Size};

use egui_grid::GridBuilder;
use egui_modal::Modal;
use egui_notify::Toasts;
use fluent_templates::Loader;
use itertools::Itertools;
use kerbtk::{
    arena::Arena,
    bodies::Body,
    time::{GET, UT},
    vessel::{Decouplers, PartId, VesselClass},
};
use mission::Mission;
use num_enum::FromPrimitive;
use parking_lot::RwLock;
use time::Duration;
use tracing::{error, info};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};
use unic_langid::LanguageIdentifier;
use vectors::{VectorComparison, VectorPanelSummary};

#[path = "kerbtk/backend.rs"]
mod backend;
#[path = "kerbtk/mission.rs"]
mod mission;
#[path = "kerbtk/vectors.rs"]
mod vectors;

fluent_templates::static_loader! {
    static LOCALES = {
        locales: "src/bin/kerbtk/locales",
        fallback_language: "en-US",
    };
}

const US_ENGLISH: LanguageIdentifier = unic_langid::langid!("en-US");

macro_rules! i18n {
    ($v:expr) => {
        LOCALES.lookup(&US_ENGLISH, $v)
    };
}

macro_rules! i18n_args {
    ($v:expr, $($arg:expr => $val:expr),*) => {{
	let mut args = ::std::collections::HashMap::new();
	$(
	    args.insert(String::from($arg), ::fluent::FluentValue::from($val));
	)*
	LOCALES.lookup_with_args(&US_ENGLISH, $v, &args)
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
    let _ = thread::spawn(|| handler_thread(handler_rx, handler_tx, mission1));
    eframe::run_native(
        &i18n!("title"),
        native_options,
        Box::new(|cc| Box::new(App::new(main_rx, main_tx, cc, mission))),
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
    toasts: Toasts,
    logs: Logs,
    backend: Backend,
}

struct Backend {
    tx: Sender<(usize, HReq)>,
    rx: Receiver<(usize, eyre::Result<HRes>)>,
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
        open: &mut bool,
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
            toasts: Default::default(),
            logs: Default::default(),
            backend: Backend {
                tx,
                rx,
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

impl TimeInput {
    pub fn parse(self, s: &str) -> Option<UTorGET> {
        match self {
            TimeInput::UTSeconds => s
                .parse::<f64>()
                .ok()
                .map(|x| UTorGET::UT(UT::from_duration(Duration::seconds_f64(x)))),
            TimeInput::UTDHMS => todo!(),
            TimeInput::GETDHMS => todo!(),
        }
    }
}

impl Default for TimeInput {
    fn default() -> Self {
        Self::GETDHMS
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
    ui_id: egui::Id,
    loading: bool,
}

impl Default for SystemConfiguration {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
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
        open: &mut bool,
    ) {
        egui::Window::new(i18n!("syscfg-title"))
            .open(open)
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
    ui_id: egui::Id,
    ip: String,
    rpc_port: String,
    stream_port: String,
    status: String,
    loading: bool,
}

impl Default for KRPCConfig {
    fn default() -> Self {
        Self {
            ui_id: egui::Id::new(Instant::now()),
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
        open: &mut bool,
    ) {
        egui::Window::new(i18n!("krpc-title"))
            .open(open)
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
        open: &mut bool,
    ) {
        egui::Window::new("Mission Plan Table")
            .open(open)
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
                                    egui::Label::new(egui::RichText::new("GETI").heading())
                                        .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(egui::RichText::new("ΔT").heading())
                                        .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(egui::RichText::new("ΔV").heading())
                                        .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(egui::RichText::new("ΔVREM").heading())
                                        .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(egui::RichText::new("HA").heading())
                                        .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(egui::RichText::new("HP").heading())
                                        .wrap(false),
                                );
                            });
                        });
                        header.col(|ui| {
                            ui.with_layout(layout, |ui| {
                                ui.add(
                                    egui::Label::new(egui::RichText::new("Code").heading())
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
            DisplaySelect::VC => self.dis.vc = true,
            DisplaySelect::VPS => self.dis.vps = true,
            DisplaySelect::Classes => self.dis.classes = true,
            _ => {}
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
                    .add(egui::Button::new("Organize Windows").wrap(false))
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
                        });
                });
            });

        self.syscfg.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis.syscfg,
        );
        self.krpc.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis.krpc,
        );
        self.mpt.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis.mpt,
        );
        self.vc.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis.vc,
        );
        self.vps.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis.vps,
        );
        self.classes.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis.classes,
        );
        self.logs.show(
            &self.mission,
            &mut self.toasts,
            &mut self.backend,
            ctx,
            frame,
            &mut self.dis.logs,
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
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash, FromPrimitive)]
#[repr(u16)]
pub enum DisplaySelect {
    SysCfg = 0,
    Krpc = 1,
    Logs = 2,
    MPT = 100,
    VC = 200,
    VPS = 201,
    Classes = 300,
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
        open: &mut bool,
    ) {
        egui::Window::new("Logs").open(open).show(ctx, |_ui| {});
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
struct Classes {
    ui_id: egui::Id,
    search: String,
    current_class: Option<Arc<RwLock<VesselClass>>>,
    renaming: bool,
    just_clicked_rename: bool,
    classes_filtered: Vec<Arc<RwLock<VesselClass>>>,
    force_refilter: bool,
    loading: bool,
    checkboxes: HashMap<PartId, bool>,
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
                        .hint_text("Search or create")
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
                            && ui.button(format!("Create \"{}\"", self.search)).clicked())
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
                    ui.label(egui::RichText::new(format!("Subvessel {}", ix + 1)).strong());
                    egui::ComboBox::from_id_source(self.ui_id.with("SubvesselComboBox").with(ix))
                        .selected_text(self.subvessel_options[ix].to_string())
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self.subvessel_options[ix],
                                SubvesselOption::Keep,
                                "Keep",
                            );
                            ui.selectable_value(
                                &mut self.subvessel_options[ix],
                                SubvesselOption::Discard,
                                "Ignore",
                            );
                        });
                    if self.subvessel_options[ix] == SubvesselOption::Keep {
                        ui.text_edit_singleline(&mut self.subvessel_names[ix]);
                    }
                });
                state.show_body_indented(&header_res.response, ui, |ui| {
                    ui.vertical(|ui| {
                        for part in subvessel {
                            ui.label(egui::RichText::new(format!(
                                " ▪ {}",
                                class.parts[*part].title
                            )));
                        }
                    })
                });
            }
            ui.horizontal(|ui| {
                if ui.button("Cancel").clicked() {
                    modal.close();
                }
                if ui.button("Finish").clicked() {
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

                        if option == SubvesselOption::Keep {
                            let vessel = Arc::new(RwLock::new(VesselClass {
                                name: name.clone(),
                                description: String::new(),
                                shortcode: String::new(),
                                parts,
                                root,
                            }));
                            mission.write().classes.push(vessel);
                        }
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
        egui::Window::new("Vessel Classes")
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
                                                        .on_hover_text("Save")
                                                        .clicked()
                                                {
                                                    self.renaming = false;
                                                } else if !self.renaming
                                                    && ui
                                                        .button(icon("\u{e3c9}"))
                                                        .on_hover_text("Rename")
                                                        .clicked()
                                                {
                                                    self.renaming = true;
                                                    self.just_clicked_rename = true;
                                                    self.force_refilter = true;
                                                }

                                                // TODO: confirm delete
                                                if ui
                                                    .button(icon("\u{e872}"))
                                                    .on_hover_text("Delete")
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
                                                        .button("Load from Editor")
                                                        .on_hover_text(LOCALES.lookup(
                                                            &US_ENGLISH,
                                                            "classes-load-editor-explainer",
                                                        ));
                                                    if load_btn.clicked() {
                                                        backend.tx(
                                                            DisplaySelect::Classes,
                                                            HReq::LoadVesselPartsFromEditor,
                                                        )?;
                                                        self.loading = true;
                                                    }
                                                    let load_btn = ui
                                                        .button("Load from Flight")
                                                        .on_hover_text(LOCALES.lookup(
                                                            &US_ENGLISH,
                                                            "classes-load-flight-explainer",
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

                                            ui.label("Description");
                                            egui::TextEdit::multiline(&mut class.description)
                                                .show(ui);
                                            ui.horizontal(|ui| {
                                                ui.label("Shortcode");
                                                let shortcode = egui::TextEdit::singleline(
                                                    &mut class.shortcode,
                                                )
                                                .char_limit(5)
                                                .show(ui);
                                                shortcode.response.on_hover_ui(|ui| {
                                                    ui.horizontal_wrapped(|ui| {
                                                        ui.label(LOCALES.lookup(
                                                            &US_ENGLISH,
                                                            "classes-shortcode-explainer",
                                                        ));
                                                        ui.label(
                                                            egui::RichText::new("Must be unique.")
                                                                .strong(),
                                                        );
                                                    });
                                                });
                                            });
                                            ui.heading("Decouplers");
                                            ui.label(
                                                LOCALES.lookup(
                                                    &US_ENGLISH,
                                                    "classes-calcsep-explainer",
                                                ),
                                            );
                                            for (partid, part) in class
                                                .parts
                                                .iter()
                                                .filter(|x| x.1.decouplers.is_some())
                                                .sorted_by_key(|(_, x)| (&x.title, &x.tag))
                                            {
                                                ui.add_space(0.2);
                                                match part.decouplers.as_ref().expect("decoupler") {
                                                    Decouplers::Single(_) => {
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
                                                                format!(
                                                                    "{} (tag: \"{}\")",
                                                                    &part.title, &part.tag
                                                                ),
                                                            );
                                                        }
                                                    }
                                                    Decouplers::RODecoupler { .. } => {
                                                        let &mut (ref mut top, ref mut bot) = self
                                                            .rocheckboxes
                                                            .entry(partid)
                                                            .or_insert((false, false));
                                                        ui.horizontal_wrapped(|ui| {
                                                            if part.tag.trim().is_empty() {
                                                                ui.checkbox(top, "Top");
                                                                ui.checkbox(
                                                                    bot,
                                                                    format!(
                                                                        "Bottom - {}",
                                                                        &part.title
                                                                    ),
                                                                );
                                                            } else {
                                                                ui.checkbox(top, "Top");
                                                                ui.checkbox(
                                                                    bot,
                                                                    format!(
                                                                        "Bottom - {} (tag: \"{}\")",
                                                                        &part.title, &part.tag
                                                                    ),
                                                                );
                                                            }
                                                        });
                                                    }
                                                    Decouplers::ProceduralFairing(_) => todo!(),
                                                }
                                            }

                                            let modal = Modal::new(ctx, "SeparationModal");

                                            modal.show(|ui| {
                                                self.modal(ctx, ui, &modal, &mut class, mission)
                                            });

                                            if ui.button("Calculate Separation").clicked() {
                                                let parts = self
                                                    .checkboxes
                                                    .iter()
                                                    .filter_map(|(id, checked)| {
                                                        checked.then_some(*id)
                                                    })
                                                    .collect::<Vec<_>>();
                                                let subvessels = kerbtk::vessel::decoupled_vessels(
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
                                                        format!("{} {}", class.name, i + 1)
                                                    })
                                                    .collect();
                                                self.subvessels = subvessels;

                                                modal.open();
                                                self.checkboxes.clear();
                                                self.rocheckboxes.clear();
                                            }
                                        } else {
                                            ui.heading("No Class Selected");
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
        if let Ok(HRes::LoadedVesselClass(parts, root)) = res {
            if let Some(class) = self.current_class.as_ref() {
                let mut class = class.write();
                class.parts = parts;
                class.root = root;
            }
            Ok(())
        } else {
            res.map(|_| ())
        }
    }
}
