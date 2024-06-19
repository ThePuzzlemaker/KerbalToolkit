use kerbtk::time::{GET, UT};
use time::Duration;

use crate::{i18n, parse, UTorGET};

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

pub struct TimeInput1<'a> {
    buf: &'a mut String,
    parsed: &'a mut Option<UTorGET>,
    desired_width: Option<f32>,
    kind: TimeInputKind2,
    disp: TimeDisplayKind,
    interactive: bool,
    allow_neg: bool,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TimeInputKind2 {
    UT,
    #[allow(clippy::upper_case_acronyms)]
    GET,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TimeDisplayKind {
    Dhms,
    Sec,
}

pub struct TimeDisplayBtn<'a>(pub &'a mut TimeDisplayKind);

impl egui::Widget for TimeDisplayBtn<'_> {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        let mut dirty = false;
        let mut res = ui
            .scope(|ui| {
                if ui
                    .button(icon("\u{e425}"))
                    .on_hover_text(i18n!("time-display-toggle"))
                    .clicked()
                {
                    *self.0 = match self.0 {
                        TimeDisplayKind::Dhms => TimeDisplayKind::Sec,
                        TimeDisplayKind::Sec => TimeDisplayKind::Dhms,
                    };
                    dirty = true;
                }
            })
            .response;
        if dirty {
            res.mark_changed();
        }
        res
    }
}

impl TimeInputKind2 {
    pub fn with_duration(self, duration: Duration) -> UTorGET {
        match self {
            Self::UT => UTorGET::UT(UT::from_duration(duration)),
            Self::GET => UTorGET::GET(GET::from_duration(duration)),
        }
    }
}

impl<'a> TimeInput1<'a> {
    pub fn new(
        buf: &'a mut String,
        parsed: &'a mut Option<UTorGET>,
        desired_width: Option<f32>,
        kind: TimeInputKind2,
        disp: TimeDisplayKind,
        interactive: bool,
        allow_neg: bool,
    ) -> Self {
        Self {
            buf,
            parsed,
            desired_width,
            kind,
            disp,
            interactive,
            allow_neg,
        }
    }
}

impl egui::Widget for TimeInput1<'_> {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.scope(|ui| {
            let output = if self.interactive {
                if let Ok((_, parsed)) = parse::parse_dhms_duration(self.buf, self.allow_neg) {
                    *self.parsed = Some(self.kind.with_duration(parsed));
                } else if let Ok((_, parsed)) = parse::parse_dhms_time(self.buf, self.allow_neg) {
                    *self.parsed = Some(self.kind.with_duration(parsed));
                } else if let Ok((_, parsed)) = parse::parse_sec_time(self.buf, self.allow_neg) {
                    *self.parsed = Some(self.kind.with_duration(parsed));
                } else {
                    *self.parsed = None;

                    let visuals = ui.visuals_mut();
                    visuals.selection.stroke.color = egui::Color32::from_rgb(255, 0, 0);
                    visuals.widgets.active.bg_stroke.color = egui::Color32::from_rgb(255, 0, 0);
                    visuals.widgets.active.bg_stroke.width = 1.0;
                    // Only show a passive red border if our buffer is not empty
                    if !self.buf.trim().is_empty() {
                        visuals.widgets.inactive.bg_stroke.color =
                            egui::Color32::from_rgb(255, 0, 0);
                        visuals.widgets.inactive.bg_stroke.width = 1.0;
                    }
                    visuals.widgets.hovered.bg_stroke.color = egui::Color32::from_rgb(255, 0, 0);
                    visuals.widgets.hovered.bg_stroke.width = 1.0;
                }

                let edit = egui::TextEdit::singleline(self.buf);
                let edit = if let Some(desired_width) = self.desired_width {
                    edit.desired_width(desired_width)
                } else {
                    edit
                };
                let edit = edit.interactive(self.interactive);
                let output = edit.show(ui);
                output.response
            } else {
                egui::Frame::none()
                    .inner_margin(egui::Margin::symmetric(4.0, 2.0))
                    .fill(ui.visuals().extreme_bg_color)
                    .rounding(ui.visuals().widgets.noninteractive.rounding)
                    .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
                    .show(ui, |ui| {
                        let res = ui.label(&*self.buf);
                        if let Some(desired_width) = self.desired_width {
                            if desired_width > res.rect.width() {
                                ui.add_space(desired_width - res.rect.width() - 8.0);
                            }
                        }
                    })
                    .response
            };

            if output.lost_focus() || !output.has_focus() {
                if let Some(parsed) = *self.parsed {
                    let t = match parsed {
                        UTorGET::UT(t) => t,
                        UTorGET::GET(t) => UT::from_duration(t.into_duration()),
                    };
                    let (d, h, m, s, ms) = (
                        t.days().abs(),
                        t.hours(),
                        t.minutes(),
                        t.seconds(),
                        t.millis(),
                    );
                    let n = if t.is_negative() { "-" } else { "" };
                    match self.disp {
                        TimeDisplayKind::Dhms => {
                            *self.buf = format!("{n}{d:>03}:{h:>02}:{m:>02}:{s:>02}.{ms:>03}");
                        }
                        TimeDisplayKind::Sec => {
                            *self.buf = format!("{:.3}", t.into_duration().as_seconds_f64());
                        }
                    }
                }
            }

            output
        })
        .inner
    }
}

pub struct DurationInput<'a> {
    buf: &'a mut String,
    parsed: &'a mut Option<Duration>,
    desired_width: Option<f32>,
    interactive: bool,
    allow_neg: bool,
}

impl<'a> DurationInput<'a> {
    pub fn new(
        buf: &'a mut String,
        parsed: &'a mut Option<Duration>,
        desired_width: Option<f32>,
        interactive: bool,
        allow_neg: bool,
    ) -> Self {
        Self {
            parsed,
            buf,
            desired_width,
            interactive,
            allow_neg,
        }
    }
}

impl egui::Widget for DurationInput<'_> {
    fn ui(self, ui: &mut egui::Ui) -> egui::Response {
        ui.scope(|ui| {
            let output = if self.interactive {
                if let Ok((_, parsed)) = parse::parse_dhms_duration(self.buf, self.allow_neg) {
                    *self.parsed = Some(parsed);
                } else {
                    *self.parsed = None;

                    let visuals = ui.visuals_mut();
                    visuals.selection.stroke.color = egui::Color32::from_rgb(255, 0, 0);
                    visuals.widgets.active.bg_stroke.color = egui::Color32::from_rgb(255, 0, 0);
                    visuals.widgets.active.bg_stroke.width = 1.0;
                    // Only show a passive red border if our buffer is not empty
                    if !self.buf.trim().is_empty() {
                        visuals.widgets.inactive.bg_stroke.color =
                            egui::Color32::from_rgb(255, 0, 0);
                        visuals.widgets.inactive.bg_stroke.width = 1.0;
                    }
                    visuals.widgets.hovered.bg_stroke.color = egui::Color32::from_rgb(255, 0, 0);
                    visuals.widgets.hovered.bg_stroke.width = 1.0;
                }
                let edit = egui::TextEdit::singleline(self.buf);
                let edit = if let Some(desired_width) = self.desired_width {
                    edit.desired_width(desired_width)
                } else {
                    edit
                };
                // TODO: replace with a label & frame when !self.interactive to allow copy+paste
                let edit = edit.interactive(self.interactive);
                let output = edit.show(ui);
                output.response
            } else {
                egui::Frame::none()
                    .inner_margin(egui::Margin::symmetric(4.0, 2.0))
                    .fill(ui.visuals().extreme_bg_color)
                    .rounding(ui.visuals().widgets.noninteractive.rounding)
                    .stroke(ui.visuals().widgets.noninteractive.bg_stroke)
                    .show(ui, |ui| {
                        let res = ui.label(&*self.buf);
                        if let Some(desired_width) = self.desired_width {
                            if desired_width > res.rect.width() {
                                ui.add_space(desired_width - res.rect.width() - 8.0);
                            }
                        }
                    })
                    .response
            };

            if output.lost_focus() || !output.has_focus() {
                if let Some(parsed) = *self.parsed {
                    let t = UT::from_duration(parsed);
                    let (d, h, m, s, ms) = (
                        t.days().abs(),
                        t.hours(),
                        t.minutes(),
                        t.seconds(),
                        t.millis(),
                    );
                    let n = if t.is_negative() { "-" } else { "" };
                    *self.buf = format!("{n}{d}d {h:>02}h {m:>02}m {s:>02}.{ms:>03}s");
                }
            }
            output
        })
        .inner
    }
}
