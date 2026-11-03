//! The "Roommate Radar" dashboard tab: header with a status line, then one row per setting. Drawn on the CPU like the
//! labels (draw.rs); the dashboard's laser pointer arrives as mouse events in texture pixels.
use crate::detector;
use crate::draw::{self, Image, Rgb, WHITE, over, rounded_rect_sdf};
use crate::openvr::{self, VROverlayHandle_t, vr};
use crate::settings::{MAX_DIST, MIN_DIST, RATES, SIZES, Settings};
use crate::textures::Textures;

const W: usize = 1000;
const SETTINGS_H: usize = (TOP + ROWS as f32 * ROW_H + (SECTIONS.len() - 1) as f32 * GAP + 30.0) as usize; // 30 px below
const PAD: f32 = 48.0;
const TOP: f32 = 190.0;
const ROW_H: f32 = 92.0;
const GAP: f32 = 36.0; // extra space above each section after the first

/// The tab's controls, one per row, top to bottom, in sections (a gap and a stronger line between them): what each
/// shows and changes, and when it's greyed out.
static SECTIONS: [&[Control]; 3] = [
    &[
        Control { title: "Enabled", detail: "", dimmed: |_| false, kind: ControlKind::Switch(|s| &mut s.enabled) },
        // greyed out when never started by Steam: no game id to launch it by
        Control {
            title: "Autorun on startup",
            detail: "",
            dimmed: |s| s.gameid.is_none(),
            kind: ControlKind::Switch(|s| &mut s.autostart),
        },
        Control {
            title: "Only in Games",
            detail: "",
            dimmed: NEEDS_ENABLED,
            kind: ControlKind::Switch(|s| &mut s.games_only),
        },
    ],
    &[
        Control {
            title: "Detect People",
            detail: "",
            dimmed: NEEDS_ENABLED,
            kind: ControlKind::Switch(|s| &mut s.people),
        },
        Control {
            title: "Detect Cats and Dogs",
            detail: "",
            dimmed: NEEDS_ENABLED,
            kind: ControlKind::Switch(|s| &mut s.pets),
        },
        Control {
            title: "Ignore My Hands",
            detail: "Works best when holding controllers",
            dimmed: NEEDS_ENABLED,
            kind: ControlKind::Switch(|s| &mut s.ignore_hands),
        },
        Control {
            title: "Only Detect Near Play Area",
            detail: "Inside your boundary or within 1m of it", // PLAY_AREA_MARGIN, tested
            dimmed: NEEDS_ENABLED,
            kind: ControlKind::Switch(|s| &mut s.play_area_only),
        },
        Control {
            title: "Max Range",
            detail: "",
            dimmed: NEEDS_ENABLED,
            kind: ControlKind::Stepper { val: |s| &mut s.max_dist, step: 0.5, min: MIN_DIST, max: MAX_DIST },
        },
    ],
    &[
        Control {
            title: "Detection Resolution",
            detail: "",
            dimmed: NEEDS_ENABLED,
            kind: ControlKind::Choice {
                count: SIZES.len(),
                label: |i| format!("{} ({}px)", SIZE_NAMES[i], SIZES[i]),
                get: |s| SIZES.iter().position(|&x| x == s.size).unwrap_or(0),
                pick: |s, i| {
                    s.size = SIZES[i];
                    detector::SIZE.store(s.size, std::sync::atomic::Ordering::Relaxed); // the detector reloads
                },
            },
        },
        Control {
            title: "Detection Rate",
            detail: "",
            dimmed: NEEDS_ENABLED,
            kind: ControlKind::Choice {
                count: RATES.len(),
                label: |i| format!("{}Hz", RATES[i]),
                get: |s| RATES.iter().position(|&x| x == s.hz).unwrap_or(0),
                pick: |s, i| {
                    s.hz = RATES[i];
                    detector::HZ.store(s.hz);
                },
            },
        },
    ],
];
const ROWS: usize = {
    let (mut n, mut i) = (0, 0);
    while i < SECTIONS.len() {
        n += SECTIONS[i].len();
        i += 1;
    }
    n
};

/// Detection Resolution's names for SIZES.
const SIZE_NAMES: [&str; SIZES.len()] = ["Low", "Medium", "High"];

/// Everything but Enabled and Autorun on startup is greyed out while Enabled is off.
const NEEDS_ENABLED: fn(&Settings) -> bool = |s| !s.enabled;

struct Control {
    title: &'static str,
    detail: &'static str, // a smaller line under the title, or ""
    dimmed: fn(&Settings) -> bool,
    kind: ControlKind,
}

#[derive(Clone, Copy)]
enum ControlKind {
    Switch(fn(&mut Settings) -> &mut bool),
    Stepper { val: fn(&mut Settings) -> &mut f32, step: f32, min: f32, max: f32 }, // (-) value (+)
    Choice { count: usize, label: fn(usize) -> String, get: fn(&Settings) -> usize, pick: fn(&mut Settings, usize) }, // dropdown
}

/// What's under the pointer.
#[derive(Clone, Copy, PartialEq, Debug)]
enum Target {
    Row(usize),       // a switch's or dropdown's row
    Step(usize, i32), // a stepper's (-) (-1) or (+) (1)
    Item(usize),      // the open popup's choice
    Ack,              // the first-run warning's button
}

/// Shown in the tab, instead of the settings, until its button is pressed (the dashboard opens on it at first start).
/// Broken by hand, each sentence on its own lines.
const WARNING: [&str; 4] = [
    "Roommate Radar won't always",
    "see people and pets.",
    "Don't rely on it to avoid collisions:",
    "keep your play area clear.",
];
// Centered, top to bottom: the icon, a heading, the warning, the button.
const ICON_Y0: f32 = 72.0;
const HEAD_Y: f32 = 360.0; // baselines
const HEAD_SIZE: f32 = 72.0;
const WARN_Y: f32 = 475.0;
const WARN_SIZE: f32 = 50.0;
const WARN_LINE_SPACING: f32 = 70.0;
const ACK_W: f32 = 560.0;
const ACK_H: f32 = 120.0;
const ACK_Y0: f32 = 780.0;
const ACK_X0: f32 = (W as f32 - ACK_W) / 2.0;
const ACK_SIZE: f32 = 52.0;
const WARNING_H: usize = (ACK_Y0 + ACK_H) as usize + 72; // shorter than the settings

/// The tab's height in texture pixels: the warning's until it's acknowledged.
fn height(settings: &Settings) -> usize {
    if settings.warning_acked { SETTINGS_H } else { WARNING_H }
}

const TOGGLE_W: f32 = 104.0;
const TOGGLE_H: f32 = 56.0;
const BTN_R: f32 = 28.0;
const DROP_W: f32 = 300.0;
const DROP_H: f32 = 60.0;
const ITEM_H: f32 = 60.0;
const DROP_X0: f32 = W as f32 - PAD - DROP_W;
const MINUS_CX: f32 = W as f32 - PAD - BTN_R - 220.0;
const PLUS_CX: f32 = W as f32 - PAD - BTN_R;
const HOVER_TICK_US: u16 = 1500; // hover haptic
const DASH_KEY: &str = "steamframe.roommate-radar.dashboard"; // the tab's overlay key
pub const BG: Rgb = [20, 20, 24];
pub const GREY: Rgb = [150, 150, 160];
pub const LIGHT: Rgb = [205, 205, 212]; // body text: between WHITE and GREY
pub const TRACK_OFF: Rgb = [68, 68, 76];
pub const BTN_FILL: Rgb = [52, 52, 60];
const ACCENT: Rgb = draw::class_rgb(0); // the person color

/// Row `row` (0..ROWS): its section and control.
fn at(row: usize) -> (usize, &'static Control) {
    SECTIONS
        .iter()
        .enumerate()
        .flat_map(|(section, cs)| cs.iter().map(move |c| (section, c)))
        .nth(row)
        .expect("no such row")
}

fn row_cy(row: usize) -> f32 {
    TOP + (row as f32 + 0.5) * ROW_H + at(row).0 as f32 * GAP
}

impl Control {
    /// Which part of this control, in row `row`, is at (x, y): the whole row, but a stepper's buttons.
    fn hit(&self, row: usize, x: f32, y: f32) -> Option<Target> {
        let ControlKind::Stepper { .. } = self.kind else {
            return Some(Target::Row(row));
        };
        let near = |cx: f32| (x - cx).hypot(y - row_cy(row)) < BTN_R + 12.0;
        if near(MINUS_CX) {
            Some(Target::Step(row, -1))
        } else if near(PLUS_CX) {
            Some(Target::Step(row, 1))
        } else {
            None
        }
    }

    /// Changes and saves the setting; true: a dropdown's row, which opens its popup instead.
    fn press(&self, target: Target, settings: &mut Settings) -> bool {
        match (target, self.kind) {
            (Target::Row(_), ControlKind::Switch(f)) => *f(settings) = !*f(settings),
            (Target::Step(_, d), ControlKind::Stepper { val, step, min, max }) => {
                *val(settings) = (*val(settings) + d as f32 * step).clamp(min, max)
            }
            (Target::Item(i), ControlKind::Choice { pick, .. }) => pick(settings, i),
            (Target::Row(_), ControlKind::Choice { .. }) => return true,
            _ => return false,
        }
        settings.save();
        false
    }

    /// Draws row `row`: title (and detail) on the left, the control on the right.
    fn render(&self, img: &mut Image, row: usize, hover: Option<Target>, settings: &Settings) {
        let wf = W as f32;
        let cy = row_cy(row);
        let hovered = hover == Some(Target::Row(row));
        if hovered {
            fill_rrect(img, PAD - 20.0, cy - ROW_H / 2.0 + 4.0, wf - 2.0 * PAD + 40.0, ROW_H - 8.0, 18.0, WHITE, 0.05);
        }
        // a line above each row but the first, stronger mid-gap between sections
        if row > 0 {
            let (h, a, y) = if at(row).0 != at(row - 1).0 { (3.0, 0.25, GAP / 2.0) } else { (1.5, 0.08, 0.0) };
            fill_rrect(img, PAD, cy - ROW_H / 2.0 - y - h / 2.0, wf - 2.0 * PAD, h, 0.0, WHITE, a);
        }
        let dim = (self.dimmed)(settings);
        let (fg, fade) = if dim { (GREY, 0.5) } else { (WHITE, 1.0) };
        if self.detail.is_empty() {
            img.text(PAD as i32, (cy + 13.0) as i32, self.title, fg, 36.0);
        } else {
            // title nudged up, detail line below it
            img.text(PAD as i32, (cy + 1.0) as i32, self.title, fg, 36.0);
            img.text(PAD as i32, (cy + 33.0) as i32, self.detail, GREY, 24.0);
        }
        let mut scratch = settings.clone(); // for the accessors, which take &mut
        match self.kind {
            ControlKind::Switch(f) => {
                // accent track with the knob right when on; grey when dimmed
                let on = *f(&mut scratch);
                let (x0, y0) = (wf - PAD - TOGGLE_W, cy - TOGGLE_H / 2.0);
                let track = if on && !dim { ACCENT } else { TRACK_OFF };
                fill_rrect(img, x0, y0, TOGGLE_W, TOGGLE_H, TOGGLE_H / 2.0, track, fade);
                let knob = if on { x0 + TOGGLE_W - TOGGLE_H / 2.0 } else { x0 + TOGGLE_H / 2.0 };
                fill_circle(img, knob, cy, TOGGLE_H / 2.0 - 5.0, WHITE, if dim { 0.6 } else { 1.0 });
            }
            ControlKind::Choice { label, get, .. } => {
                // current choice and a chevron
                let a = if hovered { 1.0 } else { 0.8 } * fade;
                fill_rrect(img, DROP_X0, cy - DROP_H / 2.0, DROP_W, DROP_H, 14.0, BTN_FILL, a);
                img.text((DROP_X0 + 22.0) as i32, (cy + 10.0) as i32, &label(get(settings)), fg, 28.0);
                for k in -8i32..=8 {
                    let x = wf - PAD - 34.0 + k as f32;
                    fill_circle(img, x, cy - 3.0 + (8 - k.abs()) as f32 * 0.6, 2.0, fg, 1.0);
                }
            }
            ControlKind::Stepper { val, .. } => {
                for (cx, d) in [(MINUS_CX, -1), (PLUS_CX, 1)] {
                    let a = if hover == Some(Target::Step(row, d)) { 1.0 } else { 0.8 } * fade;
                    fill_circle(img, cx, cy, BTN_R, BTN_FILL, a);
                    fill_rrect(img, cx - 11.0, cy - 1.5, 22.0, 3.0, 1.5, fg, 1.0);
                    if d == 1 {
                        fill_rrect(img, cx - 1.5, cy - 11.0, 3.0, 22.0, 1.5, fg, 1.0);
                    }
                }
                let v = format!("{:.1} m", *val(&mut scratch));
                let tw = draw::text_end_x(0, &v, 36.0);
                img.text(((MINUS_CX + PLUS_CX) / 2.0 - tw as f32 / 2.0) as i32, (cy + 13.0) as i32, &v, fg, 36.0);
            }
        }
    }

    /// A dropdown's number of choices, their labels and which one is current.
    fn choices(&self, settings: &Settings) -> (usize, fn(usize) -> String, usize) {
        let ControlKind::Choice { count, label, get, .. } = self.kind else { unreachable!("not a dropdown") };
        (count, label, get(settings))
    }

    /// The popup opens with the current choice over the dropdown, moved up as needed to stay inside the panel.
    fn popup_top(&self, row: usize, settings: &Settings) -> f32 {
        let (n, _, i) = self.choices(settings);
        (row_cy(row) - (i as f32 + 0.5) * ITEM_H).min(SETTINGS_H as f32 - n as f32 * ITEM_H - 8.0)
    }

    fn hit_popup(&self, row: usize, x: f32, y: f32, settings: &Settings) -> Option<Target> {
        let i = ((y - self.popup_top(row, settings)) / ITEM_H).floor();
        let inside = (DROP_X0..=DROP_X0 + DROP_W).contains(&x) && i >= 0.0 && (i as usize) < self.choices(settings).0;
        inside.then_some(Target::Item(i as usize))
    }

    /// Over the rows: the choices, the current one in the accent color.
    fn render_popup(&self, img: &mut Image, row: usize, hover: Option<Target>, settings: &Settings) {
        let (y0, (n, label, cur)) = (self.popup_top(row, settings), self.choices(settings));
        fill_rrect(img, DROP_X0 - 2.0, y0 - 2.0, DROP_W + 4.0, n as f32 * ITEM_H + 4.0, 16.0, WHITE, 0.15);
        fill_rrect(img, DROP_X0, y0, DROP_W, n as f32 * ITEM_H, 14.0, BTN_FILL, 1.0);
        for i in 0..n {
            let iy = y0 + i as f32 * ITEM_H;
            if hover == Some(Target::Item(i)) {
                fill_rrect(img, DROP_X0 + 6.0, iy + 4.0, DROP_W - 12.0, ITEM_H - 8.0, 10.0, WHITE, 0.1);
            }
            let fg = if i == cur { ACCENT } else { WHITE };
            img.text((DROP_X0 + 22.0) as i32, (iy + ITEM_H / 2.0 + 10.0) as i32, &label(i), fg, 28.0);
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn fill_rrect(img: &mut Image, x0: f32, y0: f32, w: f32, h: f32, r: f32, rgb: Rgb, a: f32) {
    for y in (y0 as i32 - 1).max(0)..((y0 + h) as i32 + 2).min(img.h as i32) {
        for x in (x0 as i32 - 1).max(0)..((x0 + w) as i32 + 2).min(img.w as i32) {
            let sd = rounded_rect_sdf(x as f32 + 0.5 - x0, y as f32 + 0.5 - y0, w, h, 0.0, r);
            over(img.at(x as usize, y as usize), rgb, a * (0.5 - sd).clamp(0.0, 1.0));
        }
    }
}

pub fn fill_circle(img: &mut Image, cx: f32, cy: f32, r: f32, rgb: Rgb, a: f32) {
    fill_rrect(img, cx - r, cy - r, 2.0 * r, 2.0 * r, r, rgb, a);
}

/// Tab icon: assets/icon.svg, rasterized (straight-alpha RGBA) with
/// `rsvg-convert -w 128 -h 128 icon.svg | magick - -depth 8 rgba:icon-128.rgba` (and 176 for the warning's)
pub fn icon() -> Image {
    Image { w: 128, h: 128, px: include_bytes!("../assets/icon-128.rgba").to_vec() }
}

#[derive(Default)]
pub struct Panel {
    popup: Option<usize>, // the dropdown row whose popup is showing
    hover: Option<Target>,
    drawn_status: String, // the status the tab was last drawn with ("" = redraw)
    dash_overlay: VROverlayHandle_t,
}

impl Panel {
    /// Creates the dashboard overlay (the tab) and its icon.
    pub fn create(&mut self, textures: &mut Textures, settings: &Settings) -> Result<(), String> {
        let (dash_overlay, icon_overlay) = vr()
            .create_dashboard_overlay(DASH_KEY, "Roommate Radar")
            .map_err(|e| format!("CreateDashboardOverlay: {e}"))?;
        self.dash_overlay = dash_overlay;
        textures.set(icon_overlay, &icon());
        vr().set_width(dash_overlay, 1.5);
        vr().set_mouse_input(dash_overlay, W as f32, height(settings) as f32);
        textures.set(dash_overlay, &self.render("Starting", settings));
        if !settings.warning_acked {
            vr().show_dashboard(DASH_KEY);
        }
        Ok(())
    }

    /// Which control (texture pixel, y down) is under the pointer; greyed-out rows don't hover or press.
    fn hit(&self, x: f32, y: f32, settings: &Settings) -> Option<Target> {
        if !settings.warning_acked {
            let on = (ACK_X0..=ACK_X0 + ACK_W).contains(&x) && (ACK_Y0..=ACK_Y0 + ACK_H).contains(&y);
            return on.then_some(Target::Ack);
        }
        if let Some(row) = self.popup {
            return at(row).1.hit_popup(row, x, y, settings);
        }
        let row = (0..ROWS).find(|&r| (y - row_cy(r)).abs() < ROW_H / 2.0)?; // none in the gaps
        let c = Some(at(row).1).filter(|c| x >= PAD && x <= W as f32 - PAD && !(c.dimmed)(settings))?;
        c.hit(row, x, y)
    }

    /// While a popup is open any press closes it (None: outside it); an item also picks that choice.
    fn press(&mut self, target: Option<Target>, settings: &mut Settings) {
        if target == Some(Target::Ack) {
            settings.warning_acked = true;
            settings.save();
        } else if let Some(row) = self.popup.take() {
            if let Some(target @ Target::Item(_)) = target {
                at(row).1.press(target, settings);
            }
        } else if let Some(target @ (Target::Row(row) | Target::Step(row, _))) = target
            && at(row).1.press(target, settings)
        {
            self.popup = Some(row);
        }
    }

    /// The pointer's events since the last call: hover and presses.
    pub fn poll(&mut self, settings: &mut Settings) {
        while let Some(ev) = vr().poll_overlay_event(self.dash_overlay) {
            let mouse = unsafe { ev.data.mouse };
            // Mouse y is up from the bottom of the texture.
            let target = self.hit(mouse.x, height(settings) as f32 - mouse.y, settings);
            if ev.eventType == openvr::EVREventType_VREvent_MouseMove && target != self.hover {
                self.hover = target;
                self.drawn_status.clear();
                if target.is_some() {
                    // the tick the Steam UI gives when the pointer reaches a new item
                    vr().haptic_pulse(ev.trackedDeviceIndex, HOVER_TICK_US);
                }
            }
            // outside an open popup closes it
            if ev.eventType == openvr::EVREventType_VREvent_MouseButtonDown
                && (target.is_some() || self.popup.is_some())
            {
                self.press(target, settings);
                self.drawn_status.clear();
                if target == Some(Target::Ack) {
                    vr().set_mouse_input(self.dash_overlay, W as f32, SETTINGS_H as f32); // the settings' height from now on
                }
            }
        }
    }

    /// Redraws the tab, while it's showing, when the status or anything on it changed.
    pub fn update(&mut self, textures: &mut Textures, status: &str, settings: &Settings) {
        if vr().is_visible(self.dash_overlay) && self.drawn_status != status {
            self.drawn_status = status.to_string();
            textures.set(self.dash_overlay, &self.render(status, settings));
        }
    }

    fn render(&self, status: &str, settings: &Settings) -> Image {
        let hover = self.hover;
        let mut img = Image::new(W, height(settings));
        fill_rrect(&mut img, 0.0, 0.0, W as f32, height(settings) as f32, 32.0, BG, 0.94);
        if !settings.warning_acked {
            let mut centered = |y: f32, text: &str, rgb: Rgb, size: f32| {
                img.text((W as f32 - draw::text_end_x(0, text, size) as f32) as i32 / 2, y as i32, text, rgb, size)
            };
            centered(HEAD_Y, "Before You Play", WHITE, HEAD_SIZE);
            for (i, line) in WARNING.iter().enumerate() {
                centered(WARN_Y + i as f32 * WARN_LINE_SPACING, line, LIGHT, WARN_SIZE);
            }
            let ic = Image { w: 176, h: 176, px: include_bytes!("../assets/icon-176.rgba").to_vec() }; // icon()'s, bigger
            let x0 = (W - ic.w) / 2;
            for (i, p) in ic.px.chunks(4).enumerate() {
                over(img.at(x0 + i % ic.w, ICON_Y0 as usize + i / ic.w), [p[0], p[1], p[2]], p[3] as f32 / 255.0);
            }
            let a = if hover == Some(Target::Ack) { 1.0 } else { 0.85 };
            fill_rrect(&mut img, ACK_X0, ACK_Y0, ACK_W, ACK_H, ACK_H / 2.0, ACCENT, a);
            let label = "I Understand";
            let x = (W as f32 - draw::text_end_x(0, label, ACK_SIZE) as f32) / 2.0;
            img.text(x as i32, (ACK_Y0 + ACK_H / 2.0 + ACK_SIZE * 0.36) as i32, label, WHITE, ACK_SIZE);
            return img;
        }
        img.text(PAD as i32, 100, "Roommate Radar", WHITE, 56.0);
        img.text(PAD as i32, 150, status, GREY, 30.0);
        for row in 0..ROWS {
            at(row).1.render(&mut img, row, hover, settings);
        }
        if let Some(row) = self.popup {
            at(row).1.render_popup(&mut img, row, hover, settings);
        }
        img
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::Ordering;

    fn row(title: &str) -> usize {
        (0..ROWS).find(|&r| at(r).1.title == title).unwrap()
    }

    /// Past the first-run warning.
    fn warned() -> Settings {
        Settings { warning_acked: true, ..Settings::default() }
    }

    #[test]
    fn first_run_warning() {
        // Only the button hits until it's pressed; the text ends above it.
        let (mut p, mut settings) = (Panel::default(), Settings::default());
        assert_eq!(p.hit(500.0, row_cy(row("Detect People")), &settings), None);
        let ack = p.hit(W as f32 / 2.0, ACK_Y0 + ACK_H / 2.0, &settings);
        assert_eq!(ack, Some(Target::Ack));
        assert!(WARNING.iter().all(|l| draw::text_end_x(0, l, WARN_SIZE) as f32 <= W as f32 - 2.0 * PAD));
        assert!(WARN_Y + (WARNING.len() - 1) as f32 * WARN_LINE_SPACING + 40.0 < ACK_Y0);
        let mut img = p.render("", &settings);
        assert!(img.at(W / 2, (ACK_Y0 + 4.0) as usize)[0] > 200, "the button");
        p.press(ack, &mut settings);
        assert!(settings.warning_acked && p.hit(500.0, row_cy(row("Detect People")), &settings).is_some());
    }

    #[test]
    fn sections() {
        // A gap (that hits nothing) above each section after the first; the last row ends 30 px above the bottom.
        let people = row("Detect People");
        assert!(at(people - 1).0 == 0 && at(people).0 == 1 && at(ROWS - 1).0 == 2);
        assert_eq!(Panel::default().hit(500.0, row_cy(people) - ROW_H / 2.0 - GAP / 2.0, &Settings::default()), None);
        assert_eq!(row_cy(ROWS - 1) + ROW_H / 2.0 + 30.0, SETTINGS_H as f32);
    }

    #[test]
    fn hit_test() {
        // Rows and buttons hit-test to their controls; greyed-out ones don't.
        let (p, mut settings) = (Panel::default(), warned());
        let (people, range, hands) = (row("Detect People"), row("Max Range"), row("Ignore My Hands"));
        assert_eq!(p.hit(500.0, row_cy(people), &settings), Some(Target::Row(people)));
        assert_eq!(p.hit(PLUS_CX, row_cy(range), &settings), Some(Target::Step(range, 1)));
        assert_eq!(p.hit(500.0, row_cy(hands), &settings), Some(Target::Row(hands)));
        assert_eq!(p.hit(500.0, 50.0, &settings), None);
        assert_eq!(
            p.hit(500.0, row_cy(row("Autorun on startup")), &settings),
            None,
            "greyed out: not started by Steam"
        );
        settings.enabled = false;
        assert_eq!(p.hit(500.0, row_cy(people), &settings), None);
        assert_eq!(p.hit(500.0, row_cy(0), &settings), Some(Target::Row(0)));
    }

    #[test]
    fn press_and_render() {
        let (mut p, mut settings) = (Panel::default(), Settings::default());
        p.press(Some(Target::Row(row("Detect People"))), &mut settings);
        for _ in 0..20 {
            p.press(Some(Target::Step(row("Max Range"), 1)), &mut settings);
        }
        assert!(!settings.people && settings.max_dist == MAX_DIST);
        p.hover = Some(Target::Row(row("Detect Cats and Dogs")));
        let mut img = p.render("Watching", &settings);
        assert!(img.at(0, 0)[3] == 0 && img.at(500, 5)[3] > 200);
    }

    #[test]
    fn play_area_detail() {
        // The row's text (a static, so not formatted) says measure.rs's margin.
        let m = format!("within {}m ", crate::measure::PLAY_AREA_MARGIN);
        assert!(at(row("Only Detect Near Play Area")).1.detail.contains(&m));
    }

    #[test]
    fn icon_raster() {
        // The size icon() claims; transparent outside the rounded tile, the white "you" dot at the fan's apex.
        let mut i = icon();
        assert_eq!(i.px.len(), 128 * 128 * 4);
        assert!(i.at(0, 0)[3] == 0 && i.at(64, 106) == [245, 245, 247, 255]);
    }

    #[test]
    fn dropdowns() {
        // Pick, a press outside closes, the rate popup fits the panel.
        let (mut p, mut settings) = (Panel::default(), warned());
        let (size_row, rate_row) = (row("Detection Resolution"), row("Detection Rate"));
        assert_eq!(
            (at(size_row).1.choices(&settings).1(2), at(rate_row).1.choices(&settings).1(0)),
            ("High (640px)".into(), "2Hz".into())
        );
        p.press(Some(Target::Row(size_row)), &mut settings);
        let iy = at(size_row).1.popup_top(size_row, &settings) + 2.5 * ITEM_H; // the third choice
        assert_eq!(p.popup, Some(size_row));
        assert_eq!(p.hit(500.0, iy, &settings), None);
        assert_eq!(p.hit(DROP_X0 + 10.0, iy, &settings), Some(Target::Item(2)));
        p.press(Some(Target::Item(2)), &mut settings);
        assert!(settings.size == 640 && detector::SIZE.load(Ordering::Relaxed) == 640 && p.popup.is_none());
        p.press(Some(Target::Row(size_row)), &mut settings);
        p.press(None, &mut settings);
        p.press(Some(Target::Row(rate_row)), &mut settings);
        assert!(at(rate_row).1.popup_top(rate_row, &settings) + 4.0 * ITEM_H <= SETTINGS_H as f32);
        p.press(Some(Target::Item(3)), &mut settings);
        assert!(settings.size == 640 && settings.hz == 16.0 && detector::HZ.load() == 16.0 && p.popup.is_none());
    }
}
