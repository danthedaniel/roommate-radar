//! The "Guardian" dashboard tab: header with a status line, then one row per setting. Drawn on the CPU like the labels
//! (draw.rs); the dashboard's laser pointer arrives as mouse events in texture pixels.
use crate::detector;
use crate::draw::{self, Image, Rgb, WHITE, over, rrect_sd};
use crate::openvr::{self, VROverlayHandle_t, vr};
use crate::settings::{RATES, SIZES, Settings};
use crate::textures::Textures;

pub const W: usize = 1000;
pub const H: usize = 1048;
const PAD: f32 = 48.0;
const TOP: f32 = 190.0;
const ROW_H: f32 = 92.0;

// Controls: one row per switch, then the dropdown rows and the range row
const ENABLED: usize = 0;
const AUTO_LAUNCH: usize = 1; // SteamVR's auto-launch for this app, not a Settings field
const GAMES_ONLY: usize = 2;
const PEOPLE: usize = 3;
const PETS: usize = 4;
const HANDS: usize = 5;
const SIZE: usize = 6; // dropdowns: open their popup
const RATE: usize = 7;
const RANGE_MINUS: usize = 8;
const RANGE_PLUS: usize = 9;
const ITEM: usize = 10; // the open popup's items: ITEM + choice index
const TOGGLES: usize = SIZE;
const RANGE_ROW: usize = RATE + 1;

const ROWS: [&str; 9] = [
    "Enabled",
    "Start with SteamVR",
    "Only in Games",
    "Detect People",
    "Detect Cats and Dogs",
    "Ignore My Hands",
    "Detection Resolution",
    "Detection Rate",
    "Range",
];
const SIZE_LABELS: [&str; 3] = ["Low (320px)", "Medium (480px)", "High (640px)"];
const RATE_LABELS: [&str; 4] = ["2Hz", "4Hz", "8Hz", "16Hz"];
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
const BG: Rgb = [20, 20, 24];

fn row_cy(row: usize) -> f32 {
    TOP + (row as f32 + 0.5) * ROW_H
}

fn choices(drop: usize) -> usize {
    if drop == SIZE { SIZES.len() } else { RATES.len() }
}

fn choice_label(drop: usize, i: usize) -> &'static str {
    if drop == SIZE { SIZE_LABELS[i] } else { RATE_LABELS[i] }
}

/// 0 for a --size / --max-hz that isn't one of the choices
fn choice(drop: usize, set: &Settings) -> usize {
    let i =
        if drop == SIZE { SIZES.iter().position(|&s| s == set.size) } else { RATES.iter().position(|&r| r == set.hz) };
    i.unwrap_or(0)
}

/// The popup opens with the current choice over the dropdown, moved up as needed to stay inside the panel.
fn popup_top(drop: usize, set: &Settings) -> f32 {
    (row_cy(drop) - (choice(drop, set) as f32 + 0.5) * ITEM_H).min(H as f32 - choices(drop) as f32 * ITEM_H - 8.0)
}

#[allow(clippy::too_many_arguments)]
fn fill_rrect(img: &mut Image, x0: f32, y0: f32, w: f32, h: f32, r: f32, rgb: Rgb, a: f32) {
    for y in (y0 as i32 - 1).max(0)..((y0 + h) as i32 + 2).min(H as i32) {
        for x in (x0 as i32 - 1).max(0)..((x0 + w) as i32 + 2).min(W as i32) {
            let sd = rrect_sd(x as f32 + 0.5 - x0, y as f32 + 0.5 - y0, w, h, 0.0, r);
            over(img.at(x as usize, y as usize), rgb, a * (0.5 - sd).clamp(0.0, 1.0));
        }
    }
}

fn fill_circle(img: &mut Image, cx: f32, cy: f32, r: f32, rgb: Rgb, a: f32) {
    fill_rrect(img, cx - r, cy - r, 2.0 * r, 2.0 * r, r, rgb, a);
}

/// Tab icon: the box's corner brackets on a dark tile.
fn icon() -> Image {
    let size = 128;
    let mut img = Image::new(size, size);
    let mut bx = draw::outline(80, 96, 0, 5.0);
    for y in 0..size {
        for x in 0..size {
            let p = img.at(x, y);
            over(
                p,
                BG,
                (0.5 - rrect_sd(x as f32 + 0.5, y as f32 + 0.5, size as f32, size as f32, 0.0, 24.0)).clamp(0.0, 1.0),
            );
            if (24..24 + 80).contains(&x) && (16..16 + 96).contains(&y) {
                let b = bx.at(x - 24, y - 16);
                over(p, [b[0], b[1], b[2]], b[3] as f32 / 255.0);
            }
        }
    }
    img
}

pub struct Panel {
    app_key: Option<String>, // SteamVR's key for this app; None if not started by Steam
    popup: Option<usize>,    // the dropdown (SIZE, RATE) whose popup is showing
    hover: Option<usize>,
    drawn: String, // the status the tab was last drawn with ("" = redraw)
    dash: VROverlayHandle_t,
}

impl Panel {
    pub fn new(app_key: Option<String>) -> Panel {
        Panel { app_key, popup: None, hover: None, drawn: String::new(), dash: 0 }
    }

    /// Creates the dashboard overlay (the tab) and its icon.
    pub fn create(&mut self, textures: &mut Textures, set: &Settings) -> Result<(), String> {
        let (dash, icon_ov) = vr()
            .create_dashboard_overlay("steamframe.guardian.dashboard", "Guardian")
            .map_err(|e| format!("CreateDashboardOverlay: {e}"))?;
        self.dash = dash;
        textures.set(icon_ov, &icon());
        vr().set_width(dash, 1.5);
        vr().set_mouse_input(dash, W as f32, H as f32);
        textures.set(dash, &self.render(None, "Starting", set));
        Ok(())
    }

    fn autolaunch(&self) -> bool {
        self.app_key.as_ref().is_some_and(|k| vr().auto_launch(k))
    }

    /// Greyed out: the detection settings while Enabled is off, Start with SteamVR when not started by Steam.
    fn dimmed(&self, row: usize, set: &Settings) -> bool {
        if row == AUTO_LAUNCH { self.app_key.is_none() } else { row > AUTO_LAUNCH && !set.enabled }
    }

    /// Which control (texture pixel, y down) is under the pointer: the whole row for toggles, the buttons for range.
    fn hit(&self, x: f32, y: f32, set: &Settings) -> Option<usize> {
        if let Some(drop) = self.popup {
            let i = ((y - popup_top(drop, set)) / ITEM_H).floor();
            let inside = (DROP_X0..=DROP_X0 + DROP_W).contains(&x) && i >= 0.0 && (i as usize) < choices(drop);
            return inside.then(|| ITEM + i as usize);
        }
        let row = ((y - TOP) / ROW_H).floor();
        if x < PAD || x > W as f32 - PAD || row < 0.0 || row as usize > RANGE_ROW || self.dimmed(row as usize, set) {
            return None; // greyed-out rows don't hover or press
        }
        let row = row as usize;
        if row < RANGE_ROW {
            return Some(row);
        }
        let near = |cx: f32| (x - cx).hypot(y - row_cy(RANGE_ROW)) < BTN_R + 12.0;
        if near(MINUS_CX) {
            Some(RANGE_MINUS)
        } else if near(PLUS_CX) {
            Some(RANGE_PLUS)
        } else {
            None
        }
    }

    /// While a popup is open any press closes it (None: outside it); an item also picks that choice.
    fn press(&mut self, ctl: Option<usize>, set: &mut Settings) {
        if let Some(drop) = self.popup.take() {
            let Some(i) = ctl.and_then(|c| c.checked_sub(ITEM)) else {
                return;
            };
            if drop == SIZE {
                set.size = SIZES[i];
                detector::SIZE.store(set.size, std::sync::atomic::Ordering::Relaxed); // the detector reloads
            } else {
                set.hz = RATES[i];
                detector::HZ.store(set.hz);
            }
            set.save();
            return;
        }
        match ctl {
            Some(ENABLED) => set.enabled = !set.enabled,
            Some(AUTO_LAUNCH) => {
                if let Some(k) = &self.app_key {
                    vr().set_auto_launch(k, !self.autolaunch());
                }
                return;
            }
            Some(PEOPLE) => set.people = !set.people,
            Some(PETS) => set.pets = !set.pets,
            Some(HANDS) => set.hands = !set.hands,
            Some(GAMES_ONLY) => set.games_only = !set.games_only,
            Some(RANGE_MINUS) => set.max_dist = (set.max_dist - 0.5).max(1.0),
            Some(RANGE_PLUS) => set.max_dist = (set.max_dist + 0.5).min(8.0),
            Some(c @ (SIZE | RATE)) => {
                self.popup = Some(c);
                return;
            }
            _ => return,
        }
        set.save();
    }

    /// The pointer's events since the last call: hover and presses.
    pub fn poll(&mut self, set: &mut Settings) {
        while let Some(ev) = vr().poll_overlay_event(self.dash) {
            let mouse = unsafe { ev.data.mouse };
            // Mouse y is up from the bottom of the texture.
            let c = self.hit(mouse.x, H as f32 - mouse.y, set);
            if ev.eventType == openvr::EVREventType_VREvent_MouseMove && c != self.hover {
                self.hover = c;
                self.drawn.clear();
                if c.is_some() {
                    // the tick the Steam UI gives when the pointer reaches a new item
                    vr().haptic_pulse(ev.trackedDeviceIndex, HOVER_TICK_US);
                }
            }
            // outside an open popup closes it
            if ev.eventType == openvr::EVREventType_VREvent_MouseButtonDown && (c.is_some() || self.popup.is_some()) {
                self.press(c, set);
                self.drawn.clear();
            }
        }
    }

    /// Redraws the tab, while it's showing, when the status or anything on it changed.
    pub fn update(&mut self, textures: &mut Textures, status: &str, set: &Settings) {
        if vr().is_visible(self.dash) && self.drawn != status {
            self.drawn = status.to_string();
            textures.set(self.dash, &self.render(self.hover, status, set));
        }
    }

    fn render(&self, hover: Option<usize>, status: &str, set: &Settings) -> Image {
        const GREY: Rgb = [150, 150, 160];
        const OFF: Rgb = [68, 68, 76];
        const BTN: Rgb = [52, 52, 60];
        let accent = draw::class_rgb(0);
        let (wf, hf) = (W as f32, H as f32);
        let mut img = Image::new(W, H);
        fill_rrect(&mut img, 0.0, 0.0, wf, hf, 32.0, BG, 0.94);
        img.text(PAD as i32, 100, "Guardian", WHITE, 56.0);
        img.text(PAD as i32, 150, status, GREY, 30.0);
        let on = [set.enabled, self.autolaunch(), set.games_only, set.people, set.pets, set.hands];
        for row in 0..=RANGE_ROW {
            let cy = row_cy(row);
            if hover == Some(row) {
                fill_rrect(
                    &mut img,
                    PAD - 20.0,
                    cy - ROW_H / 2.0 + 4.0,
                    wf - 2.0 * PAD + 40.0,
                    ROW_H - 8.0,
                    18.0,
                    WHITE,
                    0.05,
                );
            }
            if row > 0 {
                fill_rrect(&mut img, PAD, cy - ROW_H / 2.0, wf - 2.0 * PAD, 1.5, 0.0, WHITE, 0.08);
            }
            let dim = self.dimmed(row, set);
            let (fg, fade) = if dim { (GREY, 0.5) } else { (WHITE, 1.0) };
            if row == HANDS {
                // title nudged up, detail line below it
                img.text(PAD as i32, (cy + 1.0) as i32, ROWS[row], fg, 36.0);
                img.text(PAD as i32, (cy + 33.0) as i32, "Works best when holding controllers", GREY, 24.0);
            } else {
                img.text(PAD as i32, (cy + 13.0) as i32, ROWS[row], fg, 36.0);
            }
            if row < TOGGLES {
                // switch: accent track with the knob right when on; grey when dimmed
                let (x0, y0) = (wf - PAD - TOGGLE_W, cy - TOGGLE_H / 2.0);
                let track = if on[row] && !dim { accent } else { OFF };
                fill_rrect(&mut img, x0, y0, TOGGLE_W, TOGGLE_H, TOGGLE_H / 2.0, track, fade);
                let knob = if on[row] { x0 + TOGGLE_W - TOGGLE_H / 2.0 } else { x0 + TOGGLE_H / 2.0 };
                fill_circle(&mut img, knob, cy, TOGGLE_H / 2.0 - 5.0, WHITE, if dim { 0.6 } else { 1.0 });
            } else if row < RANGE_ROW {
                // dropdown: current choice and a chevron
                let a = if hover == Some(row) { 1.0 } else { 0.8 } * fade;
                fill_rrect(&mut img, DROP_X0, cy - DROP_H / 2.0, DROP_W, DROP_H, 14.0, BTN, a);
                img.text((DROP_X0 + 22.0) as i32, (cy + 10.0) as i32, choice_label(row, choice(row, set)), fg, 28.0);
                for k in -8i32..=8 {
                    let x = wf - PAD - 34.0 + k as f32;
                    fill_circle(&mut img, x, cy - 3.0 + (8 - k.abs()) as f32 * 0.6, 2.0, fg, 1.0);
                }
            } else {
                // range: (-) value (+)
                for (b, cx) in [MINUS_CX, PLUS_CX].into_iter().enumerate() {
                    let a = if hover == Some(RANGE_MINUS + b) { 1.0 } else { 0.8 } * fade;
                    fill_circle(&mut img, cx, cy, BTN_R, BTN, a);
                    fill_rrect(&mut img, cx - 11.0, cy - 1.5, 22.0, 3.0, 1.5, fg, 1.0);
                    if b == 1 {
                        fill_rrect(&mut img, cx - 1.5, cy - 11.0, 3.0, 22.0, 1.5, fg, 1.0);
                    }
                }
                let v = format!("{:.1} m", set.max_dist);
                let tw = draw::text_width(0, &v, 36.0);
                img.text(((MINUS_CX + PLUS_CX) / 2.0 - tw as f32 / 2.0) as i32, (cy + 13.0) as i32, &v, fg, 36.0);
            }
        }
        if let Some(drop) = self.popup {
            // over the rows: the choices, the current one in the accent color
            let (y0, n) = (popup_top(drop, set), choices(drop));
            fill_rrect(&mut img, DROP_X0 - 2.0, y0 - 2.0, DROP_W + 4.0, n as f32 * ITEM_H + 4.0, 16.0, WHITE, 0.15);
            fill_rrect(&mut img, DROP_X0, y0, DROP_W, n as f32 * ITEM_H, 14.0, BTN, 1.0);
            for i in 0..n {
                let iy = y0 + i as f32 * ITEM_H;
                if hover == Some(ITEM + i) {
                    fill_rrect(&mut img, DROP_X0 + 6.0, iy + 4.0, DROP_W - 12.0, ITEM_H - 8.0, 10.0, WHITE, 0.1);
                }
                let fg = if i == choice(drop, set) { accent } else { WHITE };
                img.text((DROP_X0 + 22.0) as i32, (iy + ITEM_H / 2.0 + 10.0) as i32, choice_label(drop, i), fg, 28.0);
            }
        }
        img
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::Ordering;

    #[test]
    fn hit_test() {
        // Rows and buttons hit-test to their controls; greyed-out ones don't.
        let (p, mut set) = (Panel::new(None), Settings::default());
        assert_eq!(p.hit(500.0, row_cy(PEOPLE), &set), Some(PEOPLE));
        assert_eq!(p.hit(PLUS_CX, row_cy(RANGE_ROW), &set), Some(RANGE_PLUS));
        assert_eq!(p.hit(500.0, row_cy(HANDS), &set), Some(HANDS));
        assert_eq!(p.hit(500.0, 50.0, &set), None);
        assert_eq!(p.hit(500.0, row_cy(AUTO_LAUNCH), &set), None, "greyed out: not started by Steam");
        set.enabled = false;
        assert_eq!(p.hit(500.0, row_cy(PEOPLE), &set), None);
        assert_eq!(p.hit(500.0, row_cy(ENABLED), &set), Some(ENABLED));
    }

    #[test]
    fn press_and_render() {
        let (mut p, mut set) = (Panel::new(None), Settings::default());
        p.press(Some(PEOPLE), &mut set);
        for _ in 0..20 {
            p.press(Some(RANGE_PLUS), &mut set);
        }
        assert!(!set.people && set.max_dist == 8.0);
        let mut img = p.render(Some(PETS), "Watching", &set);
        assert!(img.at(0, 0)[3] == 0 && img.at(500, 5)[3] > 200);
    }

    #[test]
    fn dropdowns() {
        // Pick, a press outside closes, the rate popup fits the panel.
        let (mut p, mut set) = (Panel::new(None), Settings::default());
        p.press(Some(SIZE), &mut set);
        let iy = popup_top(SIZE, &set) + 2.5 * ITEM_H; // the third choice
        assert_eq!(p.popup, Some(SIZE));
        assert_eq!(p.hit(500.0, iy, &set), None);
        assert_eq!(p.hit(DROP_X0 + 10.0, iy, &set), Some(ITEM + 2));
        p.press(Some(ITEM + 2), &mut set);
        assert!(set.size == 640 && detector::SIZE.load(Ordering::Relaxed) == 640 && p.popup.is_none());
        p.press(Some(SIZE), &mut set);
        p.press(None, &mut set);
        p.press(Some(RATE), &mut set);
        assert!(popup_top(RATE, &set) + 4.0 * ITEM_H <= H as f32);
        p.press(Some(ITEM + 3), &mut set);
        assert!(set.size == 640 && set.hz == 16.0 && detector::HZ.load() == 16.0 && p.popup.is_none());
    }
}
