//! CPU drawing into straight-alpha RGBA pixels (anti-aliased shapes, text): the box, label and chevron textures here,
//! the dashboard tab in panel.rs.
use std::sync::OnceLock;

#[cfg(not(test))]
const FONT: &str = "NotoSans-Medium.ttf"; // shipped with the app
#[cfg(test)]
const FONT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../third_party/fonts/NotoSans-Medium.ttf");

pub type Rgb = [u8; 3];
pub const WHITE: Rgb = [245, 245, 247];

pub fn class_rgb(cls: usize) -> Rgb {
    if cls == 0 { [255, 69, 58] } else { [255, 184, 0] } // person, pet
}

/// Straight-alpha "over" of color rgb at coverage a onto RGBA pixel p.
pub fn over(p: &mut [u8], rgb: Rgb, a: f32) {
    if a <= 0.0 {
        return;
    }
    let da = p[3] as f32 / 255.0;
    let oa = a + da * (1.0 - a);
    for c in 0..3 {
        p[c] = ((rgb[c] as f32 * a + p[c] as f32 * da * (1.0 - a)) / oa).round() as u8;
    }
    p[3] = (oa * 255.0).round() as u8;
}

/// Signed distance (px, < 0 inside) from (x, y) to a w x h rounded rect inset by `inset`, corner radius r.
pub fn rrect_sd(x: f32, y: f32, w: f32, h: f32, inset: f32, r: f32) -> f32 {
    let qx = (x - w / 2.0).abs() - (w / 2.0 - inset - r);
    let qy = (y - h / 2.0).abs() - (h / 2.0 - inset - r);
    qx.max(0.0).hypot(qy.max(0.0)) + qx.max(qy).min(0.0) - r
}

/// RGBA pixels, w x h, row-major.
pub struct Image {
    pub w: usize,
    pub h: usize,
    pub px: Vec<u8>,
}

impl Image {
    pub fn new(w: usize, h: usize) -> Image {
        Image { w, h, px: vec![0; w * h * 4] }
    }

    pub fn at(&mut self, x: usize, y: usize) -> &mut [u8] {
        let i = (y * self.w + x) * 4;
        &mut self.px[i..i + 4]
    }

    /// Draws UTF-8 text `size` px tall with its baseline at y.
    pub fn text(&mut self, x: i32, y: i32, s: &str, rgb: Rgb, size: f32) {
        draw_text(Some(self), x, y, s, rgb, size);
    }
}

fn font() -> &'static fontdue::Font {
    static F: OnceLock<fontdue::Font> = OnceLock::new();
    F.get_or_init(|| {
        let bytes = std::fs::read(FONT).unwrap_or_else(|e| panic!("can't load {FONT}: {e}"));
        fontdue::Font::from_bytes(bytes, fontdue::FontSettings::default()).unwrap_or_else(|e| panic!("{FONT}: {e}"))
    })
}

/// The pen x after `s` drawn from x, `size` px tall.
pub fn text_width(x: i32, s: &str, size: f32) -> i32 {
    draw_text(None, x, 0, s, WHITE, size)
}

fn draw_text(mut img: Option<&mut Image>, mut x: i32, y: i32, s: &str, rgb: Rgb, size: f32) -> i32 {
    for ch in s.chars() {
        let m = font().metrics(ch, size);
        if let Some(img) = img.as_deref_mut() {
            let (m, bitmap) = font().rasterize(ch, size);
            let top = y - (m.ymin + m.height as i32);
            for r in 0..m.height {
                for c in 0..m.width {
                    let (px, py) = (x + m.xmin + c as i32, top + r as i32);
                    if px >= 0 && py >= 0 && (px as usize) < img.w && (py as usize) < img.h {
                        over(img.at(px as usize, py as usize), rgb, bitmap[r * m.width + c] as f32 / 255.0);
                    }
                }
            }
        }
        x += m.advance_width.round() as i32;
    }
    x
}

/// Anti-aliased rounded outline, `line` px wide (thinner and fainter along the sides, bold at the corners), over a
/// barely-there fill. Sized to the box's aspect.
pub fn outline(w: usize, h: usize, cls: usize, line: f32) -> Image {
    let rgb = class_rgb(cls);
    let mut img = Image::new(w, h);
    let (wf, hf) = (w as f32, h as f32);
    let m = wf.min(hf);
    let (r, corner) = (m * 0.08, m * 0.2);
    for y in 0..h {
        for x in 0..w {
            let (fx, fy) = (x as f32 + 0.5, y as f32 + 0.5);
            let sd = rrect_sd(fx, fy, wf, hf, 1.0, r);
            let c = (fx - wf / 2.0).abs() > wf / 2.0 - corner && (fy - hf / 2.0).abs() > hf / 2.0 - corner;
            let lw = if c { line * 2.5 } else { line }; // stroke lies just inside the edge: sd in [-lw, 0]
            let a = (lw / 2.0 + 0.5 - (sd + lw / 2.0).abs()).clamp(0.0, 1.0) * if c { 1.0 } else { 0.6 };
            over(img.at(x, y), rgb, a.max(if sd < 0.0 { 0.06 } else { 0.0 }));
        }
    }
    img
}

/// Dark translucent pill: class-colored dot, class name in white, confidence (if given) in the class color.
pub fn label(name: &str, pct: Option<i32>, cls: usize) -> Image {
    let (rgb, bg) = (class_rgb(cls), [16, 16, 20]);
    let num = pct.map(|p| format!("{p}%")).unwrap_or_default();
    let h = 64;
    let (dot, gap, space, pad) = (14.0, 12, 12, h / 2 - 6);
    let x_name = pad + dot as i32 + gap;
    let x_num = text_width(x_name, name, 40.0) + if num.is_empty() { 0 } else { space };
    let w = (text_width(x_num, &num, 40.0) + pad) as usize;
    let mut img = Image::new(w, h as usize);
    for y in 0..h as usize {
        for x in 0..w {
            let (fx, fy) = (x as f32 + 0.5, y as f32 + 0.5);
            let p = img.at(x, y);
            over(p, bg, 0.72 * (0.5 - rrect_sd(fx, fy, w as f32, h as f32, 0.0, h as f32 / 2.0)).clamp(0.0, 1.0));
            let d = (fx - (pad as f32 + dot / 2.0)).hypot(fy - h as f32 / 2.0);
            over(p, rgb, (dot / 2.0 + 0.5 - d).clamp(0.0, 1.0));
        }
    }
    let m = font().horizontal_line_metrics(40.0).unwrap();
    let base = h / 2 + ((m.ascent + m.descent) / 2.0).round() as i32; // text centered vertically
    img.text(x_name, base, name, WHITE, 40.0);
    img.text(x_num, base, &num, rgb, 40.0);
    img
}

/// Distance (px) from p to the segment a-b.
fn seg_dist(p: [f32; 2], a: [f32; 2], b: [f32; 2]) -> f32 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
    (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
}

/// A chevron pointing left (or right), w x h (2:3), in the class's color over a dark halo so it shows on any scene.
pub fn chevron(w: usize, h: usize, left: bool, cls: usize) -> Image {
    let mut img = Image::new(w, h);
    let (wf, hf) = (w as f32, h as f32);
    let (line, halo) = (wf * 0.18, wf * 0.06);
    let (tip, top, bottom) = ([wf * 0.28, hf / 2.0], [wf * 0.72, hf * 0.12], [wf * 0.72, hf * 0.88]);
    for y in 0..h {
        for x in 0..w {
            let fx = x as f32 + 0.5;
            let p = [if left { fx } else { wf - fx }, y as f32 + 0.5];
            let d = seg_dist(p, tip, top).min(seg_dist(p, tip, bottom));
            let px = img.at(x, y);
            over(px, [0, 0, 0], 0.5 * (line / 2.0 + halo + 0.5 - d).clamp(0.0, 1.0));
            over(px, class_rgb(cls), (line / 2.0 + 0.5 - d).clamp(0.0, 1.0));
        }
    }
    img
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn label_texture() {
        // Pill sized to the text (transparent outside its rounded ends), white text, colored dot.
        let mut l = label("person", Some(87), 0);
        let white = l.px.chunks(4).filter(|p| p[0] > 230 && p[1] > 230 && p[3] > 230).count();
        assert!(l.h == 64 && l.w > 3 * l.h && white > 200);
        assert!(l.at(0, 0)[3] == 0 && l.at(l.w / 2, 1)[3] > 150 && l.at(64 / 2 - 6 + 7, 64 / 2)[1] == 69);
    }

    #[test]
    fn chevron_texture() {
        // Opaque red at the tip, clear in the corners and the open side; the right one mirrors the left.
        let mut l = chevron(128, 192, true, 0);
        let mut r = chevron(128, 192, false, 0);
        assert!(l.at(36, 96)[3] == 255 && l.at(36, 96)[0] == 255 && l.at(0, 0)[3] == 0 && l.at(110, 96)[3] == 0);
        assert!((0..192).all(|y| (0..128).all(|x| l.at(x, y)[3] == r.at(127 - x, y)[3])));
    }

    #[test]
    fn box_texture() {
        // Bold opaque corners, fainter sides, nearly clear middle, nothing outside the rounded corner.
        let mut b = outline(256, 512, 0, 4.0);
        assert!(b.at(0, 0)[3] == 0 && b.at(3, 40)[3] > 230 && b.at(128, 256)[3] < 20);
        assert!(b.at(3, 256)[3] > 100 && b.at(3, 256)[3] < 180);
    }
}
