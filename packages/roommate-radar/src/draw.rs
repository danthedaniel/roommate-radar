//! CPU drawing into straight-alpha RGBA pixels (anti-aliased shapes, text): the box and label textures here, the
//! dashboard tab in panel.rs.
use std::sync::OnceLock;

#[cfg(not(test))]
const FONT_PATH: &str = "NotoSans-Medium.ttf"; // shipped with the app
#[cfg(test)]
const FONT_PATH: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../third_party/fonts/NotoSans-Medium.ttf");

pub type Rgb = [u8; 3];
pub const WHITE: Rgb = [245, 245, 247];

pub const fn class_rgb(cls: usize) -> Rgb {
    if cls == 0 { [255, 69, 58] } else { [255, 184, 0] } // person, pet
}

/// Straight-alpha "over" of color rgb at coverage `coverage` onto RGBA pixel p.
pub fn over(p: &mut [u8], rgb: Rgb, coverage: f32) {
    if coverage <= 0.0 {
        return;
    }
    let dst_alpha = p[3] as f32 / 255.0;
    let out_alpha = coverage + dst_alpha * (1.0 - coverage);
    for c in 0..3 {
        p[c] = ((rgb[c] as f32 * coverage + p[c] as f32 * dst_alpha * (1.0 - coverage)) / out_alpha).round() as u8;
    }
    p[3] = (out_alpha * 255.0).round() as u8;
}

/// Signed distance (px, < 0 inside) from (x, y) to a w x h rounded rect inset by `inset`, corner radius `radius`.
pub fn rounded_rect_sdf(x: f32, y: f32, w: f32, h: f32, inset: f32, radius: f32) -> f32 {
    let qx = (x - w / 2.0).abs() - (w / 2.0 - inset - radius);
    let qy = (y - h / 2.0).abs() - (h / 2.0 - inset - radius);
    qx.max(0.0).hypot(qy.max(0.0)) + qx.max(qy).min(0.0) - radius
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
        let bytes = std::fs::read(FONT_PATH).unwrap_or_else(|e| panic!("can't load {FONT_PATH}: {e}"));
        fontdue::Font::from_bytes(bytes, fontdue::FontSettings::default())
            .unwrap_or_else(|e| panic!("{FONT_PATH}: {e}"))
    })
}

/// The pen x after `s` drawn from x, `size` px tall.
pub fn text_end_x(x: i32, s: &str, size: f32) -> i32 {
    draw_text(None, x, 0, s, WHITE, size)
}

fn draw_text(mut img: Option<&mut Image>, mut x: i32, y: i32, s: &str, rgb: Rgb, size: f32) -> i32 {
    for ch in s.chars() {
        let glyph = font().metrics(ch, size);
        if let Some(img) = img.as_deref_mut() {
            let (raster, bitmap) = font().rasterize(ch, size);
            let top = y - (raster.ymin + raster.height as i32);
            for row in 0..raster.height {
                for col in 0..raster.width {
                    let (dst_x, dst_y) = (x + raster.xmin + col as i32, top + row as i32);
                    if dst_x >= 0 && dst_y >= 0 && (dst_x as usize) < img.w && (dst_y as usize) < img.h {
                        let coverage = bitmap[row * raster.width + col] as f32 / 255.0;
                        over(img.at(dst_x as usize, dst_y as usize), rgb, coverage);
                    }
                }
            }
        }
        x += glyph.advance_width.round() as i32;
    }
    x
}

/// Anti-aliased rounded outline, `line_px` wide (thinner and fainter along the sides, bold at the corners), over a
/// barely-there fill. Sized to the box's aspect.
pub fn outline(w: usize, h: usize, cls: usize, line_px: f32) -> Image {
    let rgb = class_rgb(cls);
    let mut img = Image::new(w, h);
    let (wf, hf) = (w as f32, h as f32);
    let short_side_px = wf.min(hf);
    let (radius, corner_zone_px) = (short_side_px * 0.08, short_side_px * 0.2);
    for y in 0..h {
        for x in 0..w {
            let (fx, fy) = (x as f32 + 0.5, y as f32 + 0.5);
            let sd = rounded_rect_sdf(fx, fy, wf, hf, 1.0, radius);
            let in_corner =
                (fx - wf / 2.0).abs() > wf / 2.0 - corner_zone_px && (fy - hf / 2.0).abs() > hf / 2.0 - corner_zone_px;
            // the stroke lies just inside the edge: sd in [-stroke_px, 0]
            let stroke_px = if in_corner { line_px * 2.5 } else { line_px };
            let a = (stroke_px / 2.0 + 0.5 - (sd + stroke_px / 2.0).abs()).clamp(0.0, 1.0)
                * if in_corner { 1.0 } else { 0.6 };
            over(img.at(x, y), rgb, a.max(if sd < 0.0 { 0.06 } else { 0.0 }));
        }
    }
    img
}

/// Dark translucent pill: class-colored dot, class name in white, confidence (if given) in the class color.
pub fn label_pill(name: &str, pct: Option<i32>, cls: usize) -> Image {
    let (rgb, bg) = (class_rgb(cls), [16, 16, 20]);
    let pct_text = pct.map(|p| format!("{p}%")).unwrap_or_default();
    let h = 64;
    let (dot_px, dot_gap_px, pct_gap_px, pad_px) = (14.0, 12, 12, h / 2 - 6);
    let x_name = pad_px + dot_px as i32 + dot_gap_px;
    let x_pct = text_end_x(x_name, name, 40.0) + if pct_text.is_empty() { 0 } else { pct_gap_px };
    let w = (text_end_x(x_pct, &pct_text, 40.0) + pad_px) as usize;
    let mut img = Image::new(w, h as usize);
    for y in 0..h as usize {
        for x in 0..w {
            let (fx, fy) = (x as f32 + 0.5, y as f32 + 0.5);
            let p = img.at(x, y);
            over(
                p,
                bg,
                0.72 * (0.5 - rounded_rect_sdf(fx, fy, w as f32, h as f32, 0.0, h as f32 / 2.0)).clamp(0.0, 1.0),
            );
            let d = (fx - (pad_px as f32 + dot_px / 2.0)).hypot(fy - h as f32 / 2.0);
            over(p, rgb, (dot_px / 2.0 + 0.5 - d).clamp(0.0, 1.0));
        }
    }
    let line = font().horizontal_line_metrics(40.0).unwrap();
    let baseline = h / 2 + ((line.ascent + line.descent) / 2.0).round() as i32; // text centered vertically
    img.text(x_name, baseline, name, WHITE, 40.0);
    img.text(x_pct, baseline, &pct_text, rgb, 40.0);
    img
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn label_texture() {
        // Pill sized to the text (transparent outside its rounded ends), white text, colored dot.
        let mut l = label_pill("person", Some(87), 0);
        let white = l.px.chunks(4).filter(|p| p[0] > 230 && p[1] > 230 && p[3] > 230).count();
        assert!(l.h == 64 && l.w > 3 * l.h && white > 200);
        assert!(l.at(0, 0)[3] == 0 && l.at(l.w / 2, 1)[3] > 150 && l.at(64 / 2 - 6 + 7, 64 / 2)[1] == 69);
    }

    #[test]
    fn box_texture() {
        // Bold opaque corners, fainter sides, nearly clear middle, nothing outside the rounded corner.
        let mut b = outline(256, 512, 0, 4.0);
        assert!(b.at(0, 0)[3] == 0 && b.at(3, 40)[3] > 230 && b.at(128, 256)[3] < 20);
        assert!(b.at(3, 256)[3] > 100 && b.at(3, 256)[3] < 180);
    }
}
