//! roommate-radar's settings: set from the dashboard tab (panel.rs), saved to ~/.config/roommate-radar.conf.
use std::path::PathBuf;

pub const SIZES: [i32; 3] = [320, 480, 640];
pub const RATES: [f32; 4] = [2.0, 4.0, 8.0, 16.0];
pub const MIN_DIST: f32 = 1.5; // max_dist's range (m)
pub const MAX_DIST: f32 = 6.0; // stereo depth noise grows like distance^2: past this it's too rough to show

#[derive(Clone, Debug)]
pub struct Settings {
    pub enabled: bool,
    pub people: bool,
    pub pets: bool,            // cats and dogs
    pub hands: bool,           // ignore boxes around a controller: your hands (measure.rs)
    pub games_only: bool,      // pause while no game is running (SteamVR home, dashboard)
    pub play_area: bool,       // ignore detections outside the play area's boundary (measure.rs)
    pub max_dist: f32,         // ignore detections farther than this (m, horizontal)
    pub size: i32,             // YOLO per-view size, one of SIZES
    pub hz: f32,               // YOLO runs per second, one of RATES
    pub autostart: bool,       // start with SteamVR (the driver launches the app, autostart.rs)
    pub gameid: Option<u64>,   // not in the tab: Steam's id for this app (SteamGameId), for the driver to launch it
    pub warned: bool,          // the tab's first-run warning was acknowledged (panel.rs)
    pub align: [f32; 3],       // not in the tab: the rectification's correction, rad (selfcal.rs); saved in degrees
    pub path: Option<PathBuf>, // where save() writes; None: nowhere (tests)
}

impl Default for Settings {
    fn default() -> Settings {
        Settings {
            enabled: true,
            people: true,
            pets: true,
            hands: true,
            games_only: false,
            play_area: false,
            max_dist: 3.0,
            size: 480,
            hz: 4.0,
            autostart: true,
            gameid: None,
            warned: false,
            align: [0.0; 3],
            path: None,
        }
    }
}

impl Settings {
    /// The defaults, overridden by what's saved at `path`.
    pub fn load(path: PathBuf) -> Settings {
        let mut s = Settings { path: Some(path), ..Settings::default() };
        let text = std::fs::read_to_string(s.path.as_ref().unwrap()).unwrap_or_default();
        for (k, v) in text.lines().filter_map(|l| l.split_once('=')) {
            let v = v.trim();
            if k.trim() == "align" {
                let w: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
                if let [x, y, z] = w[..]
                    && [x, y, z].iter().all(|a| a.abs() < 5.0)
                {
                    s.align = [x, y, z].map(f32::to_radians);
                }
                continue;
            }
            let Ok(f) = v.parse::<f32>() else {
                continue;
            };
            match k.trim() {
                "enabled" => s.enabled = f != 0.0,
                "people" => s.people = f != 0.0,
                "pets" => s.pets = f != 0.0,
                "hands" => s.hands = f != 0.0,
                "games_only" => s.games_only = f != 0.0,
                "play_area" => s.play_area = f != 0.0,
                "autostart" => s.autostart = f != 0.0,
                "warned" => s.warned = f != 0.0,
                "gameid" => s.gameid = v.parse().ok(),
                // A range saved when the maximum was larger: the largest now
                "max_dist" if (MIN_DIST..=20.0).contains(&f) => s.max_dist = f.min(MAX_DIST),
                "size" if SIZES.contains(&(f as i32)) => s.size = f as i32,
                "hz" if RATES.contains(&f) => s.hz = f,
                _ => {}
            }
        }
        s
    }

    pub fn save(&self) {
        let Some(path) = &self.path else {
            return;
        };
        let b = |x: bool| x as i32;
        let _ = std::fs::write(
            path,
            format!(
                "enabled={}\npeople={}\npets={}\nhands={}\ngames_only={}\nplay_area={}\nmax_dist={:.1}\nsize={}\nhz={}\nautostart={}\nwarned={}\nalign={}\n{}",
                b(self.enabled),
                b(self.people),
                b(self.pets),
                b(self.hands),
                b(self.games_only),
                b(self.play_area),
                self.max_dist,
                self.size,
                self.hz,
                b(self.autostart),
                b(self.warned),
                self.align.map(|a| format!("{:.4}", a.to_degrees())).join(","),
                self.gameid.map(|g| format!("gameid={g}\n")).unwrap_or_default()
            ),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn saved_settings_load_back() {
        let path = std::env::temp_dir().join(format!("roommate-radar-test-{}.conf", std::process::id()));
        let s = Settings {
            people: false,
            play_area: true,
            max_dist: 4.5,
            size: 640,
            hz: 16.0,
            autostart: false,
            warned: true,
            gameid: Some(16375998297183092736),
            align: [0.0115, -0.0193, 0.0087],
            path: Some(path.clone()),
            ..Settings::default()
        };
        s.save();
        let l = Settings::load(path.clone());
        assert!(!l.people && l.pets && l.play_area && l.max_dist == 4.5 && l.size == 640 && l.hz == 16.0);
        assert!(!l.autostart && l.warned && l.gameid == Some(16375998297183092736), "a game id doesn't fit in an f32");
        assert!(l.align.iter().zip(s.align).all(|(a, b)| (a - b).abs() < 1e-5), "{:?}", l.align);
        std::fs::write(&path, "max_dist=8\nsize=100\n").unwrap();
        let l = Settings::load(path.clone());
        assert!(l.max_dist == MAX_DIST && l.size == 480, "a larger range: the maximum; a bad size: the default");
        std::fs::write(&path, "max_dist=30\n").unwrap();
        let l = Settings::load(path.clone());
        std::fs::remove_file(path).unwrap();
        assert!(l.max_dist == Settings::default().max_dist, "nonsense: the default");
    }
}
