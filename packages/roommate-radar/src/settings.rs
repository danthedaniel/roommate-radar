//! roommate-radar's settings: set from the dashboard tab (panel.rs), saved to ~/.config/roommate-radar.conf.
use std::path::PathBuf;

const CONF_PATH: &str = ".config/roommate-radar.conf"; // under $HOME; the driver's autostart reads it too
pub const SIZES: [i32; 3] = [320, 480, 640];
pub const RATES: [f32; 4] = [2.0, 4.0, 8.0, 16.0];
pub const MIN_DIST: f32 = 1.5; // max_dist's range (m)
pub const MAX_DIST: f32 = 6.0; // stereo depth noise grows like distance^2: past this it's too rough to show

#[derive(Clone)]
pub struct Settings {
    pub enabled: bool,
    pub people: bool,
    pub pets: bool,            // cats and dogs
    pub ignore_hands: bool,    // ignore boxes around a controller: your hands (measure.rs)
    pub games_only: bool,      // pause while no game is running (SteamVR home, dashboard)
    pub play_area_only: bool,  // ignore detections outside the play area's boundary (measure.rs)
    pub max_dist: f32,         // ignore detections farther than this (m, horizontal)
    pub size: i32,             // YOLO per-view size, one of SIZES
    pub hz: f32,               // YOLO runs per second, one of RATES
    pub autostart: bool,       // start with SteamVR (the driver launches the app, autostart.rs)
    pub gameid: Option<u64>,   // not in the tab: Steam's id for this app (SteamGameId), for the driver to launch it
    pub warning_acked: bool,   // the tab's first-run warning was acknowledged (panel.rs)
    pub correction: [f32; 3],  // not in the tab: the rectification's correction, rad (selfcal.rs); saved in degrees
    pub path: Option<PathBuf>, // where save() writes; None: nowhere (tests)
}

impl Default for Settings {
    fn default() -> Settings {
        Settings {
            enabled: true,
            people: true,
            pets: true,
            ignore_hands: true,
            games_only: false,
            play_area_only: false,
            max_dist: 3.0,
            size: 480,
            hz: 4.0,
            autostart: true,
            gameid: None,
            warning_acked: false,
            correction: [0.0; 3],
            path: None,
        }
    }
}

impl Settings {
    /// What's saved in ~/.config/roommate-radar.conf.
    pub fn load_saved() -> Settings {
        Settings::load(format!("{}/{CONF_PATH}", std::env::var("HOME").unwrap_or_default()).into())
    }

    /// The defaults, overridden by what's saved at `path`.
    pub fn load(path: PathBuf) -> Settings {
        let text = std::fs::read_to_string(&path).unwrap_or_default();
        let mut s = Settings { path: Some(path), ..Settings::default() };
        for (k, v) in text.lines().filter_map(|l| l.split_once('=')) {
            let (k, v) = (k.trim(), v.trim());
            if k == "align" {
                let deg: Vec<f32> = v.split(',').filter_map(|x| x.trim().parse().ok()).collect();
                if let [x, y, z] = deg[..]
                    && [x, y, z].iter().all(|a| a.abs() < 5.0)
                {
                    s.correction = [x, y, z].map(f32::to_radians);
                }
                continue;
            }
            let Ok(num) = v.parse::<f32>() else {
                continue;
            };
            match k {
                "enabled" => s.enabled = num != 0.0,
                "people" => s.people = num != 0.0,
                "pets" => s.pets = num != 0.0,
                "hands" => s.ignore_hands = num != 0.0,
                "games_only" => s.games_only = num != 0.0,
                "play_area" => s.play_area_only = num != 0.0,
                "autostart" => s.autostart = num != 0.0,
                "warned" => s.warning_acked = num != 0.0,
                "gameid" => s.gameid = v.parse().ok(),
                // A range saved when the maximum was larger: the largest now
                "max_dist" if (MIN_DIST..=20.0).contains(&num) => s.max_dist = num.min(MAX_DIST),
                "size" if SIZES.contains(&(num as i32)) => s.size = num as i32,
                "hz" if RATES.contains(&num) => s.hz = num,
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
                b(self.ignore_hands),
                b(self.games_only),
                b(self.play_area_only),
                self.max_dist,
                self.size,
                self.hz,
                b(self.autostart),
                b(self.warning_acked),
                self.correction.map(|a| format!("{:.4}", a.to_degrees())).join(","),
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
            play_area_only: true,
            max_dist: 4.5,
            size: 640,
            hz: 16.0,
            autostart: false,
            warning_acked: true,
            gameid: Some(16375998297183092736),
            correction: [0.0115, -0.0193, 0.0087],
            path: Some(path.clone()),
            ..Settings::default()
        };
        s.save();
        let l = Settings::load(path.clone());
        assert!(!l.people && l.pets && l.play_area_only && l.max_dist == 4.5 && l.size == 640 && l.hz == 16.0);
        assert!(
            !l.autostart && l.warning_acked && l.gameid == Some(16375998297183092736),
            "a game id doesn't fit in an f32"
        );
        assert!(l.correction.iter().zip(s.correction).all(|(a, b)| (a - b).abs() < 1e-5), "{:?}", l.correction);
        std::fs::write(&path, "max_dist=8\nsize=100\n").unwrap();
        let l = Settings::load(path.clone());
        assert!(l.max_dist == MAX_DIST && l.size == 480, "a larger range: the maximum; a bad size: the default");
        std::fs::write(&path, "max_dist=30\n").unwrap();
        let l = Settings::load(path.clone());
        std::fs::remove_file(path).unwrap();
        assert!(l.max_dist == Settings::default().max_dist, "nonsense: the default");
    }
}
