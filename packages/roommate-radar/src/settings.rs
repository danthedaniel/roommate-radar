//! roommate-radar's settings: set from the dashboard tab (panel.rs), saved to ~/.config/roommate-radar.conf.
use std::path::PathBuf;

pub const SIZES: [i32; 3] = [320, 480, 640];
pub const RATES: [f32; 4] = [2.0, 4.0, 8.0, 16.0];

#[derive(Clone, Debug)]
pub struct Settings {
    pub enabled: bool,
    pub people: bool,
    pub pets: bool,            // cats and dogs
    pub hands: bool,           // ignore boxes around a controller: your hands (measure.rs)
    pub games_only: bool,      // pause while no game is running (SteamVR home, dashboard)
    pub max_dist: f32,         // ignore detections farther than this (m, horizontal)
    pub size: i32,             // YOLO per-view size, one of SIZES
    pub hz: f32,               // YOLO runs per second, one of RATES
    pub autolaunch_set: bool,  // not in the tab: auto-launch was switched on once (--register)
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
            max_dist: 3.0,
            size: 320,
            hz: 4.0,
            autolaunch_set: false,
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
            let Ok(v) = v.trim().parse::<f32>() else {
                continue;
            };
            match k.trim() {
                "enabled" => s.enabled = v != 0.0,
                "people" => s.people = v != 0.0,
                "pets" => s.pets = v != 0.0,
                "hands" => s.hands = v != 0.0,
                "games_only" => s.games_only = v != 0.0,
                "autolaunch_set" => s.autolaunch_set = v != 0.0,
                "max_dist" => s.max_dist = v,
                "size" if SIZES.contains(&(v as i32)) => s.size = v as i32,
                "hz" if RATES.contains(&v) => s.hz = v,
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
                "enabled={}\npeople={}\npets={}\nhands={}\ngames_only={}\nmax_dist={:.1}\nsize={}\nhz={}\nautolaunch_set={}\n",
                b(self.enabled),
                b(self.people),
                b(self.pets),
                b(self.hands),
                b(self.games_only),
                self.max_dist,
                self.size,
                self.hz,
                b(self.autolaunch_set)
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
            max_dist: 4.5,
            size: 640,
            hz: 16.0,
            path: Some(path.clone()),
            ..Settings::default()
        };
        s.save();
        let l = Settings::load(path.clone());
        std::fs::remove_file(path).unwrap();
        assert!(!l.people && l.pets && l.max_dist == 4.5 && l.size == 640 && l.hz == 16.0);
    }
}
