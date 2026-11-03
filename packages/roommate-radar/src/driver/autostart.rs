//! Starts Roommate Radar when SteamVR starts, if its "Start with SteamVR" is on (`autostart` in
//! ~/.config/roommate-radar.conf, next to the `gameid` the app records there).
//!
//! Not SteamVR's own auto-launch: on the Frame SteamVR starts before Steam has logged in. Steam holds the launch SteamVR
//! asks for until it has, then runs it a few milliseconds before its UI can answer the launch's first step
//! (ShowInterstitials), and it hangs at "Launching…" for good. So this waits for Steam's login (config/loginusers.vdf
//! written since the running Steam, the pid in ~/.steam/steam.pid, started: at boot SteamVR can start before Steam has
//! rewritten that file), then LOGIN_DELAY, then asks Steam itself: steam://rungameid on ~/.steam/steam.pipe, as
//! `steam <url>` does.
use std::fs;
use std::io::Write;
use std::os::unix::fs::OpenOptionsExt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::settings::Settings;

static STOP: AtomicBool = AtomicBool::new(false);
const LOGIN_DELAY: Duration = Duration::from_secs(10); // after the login; Steam's UI is ready ~0.1 s after it

/// The app's launch URL, if it should start with SteamVR and has recorded its game id.
fn launch_url(s: &Settings) -> Option<String> {
    if !s.autostart {
        return None;
    }
    Some(format!("steam://rungameid/{}", s.gameid?))
}

/// When process `pid` started; None if there's no such process.
fn process_start_time(pid: u32) -> Option<SystemTime> {
    // starttime: field 22 of /proc/<pid>/stat, in clock ticks since boot (the fields after the parenthesized name are 3+)
    let stat = fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
    let start_ticks: u64 = stat.rsplit(')').next()?.split_whitespace().nth(19)?.parse().ok()?;
    let boot_time_s: u64 =
        fs::read_to_string("/proc/stat").ok()?.lines().find_map(|l| l.strip_prefix("btime "))?.parse().ok()?;
    let ticks_per_s = unsafe { libc::sysconf(libc::_SC_CLK_TCK) } as u64;
    Some(UNIX_EPOCH + Duration::from_secs(boot_time_s) + Duration::from_millis(start_ticks * 1000 / ticks_per_s))
}

fn app_running() -> bool {
    let Ok(procs) = fs::read_dir("/proc") else {
        return false;
    };
    procs.flatten().any(|p| fs::read_to_string(p.path().join("comm")).is_ok_and(|c| c.trim_end() == "roommate-radar"))
}

pub fn stop() {
    STOP.store(true, Ordering::Relaxed);
}

/// Waits for Steam (until stop()), then launches the app unless it's running already.
pub fn run() {
    let Some(url) = launch_url(&Settings::load_saved()) else {
        return;
    };
    let home = std::env::var("HOME").unwrap_or_default();
    let logged_in = || -> Option<bool> {
        let steam_start_time =
            process_start_time(fs::read_to_string(format!("{home}/.steam/steam.pid")).ok()?.trim().parse().ok()?)?;
        Some(
            fs::metadata(format!("{home}/.steam/root/config/loginusers.vdf")).ok()?.modified().ok()? > steam_start_time,
        )
    };
    let mut launch_at = None;
    while !STOP.load(Ordering::Relaxed) {
        if launch_at.is_none() && logged_in() == Some(true) {
            launch_at = Some(Instant::now() + LOGIN_DELAY);
        }
        if launch_at.is_some_and(|t| Instant::now() >= t) {
            break;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    if STOP.load(Ordering::Relaxed) || app_running() {
        return;
    }
    // Steam closing the pipe under the write would SIGPIPE vrserver as a whole: blocked on this thread, the write gets
    // EPIPE instead (and the pending signal goes with the thread).
    unsafe {
        let mut signals: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut signals);
        libc::sigaddset(&mut signals, libc::SIGPIPE);
        libc::pthread_sigmask(libc::SIG_BLOCK, &signals, std::ptr::null_mut());
    }
    // Non-blocking: with no Steam reading the pipe, the open fails instead of waiting for one.
    let pipe =
        fs::OpenOptions::new().write(true).custom_flags(libc::O_NONBLOCK).open(format!("{home}/.steam/steam.pipe"));
    match pipe.and_then(|mut p| p.write_all(format!("{url}\n").as_bytes())) {
        Ok(()) => log!("started with SteamVR ({url})\n"),
        Err(e) => log!("can't ask Steam to start it ({url}): {e}\n"),
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn process_start_time() {
        let t = super::process_start_time(std::process::id()).unwrap();
        let age = std::time::SystemTime::now().duration_since(t).unwrap();
        assert!(age.as_secs() < 60, "this test started {age:?} ago");
        assert!(super::process_start_time(u32::MAX).is_none());
    }

    #[test]
    fn launch_url_from_the_apps_settings() {
        use crate::settings::Settings;
        let url = |autostart, gameid| super::launch_url(&Settings { autostart, gameid, ..Settings::default() });
        let on = super::launch_url(&Settings { gameid: Some(16375998297183092736), ..Settings::default() });
        let off = url(false, Some(16375998297183092736));
        let never_started_by_steam = url(true, None);
        assert_eq!(on.as_deref(), Some("steam://rungameid/16375998297183092736"), "on by default");
        assert!(off.is_none() && never_started_by_steam.is_none());
    }
}
