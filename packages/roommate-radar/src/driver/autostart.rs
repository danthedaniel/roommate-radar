//! Starts Roommate Radar when SteamVR starts, if its "Start with SteamVR" is on (`autostart` in
//! ~/.config/roommate-radar.conf, next to the `gameid` the app records there).
//!
//! Not SteamVR's own auto-launch: on the Frame SteamVR starts before Steam has logged in. Steam holds the launch SteamVR
//! asks for until it has, then runs it a few milliseconds before its UI can answer the launch's first step
//! (ShowInterstitials), and it hangs at "Launching…" for good. So this waits for Steam's login (config/loginusers.vdf
//! written since the running Steam, the pid in ~/.steam/steam.pid, started: at boot SteamVR can start before Steam has
//! rewritten that file), then a margin, then asks Steam itself: steam://rungameid on ~/.steam/steam.pipe, as
//! `steam <url>` does.
use std::fs;
use std::io::Write;
use std::os::unix::fs::OpenOptionsExt;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::shm;

static STOP: AtomicBool = AtomicBool::new(false);
const MARGIN: Duration = Duration::from_secs(10); // after the login; Steam's UI is ready ~0.1 s after it

/// The app's launch URL, if it should start with SteamVR and has recorded its game id.
fn launch_url(home: &str) -> Option<String> {
    let conf = fs::read_to_string(format!("{home}/{}", shm::CONF)).ok()?;
    let get = |k: &str| conf.lines().find_map(|l| l.strip_prefix(k)?.strip_prefix('=')).map(str::trim);
    if get("autostart") == Some("0") {
        return None;
    }
    Some(format!("steam://rungameid/{}", get("gameid")?))
}

/// When process `pid` started; None if there's no such process.
fn started(pid: u32) -> Option<SystemTime> {
    // starttime: field 22 of /proc/<pid>/stat, in clock ticks since boot (the fields after the parenthesized name are 3+)
    let stat = fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
    let ticks: u64 = stat.rsplit(')').next()?.split_whitespace().nth(19)?.parse().ok()?;
    let boot: u64 =
        fs::read_to_string("/proc/stat").ok()?.lines().find_map(|l| l.strip_prefix("btime "))?.parse().ok()?;
    let hz = unsafe { libc::sysconf(libc::_SC_CLK_TCK) } as u64;
    Some(UNIX_EPOCH + Duration::from_secs(boot) + Duration::from_millis(ticks * 1000 / hz))
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
    let home = std::env::var("HOME").unwrap_or_default();
    let Some(url) = launch_url(&home) else {
        return;
    };
    let logged_in = || -> Option<bool> {
        let steam = started(fs::read_to_string(format!("{home}/.steam/steam.pid")).ok()?.trim().parse().ok()?)?;
        Some(fs::metadata(format!("{home}/.steam/root/config/loginusers.vdf")).ok()?.modified().ok()? > steam)
    };
    let mut due = None;
    while !STOP.load(Ordering::Relaxed) {
        if due.is_none() && logged_in() == Some(true) {
            due = Some(Instant::now() + MARGIN);
        }
        if due.is_some_and(|t| Instant::now() >= t) {
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
        let mut set: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut set);
        libc::sigaddset(&mut set, libc::SIGPIPE);
        libc::pthread_sigmask(libc::SIG_BLOCK, &set, std::ptr::null_mut());
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
        let t = super::started(std::process::id()).unwrap();
        let age = std::time::SystemTime::now().duration_since(t).unwrap();
        assert!(age.as_secs() < 60, "this test started {age:?} ago");
        assert!(super::started(u32::MAX).is_none());
    }

    #[test]
    fn launch_url_from_the_apps_settings() {
        let home = std::env::temp_dir().join(format!("roommate-radar-autostart-{}", std::process::id()));
        std::fs::create_dir_all(home.join(".config")).unwrap();
        let url = |conf: &str| {
            std::fs::write(home.join(crate::shm::CONF), conf).unwrap();
            super::launch_url(home.to_str().unwrap())
        };
        let on = url("enabled=1\ngameid=16375998297183092736\n");
        let off = url("autostart=0\ngameid=16375998297183092736\n");
        let never_started_by_steam = url("autostart=1\n");
        std::fs::remove_dir_all(&home).unwrap();
        assert_eq!(on.as_deref(), Some("steam://rungameid/16375998297183092736"), "on by default");
        assert!(off.is_none() && never_started_by_steam.is_none());
    }
}
