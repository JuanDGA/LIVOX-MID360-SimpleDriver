// Copyright 2026 Juan David Guevara Arévalo
// 
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
// 
//        http://www.apache.org/licenses/LICENSE-2.0
// 
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

//! 3D point-cloud viewer for the Livox MID360.
//!
//! Build/run with the `viewer` feature. There are two modes:
//!   cargo run --features viewer --bin lidar_viewer -- live <host_ip> <lidar_ip>
//!   cargo run --features viewer --bin lidar_viewer -- replay <points.csv> [imu.csv]
//!
//! The legacy form `lidar_viewer <host_ip> <lidar_ip>` is still accepted and
//! is equivalent to passing `live` first.
//!
//! Controls:
//!   drag (left)   - orbit
//!   drag (right)  - pan
//!   scroll        - zoom
//!   Up / Down     - increase / decrease point retention window
//!   C             - clear the point buffer
//!   F             - toggle FOV clip (hide points outside the LiDAR's current view)
//!   Esc           - quit
//!
//! Replay-only controls:
//!   Space         - play / pause
//!   R             - restart from the beginning
//!   Z / X         - slow down / speed up playback
//!
//! Points are expired by LiDAR timestamp age, so moving objects stop leaving
//! trails once they leave a region. The retention window (default 500 ms) is
//! adjustable at runtime; larger windows build a denser map of static scenes.
//!
//! FOV clip (off by default): the MID360 scans 360 deg about Z and covers
//! elevation -7 deg to +59 deg from the horizontal plane. When enabled, the
//! view splits into two layers: live points (captured within the last
//! ~150 ms) are always drawn as the current sensor reading, while older
//! points are drawn only where the LiDAR is no longer pointing (outside its
//! current FOV) to keep a persistent background map. Old points inside the
//! current FOV are hidden so the live data replaces them. Flipping the
//! sensor therefore keeps the previous view (e.g. a ceiling) as a
//! persistent layer while the new view (e.g. a floor) is drawn live.
//! While the clip is on, age-based expiry is disabled so out-of-FOV points
//! survive reorientation; the `MAX_POINTS` cap still bounds memory.
//!
//! Coordinate convention: the LiDAR frame is (x, y, z) with Z as height. The
//! viewer remaps each point to world (x, z, -y) so that Z is drawn upward and
//! positive LiDAR Y points the correct way (the sensor frame is left-handed
//! relative to the viewer's right-handed one).
//!
//! IMU stabilization: each point is rotated by the LiDAR's estimated attitude
//! (body -> gravity-aligned world frame) before display, so rotating the
//! sensor does not rotate the view. Only orientation is corrected; walking
//! the LiDAR sideways will still translate the cloud (an IMU cannot recover
//! position).

use std::net::{Ipv4Addr, SocketAddr};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use glam::{Mat4, Quat, Vec3, Vec4};
use minifb::{Key, MouseButton, MouseMode, Window, WindowOptions};

use lidar_reader::client::{DataStream, LivoxClient};
use lidar_reader::imu::{AttitudeEstimator, OrientationHistory};
use lidar_reader::packet::{DataPacket, DataPayload};
use lidar_reader::points::{Cartesian32Point, Point, Tag};
use lidar_reader::protocol::DataType;
use lidar_reader::{load_imu_csv, load_points_csv, LoadedImu, LoadedPoint};

const WIDTH: usize = 1024;
const HEIGHT: usize = 768;
const MAX_POINTS: usize = 600_000;
const DEFAULT_MAX_AGE_MS: f32 = 500.0;
const AGE_STEP_MS: f32 = 100.0;
const AGE_MIN_MS: f32 = 50.0;
const AGE_MAX_MS: f32 = 5000.0;
/// IMU orientation history length (~4 s at 200 Hz) covers point-packet latency.
const ORIENTATION_HISTORY_LEN: usize = 800;

/// LiDAR field of view, measured in the body frame. The MID360 scans a full
/// 360 deg about Z (azimuth unconstrained) and elevations from -7 deg to
/// +59 deg measured from the horizontal (XY) plane toward +/- Z. Used by the
/// optional FOV clip; off by default so points the LiDAR can no longer see
/// remain on screen as a persistent map.
const FOV_ELEV_MIN_RAD: f32 = -7.0 * std::f32::consts::PI / 180.0;
const FOV_ELEV_MAX_RAD: f32 = 59.0 * std::f32::consts::PI / 180.0;
/// A point is treated as "live" (current sensor data) if it was captured
/// within this age of the newest point in the buffer. When the FOV clip is
/// on, live points are always drawn; older points are drawn only where the
/// LiDAR is no longer pointing (out of FOV), so the live view and a
/// persistent background map coexist. ~150 ms covers a full MID360 sweep.
const LIVE_AGE_NS: u64 = 150_000_000;

/// Accumulated point cloud with the latest LiDAR timestamp used for expiry.
struct Cloud {
    /// (world metres, packet timestamp ns). World = (x, z, -y) after attitude
    /// rotation, so the LiDAR Z (height) becomes the vertical viewer axis.
    points: Vec<([f32; 3], u64)>,
    latest_ts: u64,
}

impl Cloud {
    fn new() -> Self {
        Self {
            points: Vec::with_capacity(MAX_POINTS),
            latest_ts: 0,
        }
    }

    /// Add a batch of points from one packet, rotated by the attitude `q`
    /// (body -> world) estimated at the packet timestamp. When `expire` is
    /// true, points older than `max_age_ns` are dropped (the rolling-view
    /// behaviour with the clip off). When `expire` is false (FOV clip on),
    /// age-based expiry is skipped so out-of-FOV points persist across
    /// sensor reorientation and the render-time clip decides what to show;
    /// the `MAX_POINTS` safety cap still bounds memory.
    fn add(
        &mut self,
        pts: &[lidar_reader::points::Point],
        ts: u64,
        max_age_ns: u64,
        q: Quat,
        expire: bool,
    ) {
        if ts > self.latest_ts {
            self.latest_ts = ts;
        }
        for p in pts {
            let (x, y, z) = p.coords_m();
            // Stabilize: body -> gravity-aligned world frame.
            let pw = q * Vec3::new(x, y, z);
            // Remap to viewer world (x, z, -y): Z up, +LiDAR-Y corrected.
            self.points.push(([pw.x, pw.z, -pw.y], ts));
        }
        if expire {
            self.expire(max_age_ns);
        }
        // Safety cap so a runaway stream never blows up memory.
        if self.points.len() > MAX_POINTS {
            let drop = self.points.len() - MAX_POINTS;
            self.points.drain(..drop);
        }
    }

    fn expire(&mut self, max_age_ns: u64) {
        let cutoff = self.latest_ts.saturating_sub(max_age_ns);
        // Points are appended in timestamp order, so the old ones are at the
        // front; drop them in bulk.
        let mut keep = 0;
        while keep < self.points.len() && self.points[keep].1 < cutoff {
            keep += 1;
        }
        if keep > 0 {
            self.points.drain(..keep);
        }
    }

    fn clear(&mut self) {
        self.points.clear();
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let mode = match args.get(1).map(String::as_str) {
        Some("live") => Mode::Live,
        Some("replay") => Mode::Replay,
        _ => {
            // Legacy form: `lidar_viewer <host_ip> <lidar_ip>` is live mode.
            // Detect by trying to parse the first arg as an IPv4 address.
            if args
                .get(1)
                .is_some_and(|s| s.parse::<Ipv4Addr>().is_ok())
            {
                Mode::Live
            } else {
                print_usage();
                return;
            }
        }
    };

    let cloud: Arc<Mutex<Cloud>> = Arc::new(Mutex::new(Cloud::new()));
    let max_age_ms = Arc::new(AtomicU32::new(DEFAULT_MAX_AGE_MS.to_bits()));
    let latest_q: Arc<Mutex<Quat>> = Arc::new(Mutex::new(Quat::IDENTITY));
    let fov_clip = Arc::new(AtomicBool::new(false));

    match mode {
        Mode::Live => {
            // For live mode the IP args sit at positions 2/3 (`live <host> <lidar>`)
            // or 1/2 in the legacy form (no `live` keyword).
            let offset = if args.get(1).map(String::as_str) == Some("live") {
                2
            } else {
                1
            };
            let host_ip = match args.get(offset).and_then(|s| s.parse::<Ipv4Addr>().ok()) {
                Some(ip) => ip,
                None => {
                    eprintln!("live mode requires <host_ip> <lidar_ip>");
                    print_usage();
                    return;
                }
            };
            let lidar_ip = match args.get(offset + 1).and_then(|s| s.parse::<Ipv4Addr>().ok()) {
                Some(ip) => ip,
                None => {
                    eprintln!("live mode requires <host_ip> <lidar_ip>");
                    print_usage();
                    return;
                }
            };

            spawn_data_thread(
                host_ip,
                lidar_ip,
                cloud.clone(),
                max_age_ms.clone(),
                latest_q.clone(),
                fov_clip.clone(),
            );
            run_window(cloud, max_age_ms, latest_q, fov_clip, None);
        }
        Mode::Replay => {
            let points_path = match args.get(2).map(PathBuf::from) {
                Some(p) => p,
                None => {
                    eprintln!("replay mode requires <points.csv> [imu.csv]");
                    print_usage();
                    return;
                }
            };
            let imu_path = args.get(3).map(PathBuf::from);

            let (points, imu) = match load_replay(&points_path, imu_path.as_ref()) {
                Ok(v) => v,
                Err(e) => {
                    eprintln!("failed to load CSVs: {e}");
                    return;
                }
            };
            if points.is_empty() {
                eprintln!("no points found in {}", points_path.display());
                return;
            }
            let playback = Some(Playback::new(points, imu));
            run_window(cloud, max_age_ms, latest_q, fov_clip, playback);
        }
    }
}

#[derive(Clone, Copy)]
enum Mode {
    Live,
    Replay,
}

fn print_usage() {
    eprintln!("Usage:");
    eprintln!("  lidar_viewer live <host_ip> <lidar_ip>");
    eprintln!("  lidar_viewer replay <points.csv> [imu.csv]");
    eprintln!();
    eprintln!("The legacy form `lidar_viewer <host_ip> <lidar_ip>` is still accepted.");
}

/// Load points and (optional) IMU CSVs for replay.
fn load_replay(
    points_path: &PathBuf,
    imu_path: Option<&PathBuf>,
) -> std::result::Result<(Vec<LoadedPoint>, Vec<LoadedImu>), lidar_reader::LidarError> {
    let points = load_points_csv(points_path)?;
    let imu = match imu_path {
        Some(p) => load_imu_csv(p)?,
        None => Vec::new(),
    };
    Ok((points, imu))
}

fn spawn_data_thread(
    host_ip: Ipv4Addr,
    lidar_ip: Ipv4Addr,
    cloud: Arc<Mutex<Cloud>>,
    max_age_ms: Arc<AtomicU32>,
    latest_q: Arc<Mutex<Quat>>,
    fov_clip: Arc<AtomicBool>,
) {
    std::thread::spawn(move || {
        let rt = match tokio::runtime::Runtime::new() {
            Ok(rt) => rt,
            Err(e) => {
                eprintln!("failed to start runtime: {e}");
                return;
            }
        };
        rt.block_on(async move {
            let client = match LivoxClient::with_default_cmd_port(host_ip).await {
                Ok(c) => c,
                Err(e) => {
                    eprintln!("failed to bind command socket: {e}");
                    return;
                }
            };
            let stream = match DataStream::with_default_ports(host_ip).await {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("failed to bind data sockets: {e}");
                    return;
                }
            };

            let lidar_cmd_addr = SocketAddr::from((lidar_ip, lidar_reader::protocol::CMD_PORT));
            let data_dst = SocketAddr::from((host_ip, lidar_reader::protocol::HOST_DATA_PORT));
            let imu_dst = SocketAddr::from((host_ip, lidar_reader::protocol::HOST_IMU_PORT));

            if let Err(e) = client
                .start_streaming(
                    lidar_cmd_addr,
                    data_dst,
                    imu_dst,
                    DataType::PointCloudCartesian32,
                    Duration::from_secs(2),
                )
                .await
            {
                eprintln!("failed to start streaming: {e}");
                return;
            }
            println!("LiDAR streaming; close the window or press Esc to quit.");

            let mut estimator = AttitudeEstimator::new();
            let mut history = OrientationHistory::new(ORIENTATION_HISTORY_LEN);

            loop {
                tokio::select! {
                    pkt = stream.next_imu(Duration::from_millis(200)) => {
                        if let Ok(DataPacket {
                            header,
                            payload: DataPayload::Imu(imu),
                        }) = pkt
                        {
                            let gyro = Vec3::new(imu.gyro_x, imu.gyro_y, imu.gyro_z);
                            let acc = Vec3::new(imu.acc_x, imu.acc_y, imu.acc_z);
                            estimator.update(gyro, acc, header.timestamp);
                            history.push(header.timestamp, estimator.q);
                            if let Ok(mut q) = latest_q.lock() {
                                *q = estimator.q;
                            }
                        }
                    }
                    pkt = stream.next_point_cloud(Duration::from_millis(200)) => {
                        if let Ok(DataPacket {
                            header,
                            payload: DataPayload::Points(pts),
                        }) = pkt
                        {
                            let q = history.at(header.timestamp);
                            let max_age_ns = (f32::from_bits(max_age_ms.load(Ordering::Relaxed))
                                * 1_000_000.0) as u64;
                            // With the FOV clip on we want a persistent map,
                            // so skip age-based expiry (out-of-FOV points must
                            // survive sensor reorientation). The render-time
                            // clip hides old in-FOV points behind live data.
                            let expire = !fov_clip.load(Ordering::Relaxed);
                            if let Ok(mut c) = cloud.lock() {
                                c.add(&pts, header.timestamp, max_age_ns, q, expire);
                            }
                        }
                    }
                }
            }
        });
    });
}

/// Replays recorded CSVs through the same stabilization + FOV-clip pipeline
/// as the live stream. Each frame [`tick`] advances the playhead by wall-clock
/// time scaled by `speed`, feeds IMU samples up to the playhead into the
/// attitude estimator, and emits point groups up to the playhead into the
/// cloud with the attitude looked up at each group's timestamp -- exactly
/// mirroring how the live thread ingests packets.
struct Playback {
    points: Vec<LoadedPoint>,
    imu: Vec<LoadedImu>,
    next_point: usize,
    next_imu: usize,
    estimator: AttitudeEstimator,
    history: OrientationHistory,
    playhead_ns: u64,
    first_ts: u64,
    last_ts: u64,
    playing: bool,
    speed: f32,
}

impl Playback {
    fn new(points: Vec<LoadedPoint>, imu: Vec<LoadedImu>) -> Self {
        let first_ts = points.first().map(|p| p.ts_ns).unwrap_or(0);
        let last_ts = points
            .last()
            .map(|p| p.ts_ns)
            .unwrap_or(first_ts)
            .max(imu.last().map(|i| i.ts_ns).unwrap_or(first_ts));
        Self {
            points,
            imu,
            next_point: 0,
            next_imu: 0,
            estimator: AttitudeEstimator::new(),
            history: OrientationHistory::new(ORIENTATION_HISTORY_LEN),
            playhead_ns: first_ts,
            first_ts,
            last_ts,
            playing: true,
            speed: 1.0,
        }
    }

    /// Advance the playhead and push any newly-due IMU samples and point
    /// groups into the shared cloud / attitude state.
    fn tick(
        &mut self,
        dt_secs: f32,
        cloud: &Arc<Mutex<Cloud>>,
        max_age_ms: &AtomicU32,
        latest_q: &Arc<Mutex<Quat>>,
        fov_clip: &AtomicBool,
    ) {
        if self.playing {
            let advance = (dt_secs * self.speed * 1e9) as u64;
            self.playhead_ns = self.playhead_ns.saturating_add(advance);
            if self.playhead_ns >= self.last_ts {
                self.playhead_ns = self.last_ts;
                self.playing = false;
            }
        }

        // Feed IMU samples up to the playhead (time-ordered) into the filter.
        while self.next_imu < self.imu.len() && self.imu[self.next_imu].ts_ns <= self.playhead_ns {
            let r = &self.imu[self.next_imu];
            self.estimator.update(
                Vec3::from(r.gyro),
                Vec3::from(r.acc),
                r.ts_ns,
            );
            self.history.push(r.ts_ns, self.estimator.q);
            self.next_imu += 1;
        }
        if let Ok(mut q) = latest_q.lock() {
            *q = self.estimator.q;
        }

        // Emit point groups up to the playhead. Points within one packet share
        // a timestamp; group consecutive equal-timestamp rows into one batch.
        let max_age_ns = (f32::from_bits(max_age_ms.load(Ordering::Relaxed)) * 1_000_000.0) as u64;
        let expire = !fov_clip.load(Ordering::Relaxed);
        while self.next_point < self.points.len()
            && self.points[self.next_point].ts_ns <= self.playhead_ns
        {
            let ts = self.points[self.next_point].ts_ns;
            let mut group: Vec<Point> = Vec::new();
            while self.next_point < self.points.len() && self.points[self.next_point].ts_ns == ts {
                let lp = self.points[self.next_point];
                // Body-frame metres -> mm i32 for the viewer's point path.
                // Sub-millimetre quantization is invisible at display scale.
                group.push(Point::Cartesian32(Cartesian32Point {
                    x_mm: (lp.x_m * 1000.0) as i32,
                    y_mm: (lp.y_m * 1000.0) as i32,
                    z_mm: (lp.z_m * 1000.0) as i32,
                    reflectivity: 0,
                    tag: Tag(0),
                }));
                self.next_point += 1;
            }
            let q = self.history.at(ts);
            if let Ok(mut c) = cloud.lock() {
                c.add(&group, ts, max_age_ns, q, expire);
            }
        }
    }

    /// Rewind to the start and clear the cloud so the replay begins fresh.
    fn restart(&mut self, cloud: &Arc<Mutex<Cloud>>) {
        self.next_point = 0;
        self.next_imu = 0;
        self.estimator = AttitudeEstimator::new();
        self.history = OrientationHistory::new(ORIENTATION_HISTORY_LEN);
        self.playhead_ns = self.first_ts;
        self.playing = true;
        if let Ok(mut c) = cloud.lock() {
            c.clear();
        }
        eprintln!("replay restarted");
    }

    fn slower(&mut self) {
        self.speed = (self.speed / 1.5).max(0.1);
        eprintln!("replay speed {:.2}x", self.speed);
    }

    fn faster(&mut self) {
        self.speed = (self.speed * 1.5).min(20.0);
        eprintln!("replay speed {:.2}x", self.speed);
    }
}

impl std::fmt::Display for Playback {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let elapsed = self.playhead_ns.saturating_sub(self.first_ts);
        let total = self.last_ts.saturating_sub(self.first_ts);
        let state = if self.playing { "play" } else { "pause" };
        write!(
            f,
            "{:.1}s/{:.1}s | {state} | {:.2}x",
            elapsed as f32 * 1e-9,
            total as f32 * 1e-9,
            self.speed,
        )
    }
}

#[derive(Clone, Copy)]
struct Camera {
    yaw: f32,
    pitch: f32,
    distance: f32,
    target: [f32; 3],
}

impl Camera {
    fn eye(self) -> Vec3 {
        let cp = self.pitch.cos();
        Vec3::new(
            self.target[0] + self.distance * self.yaw.cos() * cp,
            self.target[1] + self.distance * self.pitch.sin(),
            self.target[2] + self.distance * self.yaw.sin() * cp,
        )
    }
}

fn run_window(
    cloud: Arc<Mutex<Cloud>>,
    max_age_ms: Arc<AtomicU32>,
    latest_q: Arc<Mutex<Quat>>,
    fov_clip: Arc<AtomicBool>,
    mut playback: Option<Playback>,
) {
    let mut buffer = vec![0u32; WIDTH * HEIGHT];
    let mut zbuffer = vec![f32::NEG_INFINITY; WIDTH * HEIGHT];

    let mut window = match Window::new(
        "Livox MID360 viewer",
        WIDTH,
        HEIGHT,
        WindowOptions::default(),
    ) {
        Ok(w) => w,
        Err(e) => {
            eprintln!("failed to open window: {e}");
            return;
        }
    };

    let mut cam = Camera {
        yaw: 0.0,
        pitch: 0.5,
        distance: 8.0,
        target: [0.0, 0.0, 0.0],
    };
    let mut prev_mouse: Option<(f32, f32)> = None;
    let mut frame_count = 0u64;
    let mut fps_time = Instant::now();
    let mut fps = 0.0f32;
    let mut last_frame = Instant::now();

    while window.is_open() && !window.is_key_down(Key::Escape) {
        let dt = last_frame.elapsed().as_secs_f32();
        last_frame = Instant::now();

        handle_input(&window, &mut cam, &mut prev_mouse, &max_age_ms, &cloud, &fov_clip);

        // Advance replay, if any, by the wall-clock delta since last frame.
        if let Some(pb) = playback.as_mut() {
            // Replay-only keys (distinct from the always-on camera/age keys).
            for key in window.get_keys_pressed(minifb::KeyRepeat::No) {
                match key {
                    Key::Space => {
                        pb.playing = !pb.playing;
                        eprintln!("replay {}", if pb.playing { "play" } else { "pause" });
                    }
                    Key::R => pb.restart(&cloud),
                    Key::Z => pb.slower(),
                    Key::X => pb.faster(),
                    _ => {}
                }
            }
            pb.tick(dt, &cloud, &max_age_ms, &latest_q, &fov_clip);
        }

        let view = Mat4::look_at_rh(cam.eye(), Vec3::from(cam.target), Vec3::Y);
        let proj = Mat4::perspective_rh(1.0, WIDTH as f32 / HEIGHT as f32, 0.05, 1000.0);

        buffer.fill(0xFF0A0A0F); // dark background
        zbuffer.fill(f32::NEG_INFINITY);

        draw_axes(&mut buffer, &mut zbuffer, view, proj);

        let q_current = latest_q.lock().map(|q| *q).unwrap_or(Quat::IDENTITY);
        let clip = fov_clip.load(Ordering::Relaxed);
        let count = render_points(&cloud, &mut buffer, &mut zbuffer, view, proj, q_current, clip);

        window
            .update_with_buffer(&buffer, WIDTH, HEIGHT)
            .expect("update failed");

        frame_count += 1;
        let elapsed = fps_time.elapsed().as_secs_f32();
        if elapsed >= 0.5 {
            fps = frame_count as f32 / elapsed;
            frame_count = 0;
            fps_time = Instant::now();
        }
        let age = f32::from_bits(max_age_ms.load(Ordering::Relaxed));
        let fov = if clip { "on" } else { "off" };
        let title = match &playback {
            Some(pb) => format!(
                "Livox MID360 | replay {pb} | points: {count} | age: {age:.0} ms | fov: {fov} | fps: {fps:.0} | dist: {:.1} m",
                cam.distance
            ),
            None => format!(
                "Livox MID360 | points: {count} | age: {age:.0} ms | fov: {fov} | fps: {fps:.0} | dist: {:.1} m",
                cam.distance
            ),
        };
        window.set_title(&title);
    }
}

/// Render the current point cloud and return how many points were drawn.
///
/// When `fov_clip` is set, the view splits into a live layer and a
/// persisted layer (see [`should_draw_when_clipping`]): live points are
/// always drawn, and older points are drawn only where the LiDAR is not
/// currently pointing (outside its FOV, using the current attitude).
fn render_points(
    cloud: &Mutex<Cloud>,
    buffer: &mut [u32],
    zbuffer: &mut [f32],
    view: Mat4,
    proj: Mat4,
    q_current: Quat,
    fov_clip: bool,
) -> usize {
    let (snapshot, latest_ts): (Vec<([f32; 3], u64)>, u64) = {
        let c = match cloud.lock() {
            Ok(c) => c,
            Err(_) => return 0,
        };
        (c.points.iter().copied().collect(), c.latest_ts)
    };
    let inv_q = q_current.conjugate();
    let mut drawn = 0;
    for (p, ts) in &snapshot {
        if fov_clip {
            let age = latest_ts.saturating_sub(*ts);
            if !should_draw_when_clipping(age, in_fov(*p, inv_q)) {
                continue;
            }
        }
        if let Some((sx, sy, vz)) = project(*p, view, proj) {
            put_point(buffer, zbuffer, sx, sy, vz, height_color(p[1]));
            drawn += 1;
        }
    }
    drawn
}

/// Whether a buffered point should be drawn while the FOV clip is on.
///
/// Live points (age <= [`LIVE_AGE_NS`]) are always drawn: they are the
/// freshest sensor data. Older points are drawn only when they lie OUTSIDE
/// the current FOV, so regions the LiDAR is no longer pointing at stay
/// filled (persistent map) while regions it is pointing at show only live
/// data instead of stacking stale points on top of the new reading.
fn should_draw_when_clipping(age_ns: u64, in_fov: bool) -> bool {
    let is_live = age_ns <= LIVE_AGE_NS;
    is_live || !in_fov
}

/// Whether a stored viewer-world point lies inside the LiDAR's current FOV.
///
/// Stored points are in viewer world `(x, z, -y)` (see `Cloud::add`); we first
/// undo that remap to recover the gravity-aligned world vector, then rotate
/// world -> body with `inv_q` (the inverse of the current body -> world
/// attitude). The FOV cone is defined in the body frame: 360 deg about Z, so
/// azimuth is unconstrained, and elevation must lie within
/// `[FOV_ELEV_MIN_RAD, FOV_ELEV_MAX_RAD]` measured from the horizontal plane.
/// Used by [`should_draw_when_clipping`] to decide which old points become
/// persisted background (out of FOV) versus hidden behind live data (in FOV).
fn in_fov(stored: [f32; 3], inv_q: Quat) -> bool {
    // Undo viewer remap (x, z, -y) -> world (x, y, z).
    let pw = Vec3::new(stored[0], -stored[2], stored[1]);
    let body = inv_q * pw;
    let horiz = (body.x * body.x + body.y * body.y).sqrt();
    let elev = body.z.atan2(horiz);
    elev >= FOV_ELEV_MIN_RAD && elev <= FOV_ELEV_MAX_RAD
}

fn handle_input(
    window: &Window,
    cam: &mut Camera,
    prev_mouse: &mut Option<(f32, f32)>,
    max_age_ms: &AtomicU32,
    cloud: &Mutex<Cloud>,
    fov_clip: &AtomicBool,
) {
    let mouse = window.get_mouse_pos(MouseMode::Pass);
    let (dx, dy) = match (mouse, *prev_mouse) {
        (Some((x, y)), Some((px, py))) => (x - px, y - py),
        _ => (0.0, 0.0),
    };
    *prev_mouse = mouse;

    if window.get_mouse_down(MouseButton::Left) {
        cam.yaw -= dx * 0.01;
        cam.pitch += dy * 0.01;
        cam.pitch = cam.pitch.clamp(-1.5, 1.5);
    }
    if window.get_mouse_down(MouseButton::Right) {
        let scale = cam.distance * 0.0015;
        let right = Vec3::new(cam.yaw.sin(), 0.0, -cam.yaw.cos());
        let up = Vec3::Y;
        cam.target[0] -= right.x * dx * scale + up.x * dy * scale;
        cam.target[1] -= right.y * dx * scale + up.y * dy * scale;
        cam.target[2] -= right.z * dx * scale + up.z * dy * scale;
    }

    if let Some((_, sy)) = window.get_scroll_wheel() {
        cam.distance *= 1.0 - sy * 0.1;
        cam.distance = cam.distance.clamp(0.2, 500.0);
    }

    // Adjust point retention window. Up = longer trails, Down = shorter.
    if window.is_key_pressed(Key::Up, minifb::KeyRepeat::Yes) {
        bump_age(max_age_ms, AGE_STEP_MS);
    }
    if window.is_key_pressed(Key::Down, minifb::KeyRepeat::Yes) {
        bump_age(max_age_ms, -AGE_STEP_MS);
    }

    // Discrete key actions: use get_keys_pressed (queued events) instead of
    // is_key_pressed(No), which can miss presses that fall between frames.
    for key in window.get_keys_pressed(minifb::KeyRepeat::No) {
        match key {
            Key::C => {
                if let Ok(mut c) = cloud.lock() {
                    c.clear();
                    eprintln!("cleared point buffer");
                }
            }
            Key::F => {
                let next = !fov_clip.load(Ordering::Relaxed);
                fov_clip.store(next, Ordering::Relaxed);
                eprintln!(
                    "FOV clip {} -- {}",
                    if next { "ON" } else { "OFF" },
                    if next {
                        "live points always shown; older points shown only where the LiDAR \
                         is not currently pointing (persistent map), hidden where it is so \
                         live data replaces them. Age-based expiry is disabled, so points \
                         persist across reorientation until the LiDAR re-observes them or \
                         you press C"
                    } else {
                        "showing all accumulated points; age-based expiry resumed"
                    }
                );
            }
            _ => {}
        }
    }
}

fn bump_age(max_age_ms: &AtomicU32, delta: f32) {
    let current = f32::from_bits(max_age_ms.load(Ordering::Relaxed));
    let next = (current + delta).clamp(AGE_MIN_MS, AGE_MAX_MS);
    max_age_ms.store(next.to_bits(), Ordering::Relaxed);
}

fn project(p: [f32; 3], view: Mat4, proj: Mat4) -> Option<(i32, i32, f32)> {
    let view_space = view * Vec4::new(p[0], p[1], p[2], 1.0);
    if view_space.z >= -0.05 {
        return None; // behind or too close to camera
    }
    let clip = proj * view_space;
    if clip.w <= 0.0 {
        return None;
    }
    let ndc = Vec3::new(clip.x, clip.y, clip.z) / clip.w;
    let sx = (ndc.x * 0.5 + 0.5) * WIDTH as f32;
    let sy = (1.0 - (ndc.y * 0.5 + 0.5)) * HEIGHT as f32;
    Some((sx as i32, sy as i32, view_space.z))
}

fn put_point(buffer: &mut [u32], zbuffer: &mut [f32], sx: i32, sy: i32, vz: f32, color: u32) {
    // Draw a small cross so individual points are visible.
    for &(ox, oy) in &[(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)] {
        let x = sx + ox;
        let y = sy + oy;
        if x < 0 || x >= WIDTH as i32 || y < 0 || y >= HEIGHT as i32 {
            continue;
        }
        let idx = y as usize * WIDTH + x as usize;
        if vz > zbuffer[idx] {
            zbuffer[idx] = vz;
            buffer[idx] = color;
        }
    }
}

fn draw_line(
    buffer: &mut [u32],
    zbuffer: &mut [f32],
    a: [f32; 3],
    b: [f32; 3],
    view: Mat4,
    proj: Mat4,
    color: u32,
) {
    let Some((x0, y0, z0)) = project(a, view, proj) else { return };
    let Some((x1, y1, z1)) = project(b, view, proj) else { return };
    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    let steps = dx.max(dy).max(1);
    for i in 0..=steps {
        let t = i as f32 / steps as f32;
        let x = x0 as f32 + (x1 - x0) as f32 * t;
        let y = y0 as f32 + (y1 - y0) as f32 * t;
        let z = z0 + (z1 - z0) * t;
        put_point(buffer, zbuffer, x as i32, y as i32, z, color);
    }
}

fn draw_axes(buffer: &mut [u32], zbuffer: &mut [f32], view: Mat4, proj: Mat4) {
    let o = [0.0, 0.0, 0.0];
    let len = 1.0;
    draw_line(buffer, zbuffer, o, [len, 0.0, 0.0], view, proj, 0xFFFF0000); // X red
    draw_line(buffer, zbuffer, o, [0.0, len, 0.0], view, proj, 0xFF00FF00); // Y green
    draw_line(buffer, zbuffer, o, [0.0, 0.0, len], view, proj, 0xFF0000FF); // Z blue
}

/// Map a world-space height (metres) to an RGB colour using a simple gradient.
fn height_color(z: f32) -> u32 {
    // Span of ~10 m centred near ground level; clamped.
    let t = ((z + 2.0) / 10.0).clamp(0.0, 1.0);
    let (r, g, b) = gradient(t);
    (255 << 24) | ((r as u32) << 16) | ((g as u32) << 8) | (b as u32)
}

fn gradient(t: f32) -> (u8, u8, u8) {
    // Blue -> cyan -> green -> yellow -> red.
    let v = t * 4.0;
    match v as i32 {
        0 => {
            let k = v;
            (0, (k * 255.0) as u8, 255)
        }
        1 => {
            let k = v - 1.0;
            (0, 255, (255.0 - k * 255.0) as u8)
        }
        2 => {
            let k = v - 2.0;
            ((k * 255.0) as u8, 255, 0)
        }
        _ => {
            let k = v - 3.0;
            (255, (255.0 - k * 255.0) as u8, 0)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use lidar_reader::points::{Cartesian32Point, Point, Tag};

    /// Convert a body-frame direction to the stored viewer-world form the
    /// renderer uses: world = q * body, then stored = (world.x, world.z, -world.y).
    fn stored_from_body(body: Vec3, q: Quat) -> [f32; 3] {
        let w = q * body;
        [w.x, w.z, -w.y]
    }

    #[test]
    fn in_fov_accepts_cone_with_identity_attitude() {
        let q = Quat::IDENTITY;
        // Horizontal point: elevation 0 deg -> inside [-7, 59].
        assert!(in_fov(stored_from_body(Vec3::new(5.0, 0.0, 0.0), q), q));
        // +45 deg elevation -> inside.
        let z = 45.0_f32.to_radians().tan() * 5.0;
        assert!(in_fov(stored_from_body(Vec3::new(5.0, 0.0, z), q), q));
        // Any azimuth at elevation 0 is inside (360 deg about Z).
        assert!(in_fov(stored_from_body(Vec3::new(0.0, -3.0, 0.0), q), q));
        assert!(in_fov(stored_from_body(Vec3::new(-2.0, 4.0, 0.0), q), q));
    }

    #[test]
    fn in_fov_rejects_outside_elevation() {
        let q = Quat::IDENTITY;
        // +80 deg elevation: above the +59 deg ceiling -> outside.
        let z = 80.0_f32.to_radians().tan() * 5.0;
        assert!(!in_fov(stored_from_body(Vec3::new(5.0, 0.0, z), q), q));
        // -20 deg elevation: below the -7 deg floor -> outside.
        let z = -20.0_f32.to_radians().tan() * 5.0;
        assert!(!in_fov(stored_from_body(Vec3::new(5.0, 0.0, z), q), q));
    }

    #[test]
    fn in_fov_boundary_is_inclusive() {
        let q = Quat::IDENTITY;
        // Exactly +59 deg and -7 deg should be kept (inclusive bounds).
        let up_z = 59.0_f32.to_radians().tan() * 5.0;
        assert!(in_fov(stored_from_body(Vec3::new(5.0, 0.0, up_z), q), q));
        let down_z = -7.0_f32.to_radians().tan() * 5.0;
        assert!(in_fov(stored_from_body(Vec3::new(5.0, 0.0, down_z), q), q));
    }

    #[test]
    fn in_fov_uses_current_attitude() {
        // LiDAR now points such that world +X lies along its body +Z (straight
        // up): a world-horizontal point is seen at +90 deg elevation in the
        // body frame, above the +59 deg ceiling, so it is out of FOV.
        let q_now = Quat::from_rotation_y(90.0_f32.to_radians());
        let inv_q = q_now.conjugate().normalize();
        let stored = [5.0, 0.0, 0.0]; // world (5, 0, 0)
        assert!(!in_fov(stored, inv_q));
        // With no rotation the same point is in-FOV.
        assert!(in_fov(stored, Quat::IDENTITY));
    }

    #[test]
    fn clip_keeps_live_points_everywhere() {
        // Live (age <= threshold) is drawn regardless of whether it is in the
        // current FOV: it is the fresh sensor reading.
        assert!(should_draw_when_clipping(0, true));
        assert!(should_draw_when_clipping(0, false));
        assert!(should_draw_when_clipping(LIVE_AGE_NS, true)); // boundary inclusive
        assert!(should_draw_when_clipping(LIVE_AGE_NS, false));
    }

    #[test]
    fn clip_hides_old_in_fov_and_keeps_old_out_of_fov() {
        // Just past the live threshold -> "persisted".
        let old = LIVE_AGE_NS + 1;
        // Old + in FOV -> hidden: live data supersedes it where the LiDAR
        // is currently pointing.
        assert!(!should_draw_when_clipping(old, true));
        // Old + out of FOV -> kept as persistent background map.
        assert!(should_draw_when_clipping(old, false));
        // Much older, out of FOV -> still kept (persistent layer).
        assert!(should_draw_when_clipping(LIVE_AGE_NS * 10, false));
        // Much older, in FOV -> still hidden.
        assert!(!should_draw_when_clipping(LIVE_AGE_NS * 10, true));
    }

    #[test]
    fn clip_persists_ceiling_when_lidar_flips_down() {
        // Model the user's scenario at the decision level. A ceiling point
        // was captured earlier (old) and, after the LiDAR flips down, lies
        // out of its current FOV -> it must be drawn (persisted), not hidden.
        let ceiling_age = LIVE_AGE_NS * 5; // captured a while ago
        let ceiling_in_fov_after_flip = false;
        assert!(should_draw_when_clipping(ceiling_age, ceiling_in_fov_after_flip));
        // A freshly captured floor point (live) is in the new FOV -> drawn.
        assert!(should_draw_when_clipping(0, true));
        // An old floor point now in the FOV -> hidden behind the live data.
        assert!(!should_draw_when_clipping(ceiling_age, true));
    }

    #[test]
    fn add_with_expire_off_keeps_old_points() {
        // Reproduces the user's bug: with the FOV clip on, expiry must be
        // disabled so a point captured ~1 s ago survives a slow flip and can
        // be persisted by the render-time clip. With expiry on (clip off)
        // the same old point is dropped.
        let q = Quat::IDENTITY;
        let now = 1_000_000_000; // 1 s in ns
        let ceiling = Point::Cartesian32(Cartesian32Point {
            x_mm: 0,
            y_mm: 0,
            z_mm: 3000,
            reflectivity: 0,
            tag: Tag(0),
        });
        let floor = Point::Cartesian32(Cartesian32Point {
            x_mm: 0,
            y_mm: 0,
            z_mm: -3000,
            reflectivity: 0,
            tag: Tag(0),
        });

        // Both clouds start with the ceiling captured at t = 0.
        let mut clip_off = Cloud::new();
        clip_off.add(&[ceiling], 0, 0, q, false);
        let mut clip_on = Cloud::new();
        clip_on.add(&[ceiling], 0, 0, q, false);

        // A floor point arrives 1 s later; latest_ts jumps to `now`.
        // Clip OFF -> expire on, 500 ms window drops the 1 s-old ceiling.
        clip_off.add(&[floor], now, 500_000_000, q, true);
        assert_eq!(clip_off.points.len(), 1, "ceiling should be expired");
        // Clip ON -> expire off, ceiling survives so the clip can persist it.
        clip_on.add(&[floor], now, 500_000_000, q, false);
        assert_eq!(clip_on.points.len(), 2, "ceiling must survive when clip is on");
    }

    /// Build a `Playback` with a couple of point groups and identity-keeping
    /// IMU samples, plus the shared state `tick` writes into.
    fn make_playback() -> (
        Playback,
        Arc<Mutex<Cloud>>,
        Arc<AtomicU32>,
        Arc<Mutex<Quat>>,
        Arc<AtomicBool>,
    ) {
        let points = vec![
            LoadedPoint { ts_ns: 0, x_m: 1.0, y_m: 2.0, z_m: 3.0 },
            LoadedPoint { ts_ns: 0, x_m: 4.0, y_m: 5.0, z_m: 6.0 }, // same ts -> one group
            LoadedPoint { ts_ns: 1_000_000_000, x_m: 7.0, y_m: 8.0, z_m: 9.0 },
        ];
        // Zero gyro + gravity along Z keeps the estimator at identity so the
        // stored remap is exactly (x, z, -y) and easy to assert.
        let imu = vec![
            LoadedImu { ts_ns: 0, gyro: [0.0; 3], acc: [0.0, 0.0, 1.0] },
            LoadedImu { ts_ns: 500_000_000, gyro: [0.0; 3], acc: [0.0, 0.0, 1.0] },
        ];
        let cloud = Arc::new(Mutex::new(Cloud::new()));
        let max_age_ms = Arc::new(AtomicU32::new(5000.0f32.to_bits())); // 5 s window
        let latest_q = Arc::new(Mutex::new(Quat::IDENTITY));
        let fov_clip = Arc::new(AtomicBool::new(false));
        (Playback::new(points, imu), cloud, max_age_ms, latest_q, fov_clip)
    }

    #[test]
    fn playback_ticks_through_all_groups() {
        let (mut pb, cloud, max_age_ms, latest_q, fov_clip) = make_playback();
        // One big tick jumps the playhead to the last timestamp.
        pb.tick(10.0, &cloud, &max_age_ms, &latest_q, &fov_clip);

        let c = cloud.lock().unwrap();
        assert_eq!(c.points.len(), 3, "all three loaded points emitted");
        // Identity attitude: stored viewer-world = (body.x, body.z, -body.y).
        assert_eq!(c.points[0].0, [1.0, 3.0, -2.0]);
        assert_eq!(c.points[1].0, [4.0, 6.0, -5.0]);
        assert_eq!(c.points[2].0, [7.0, 9.0, -8.0]);
        assert!(!pb.playing, "playback should pause at the end");
        assert_eq!(pb.next_point, 3);
        assert_eq!(pb.next_imu, 2);
    }

    #[test]
    fn playback_advances_progressively_with_small_ticks() {
        let (mut pb, cloud, max_age_ms, latest_q, fov_clip) = make_playback();
        // First group is at t = 0; its points are == playhead so they emit on
        // the very first tick even with dt = 0.
        pb.tick(0.0, &cloud, &max_age_ms, &latest_q, &fov_clip);
        assert_eq!(cloud.lock().unwrap().points.len(), 2, "first group only");
        assert!(pb.playing, "not at the end yet");

        // Advance past the 1 s mark -> remaining group emits and it pauses.
        pb.tick(2.0, &cloud, &max_age_ms, &latest_q, &fov_clip);
        assert_eq!(cloud.lock().unwrap().points.len(), 3);
        assert!(!pb.playing);
    }

    #[test]
    fn playback_restart_clears_and_rewinds() {
        let (mut pb, cloud, max_age_ms, latest_q, fov_clip) = make_playback();
        pb.tick(10.0, &cloud, &max_age_ms, &latest_q, &fov_clip);
        assert!(!cloud.lock().unwrap().points.is_empty());

        pb.restart(&cloud);
        assert!(cloud.lock().unwrap().points.is_empty(), "restart clears cloud");
        assert_eq!(pb.next_point, 0);
        assert_eq!(pb.next_imu, 0);
        assert_eq!(pb.playhead_ns, pb.first_ts);
        assert!(pb.playing);
    }
}
