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
//!   cargo run --features viewer --bin lidar_viewer -- live <host_ip> <lidar_ip> [--encode-dir <folder>] [--rounds N]
//!   cargo run --features viewer --bin lidar_viewer -- replay <points.csv> [imu.csv] [--encode-dir <folder>] [--rounds N]
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
//!   E             - start/stop saving frame encodings (requires --encode-dir)
//!   B             - encode the current frame and print the time (saves nothing)
//!   Esc           - quit
//!
//! Encoding: with `--encode-dir <folder>` on the command line, pressing E
//! starts a recording: every frame's `utils::encode` token vector of the
//! currently buffered points is appended to a single file as one
//! comma-separated line (4 * 8^rounds values per line) until E is pressed
//! again. Each recording opens a new incrementally numbered file
//! (encoding_000001.csv, encoding_000002.csv, ...), so stopping and resuming
//! never overwrites. `--rounds N` (1-5, default 2) sets the subdivision
//! depth. Pressing B encodes the current frame and prints the elapsed time
//! without saving, so the real-time cost can be measured before recording.
//! Without `--encode-dir` the save toggle is disabled; B still works.
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

use std::fs::File;
use std::io::{BufWriter, Write};
use std::net::{Ipv4Addr, SocketAddr};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use glam::{Mat4, Vec3, Vec4};
use minifb::{Key, MouseButton, MouseMode, Window, WindowOptions};

use lidar_reader::client::{DataStream, LivoxClient};
use lidar_reader::imu::{AttitudeEstimator, OrientationHistory};
use lidar_reader::packet::{DataPacket, DataPayload};
use lidar_reader::points::{Cartesian32Point, Point, Tag};
use lidar_reader::protocol::DataType;
use lidar_reader::utils::cloud::Cloud;
use lidar_reader::utils::encode::{self, FLOATS_PER_SEGMENT, MAX_ROUNDS, MIN_ROUNDS};
use lidar_reader::{load_imu_csv, load_points_csv, LoadedImu, LoadedPoint};

const WIDTH: usize = 1024;
const HEIGHT: usize = 768;
const DEFAULT_MAX_AGE_MS: f32 = 500.0;
const AGE_STEP_MS: f32 = 100.0;
const AGE_MIN_MS: f32 = 50.0;
const AGE_MAX_MS: f32 = 5000.0;
/// IMU orientation history length (~4 s at 200 Hz) covers point-packet latency.
const ORIENTATION_HISTORY_LEN: usize = 800;
/// Octree rounds used for frame encodings unless `--rounds` overrides it.
const DEFAULT_ENCODE_ROUNDS: u8 = 2;

fn main() {
    let raw: Vec<String> = std::env::args().collect();
    let (args, encode_dir, rounds) = match split_encoding_flags(&raw) {
        Ok(parsed) => parsed,
        Err(e) => {
            eprintln!("{e}");
            print_usage();
            return;
        }
    };
    let encoder = Encoder::new(encode_dir, rounds);

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

            spawn_data_thread(host_ip, lidar_ip, cloud.clone(), max_age_ms.clone());
            run_window(cloud, max_age_ms, None, encoder);
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
            run_window(cloud, max_age_ms, playback, encoder);
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
    eprintln!("  lidar_viewer live <host_ip> <lidar_ip> [--encode-dir <folder>] [--rounds N]");
    eprintln!("  lidar_viewer replay <points.csv> [imu.csv] [--encode-dir <folder>] [--rounds N]");
    eprintln!();
    eprintln!("Options:");
    eprintln!("  --encode-dir <folder>  enable saving frame encodings with the E key:");
    eprintln!("                         each recording creates encoding_000001.csv,");
    eprintln!("                         encoding_000002.csv, ... in <folder>, one");
    eprintln!("                         comma-separated line per frame. Without it,");
    eprintln!("                         saving is disabled.");
    eprintln!("  --rounds N             octree subdivision rounds ({MIN_ROUNDS}-{MAX_ROUNDS},");
    eprintln!("                         default {DEFAULT_ENCODE_ROUNDS}); line size is 4 * 8^N floats");
    eprintln!();
    eprintln!("The legacy form `lidar_viewer <host_ip> <lidar_ip>` is still accepted.");
}

/// Prints the key bindings once at startup so they are always at hand.
fn print_controls(encoder: &Encoder) {
    eprintln!();
    eprintln!("Controls:");
    eprintln!("  drag (left)   orbit");
    eprintln!("  drag (right)  pan");
    eprintln!("  scroll        zoom");
    eprintln!("  Up / Down     increase / decrease point retention window");
    eprintln!("  C             clear the point buffer");
    if encoder.saving_enabled() {
        eprintln!(
            "  E             start/stop saving frame encodings to {}",
            encoder.dir.as_ref().expect("checked").display()
        );
    } else {
        eprintln!("  E             (disabled: relaunch with --encode-dir <folder> to enable)");
    }
    eprintln!(
        "  B             benchmark: encode current frame (rounds {}) and print the time",
        encoder.rounds
    );
    eprintln!("  Esc           quit");
    eprintln!("Replay only: Space play/pause, R restart, Z/X slow down/speed up");
    eprintln!();
}

/// Pulls `--encode-dir <folder>` and `--rounds N` out of the argument list,
/// returning the remaining positional arguments plus the parsed options.
fn split_encoding_flags(
    raw: &[String],
) -> std::result::Result<(Vec<String>, Option<PathBuf>, u8), String> {
    let mut args = Vec::with_capacity(raw.len());
    args.push(raw[0].clone());
    let mut dir = None;
    let mut rounds = DEFAULT_ENCODE_ROUNDS;

    let mut i = 1;
    while i < raw.len() {
        match raw[i].as_str() {
            "--encode-dir" => {
                i += 1;
                let Some(path) = raw.get(i) else {
                    return Err("--encode-dir requires a folder path".to_string());
                };
                dir = Some(PathBuf::from(path));
            }
            "--rounds" => {
                i += 1;
                let Some(value) = raw.get(i) else {
                    return Err("--rounds requires a value".to_string());
                };
                rounds = value
                    .parse::<u8>()
                    .ok()
                    .filter(|r| (MIN_ROUNDS..=MAX_ROUNDS).contains(r))
                    .ok_or_else(|| {
                        format!("--rounds must be {MIN_ROUNDS}-{MAX_ROUNDS}, got '{value}'")
                    })?;
            }
            other => args.push(other.to_string()),
        }
        i += 1;
    }
    Ok((args, dir, rounds))
}

/// Runtime state of the frame-encoding feature. Benchmarking (key B) is
/// always available; saving (key E) only when a folder was given at launch.
struct Encoder {
    rounds: u8,
    dir: Option<PathBuf>,
    /// Open output file while recording; `None` when stopped.
    out: Option<BufWriter<File>>,
    next_index: u32,
    /// Frames written to the current file.
    frames: u32,
}

impl Encoder {
    fn new(dir: Option<PathBuf>, rounds: u8) -> Self {
        let dir = dir.and_then(|d| match std::fs::create_dir_all(&d) {
            Ok(()) => Some(d),
            Err(e) => {
                eprintln!("encoding saves disabled: cannot create {}: {e}", d.display());
                None
            }
        });
        Self {
            rounds,
            dir,
            out: None,
            next_index: 1,
            frames: 0,
        }
    }

    fn saving_enabled(&self) -> bool {
        self.dir.is_some()
    }

    fn is_recording(&self) -> bool {
        self.out.is_some()
    }

    /// Toggles saving on/off. Each start opens a new incrementally numbered
    /// file, so resumed recordings never overwrite earlier ones.
    fn toggle_recording(&mut self) {
        if self.is_recording() {
            self.stop();
        } else {
            self.start();
        }
    }

    fn start(&mut self) {
        let Some(dir) = &self.dir else {
            eprintln!("encoding saves are disabled: relaunch with --encode-dir <folder>");
            return;
        };
        let path = dir.join(format!("encoding_{:06}.csv", self.next_index));
        match File::create(&path) {
            Ok(file) => {
                self.out = Some(BufWriter::new(file));
                self.next_index += 1;
                self.frames = 0;
                eprintln!(
                    "encoding REC ON  -> {} | rounds {} | {} floats/line | press E to stop",
                    path.display(),
                    self.rounds,
                    FLOATS_PER_SEGMENT * 8usize.pow(self.rounds as u32),
                );
            }
            Err(e) => eprintln!("failed to create {}: {e}", path.display()),
        }
    }

    fn stop(&mut self) {
        if let Some(mut out) = self.out.take() {
            if let Err(e) = out.flush() {
                eprintln!("failed to flush encoding file: {e}");
            }
        }
        eprintln!("encoding REC OFF -- {} frame(s) written", self.frames);
    }

    /// Encodes one frame and appends it as a comma-separated line.
    /// No-op while not recording; empty clouds are skipped.
    fn save_frame(&mut self, cloud: &[[f32; 3]]) {
        let Some(out) = self.out.as_mut() else { return };
        let tokens = match encode::encode(cloud, self.rounds) {
            Ok(t) => t,
            Err(_) => return,
        };
        if let Err(e) = write_frame(out, &tokens) {
            eprintln!("failed to write frame: {e} -- recording stopped");
            self.stop();
            return;
        }
        self.frames += 1;
    }

    /// Encodes the current frame without saving and prints the elapsed time.
    fn benchmark(&self, cloud: &[[f32; 3]]) {
        let start = Instant::now();
        match encode::encode(cloud, self.rounds) {
            Ok(tokens) => eprintln!(
                "encode benchmark: {} points | rounds {} -> {} floats in {:.2} ms",
                cloud.len(),
                self.rounds,
                tokens.len(),
                start.elapsed().as_secs_f64() * 1000.0,
            ),
            Err(e) => eprintln!("encode benchmark: {e}"),
        }
    }
}

/// Appends one frame as a single comma-separated line.
fn write_frame(out: &mut impl Write, tokens: &[f32]) -> std::io::Result<()> {
    let line = tokens
        .iter()
        .map(|t| t.to_string())
        .collect::<Vec<_>>()
        .join(",");
    writeln!(out, "{line}")
}

/// Copies the buffered viewer-world points for encoding.
fn cloud_points(cloud: &Mutex<Cloud>) -> Vec<[f32; 3]> {
    match cloud.lock() {
        Ok(c) => c.positions(),
        Err(_) => Vec::new(),
    }
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
                            if let Ok(mut c) = cloud.lock() {
                                c.add(&pts, header.timestamp, max_age_ns, q);
                            }
                        }
                    }
                }
            }
        });
    });
}

/// Replays recorded CSVs through the same stabilization pipeline as the live
/// stream. Each frame [`tick`] advances the playhead by wall-clock time
/// scaled by `speed`, feeds IMU samples up to the playhead into the attitude
/// estimator, and emits point groups up to the playhead into the cloud with
/// the attitude looked up at each group's timestamp -- exactly mirroring how
/// the live thread ingests packets.
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
    /// groups into the shared cloud.
    fn tick(&mut self, dt_secs: f32, cloud: &Arc<Mutex<Cloud>>, max_age_ms: &AtomicU32) {
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

        // Emit point groups up to the playhead. Points within one packet share
        // a timestamp; group consecutive equal-timestamp rows into one batch.
        let max_age_ns = (f32::from_bits(max_age_ms.load(Ordering::Relaxed)) * 1_000_000.0) as u64;
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
                c.add(&group, ts, max_age_ns, q);
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
    mut playback: Option<Playback>,
    mut encoder: Encoder,
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

    print_controls(&encoder);

    while window.is_open() && !window.is_key_down(Key::Escape) {
        let dt = last_frame.elapsed().as_secs_f32();
        last_frame = Instant::now();

        handle_input(
            &window,
            &mut cam,
            &mut prev_mouse,
            &max_age_ms,
            &cloud,
            &mut encoder,
        );

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
            pb.tick(dt, &cloud, &max_age_ms);
        }

        let view = Mat4::look_at_rh(cam.eye(), Vec3::from(cam.target), Vec3::Y);
        let proj = Mat4::perspective_rh(1.0, WIDTH as f32 / HEIGHT as f32, 0.05, 1000.0);

        buffer.fill(0xFF0A0A0F); // dark background
        zbuffer.fill(f32::NEG_INFINITY);

        draw_axes(&mut buffer, &mut zbuffer, view, proj);

        let count = render_points(&cloud, &mut buffer, &mut zbuffer, view, proj);

        // Append this frame's encoding while recording.
        if encoder.is_recording() {
            encoder.save_frame(&cloud_points(&cloud));
        }

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
        let title = match &playback {
            Some(pb) => format!(
                "Livox MID360 | replay {pb} | points: {count} | age: {age:.0} ms | fps: {fps:.0} | dist: {:.1} m",
                cam.distance
            ),
            None => format!(
                "Livox MID360 | points: {count} | age: {age:.0} ms | fps: {fps:.0} | dist: {:.1} m",
                cam.distance
            ),
        };
        window.set_title(&title);
    }
}

/// Render the current point cloud and return how many points were drawn.
fn render_points(
    cloud: &Mutex<Cloud>,
    buffer: &mut [u32],
    zbuffer: &mut [f32],
    view: Mat4,
    proj: Mat4,
) -> usize {
    let snapshot: Vec<[f32; 3]> = match cloud.lock() {
        Ok(c) => c.positions(),
        Err(_) => return 0,
    };
    let mut drawn = 0;
    for p in &snapshot {
        if let Some((sx, sy, vz)) = project(*p, view, proj) {
            put_point(buffer, zbuffer, sx, sy, vz, height_color(p[1]));
            drawn += 1;
        }
    }
    drawn
}

fn handle_input(
    window: &Window,
    cam: &mut Camera,
    prev_mouse: &mut Option<(f32, f32)>,
    max_age_ms: &AtomicU32,
    cloud: &Mutex<Cloud>,
    encoder: &mut Encoder,
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
            Key::B => encoder.benchmark(&cloud_points(cloud)),
            Key::E => encoder.toggle_recording(),
            Key::C => {
                if let Ok(mut c) = cloud.lock() {
                    c.clear();
                    eprintln!("cleared point buffer");
                }
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

    /// Build a `Playback` with a couple of point groups and identity-keeping
    /// IMU samples, plus the shared state `tick` writes into.
    fn make_playback() -> (Playback, Arc<Mutex<Cloud>>, Arc<AtomicU32>) {
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
        (Playback::new(points, imu), cloud, max_age_ms)
    }

    #[test]
    fn playback_ticks_through_all_groups() {
        let (mut pb, cloud, max_age_ms) = make_playback();
        // One big tick jumps the playhead to the last timestamp.
        pb.tick(10.0, &cloud, &max_age_ms);

        let c = cloud.lock().unwrap();
        assert_eq!(c.len(), 3, "all three loaded points emitted");
        // Identity attitude: stored viewer-world = (body.x, body.z, -body.y).
        assert_eq!(c.points()[0].0, [1.0, 3.0, -2.0]);
        assert_eq!(c.points()[1].0, [4.0, 6.0, -5.0]);
        assert_eq!(c.points()[2].0, [7.0, 9.0, -8.0]);
        assert!(!pb.playing, "playback should pause at the end");
        assert_eq!(pb.next_point, 3);
        assert_eq!(pb.next_imu, 2);
    }

    #[test]
    fn playback_advances_progressively_with_small_ticks() {
        let (mut pb, cloud, max_age_ms) = make_playback();
        // First group is at t = 0; its points are == playhead so they emit on
        // the very first tick even with dt = 0.
        pb.tick(0.0, &cloud, &max_age_ms);
        assert_eq!(cloud.lock().unwrap().len(), 2, "first group only");
        assert!(pb.playing, "not at the end yet");

        // Advance past the 1 s mark -> remaining group emits and it pauses.
        pb.tick(2.0, &cloud, &max_age_ms);
        assert_eq!(cloud.lock().unwrap().len(), 3);
        assert!(!pb.playing);
    }

    #[test]
    fn playback_restart_clears_and_rewinds() {
        let (mut pb, cloud, max_age_ms) = make_playback();
        pb.tick(10.0, &cloud, &max_age_ms);
        assert!(!cloud.lock().unwrap().is_empty());

        pb.restart(&cloud);
        assert!(cloud.lock().unwrap().is_empty(), "restart clears cloud");
        assert_eq!(pb.next_point, 0);
        assert_eq!(pb.next_imu, 0);
        assert_eq!(pb.playhead_ns, pb.first_ts);
        assert!(pb.playing);
    }

    fn str_args(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn flags_extract_encode_dir_and_rounds() {
        let raw = str_args(&[
            "viewer", "live", "1.1.1.1", "2.2.2.2", "--encode-dir", "/tmp/out", "--rounds", "3",
        ]);
        let (args, dir, rounds) = split_encoding_flags(&raw).unwrap();
        assert_eq!(args, ["viewer", "live", "1.1.1.1", "2.2.2.2"]);
        assert_eq!(dir, Some(PathBuf::from("/tmp/out")));
        assert_eq!(rounds, 3);
    }

    #[test]
    fn flags_default_to_no_dir_and_default_rounds() {
        let raw = str_args(&["viewer", "replay", "points.csv"]);
        let (args, dir, rounds) = split_encoding_flags(&raw).unwrap();
        assert_eq!(args, ["viewer", "replay", "points.csv"]);
        assert_eq!(dir, None);
        assert_eq!(rounds, DEFAULT_ENCODE_ROUNDS);
    }

    #[test]
    fn flags_reject_bad_rounds() {
        assert!(split_encoding_flags(&str_args(&["viewer", "--rounds", "9"])).is_err());
        assert!(split_encoding_flags(&str_args(&["viewer", "--rounds"])).is_err());
        assert!(split_encoding_flags(&str_args(&["viewer", "--encode-dir"])).is_err());
    }

    #[test]
    fn toggle_without_dir_keeps_recording_disabled() {
        let mut encoder = Encoder::new(None, 2);
        encoder.toggle_recording();
        assert!(!encoder.is_recording());
    }

    #[test]
    fn recording_writes_one_file_per_session_one_line_per_frame() {
        let dir = std::env::temp_dir().join(format!("lidar_viewer_encode_test_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut encoder = Encoder::new(Some(dir.clone()), 1);
        let cloud = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]];

        encoder.toggle_recording(); // on -> encoding_000001.csv
        encoder.save_frame(&cloud);
        encoder.save_frame(&cloud);
        encoder.toggle_recording(); // off
        encoder.save_frame(&cloud); // not recording: skipped
        encoder.toggle_recording(); // on again -> encoding_000002.csv
        encoder.save_frame(&cloud);
        encoder.toggle_recording(); // off

        let first = std::fs::read_to_string(dir.join("encoding_000001.csv")).unwrap();
        assert_eq!(first.lines().count(), 2, "one line per recorded frame");
        for line in first.lines() {
            assert_eq!(line.split(',').count(), FLOATS_PER_SEGMENT * 8);
        }
        let second = std::fs::read_to_string(dir.join("encoding_000002.csv")).unwrap();
        assert_eq!(second.lines().count(), 1);
        assert_eq!(second.split(',').count(), FLOATS_PER_SEGMENT * 8);
        assert!(!dir.join("encoding_000003.csv").exists());
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
