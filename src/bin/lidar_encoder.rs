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

//! Headless point-cloud encoder for the Livox MID360.
//!
//! Same data path as `lidar_viewer` (IMU stabilization + rolling retention
//! window over a shared [`Cloud`]) but with no window: every incoming frame
//! is encoded with `utils::encode` and appended as one comma-separated line
//! to a single CSV in the folder given by `--encode-dir`.
//!
//! Build/run with the `encoder` feature:
//!   cargo run --features encoder --bin lidar_encoder -- live <host_ip> <lidar_ip> --encode-dir <folder> [--rounds N] [--window MS]
//!   cargo run --features encoder --bin lidar_encoder -- replay <points.csv> [imu.csv] --encode-dir <folder> [--rounds N] [--window MS]
//!
//! The legacy form `lidar_encoder <host_ip> <lidar_ip> --encode-dir ...` is
//! equivalent to passing `live` first.
//!
//! Live mode encodes every received point packet until Ctrl-C. Replay mode
//! encodes every recorded frame as fast as the CPU allows and exits.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::net::{Ipv4Addr, SocketAddr};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use glam::Vec3;

use lidar_reader::client::{DataStream, LivoxClient};
use lidar_reader::imu::{AttitudeEstimator, OrientationHistory};
use lidar_reader::packet::{DataPacket, DataPayload};
use lidar_reader::points::{Cartesian32Point, Point, Tag};
use lidar_reader::protocol::DataType;
use lidar_reader::utils::cloud::Cloud;
use lidar_reader::utils::encode::{self, FLOATS_PER_SEGMENT, MAX_ROUNDS, MIN_ROUNDS};
use lidar_reader::{load_imu_csv, load_points_csv};

/// IMU orientation history length (~4 s at 200 Hz) covers point-packet latency.
const ORIENTATION_HISTORY_LEN: usize = 800;
/// Octree rounds used unless `--rounds` overrides it.
const DEFAULT_ENCODE_ROUNDS: u8 = 2;
/// Point retention window used unless `--window` overrides it.
const DEFAULT_WINDOW_MS: f32 = 500.0;
/// Name of the single output file created per run.
const OUTPUT_FILE: &str = "encoding_000001.csv";

struct Options {
    dir: PathBuf,
    rounds: u8,
    window_ms: f32,
}

impl Options {
    fn window_ns(&self) -> u64 {
        (self.window_ms * 1_000_000.0) as u64
    }
}

enum Mode {
    Live,
    Replay,
}

#[tokio::main]
async fn main() {
    let raw: Vec<String> = std::env::args().collect();
    let (args, encode_dir, rounds, window_ms) = match split_flags(&raw) {
        Ok(parsed) => parsed,
        Err(e) => {
            eprintln!("{e}");
            print_usage();
            return;
        }
    };

    let mode = match args.get(1).map(String::as_str) {
        Some("live") => Mode::Live,
        Some("replay") => Mode::Replay,
        _ => {
            // Legacy form: `lidar_encoder <host_ip> <lidar_ip>` is live mode.
            if args.get(1).is_some_and(|s| s.parse::<Ipv4Addr>().is_ok()) {
                Mode::Live
            } else {
                print_usage();
                return;
            }
        }
    };

    let Some(dir) = encode_dir else {
        eprintln!("--encode-dir <folder> is required");
        print_usage();
        return;
    };
    let opts = Options {
        dir,
        rounds,
        window_ms,
    };

    match mode {
        Mode::Live => {
            // IP args sit at positions 2/3 (`live <host> <lidar>`) or 1/2 in
            // the legacy form (no `live` keyword).
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
            run_live(host_ip, lidar_ip, &opts).await;
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
            run_replay(&points_path, imu_path.as_deref(), &opts);
        }
    }
}

fn print_usage() {
    eprintln!("Usage:");
    eprintln!("  lidar_encoder live <host_ip> <lidar_ip> --encode-dir <folder> [--rounds N] [--window MS]");
    eprintln!("  lidar_encoder replay <points.csv> [imu.csv] --encode-dir <folder> [--rounds N] [--window MS]");
    eprintln!();
    eprintln!("Writes one comma-separated line per frame (4 * 8^N floats) to");
    eprintln!("<folder>/{OUTPUT_FILE}. Live runs until Ctrl-C; replay runs as fast as");
    eprintln!("possible and exits. The legacy form `lidar_encoder <host_ip> <lidar_ip>`");
    eprintln!("is still accepted.");
    eprintln!();
    eprintln!("Options:");
    eprintln!("  --encode-dir <folder>  target folder for the encoding file (required)");
    eprintln!("  --rounds N             octree subdivision rounds ({MIN_ROUNDS}-{MAX_ROUNDS}, default {DEFAULT_ENCODE_ROUNDS})");
    eprintln!("  --window MS            point retention window in milliseconds (default {DEFAULT_WINDOW_MS})");
}

/// Pulls `--encode-dir <folder>`, `--rounds N` and `--window MS` out of the
/// argument list, returning the remaining positional arguments plus the
/// parsed options.
fn split_flags(raw: &[String]) -> Result<(Vec<String>, Option<PathBuf>, u8, f32), String> {
    let mut args = Vec::with_capacity(raw.len());
    args.push(raw[0].clone());
    let mut dir = None;
    let mut rounds = DEFAULT_ENCODE_ROUNDS;
    let mut window_ms = DEFAULT_WINDOW_MS;

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
            "--window" => {
                i += 1;
                let Some(value) = raw.get(i) else {
                    return Err("--window requires a value in milliseconds".to_string());
                };
                window_ms = value
                    .parse::<f32>()
                    .ok()
                    .filter(|w| *w > 0.0)
                    .ok_or_else(|| {
                        format!("--window must be a positive number of milliseconds, got '{value}'")
                    })?;
            }
            other => args.push(other.to_string()),
        }
        i += 1;
    }
    Ok((args, dir, rounds, window_ms))
}

/// Bind the command/data sockets and configure the LiDAR to stream point
/// cloud + IMU to this host. Returns the data stream on success.
async fn start_stream(host_ip: Ipv4Addr, lidar_ip: Ipv4Addr) -> Option<DataStream> {
    let client = match LivoxClient::with_default_cmd_port(host_ip).await {
        Ok(c) => c,
        Err(e) => {
            eprintln!("failed to bind command socket: {e}");
            return None;
        }
    };
    let stream = match DataStream::with_default_ports(host_ip).await {
        Ok(s) => s,
        Err(e) => {
            eprintln!("failed to bind data sockets: {e}");
            return None;
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
        return None;
    }
    Some(stream)
}

/// Live mode: encode every incoming point packet until Ctrl-C.
async fn run_live(host_ip: Ipv4Addr, lidar_ip: Ipv4Addr, opts: &Options) {
    let Some(stream) = start_stream(host_ip, lidar_ip).await else {
        return;
    };
    let mut out = match open_output(&opts.dir) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("cannot open output in {}: {e}", opts.dir.display());
            return;
        }
    };
    print_banner("live", opts);
    eprintln!("streaming; press Ctrl-C to stop.");

    let mut estimator = AttitudeEstimator::new();
    let mut history = OrientationHistory::new(ORIENTATION_HISTORY_LEN);
    let mut cloud = Cloud::new();
    let mut frames = 0u64;
    let started = Instant::now();

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => break,
            pkt = stream.next_imu(Duration::from_millis(200)) => {
                if let Ok(DataPacket {
                    header,
                    payload: DataPayload::Imu(imu),
                }) = pkt
                {
                    estimator.update(
                        Vec3::new(imu.gyro_x, imu.gyro_y, imu.gyro_z),
                        Vec3::new(imu.acc_x, imu.acc_y, imu.acc_z),
                        header.timestamp,
                    );
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
                    cloud.add(&pts, header.timestamp, opts.window_ns(), q);
                    if !encode_frame(&cloud, opts.rounds, &mut out, &mut frames) {
                        break;
                    }
                }
            }
        }
    }
    finish(&mut out, frames, &started);
}

/// Replay mode: walk the recorded frames in timestamp order as fast as
/// possible, feeding IMU samples and point groups through the same
/// stabilization pipeline as live mode.
fn run_replay(points_path: &Path, imu_path: Option<&Path>, opts: &Options) {
    let points = match load_points_csv(points_path) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("failed to load {}: {e}", points_path.display());
            return;
        }
    };
    if points.is_empty() {
        eprintln!("no points found in {}", points_path.display());
        return;
    }
    let imu = match imu_path {
        Some(p) => match load_imu_csv(p) {
            Ok(v) => v,
            Err(e) => {
                eprintln!("failed to load {}: {e}", p.display());
                return;
            }
        },
        None => Vec::new(),
    };
    let mut out = match open_output(&opts.dir) {
        Ok(f) => f,
        Err(e) => {
            eprintln!("cannot open output in {}: {e}", opts.dir.display());
            return;
        }
    };
    print_banner("replay", opts);

    let mut estimator = AttitudeEstimator::new();
    let mut history = OrientationHistory::new(ORIENTATION_HISTORY_LEN);
    let mut cloud = Cloud::new();
    let mut next_imu = 0;
    let mut next_point = 0;
    let mut frames = 0u64;
    let started = Instant::now();

    while next_point < points.len() {
        let ts = points[next_point].ts_ns;

        // Feed IMU samples up to this frame's timestamp.
        while next_imu < imu.len() && imu[next_imu].ts_ns <= ts {
            let r = &imu[next_imu];
            estimator.update(Vec3::from(r.gyro), Vec3::from(r.acc), r.ts_ns);
            history.push(r.ts_ns, estimator.q);
            next_imu += 1;
        }

        // Points within one packet share a timestamp; group them into one batch.
        let mut group: Vec<Point> = Vec::new();
        while next_point < points.len() && points[next_point].ts_ns == ts {
            let lp = points[next_point];
            // Body-frame metres -> mm i32, matching the viewer's point path.
            group.push(Point::Cartesian32(Cartesian32Point {
                x_mm: (lp.x_m * 1000.0) as i32,
                y_mm: (lp.y_m * 1000.0) as i32,
                z_mm: (lp.z_m * 1000.0) as i32,
                reflectivity: 0,
                tag: Tag(0),
            }));
            next_point += 1;
        }

        let q = history.at(ts);
        cloud.add(&group, ts, opts.window_ns(), q);
        if !encode_frame(&cloud, opts.rounds, &mut out, &mut frames) {
            break;
        }
    }
    finish(&mut out, frames, &started);
}

/// Encodes the current cloud and appends it as one line. Returns false when
/// a write fails and the run should stop. Empty clouds are skipped.
fn encode_frame(cloud: &Cloud, rounds: u8, out: &mut BufWriter<File>, frames: &mut u64) -> bool {
    let tokens = match encode::encode(&cloud.positions(), rounds) {
        Ok(t) => t,
        Err(_) => return true,
    };
    if let Err(e) = write_frame(out, &tokens) {
        eprintln!("write failed: {e} -- stopping");
        return false;
    }
    *frames += 1;
    // Periodic flush + progress so long recordings survive interruptions.
    if *frames % 100 == 0 {
        if let Err(e) = out.flush() {
            eprintln!("flush failed: {e} -- stopping");
            return false;
        }
        eprintln!("{frames} frames encoded | cloud {} points", cloud.len());
    }
    true
}

fn open_output(dir: &Path) -> std::io::Result<BufWriter<File>> {
    std::fs::create_dir_all(dir)?;
    Ok(BufWriter::new(File::create(dir.join(OUTPUT_FILE))?))
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

fn print_banner(mode: &str, opts: &Options) {
    eprintln!(
        "{mode} -> {} | rounds {} | {} floats/line | window {:.0} ms",
        opts.dir.join(OUTPUT_FILE).display(),
        opts.rounds,
        FLOATS_PER_SEGMENT * 8usize.pow(opts.rounds as u32),
        opts.window_ms,
    );
}

fn finish(out: &mut BufWriter<File>, frames: u64, started: &Instant) {
    if let Err(e) = out.flush() {
        eprintln!("final flush failed: {e}");
    }
    let secs = started.elapsed().as_secs_f64();
    if frames > 0 {
        eprintln!(
            "done: {frames} frames in {secs:.1} s ({:.2} ms/frame)",
            secs * 1000.0 / frames as f64
        );
    } else {
        eprintln!("done: no frames encoded");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn str_args(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn flags_parse_all_options() {
        let raw = str_args(&[
            "enc", "live", "1.1.1.1", "2.2.2.2", "--encode-dir", "/tmp/o", "--rounds", "3",
            "--window", "250",
        ]);
        let (args, dir, rounds, window_ms) = split_flags(&raw).unwrap();
        assert_eq!(args, ["enc", "live", "1.1.1.1", "2.2.2.2"]);
        assert_eq!(dir, Some(PathBuf::from("/tmp/o")));
        assert_eq!(rounds, 3);
        assert_eq!(window_ms, 250.0);
    }

    #[test]
    fn flags_default_rounds_and_window() {
        let (args, dir, rounds, window_ms) =
            split_flags(&str_args(&["enc", "replay", "p.csv"])).unwrap();
        assert_eq!(args, ["enc", "replay", "p.csv"]);
        assert_eq!(dir, None);
        assert_eq!(rounds, DEFAULT_ENCODE_ROUNDS);
        assert_eq!(window_ms, DEFAULT_WINDOW_MS);
    }

    #[test]
    fn flags_reject_bad_values() {
        assert!(split_flags(&str_args(&["enc", "--window", "0"])).is_err());
        assert!(split_flags(&str_args(&["enc", "--window", "abc"])).is_err());
        assert!(split_flags(&str_args(&["enc", "--window"])).is_err());
        assert!(split_flags(&str_args(&["enc", "--rounds", "7"])).is_err());
        assert!(split_flags(&str_args(&["enc", "--encode-dir"])).is_err());
    }

    #[test]
    fn replay_writes_one_line_per_frame_group() {
        let dir = std::env::temp_dir().join(format!("lidar_encoder_test_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let points_csv = dir.join("points.csv");
        std::fs::write(
            &points_csv,
            "timestamp_ns,udp_cnt,frame_cnt,x_m,y_m,z_m,reflectivity,tag,a,b,c\n\
             0,0,0,1.0,0.0,0.0,0,0,0,0,0\n\
             0,0,0,2.0,0.0,0.0,0,0,0,0,0\n\
             100000000,1,0,3.0,0.0,0.0,0,0,0,0,0\n",
        )
        .unwrap();

        let opts = Options {
            dir: dir.clone(),
            rounds: 1,
            window_ms: 500.0,
        };
        run_replay(&points_csv, None, &opts);

        let out = std::fs::read_to_string(dir.join(OUTPUT_FILE)).unwrap();
        assert_eq!(out.lines().count(), 2, "two timestamp groups -> two frames");
        for line in out.lines() {
            assert_eq!(line.split(',').count(), FLOATS_PER_SEGMENT * 8);
        }
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
