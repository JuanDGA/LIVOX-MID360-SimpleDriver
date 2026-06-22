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

use std::io::Write as _;
use std::net::{Ipv4Addr, SocketAddr};
use std::time::Duration;

use lidar_reader::client::{DataStream, LivoxClient};
use lidar_reader::protocol::DataType;

#[tokio::main]
async fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        print_usage();
        return;
    }

    match args[1].as_str() {
        "discover" => {
            let host_ip = args
                .get(2)
                .map(|s| parse_ip(s))
                .unwrap_or(Ipv4Addr::UNSPECIFIED);
            run_discovery(host_ip).await;
        }
        "stream" => {
            let host_ip = match args.get(2) {
                Some(s) => parse_ip(s),
                None => {
                    eprintln!("stream requires <host_ip> <lidar_ip>");
                    print_usage();
                    return;
                }
            };
            let lidar_ip = match args.get(3) {
                Some(s) => parse_ip(s),
                None => {
                    eprintln!("stream requires <host_ip> <lidar_ip>");
                    print_usage();
                    return;
                }
            };
            run_stream(host_ip, lidar_ip).await;
        }
        "record" => {
            let host_ip = match args.get(2) {
                Some(s) => parse_ip(s),
                None => {
                    eprintln!("record requires <host_ip> <lidar_ip> <output_dir>");
                    print_usage();
                    return;
                }
            };
            let lidar_ip = match args.get(3) {
                Some(s) => parse_ip(s),
                None => {
                    eprintln!("record requires <host_ip> <lidar_ip> <output_dir>");
                    print_usage();
                    return;
                }
            };
            let out_dir = match args.get(4) {
                Some(s) => s.clone(),
                None => {
                    eprintln!("record requires <host_ip> <lidar_ip> <output_dir>");
                    print_usage();
                    return;
                }
            };
            run_record(host_ip, lidar_ip, out_dir).await;
        }
        _ => print_usage(),
    }
}

fn print_usage() {
    eprintln!("Usage:");
    eprintln!("  lidar_reader discover [host_ip]                   -- discover MID360 LiDARs");
    eprintln!("  lidar_reader stream <host_ip> <lidar_ip>          -- receive point cloud / IMU");
    eprintln!("  lidar_reader record <host_ip> <lidar_ip> <dir>    -- record point cloud / IMU to CSVs");
    eprintln!();
    eprintln!("  host_ip:  IPv4 address of the local network interface connected to the LiDAR");
    eprintln!("            (e.g. 192.168.1.50). For discover you may omit this and use 0.0.0.0.");
    eprintln!("  lidar_ip: IPv4 address of the LiDAR (e.g. 192.168.1.100).");
    eprintln!("  dir:      target folder for points.csv and imu.csv (created if missing).");
}

fn parse_ip(s: &str) -> Ipv4Addr {
    s.parse().expect("invalid IPv4 address")
}

fn bind_error_context(resource: &str, ip: Ipv4Addr) -> String {
    format!(
        "failed to bind {resource} socket to {ip}. \
         Make sure this is the IP address of a local network interface, \
         not the LiDAR's IP address.",
    )
}

async fn run_discovery(host_ip: Ipv4Addr) {
    let client = match LivoxClient::with_default_cmd_port(host_ip).await {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{}: {e}", bind_error_context("command", host_ip));
            return;
        }
    };
    let broadcast = SocketAddr::from((Ipv4Addr::BROADCAST, lidar_reader::protocol::DISCOVERY_PORT));

    println!("Sending discovery broadcast to {broadcast} from {host_ip}...");
    match client.discover(broadcast, Duration::from_secs(1)).await {
        Ok(devices) => {
            if devices.is_empty() {
                println!("No LiDARs found.");
                return;
            }
            for d in devices {
                let sn = String::from_utf8_lossy(&d.serial_number);
                println!(
                    "Found device type={} serial={} cmd_addr={}",
                    d.dev_type,
                    sn.trim_end_matches('\0'),
                    d.lidar_cmd_addr
                );
            }
        }
        Err(e) => eprintln!("Discovery failed: {e}"),
    }
}

/// Bind the command/data sockets and configure the LiDAR to stream point
/// cloud + IMU to this host. Returns the data stream on success; prints a
/// diagnostic and returns `None` on failure (so callers can early-return).
async fn start_lidar_stream(host_ip: Ipv4Addr, lidar_ip: Ipv4Addr) -> Option<DataStream> {
    let client = match LivoxClient::with_default_cmd_port(host_ip).await {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{}: {e}", bind_error_context("command", host_ip));
            return None;
        }
    };
    let stream = match DataStream::with_default_ports(host_ip).await {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{}: {e}", bind_error_context("data/imu", host_ip));
            return None;
        }
    };

    let lidar_cmd_addr = SocketAddr::from((lidar_ip, lidar_reader::protocol::CMD_PORT));
    let data_dst = SocketAddr::from((host_ip, lidar_reader::protocol::HOST_DATA_PORT));
    let imu_dst = SocketAddr::from((host_ip, lidar_reader::protocol::HOST_IMU_PORT));

    println!("Configuring LiDAR at {lidar_cmd_addr} to stream to {data_dst} / {imu_dst}");
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
        eprintln!("Failed to start streaming: {e}");
        return None;
    }
    Some(stream)
}

fn require_specific_host_ip(subcommand: &str, host_ip: Ipv4Addr) -> bool {
    if host_ip.is_unspecified() {
        eprintln!(
            "{subcommand} requires a specific host_ip (0.0.0.0 is not valid here); \
             use the IP of the interface connected to the LiDAR"
        );
        false
    } else {
        true
    }
}

async fn run_stream(host_ip: Ipv4Addr, lidar_ip: Ipv4Addr) {
    if !require_specific_host_ip("stream", host_ip) {
        return;
    }
    let Some(stream) = start_lidar_stream(host_ip, lidar_ip).await else {
        return;
    };

    println!("Streaming; press Ctrl-C to stop.");
    loop {
        tokio::select! {
            result = stream.next_point_cloud(Duration::from_secs(1)) => {
                match result {
                    Ok(packet) => {
                        if let lidar_reader::packet::DataPayload::Points(points) = &packet.payload {
                            print!(
                                "\rpoint cloud: udp={} ts={} ns points={}",
                                packet.header.udp_cnt, packet.header.timestamp, points.len()
                            );
                            let _ = std::io::stdout().flush();
                        }
                    }
                    Err(lidar_reader::LidarError::NoResponse { .. }) => {}
                    Err(e) => eprintln!("point cloud error: {e}"),
                }
            }
            result = stream.next_imu(Duration::from_secs(1)) => {
                match result {
                    Ok(packet) => {
                        if let lidar_reader::packet::DataPayload::Imu(imu) = &packet.payload {
                            print!(
                                "\rIMU gyro=({:.3},{:.3},{:.3}) acc=({:.3},{:.3},{:.3})",
                                imu.gyro_x, imu.gyro_y, imu.gyro_z,
                                imu.acc_x, imu.acc_y, imu.acc_z
                            );
                            let _ = std::io::stdout().flush();
                        }
                    }
                    Err(lidar_reader::LidarError::NoResponse { .. }) => {}
                    Err(e) => eprintln!("imu error: {e}"),
                }
            }
        }
    }
}

async fn run_record(host_ip: Ipv4Addr, lidar_ip: Ipv4Addr, out_dir: String) {
    if !require_specific_host_ip("record", host_ip) {
        return;
    }
    let Some(stream) = start_lidar_stream(host_ip, lidar_ip).await else {
        return;
    };

    let mut recorder = match lidar_reader::CsvRecorder::open(&out_dir) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("Failed to open CSV files in '{out_dir}': {e}");
            return;
        }
    };
    println!("Recording into {out_dir}; press Ctrl-C to stop.");

    let mut flush = tokio::time::interval(Duration::from_secs(1));
    let mut points_written: u64 = 0;
    let mut imu_written: u64 = 0;

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("\nStopping...");
                break;
            }
            result = stream.next_point_cloud(Duration::from_secs(1)) => {
                match result {
                    Ok(packet) => {
                        if let lidar_reader::packet::DataPayload::Points(pts) = &packet.payload {
                            if let Err(e) = recorder.write_points(&packet.header, pts) {
                                eprintln!("point CSV write failed: {e}");
                                break;
                            }
                            points_written += pts.len() as u64;
                        }
                    }
                    Err(lidar_reader::LidarError::NoResponse { .. }) => {}
                    Err(e) => eprintln!("point cloud error: {e}"),
                }
            }
            result = stream.next_imu(Duration::from_secs(1)) => {
                match result {
                    Ok(packet) => {
                        if let lidar_reader::packet::DataPayload::Imu(imu) = &packet.payload {
                            if let Err(e) = recorder.write_imu(&packet.header, imu) {
                                eprintln!("imu CSV write failed: {e}");
                                break;
                            }
                            imu_written += 1;
                        }
                    }
                    Err(lidar_reader::LidarError::NoResponse { .. }) => {}
                    Err(e) => eprintln!("imu error: {e}"),
                }
            }
            _ = flush.tick() => {
                if let Err(e) = recorder.flush() {
                    eprintln!("flush failed: {e}");
                    break;
                }
                print!("\rrecorded: points={points_written} imu={imu_written}");
                let _ = std::io::stdout().flush();
            }
        }
    }

    if let Err(e) = recorder.flush() {
        eprintln!("final flush failed: {e}");
    }
    let points_file = lidar_reader::recorder::POINTS_FILE;
    let imu_file = lidar_reader::recorder::IMU_FILE;
    println!("\nSaved {points_file} and {imu_file} to {out_dir}");
}
