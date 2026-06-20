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
        _ => print_usage(),
    }
}

fn print_usage() {
    eprintln!("Usage:");
    eprintln!("  lidar_reader discover [host_ip]          -- discover MID360 LiDARs");
    eprintln!("  lidar_reader stream <host_ip> <lidar_ip> -- receive point cloud / IMU");
    eprintln!();
    eprintln!("  host_ip:  IPv4 address of the local network interface connected to the LiDAR");
    eprintln!("            (e.g. 192.168.1.50). For discover you may omit this and use 0.0.0.0.");
    eprintln!("  lidar_ip: IPv4 address of the LiDAR (e.g. 192.168.1.100).");
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

async fn run_stream(host_ip: Ipv4Addr, lidar_ip: Ipv4Addr) {
    if host_ip.is_unspecified() {
        eprintln!(
            "stream requires a specific host_ip (0.0.0.0 is not valid here); use the IP of the interface connected to the LiDAR"
        );
        return;
    }

    let client = match LivoxClient::with_default_cmd_port(host_ip).await {
        Ok(c) => c,
        Err(e) => {
            eprintln!("{}: {e}", bind_error_context("command", host_ip));
            return;
        }
    };
    let stream = match DataStream::with_default_ports(host_ip).await {
        Ok(s) => s,
        Err(e) => {
            eprintln!("{}: {e}", bind_error_context("data/imu", host_ip));
            return;
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
        return;
    }

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
                        }
                    }
                    Err(lidar_reader::LidarError::NoResponse { .. }) => {}
                    Err(e) => eprintln!("imu error: {e}"),
                }
            }
        }
    }
}
