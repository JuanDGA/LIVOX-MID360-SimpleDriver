use std::net::Ipv4Addr;
use std::time::Duration;

use clap::Parser;
use futures_util::{SinkExt, StreamExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast;
use tokio_tungstenite::tungstenite::Message;

use lidar_reader::client::LivoxClient;
use lidar_reader::recorder::CsvRecorder;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Host IP to bind for UDP listener.
    #[arg(short, long, default_value = "192.168.1.50")]
    host_ip: String,

    /// Lidar IP.
    #[arg(short, long, default_value = "192.168.1.104")]
    lidar_ip: String,

    /// Action: live or record
    #[arg(short, long, default_value = "live")]
    action: String,

    /// Directory to output the recording
    #[arg(short, long, default_value = "recordings/temp")]
    dir: String,

    /// Maximum age of points in the live view buffer (ms)
    #[arg(short, long, default_value_t = 500.0)]
    max_age_ms: f32,
    
    /// Websocket port
    #[arg(short, long, default_value_t = 8080)]
    ws_port: u16,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    
    let is_recording = args.action == "record";
    
    let host_ip: Ipv4Addr = args.host_ip.parse()?;
    let lidar_ip: Ipv4Addr = args.lidar_ip.parse()?;
    
    // Channel for broadcasting coordinate bounds to ws clients
    // Increased bounds from 16 to 128 to buffer more frames for slower wifi clients (smartphones)
    let (tx, _rx) = broadcast::channel::<Vec<u8>>(128);

    let tx_clone = tx.clone();
    
    // Start websocket server in background
    let ws_port = args.ws_port;
    tokio::spawn(async move {
        let addr = format!("0.0.0.0:{}", ws_port);
        let listener = TcpListener::bind(&addr).await.expect("Failed to bind");
        println!("WebSocket server listening on {}", addr);

        while let Ok((stream, _)) = listener.accept().await {
            let tx = tx_clone.clone();
            tokio::spawn(handle_connection(stream, tx));
        }
    });

    println!("Starting Lidar {} mode", args.action);
    
    let command_client = match LivoxClient::with_default_cmd_port(host_ip).await {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Failed to bind command socket to {}: {}", host_ip, e);
            return Ok(());
        }
    };
    
    let stream = match lidar_reader::client::DataStream::with_default_ports(host_ip).await {
        Ok(s) => s,
        Err(e) => {
            eprintln!("Failed to bind data/imu sockets to {}: {}", host_ip, e);
            return Ok(());
        }
    };

    let lidar_cmd_addr = std::net::SocketAddr::from((lidar_ip, lidar_reader::protocol::CMD_PORT));
    let data_dst = std::net::SocketAddr::from((host_ip, lidar_reader::protocol::HOST_DATA_PORT));
    let imu_dst = std::net::SocketAddr::from((host_ip, lidar_reader::protocol::HOST_IMU_PORT));

    if let Err(e) = command_client
        .start_streaming(
            lidar_cmd_addr,
            data_dst,
            imu_dst,
            lidar_reader::protocol::DataType::PointCloudCartesian32,
            Duration::from_secs(2),
        )
        .await
    {
        eprintln!("Failed to start streaming: {}", e);
        return Ok(());
    }

    let mut recorder = if is_recording {
        println!("Recording to directory: {}", args.dir);
        Some(CsvRecorder::open(&args.dir)?)
    } else {
        None
    };

    println!("Listening for packets...");
    
    // In live view, we buffer float 32 points
    let mut point_buffer: Vec<f32> = Vec::with_capacity(100_000);
    // Rough target to send a frame every ~30ms (33fps)
    let buffer_threshold = 3000;
        
    loop {
        tokio::select! {
             result = stream.next_point_cloud(Duration::from_secs(1)) => {
                 match result {
                     Ok(packet) => {
                         if let lidar_reader::packet::DataPayload::Points(pts) = &packet.payload {
                             if let Some(rec) = &mut recorder {
                                 rec.write_points(&packet.header, pts)?;
                             }
                             
                             // Gather points for live view (even when recording)
                             for p in pts {
                                 let (x, y, z) = p.coords_m();
                                 point_buffer.push(x);
                                 point_buffer.push(y);
                                 point_buffer.push(z);
                             }
                             
                             if point_buffer.len() >= buffer_threshold * 3 {
                                 let bytes: &[u8] = bytemuck::cast_slice(&point_buffer);
                                 let _ = tx.send(bytes.to_vec());
                                 point_buffer.clear();
                             }
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
                              if let Some(rec) = &mut recorder {
                                  rec.write_imu(&packet.header, &imu)?;
                              }
                         }
                     }
                     Err(lidar_reader::LidarError::NoResponse { .. }) => {}
                     Err(e) => eprintln!("imu error: {e}"),
                 }
             }
        }
    }
}

async fn handle_connection(stream: TcpStream, tx: broadcast::Sender<Vec<u8>>) {
    let ws_stream = tokio_tungstenite::accept_async(stream)
        .await
        .expect("Error during the websocket handshake occurred");

    let (mut sender, mut _receiver) = ws_stream.split();
    let mut rx = tx.subscribe();

    println!("New WebSocket connection established");

    loop {
        match rx.recv().await {
            Ok(msg) => {
                if sender.send(Message::Binary(msg.into())).await.is_err() {
                    break;
                }
            }
            Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {
                // Client is slow, dropped some frames. Continue without disconnecting.
                continue;
            }
            Err(_) => {
                // Channel closed
                break;
            }
        }
    }
    
    println!("WebSocket connection closed");
}
