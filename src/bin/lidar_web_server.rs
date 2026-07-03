use std::net::{Ipv4Addr, SocketAddr};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use clap::Parser;
use futures_util::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast;
use tokio_tungstenite::tungstenite::Message;

use lidar_reader::client::LivoxClient;
use lidar_reader::packet::DataPacket;
use lidar_reader::points::Point;
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
    let (tx, _rx) = broadcast::channel::<Vec<u8>>(16);

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
    
    let mut client = LivoxClient::new(host_ip)?;
    client.connect(lidar_ip)?;
    let mut stream = client.start_stream()?;

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
        // This blocks, but the UDP receive logic in LIVOX-MID360-SimpleDriver is sync
        // Using it inside main loop of tokio is okay if we yield occasionally, 
        // but it's better to just do standard recv.
        // For simplicity and since stream is sync, we do it in a blocking task or block the main thread.
        // It's perfectly fine to block main thread and do websockets in spawned tasks
        if let Ok(data) = stream.recv(Duration::from_millis(100)) {
            match data {
                lidar_reader::client::DataStream::PointCloud(packet) => {
                    let pts = packet.payload.points();
                    
                    if let Some(rec) = &mut recorder {
                        rec.write_points(&packet.header, pts)?;
                    }
                    
                    if !is_recording {
                        // Gather points for live view
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
                lidar_reader::client::DataStream::Imu(header, imu) => {
                    if let Some(rec) = &mut recorder {
                        rec.write_imu(&header, &imu)?;
                    }
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

    while let Ok(msg) = rx.recv().await {
        if sender.send(Message::Binary(msg.into())).await.is_err() {
            break;
        }
    }
    
    println!("WebSocket connection closed");
}
