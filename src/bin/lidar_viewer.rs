//! 3D point-cloud viewer for the Livox MID360.
//!
//! Build/run with the `viewer` feature:
//!   cargo run --features viewer --bin lidar_viewer -- <host_ip> <lidar_ip>
//!
//! Controls:
//!   drag (left)   - orbit
//!   drag (right)  - pan
//!   scroll        - zoom
//!   Up / Down     - increase / decrease point retention window
//!   C             - clear the point buffer
//!   Esc           - quit
//!
//! Points are expired by LiDAR timestamp age, so moving objects stop leaving
//! trails once they leave a region. The retention window (default 500 ms) is
//! adjustable at runtime; larger windows build a denser map of static scenes.

use std::net::{Ipv4Addr, SocketAddr};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use glam::{Mat4, Vec3, Vec4};
use minifb::{Key, MouseButton, MouseMode, Window, WindowOptions};

use lidar_reader::client::{DataStream, LivoxClient};
use lidar_reader::packet::{DataPacket, DataPayload};
use lidar_reader::protocol::DataType;

const WIDTH: usize = 1024;
const HEIGHT: usize = 768;
const MAX_POINTS: usize = 600_000;
const DEFAULT_MAX_AGE_MS: f32 = 500.0;
const AGE_STEP_MS: f32 = 100.0;
const AGE_MIN_MS: f32 = 50.0;
const AGE_MAX_MS: f32 = 5000.0;

/// Accumulated point cloud with the latest LiDAR timestamp used for expiry.
struct Cloud {
    /// (xyz metres, packet timestamp ns)
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

    /// Add a batch of points from one packet and drop those older than the
    /// retention window. `max_age_ns` is the maximum allowed age in ns.
    fn add(&mut self, pts: &[lidar_reader::points::Point], ts: u64, max_age_ns: u64) {
        if ts > self.latest_ts {
            self.latest_ts = ts;
        }
        for p in pts {
            let (x, y, z) = p.coords_m();
            self.points.push(([x, y, z], ts));
        }
        self.expire(max_age_ns);
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
    let host_ip = match args.get(1) {
        Some(s) => s.parse().expect("invalid host IPv4 address"),
        None => {
            eprintln!("usage: lidar_viewer <host_ip> <lidar_ip>");
            return;
        }
    };
    let lidar_ip: Ipv4Addr = match args.get(2) {
        Some(s) => s.parse().expect("invalid lidar IPv4 address"),
        None => {
            eprintln!("usage: lidar_viewer <host_ip> <lidar_ip>");
            return;
        }
    };

    let cloud: Arc<Mutex<Cloud>> = Arc::new(Mutex::new(Cloud::new()));
    let max_age_ms = Arc::new(AtomicU32::new(DEFAULT_MAX_AGE_MS.to_bits()));

    spawn_data_thread(host_ip, lidar_ip, cloud.clone(), max_age_ms.clone());

    run_window(cloud, max_age_ms);
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

            loop {
                match stream.next_point_cloud(Duration::from_millis(500)).await {
                    Ok(DataPacket {
                        header,
                        payload: DataPayload::Points(pts),
                    }) => {
                        let max_age_ns = (f32::from_bits(max_age_ms.load(Ordering::Relaxed))
                            * 1_000_000.0) as u64;
                        if let Ok(mut c) = cloud.lock() {
                            c.add(&pts, header.timestamp, max_age_ns);
                        }
                    }
                    Ok(_) => {}
                    Err(lidar_reader::LidarError::NoResponse { .. }) => {}
                    Err(e) => eprintln!("data error: {e}"),
                }
            }
        });
    });
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

fn run_window(cloud: Arc<Mutex<Cloud>>, max_age_ms: Arc<AtomicU32>) {
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

    while window.is_open() && !window.is_key_down(Key::Escape) {
        handle_input(&window, &mut cam, &mut prev_mouse, &max_age_ms, &cloud);

        let view = Mat4::look_at_rh(cam.eye(), Vec3::from(cam.target), Vec3::Y);
        let proj = Mat4::perspective_rh(1.0, WIDTH as f32 / HEIGHT as f32, 0.05, 1000.0);

        buffer.fill(0xFF0A0A0F); // dark background
        zbuffer.fill(f32::NEG_INFINITY);

        draw_axes(&mut buffer, &mut zbuffer, view, proj);

        let count = render_points(&cloud, &mut buffer, &mut zbuffer, view, proj);

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
        window.set_title(&format!(
            "Livox MID360 | points: {count} | age: {age:.0} ms | fps: {fps:.0} | dist: {:.1} m",
            cam.distance
        ));
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
    let snapshot: Vec<[f32; 3]> = {
        let c = match cloud.lock() {
            Ok(c) => c,
            Err(_) => return 0,
        };
        c.points.iter().map(|(p, _)| *p).collect()
    };
    let mut drawn = 0;
    for p in &snapshot {
        if let Some((sx, sy, vz)) = project(*p, view, proj) {
            put_point(buffer, zbuffer, sx, sy, vz, height_color(p[2]));
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
    if window.is_key_pressed(Key::C, minifb::KeyRepeat::No)
        && let Ok(mut c) = cloud.lock()
    {
        c.clear();
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
