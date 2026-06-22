# lidar_reader

A small, dependency-light Rust library and command-line toolkit for the
[Livox MID360](https://www.livoxtech.com/mid-360) LiDAR. It can discover
sensors on the local network, receive their point-cloud and IMU data
streams, record them to CSV, and (with an optional feature) render a live
3D view.

The project ships two binaries backed by one library:

- `lidar_reader` -- discover, stream, and record (always built).
- `lidar_viewer` -- live 3D point-cloud window (requires the `viewer` feature).

## Table of contents

- [Requirements](#requirements)
- [Network setup](#network-setup)
- [Building](#building)
- [The `lidar_reader` CLI](#the-lidar_reader-cli)
  - [discover](#discover)
  - [stream](#stream)
  - [record](#record)
- [The `lidar_viewer` binary](#the-lidar_viewer-binary)
  - [Controls](#controls)
  - [IMU stabilization](#imu-stabilization)
  - [Optional FOV clip](#optional-fov-clip)
- [Recorded CSV format](#recorded-csv-format)
- [Network ports](#network-ports)
- [Using the library](#using-the-library)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)

## Requirements

- Rust toolchain (edition 2024). Install via [rustup](https://rustup.rs/).
- A host machine on the same Layer-2 network as the MID360, with a fixed
  IPv4 address on the interface connected to the LiDAR.
- For the viewer: a desktop environment (the `minifb` crate opens a native
  window). On headless servers, use `discover` / `stream` / `record` instead.

## Network setup

The MID360 ships configured for the `192.168.1.x` range with the sensor at
`192.168.1.100` by default. To talk to it:

1. Assign your host a static IPv4 on the same subnet, e.g. `192.168.1.50`.
   This address is what every command below calls `host_ip`.
2. Make sure no other host on the link is using the [ports](#network-ports)
   the tool binds (56101, 56301, 56401 by default).
3. If you changed the LiDAR's IP, pass that address as `lidar_ip`.

`host_ip` must be a real address of a local interface -- never `0.0.0.0`
for `stream` or `record` (the LiDAR needs a concrete destination to send
packets to). `discover` accepts `0.0.0.0` to bind all interfaces.

## Building

From the `lidar_reader` directory:

```sh
# Library + discover/stream/record CLI only (default).
cargo build --release

# Also build the 3D viewer binary.
cargo build --release --features viewer
```

Run without installing via `cargo run --bin <name> -- <args>`, or copy the
binary from `target/release/`.

## The `lidar_reader` CLI

```
lidar_reader discover [host_ip]                   -- discover MID360 LiDARs
lidar_reader stream <host_ip> <lidar_ip>          -- receive point cloud / IMU
lidar_reader record <host_ip> <lidar_ip> <dir>    -- record point cloud / IMU to CSVs
```

All three subcommands share the same argument convention:

- `host_ip` -- IPv4 address of the local network interface connected to the
  LiDAR (e.g. `192.168.1.50`). For `discover` you may omit it to use
  `0.0.0.0`.
- `lidar_ip` -- IPv4 address of the LiDAR (e.g. `192.168.1.100`).
- `dir` -- target folder for the recorded CSVs (created if missing).

### discover

Broadcasts a discovery request and prints every MID360 that responds:

```sh
cargo run --bin lidar_reader -- discover 192.168.1.50
```

Output, one line per sensor:

```
Found device type=9 serial=MID360-xxxx cmd_addr=192.168.1.100:56100
```

Use this to find the LiDAR's `lidar_ip` (the IP in `cmd_addr`) and to
confirm the host can reach it. You can run `discover` with `0.0.0.0`:

```sh
cargo run --bin lidar_reader -- discover
```

### stream

Configures the LiDAR to push point-cloud and IMU packets to this host and
prints a live one-line summary of each. Nothing is written to disk.

```sh
cargo run --bin lidar_reader -- stream 192.168.1.50 192.168.1.100
```

You will see a rolling line such as:

```
point cloud: udp=12 ts=1234567 ns points=96
IMU gyro=(0.01,0.02,0.03) acc=(0.00,0.00,1.00)
```

Press `Ctrl-C` to stop. This is the quickest way to confirm the data path
is working before recording or viewing.

### record

Same streaming setup as `stream`, but writes every point and IMU sample to
two CSV files inside a target folder.

```sh
cargo run --bin lidar_reader -- record 192.168.1.50 192.168.1.100 ./capture_001
```

The folder (`./capture_001` above) is created if it does not exist. Two
files are written (overwriting any existing ones of the same name):

- `points.csv` -- one row per point.
- `imu.csv` -- one row per IMU sample.

While recording, a rolling counter shows progress:

```
recorded: points=123456 imu=987
```

The recorder buffers writes and flushes to disk once per second, plus a
final flush on `Ctrl-C`, so a clean stop preserves all data received up to
that point. See [Recorded CSV format](#recorded-csv-format) for the schema.

## The `lidar_viewer` binary

A live 3D point-cloud window. Build and run with the `viewer` feature:

```sh
cargo run --features viewer --bin lidar_viewer -- 192.168.1.50 192.168.1.100
```

Points accumulate over a short retention window and are color-coded by
height. The view is stabilized by the IMU (see below), so rotating the
sensor does not rotate the scene.

### Controls

- **Left-drag** -- orbit the camera.
- **Right-drag** -- pan the camera target.
- **Scroll** -- zoom (change distance).
- **Up / Down** -- increase / decrease the point retention window
  (how long old points stay on screen). Default 500 ms; larger values build
  a denser map of static scenes.
- **C** -- clear the point buffer.
- **F** -- toggle the optional [FOV clip](#optional-fov-clip).
- **Esc** -- quit.

The window title shows the current point count, retention age, FOV-clip
state, FPS, and camera distance.

### IMU stabilization

Each point is rotated by the LiDAR's estimated attitude (body -> gravity-
aligned world frame) using a Mahony complementary filter before display, so
turning the sensor keeps the world fixed. Only **orientation** is corrected:
an IMU cannot recover position, so walking the LiDAR sideways will still
translate the cloud. The filter is in `src/imu.rs` and only compiled with
the `viewer` feature.

### Optional FOV clip

By default the viewer keeps every accumulated point on screen, even ones
the LiDAR can no longer see, building a persistent map as you move it.

Pressing **F** turns on the FOV clip. When enabled, only points the LiDAR
could currently see are drawn:

- **Azimuth:** full 360 degrees about the Z axis (no limit).
- **Elevation:** `-7 degrees` to `+59 degrees` measured from the horizontal
  plane (the MID360's physical FOV).

The clip is computed in the LiDAR body frame using the **current** attitude,
so the visible cone sweeps correctly as the sensor rotates. Out-of-FOV
points stay in the buffer and reappear when the LiDAR turns back toward
them (or when you toggle the clip off with **F**). Nothing is discarded.

## Recorded CSV format

Both files begin with a header row. Timestamps are the LiDAR's 8-byte
nanosecond timestamp from the data-frame header; coordinates are in metres.

### points.csv

```
timestamp_ns,udp_cnt,frame_cnt,x_m,y_m,z_m,reflectivity,tag,detection_confidence,particle_confidence,adhesion_confidence
```

- `timestamp_ns` -- packet timestamp (ns).
- `udp_cnt`, `frame_cnt` -- packet / frame counters from the header.
- `x_m`, `y_m`, `z_m` -- point coordinates in metres (LiDAR frame).
- `reflectivity` -- 8-bit reflectivity.
- `tag` -- raw 8-bit tag byte.
- `detection_confidence`, `particle_confidence`, `adhesion_confidence` --
  decoded sub-fields of the tag (0 = high confidence).

Example row:

```
12345,7,3,1.000,2.000,-3.000,128,6,0,1,2
```

### imu.csv

```
timestamp_ns,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z
```

- `gyro_*` -- gyroscope rates (rad/s).
- `acc_*` -- accelerometer readings (g).

Example row:

```
12345,0.1,0.2,0.3,0.4,0.5,0.6
```

## Network ports

Default ports used by the tool (matching the MID360 protocol):

| Purpose              | LiDAR side | Host side |
|----------------------|-----------:|----------:|
| Discovery broadcast  | 56000      | -         |
| Command (control)    | 56100      | 56101     |
| Push info            | 56200      | 56201     |
| Point-cloud data     | 56300      | 56301     |
| IMU data             | 56400      | 56401     |
| Log push             | 56500      | 56501     |

`discover` uses the discovery port; `stream`/`record`/`lidar_viewer` use
the command, data, and IMU ports. Constants live in `src/protocol.rs`.

## Using the library

`lidar_reader` is also a library for embedding MID360 support into other
Rust programs. The core types are re-exported at the crate root:

```rust
use std::net::{Ipv4Addr, SocketAddr};
use std::time::Duration;

use lidar_reader::{
    CsvRecorder, DataStream, LivoxClient, DataType, LidarError,
    packet::DataPayload, protocol::{CMD_PORT, HOST_DATA_PORT, HOST_IMU_PORT},
};

# async fn run() -> Result<(), LidarError> {
let host_ip = Ipv4Addr::new(192, 168, 1, 50);
let lidar_ip = Ipv4Addr::new(192, 168, 1, 100);

let client = LivoxClient::with_default_cmd_port(host_ip).await?;
let stream = DataStream::with_default_ports(host_ip).await?;

let lidar_cmd = SocketAddr::from((lidar_ip, CMD_PORT));
let data_dst = SocketAddr::from((host_ip, HOST_DATA_PORT));
let imu_dst  = SocketAddr::from((host_ip, HOST_IMU_PORT));

client.start_streaming(
    lidar_cmd, data_dst, imu_dst,
    DataType::PointCloudCartesian32,
    Duration::from_secs(2),
).await?;

let mut recorder = CsvRecorder::open("./capture")?;
loop {
    let pkt = stream.next_point_cloud(Duration::from_secs(1)).await?;
    if let DataPayload::Points(pts) = &pkt.payload {
        recorder.write_points(&pkt.header, pts)?;
    }
}
# Ok(())
# }
```

Key exports:

- `LivoxClient` -- async UDP control client (discovery, parameter
  get/set, start/stop streaming).
- `DataStream` -- async receiver for point-cloud and IMU UDP streams.
- `DataPacket`, `DataFrameHeader`, `DataPayload` -- parsed data frames.
- `Point`, `Cartesian32Point`, `Cartesian16Point`, `SphericalPoint`,
  `ImuSample`, `Tag` -- decoded sample types.
- `CsvRecorder` -- writes `points.csv` / `imu.csv` into a folder.
- `protocol::*` -- enums and port constants (`DataType`, `ParameterKey`,
  `LidarState`, `ReturnCode`, ports, etc.).
- `imu::AttitudeEstimator`, `imu::OrientationHistory` (viewer feature only)
  -- Mahony filter and timestamped orientation buffer for stabilization.

Data type selection: `start_streaming` requests
`DataType::PointCloudCartesian32` by default. Other supported formats are
`PointCloudCartesian16` (10 mm resolution) and `PointCloudSpherical`.

## Testing

```sh
cargo test                       # library + CLI tests
cargo test --features viewer     # also runs the viewer FOV tests
```

Tests use mock UDP sockets and temp directories; no hardware is required.
The viewer suite covers the FOV-clip math (in-cone points kept, out-of-FOV
points dropped, inclusive boundaries, and attitude-dependent clipping).

## Troubleshooting

- **"failed to bind command socket to ..."** -- `host_ip` is not an address
  of a local interface. Check `ifconfig` / `ip addr` and pass the address of
  the interface wired to the LiDAR, not the LiDAR's own IP.
- **`stream`/`record` says 0.0.0.0 is not valid** -- these subcommands need
  a concrete host IP so the LiDAR has somewhere to send packets. Only
  `discover` accepts `0.0.0.0`.
- **No devices found / no packets arrive** -- confirm the host and LiDAR are
  on the same subnet, no firewall is blocking UDP ports
  [56000-56501](#network-ports), and the LiDAR is powered and spinning.
  `discover` first, then `stream` to verify the data path.
- **`lidar_viewer` fails to open a window** -- the viewer needs a desktop.
  On a headless machine, use `record` and plot the CSVs elsewhere.
- **Build fails with a missing `minifb`/`glam`** -- those are optional; the
  viewer requires `--features viewer`.
- **Setting `LIDAR_DEBUG=1`** while running `stream` or `record` makes the
  data parser print a hex dump and CRC diagnostics for any malformed packet,
  useful when debugging firmware quirks.
