# livox_mid360

A dependency-light Rust library for the
[Livox MID360](https://www.livoxtech.com/mid-360) LiDAR. It discovers sensors
on the local network, speaks the MID360 UDP protocol, and parses point-cloud
and IMU packets.

This crate is a library. The CLI, CSV recording, and 3D viewer live in the
sibling package `lidar_apps`.

## Table of contents

- [Requirements](#requirements)
- [Network setup](#network-setup)
- [Building](#building)
- [Using the library](#using-the-library)
- [Network ports](#network-ports)
- [Python bindings](#python-bindings)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)

## Requirements

- Rust toolchain (edition 2024). Install via [rustup](https://rustup.rs/).
- A host machine on the same Layer-2 network as the MID360, with a fixed
  IPv4 address on the interface connected to the LiDAR.

## Network setup

The MID360 ships configured for the `192.168.1.x` range with the sensor at
`192.168.1.100` by default. To talk to it:

1. Assign your host a static IPv4 on the same subnet, e.g. `192.168.1.50`.
   This address is what the API calls `host_ip`.
2. Make sure no other host on the link is using the [ports](#network-ports)
   the library binds (56101, 56301, 56401 by default).
3. If you changed the LiDAR's IP, pass that address as `lidar_ip`.

`host_ip` must be a real address of a local interface. `LiveReader::connect`
rejects `0.0.0.0` because the LiDAR needs a concrete destination.
`LivoxClient::discover` accepts `0.0.0.0` to bind all interfaces.

## Building

From this directory:

```sh
cargo build --release
cargo build --release --features cloud,encode,imu
```

Optional features:

- `cloud` -- rolling point buffer (`livox_mid360::cloud::Cloud`).
- `encode` -- PCA octree tokenizer (`livox_mid360::encode`).
- `imu` -- Mahony attitude estimator (`livox_mid360::imu`).

`cloud` and `imu` pull in `glam`. The default build does not.

## Using the library

A typical live session is `LiveReader::connect`, which binds the host sockets,
configures the MID360, and yields parsed packets:

```rust
use std::net::Ipv4Addr;
use std::time::Duration;

use livox_mid360::{DataPayload, LidarError, LiveReader};

# async fn run() -> Result<(), LidarError> {
let host_ip = Ipv4Addr::new(192, 168, 1, 50);
let lidar_ip = Ipv4Addr::new(192, 168, 1, 100);

let reader = LiveReader::connect(host_ip, lidar_ip).await?;
loop {
    match reader.recv().await? {
        livox_mid360::Sample::Points { header, points } => {
            let _ = (header.timestamp, points.len());
        }
        livox_mid360::Sample::Imu { header, sample } => {
            let _ = (header.timestamp, sample.gyro_x);
        }
    }
}
# Ok(())
# }
```

`LiveReader::recv` waits for the next point-cloud or IMU packet, whichever
arrives first. `next_point_cloud` / `next_imu` wait on one socket with a
timeout (`LidarError::NoResponse` when nothing arrives).
`LiveReader::connect_with` takes a `LiveConfig` to change data type, ports,
or command timeout.

`LivoxClient` is the discovery and parameter API. `DataStream` is the raw
UDP receiver if you need to bind without starting sampling.

Key exports:

- `LiveReader`, `LiveConfig`, `Sample` -- live session.
- `LivoxClient`, `DiscoveredDevice` -- control and discovery.
- `DataStream` -- async receiver for point-cloud and IMU UDP streams.
- `DataPacket`, `DataFrameHeader`, `DataPayload` -- parsed data frames.
- `Point`, `Cartesian32Point`, `Cartesian16Point`, `SphericalPoint`,
  `ImuSample`, `Tag` -- decoded sample types.
- `protocol::*` -- enums and port constants.

`LiveReader::connect` requests `DataType::PointCloudCartesian32` and enables
IMU. Other supported formats are `PointCloudCartesian16` (10 mm resolution)
and `PointCloudSpherical`, via `LiveConfig`.

Coordinates from `Point::coords_m()` are metres in the LiDAR frame.

The APIs are async and use Tokio (`tokio::net::UdpSocket`). Call them from a
Tokio runtime.

## Python bindings

The `python/` directory is a separate PyO3 extension (`pip` name `livox-mid360`,
import `livox_mid360`). It depends on this crate and leaves the Rust API
unchanged. Python 3.12+ is required. See [python/README.md](python/README.md).

```python
from livox_mid360 import LiveReader

reader = await LiveReader.connect("192.168.1.50", "192.168.1.100")
sample = await reader.recv()
```

## Network ports

Default ports (matching the MID360 protocol):

| Purpose              | LiDAR side | Host side |
|----------------------|-----------:|----------:|
| Discovery broadcast  | 56000      | -         |
| Command (control)    | 56100      | 56101     |
| Push info            | 56200      | 56201     |
| Point-cloud data     | 56300      | 56301     |
| IMU data             | 56400      | 56401     |
| Log push             | 56500      | 56501     |

Constants live in `livox_mid360::protocol`.

## Testing

```sh
cargo test
cargo test --features cloud,encode,imu
```

Python bindings (from `python/`):

```sh
maturin develop
pytest
```

Tests use mock UDP sockets; no hardware is required.

## Troubleshooting

- **"failed to bind ... socket"** -- `host_ip` is not an address of a local
  interface. Check `ifconfig` / `ip addr` and pass the address of the
  interface wired to the LiDAR, not the LiDAR's own IP.
- **`LiveReader::connect` rejects 0.0.0.0** -- streaming needs a concrete
  host IP so the LiDAR has somewhere to send packets. Discovery may bind
  `0.0.0.0`.
- **No devices found / no packets arrive** -- confirm the host and LiDAR are
  on the same subnet, no firewall is blocking UDP ports
  [56000-56501](#network-ports), and the LiDAR is powered and spinning.
- **Setting `LIDAR_DEBUG=1`** makes the data parser print a hex dump and CRC
  diagnostics for any malformed packet.
