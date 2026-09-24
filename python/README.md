# livox-mid360 (Python)

Python bindings for the `livox_mid360` Rust library. The extension wraps live
MID360 sessions, discovery, and packet parsing. It does not change the Rust
crate: Tokio still runs inside the library, and Rust callers keep the same API.

Requires Python 3.12 or newer. Wheels use the stable ABI (`abi3-py312`).

## Install from source

A Rust toolchain is required to build.

```sh
cd python
python3 -m venv .venv
source .venv/bin/activate
pip install maturin pytest
maturin develop
pytest
```

Optional features of the Rust crate (`cloud`, `encode`, `imu`) are not wrapped
here.

## Usage

I/O methods return asyncio awaitables. Packet parse/build stays synchronous.
A multi-thread Tokio runtime inside the extension drives the Rust futures.

```python
import asyncio
from livox_mid360 import LiveReader, LivoxClient

async def main():
    client = await LivoxClient.bind("0.0.0.0")
    devices = await client.discover(wait=1.0)
    for device in devices:
        print(device.serial_number, device.lidar_cmd_addr)

    reader = await LiveReader.connect("192.168.1.50", "192.168.1.100")
    while True:
        sample = await reader.recv()
        if sample.kind == "points":
            for point in sample.points:
                x, y, z = point.coords_m()
        elif sample.kind == "imu":
            gx, gy, gz = sample.imu.gyro()

asyncio.run(main())
```

`LiveReader.connect` rejects `0.0.0.0` for the same reason as Rust: the LiDAR
needs a concrete host address. `LivoxClient.bind` may use `0.0.0.0` for discovery.

`next_point_cloud(wait)` and `next_imu(wait)` take a timeout in seconds and
raise `LidarError` when nothing arrives (`NoResponse`).

Captured UDP payloads can be parsed without a sensor:

```python
from livox_mid360 import DataPacket

packet = DataPacket.parse(udp_payload)
```
