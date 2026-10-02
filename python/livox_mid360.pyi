# Copyright 2026 Juan David Guevara Arévalo
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

"""Python bindings for the livox_mid360 Livox MID360 library."""

from collections.abc import Buffer, Sequence
from typing import Final, Literal

type _DataTypeArg = (
    DataType
    | int
    | Literal[
        "imu",
        "cartesian32",
        "point_cloud_cartesian32",
        "cartesian16",
        "point_cloud_cartesian16",
        "spherical",
        "point_cloud_spherical",
    ]
)

class LidarError(Exception):
    """Error from a Livox command, stream, or packet."""

class DataType:
    """Point or IMU record layout. Members compare equal to their protocol byte."""

    Imu: Final[DataType]
    PointCloudCartesian32: Final[DataType]
    PointCloudCartesian16: Final[DataType]
    PointCloudSpherical: Final[DataType]

    def __int__(self) -> int: ...
    def __eq__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...

class TimestampType:
    """Clock source stored in a data-frame header.

    The unsynchronized value is the attribute ``None`` (``TimestampType.None``,
    equal to ``0``). That name is a Python keyword, so a stub cannot declare it.
    """

    Ptp: Final[TimestampType]
    Gps: Final[TimestampType]

    def __int__(self) -> int: ...
    def __eq__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...

class Tag:
    """Per-point confidence bits packed into one byte."""

    def __init__(self, raw: int) -> None: ...
    @property
    def raw(self) -> int: ...
    @property
    def detection_confidence(self) -> int: ...
    @property
    def particle_confidence(self) -> int: ...
    @property
    def adhesion_confidence(self) -> int: ...
    def is_high_confidence(self) -> bool: ...
    def __repr__(self) -> str: ...

class Point:
    """One return in Cartesian32, Cartesian16, or spherical form."""

    @staticmethod
    def cartesian32(
        x_mm: int,
        y_mm: int,
        z_mm: int,
        reflectivity: int,
        tag: Tag | None = None,
    ) -> Point: ...
    @staticmethod
    def cartesian16(
        x_10mm: int,
        y_10mm: int,
        z_10mm: int,
        reflectivity: int,
        tag: Tag | None = None,
    ) -> Point: ...
    @staticmethod
    def spherical(
        depth_mm: int,
        theta_0_01_deg: int,
        phi_0_01_deg: int,
        reflectivity: int,
        tag: Tag | None = None,
    ) -> Point: ...
    @property
    def kind(self) -> Literal["cartesian32", "cartesian16", "spherical"]: ...
    @property
    def reflectivity(self) -> int: ...
    @property
    def tag(self) -> Tag: ...
    def coords_m(self) -> tuple[float, float, float]: ...
    def __repr__(self) -> str: ...

class ImuSample:
    """One IMU reading."""

    def __init__(
        self,
        gyro_x: float,
        gyro_y: float,
        gyro_z: float,
        acc_x: float,
        acc_y: float,
        acc_z: float,
    ) -> None: ...
    @property
    def gyro_x(self) -> float: ...
    @property
    def gyro_y(self) -> float: ...
    @property
    def gyro_z(self) -> float: ...
    @property
    def acc_x(self) -> float: ...
    @property
    def acc_y(self) -> float: ...
    @property
    def acc_z(self) -> float: ...
    def gyro(self) -> tuple[float, float, float]: ...
    def acc(self) -> tuple[float, float, float]: ...
    def __repr__(self) -> str: ...

class DataFrameHeader:
    """36-byte Livox data-frame header."""

    def __init__(
        self,
        timestamp: int,
        data_type: DataType,
        *,
        version: int = 0,
        time_interval: int = 0,
        dot_num: int = 0,
        udp_cnt: int = 0,
        frame_cnt: int = 0,
        time_type: TimestampType | None = None,
    ) -> None: ...
    @property
    def version(self) -> int: ...
    @property
    def length(self) -> int: ...
    @property
    def time_interval(self) -> int: ...
    @property
    def dot_num(self) -> int: ...
    @property
    def udp_cnt(self) -> int: ...
    @property
    def frame_cnt(self) -> int: ...
    @property
    def data_type(self) -> DataType: ...
    @property
    def time_type(self) -> TimestampType: ...
    @property
    def timestamp(self) -> int: ...
    def __repr__(self) -> str: ...

class DataPacket:
    """One parsed or built Livox data datagram."""

    @staticmethod
    def parse(data: Buffer) -> DataPacket:
        """Parse a captured UDP payload."""
        ...

    @staticmethod
    def from_points(header: DataFrameHeader, points: Sequence[Point]) -> DataPacket: ...
    @staticmethod
    def from_imu(header: DataFrameHeader, sample: ImuSample) -> DataPacket: ...
    @property
    def header(self) -> DataFrameHeader: ...
    @property
    def kind(self) -> Literal["points", "imu"]: ...
    @property
    def points(self) -> list[Point] | None: ...
    @property
    def imu(self) -> ImuSample | None: ...
    def to_bytes(self) -> bytes: ...
    def __repr__(self) -> str: ...

class Sample:
    """Next point-cloud or IMU packet from a live reader.

    ``points`` is set when ``kind == "points"``. ``imu`` is set when ``kind == "imu"``.
    """

    @property
    def kind(self) -> Literal["points", "imu"]: ...
    @property
    def header(self) -> DataFrameHeader: ...
    @property
    def points(self) -> list[Point] | None: ...
    @property
    def imu(self) -> ImuSample | None: ...
    def __repr__(self) -> str: ...

class DiscoveredDevice:
    """A LiDAR that answered a discovery broadcast."""

    @property
    def dev_type(self) -> int: ...
    @property
    def serial_number(self) -> str: ...
    @property
    def lidar_cmd_addr(self) -> str: ...
    def __repr__(self) -> str: ...

class LiveReader:
    """Async live MID360 session. Methods return asyncio awaitables."""

    @staticmethod
    async def connect(host_ip: str, lidar_ip: str) -> LiveReader:
        """Bind default host ports and start Cartesian32 + IMU sampling."""
        ...

    @staticmethod
    async def connect_with(
        host_ip: str,
        lidar_ip: str,
        data_type: _DataTypeArg | None = None,
        command_timeout: float = 2.0,
        lidar_cmd_port: int | None = None,
        cmd_port: int | None = None,
        data_port: int | None = None,
        imu_port: int | None = None,
    ) -> LiveReader:
        """Bind, configure stream destinations, and enter sampling.

        ``data_type`` accepts a :class:`DataType`, the protocol byte, or a name
        such as ``"cartesian32"``. The default is Cartesian32.
        ``host_ip`` must be a concrete interface address.
        """
        ...

    async def recv(self) -> Sample:
        """Wait until the next point-cloud or IMU packet arrives."""
        ...

    async def next_point_cloud(self, wait: float = 1.0) -> DataPacket:
        """Wait for the next point-cloud packet.

        ``wait`` is a timeout in seconds. Raises :class:`LidarError` when
        nothing arrives.
        """
        ...

    async def next_imu(self, wait: float = 1.0) -> DataPacket:
        """Wait for the next IMU packet.

        ``wait`` is a timeout in seconds. Raises :class:`LidarError` when
        nothing arrives.
        """
        ...

    def __repr__(self) -> str: ...

class LivoxClient:
    """Async discovery and control client. Methods return asyncio awaitables."""

    @staticmethod
    async def bind(bind_ip: str, port: int | None = None) -> LivoxClient:
        """Bind the command socket. Default host command port is 56101."""
        ...

    async def discover(
        self,
        wait: float = 1.0,
        broadcast: str | None = None,
    ) -> list[DiscoveredDevice]:
        """Broadcast a discovery request and collect responding devices.

        ``broadcast`` is ``"ip:port"``. The default is ``255.255.255.255`` on
        the discovery port.
        """
        ...

    def __repr__(self) -> str: ...

def encode(cloud: Sequence[tuple[float, float, float]], rounds: int) -> list[float]:
    """Encode a point cloud into a PCA-octree token vector.

    ``cloud`` is a sequence of ``(x, y, z)`` positions in metres. ``rounds`` is
    1 through 5. The result has ``FLOATS_PER_SEGMENT * 8**rounds`` floats.
    """
    ...

MIN_ROUNDS: Final[int]
MAX_ROUNDS: Final[int]
FLOATS_PER_SEGMENT: Final[int]

DISCOVERY_PORT: Final[int]
CMD_PORT: Final[int]
PUSH_PORT: Final[int]
DATA_PORT: Final[int]
IMU_PORT: Final[int]
LOG_PORT: Final[int]
HOST_CMD_PORT: Final[int]
HOST_PUSH_PORT: Final[int]
HOST_DATA_PORT: Final[int]
HOST_IMU_PORT: Final[int]
HOST_LOG_PORT: Final[int]

__all__ = [
    "LidarError",
    "LiveReader",
    "LivoxClient",
    "DiscoveredDevice",
    "Sample",
    "DataPacket",
    "DataFrameHeader",
    "Point",
    "Tag",
    "ImuSample",
    "DataType",
    "TimestampType",
    "encode",
    "MIN_ROUNDS",
    "MAX_ROUNDS",
    "FLOATS_PER_SEGMENT",
    "DISCOVERY_PORT",
    "CMD_PORT",
    "PUSH_PORT",
    "DATA_PORT",
    "IMU_PORT",
    "LOG_PORT",
    "HOST_CMD_PORT",
    "HOST_PUSH_PORT",
    "HOST_DATA_PORT",
    "HOST_IMU_PORT",
    "HOST_LOG_PORT",
]
