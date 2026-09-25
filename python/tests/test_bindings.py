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

import pytest

import livox_mid360 as mid


def test_port_constants():
    assert mid.DISCOVERY_PORT == 56000
    assert mid.CMD_PORT == 56100
    assert mid.DATA_PORT == 56300
    assert mid.IMU_PORT == 56400
    assert mid.HOST_CMD_PORT == 56101
    assert mid.HOST_DATA_PORT == 56301
    assert mid.HOST_IMU_PORT == 56401


def test_tag():
    tag = mid.Tag(0)
    assert tag.raw == 0
    assert tag.detection_confidence == 0
    assert tag.is_high_confidence()

    mixed = mid.Tag(0b01_10_11)
    assert mixed.adhesion_confidence == 0b11
    assert mixed.particle_confidence == 0b10
    assert mixed.detection_confidence == 0b01
    assert not mixed.is_high_confidence()


def test_cartesian32_coords_m():
    point = mid.Point.cartesian32(1234, -567, 89, 200)
    assert point.kind == "cartesian32"
    assert point.reflectivity == 200
    x, y, z = point.coords_m()
    assert x == pytest.approx(1.234)
    assert y == pytest.approx(-0.567)
    assert z == pytest.approx(0.089)


def test_imu_sample():
    sample = mid.ImuSample(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    assert sample.gyro() == pytest.approx((0.1, 0.2, 0.3))
    assert sample.acc() == pytest.approx((0.4, 0.5, 0.6))


def test_packet_roundtrip_cartesian32():
    point = mid.Point.cartesian32(1234, -567, 89, 200, mid.Tag(0))
    header = mid.DataFrameHeader(
        2_000_000,
        mid.DataType.PointCloudCartesian32,
        time_interval=50,
        udp_cnt=7,
        frame_cnt=3,
        time_type=mid.TimestampType.Ptp,
    )
    packet = mid.DataPacket.from_points(header, [point])
    parsed = mid.DataPacket.parse(bytes(packet.to_bytes()))
    assert parsed.kind == "points"
    assert parsed.header.timestamp == 2_000_000
    assert parsed.header.udp_cnt == 7
    assert parsed.header.frame_cnt == 3
    assert parsed.points is not None
    assert len(parsed.points) == 1
    assert parsed.points[0].coords_m() == pytest.approx(point.coords_m())
    assert parsed.points[0].reflectivity == 200


def test_packet_roundtrip_imu():
    imu = mid.ImuSample(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    header = mid.DataFrameHeader(1_000_000, mid.DataType.Imu, time_interval=100)
    packet = mid.DataPacket.from_imu(header, imu)
    parsed = mid.DataPacket.parse(bytes(packet.to_bytes()))
    assert parsed.kind == "imu"
    assert parsed.header.timestamp == 1_000_000
    assert parsed.imu is not None
    assert parsed.imu.gyro_x == pytest.approx(0.1)
    assert parsed.imu.acc_z == pytest.approx(0.6)


async def test_connect_rejects_unspecified_host():
    with pytest.raises(mid.LidarError, match="host_ip must be a specific local interface"):
        await mid.LiveReader.connect("0.0.0.0", "127.0.0.1")


def test_connect_rejects_invalid_ip():
    with pytest.raises(ValueError, match="invalid host_ip"):
        mid.LiveReader.connect("not-an-ip", "127.0.0.1")


def test_encode_vector_length():
    cloud = [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (2.0, 0.0, 1.0)]
    for rounds in range(mid.MIN_ROUNDS, mid.MAX_ROUNDS + 1):
        tokens = mid.encode(cloud, rounds)
        assert len(tokens) == mid.FLOATS_PER_SEGMENT * 8**rounds


def test_encode_from_point_coords():
    point = mid.Point.cartesian32(1000, 200, 100, 50)
    vector = mid.encode([point.coords_m()], 1)
    assert len(vector) == mid.FLOATS_PER_SEGMENT * 8


def test_encode_rejects_empty_cloud():
    with pytest.raises(ValueError, match="empty point cloud"):
        mid.encode([], 1)


def test_encode_rejects_invalid_rounds():
    with pytest.raises(ValueError, match="rounds"):
        mid.encode([(0.0, 0.0, 0.0)], 0)
