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

use livox_rs::data::packet::DATA_HEADER_SIZE;
use livox_rs::{
    Cartesian16Point, Cartesian32Point, DataFrameHeader, DataPacket, DataPayload, DataType,
    ImuSample, Point, Sample, SphericalPoint, Tag, TimestampType,
};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::error::map_err;

#[pyclass(eq, eq_int, frozen, name = "DataType")]
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PyDataType {
    Imu = 0,
    PointCloudCartesian32 = 1,
    PointCloudCartesian16 = 2,
    PointCloudSpherical = 3,
}

impl From<PyDataType> for DataType {
    fn from(value: PyDataType) -> Self {
        match value {
            PyDataType::Imu => DataType::Imu,
            PyDataType::PointCloudCartesian32 => DataType::PointCloudCartesian32,
            PyDataType::PointCloudCartesian16 => DataType::PointCloudCartesian16,
            PyDataType::PointCloudSpherical => DataType::PointCloudSpherical,
        }
    }
}

impl From<DataType> for PyDataType {
    fn from(value: DataType) -> Self {
        match value {
            DataType::Imu => PyDataType::Imu,
            DataType::PointCloudCartesian32 => PyDataType::PointCloudCartesian32,
            DataType::PointCloudCartesian16 => PyDataType::PointCloudCartesian16,
            DataType::PointCloudSpherical => PyDataType::PointCloudSpherical,
        }
    }
}

#[pyclass(eq, eq_int, frozen, name = "TimestampType")]
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PyTimestampType {
    None = 0,
    Ptp = 1,
    Gps = 2,
}

impl From<PyTimestampType> for TimestampType {
    fn from(value: PyTimestampType) -> Self {
        match value {
            PyTimestampType::None => TimestampType::None,
            PyTimestampType::Ptp => TimestampType::Ptp,
            PyTimestampType::Gps => TimestampType::Gps,
        }
    }
}

impl From<TimestampType> for PyTimestampType {
    fn from(value: TimestampType) -> Self {
        match value {
            TimestampType::None => PyTimestampType::None,
            TimestampType::Ptp => PyTimestampType::Ptp,
            TimestampType::Gps => PyTimestampType::Gps,
        }
    }
}

#[pyclass(frozen, name = "Tag")]
#[derive(Clone, Copy)]
pub struct PyTag {
    inner: Tag,
}

#[pymethods]
impl PyTag {
    #[new]
    fn new(raw: u8) -> Self {
        Self { inner: Tag(raw) }
    }

    #[getter]
    fn raw(&self) -> u8 {
        self.inner.raw()
    }

    #[getter]
    fn detection_confidence(&self) -> u8 {
        self.inner.detection_confidence()
    }

    #[getter]
    fn particle_confidence(&self) -> u8 {
        self.inner.particle_confidence()
    }

    #[getter]
    fn adhesion_confidence(&self) -> u8 {
        self.inner.adhesion_confidence()
    }

    fn is_high_confidence(&self) -> bool {
        self.inner.is_high_confidence()
    }

    fn __repr__(&self) -> String {
        format!("Tag({})", self.inner.raw())
    }
}

impl From<Tag> for PyTag {
    fn from(inner: Tag) -> Self {
        Self { inner }
    }
}

#[pyclass(frozen, name = "Point")]
#[derive(Clone, Copy)]
pub struct PyPoint {
    inner: Point,
}

#[pymethods]
impl PyPoint {
    #[staticmethod]
    #[pyo3(signature = (x_mm, y_mm, z_mm, reflectivity, tag=None))]
    fn cartesian32(x_mm: i32, y_mm: i32, z_mm: i32, reflectivity: u8, tag: Option<&PyTag>) -> Self {
        Self {
            inner: Point::Cartesian32(Cartesian32Point {
                x_mm,
                y_mm,
                z_mm,
                reflectivity,
                tag: tag.map(|t| t.inner).unwrap_or(Tag(0)),
            }),
        }
    }

    #[staticmethod]
    #[pyo3(signature = (x_10mm, y_10mm, z_10mm, reflectivity, tag=None))]
    fn cartesian16(
        x_10mm: i16,
        y_10mm: i16,
        z_10mm: i16,
        reflectivity: u8,
        tag: Option<&PyTag>,
    ) -> Self {
        Self {
            inner: Point::Cartesian16(Cartesian16Point {
                x_10mm,
                y_10mm,
                z_10mm,
                reflectivity,
                tag: tag.map(|t| t.inner).unwrap_or(Tag(0)),
            }),
        }
    }

    #[staticmethod]
    #[pyo3(signature = (depth_mm, theta_0_01_deg, phi_0_01_deg, reflectivity, tag=None))]
    fn spherical(
        depth_mm: u32,
        theta_0_01_deg: u16,
        phi_0_01_deg: u16,
        reflectivity: u8,
        tag: Option<&PyTag>,
    ) -> Self {
        Self {
            inner: Point::Spherical(SphericalPoint {
                depth_mm,
                theta_0_01_deg,
                phi_0_01_deg,
                reflectivity,
                tag: tag.map(|t| t.inner).unwrap_or(Tag(0)),
            }),
        }
    }

    #[getter]
    fn kind(&self) -> &'static str {
        match self.inner {
            Point::Cartesian32(_) => "cartesian32",
            Point::Cartesian16(_) => "cartesian16",
            Point::Spherical(_) => "spherical",
        }
    }

    #[getter]
    fn reflectivity(&self) -> u8 {
        self.inner.reflectivity()
    }

    #[getter]
    fn tag(&self) -> PyTag {
        self.inner.tag().into()
    }

    fn coords_m(&self) -> (f32, f32, f32) {
        self.inner.coords_m()
    }

    fn __repr__(&self) -> String {
        let (x, y, z) = self.inner.coords_m();
        format!(
            "Point(kind='{}', coords_m=({x}, {y}, {z}), reflectivity={})",
            self.kind(),
            self.inner.reflectivity()
        )
    }
}

impl From<Point> for PyPoint {
    fn from(inner: Point) -> Self {
        Self { inner }
    }
}

#[pyclass(frozen, name = "ImuSample")]
#[derive(Clone, Copy)]
pub struct PyImuSample {
    inner: ImuSample,
}

#[pymethods]
impl PyImuSample {
    #[new]
    fn new(gyro_x: f32, gyro_y: f32, gyro_z: f32, acc_x: f32, acc_y: f32, acc_z: f32) -> Self {
        Self {
            inner: ImuSample {
                gyro_x,
                gyro_y,
                gyro_z,
                acc_x,
                acc_y,
                acc_z,
            },
        }
    }

    #[getter]
    fn gyro_x(&self) -> f32 {
        self.inner.gyro_x
    }
    #[getter]
    fn gyro_y(&self) -> f32 {
        self.inner.gyro_y
    }
    #[getter]
    fn gyro_z(&self) -> f32 {
        self.inner.gyro_z
    }
    #[getter]
    fn acc_x(&self) -> f32 {
        self.inner.acc_x
    }
    #[getter]
    fn acc_y(&self) -> f32 {
        self.inner.acc_y
    }
    #[getter]
    fn acc_z(&self) -> f32 {
        self.inner.acc_z
    }

    fn gyro(&self) -> (f32, f32, f32) {
        (self.inner.gyro_x, self.inner.gyro_y, self.inner.gyro_z)
    }

    fn acc(&self) -> (f32, f32, f32) {
        (self.inner.acc_x, self.inner.acc_y, self.inner.acc_z)
    }

    fn __repr__(&self) -> String {
        format!(
            "ImuSample(gyro=({}, {}, {}), acc=({}, {}, {}))",
            self.inner.gyro_x,
            self.inner.gyro_y,
            self.inner.gyro_z,
            self.inner.acc_x,
            self.inner.acc_y,
            self.inner.acc_z
        )
    }
}

impl From<ImuSample> for PyImuSample {
    fn from(inner: ImuSample) -> Self {
        Self { inner }
    }
}

#[pyclass(frozen, name = "DataFrameHeader")]
#[derive(Clone, Copy)]
pub struct PyDataFrameHeader {
    inner: DataFrameHeader,
}

#[pymethods]
impl PyDataFrameHeader {
    #[new]
    #[pyo3(signature = (
        timestamp,
        data_type,
        *,
        version=0,
        time_interval=0,
        dot_num=0,
        udp_cnt=0,
        frame_cnt=0,
        time_type=None
    ))]
    fn new(
        timestamp: u64,
        data_type: PyDataType,
        version: u8,
        time_interval: u16,
        dot_num: u16,
        udp_cnt: u16,
        frame_cnt: u8,
        time_type: Option<PyTimestampType>,
    ) -> Self {
        Self {
            inner: DataFrameHeader {
                version,
                length: 0,
                time_interval,
                dot_num,
                udp_cnt,
                frame_cnt,
                data_type: data_type.into(),
                time_type: time_type.map(Into::into).unwrap_or(TimestampType::None),
                timestamp,
            },
        }
    }

    #[getter]
    fn version(&self) -> u8 {
        self.inner.version
    }
    #[getter]
    fn length(&self) -> u16 {
        self.inner.length
    }
    #[getter]
    fn time_interval(&self) -> u16 {
        self.inner.time_interval
    }
    #[getter]
    fn dot_num(&self) -> u16 {
        self.inner.dot_num
    }
    #[getter]
    fn udp_cnt(&self) -> u16 {
        self.inner.udp_cnt
    }
    #[getter]
    fn frame_cnt(&self) -> u8 {
        self.inner.frame_cnt
    }
    #[getter]
    fn data_type(&self) -> PyDataType {
        self.inner.data_type.into()
    }
    #[getter]
    fn time_type(&self) -> PyTimestampType {
        self.inner.time_type.into()
    }
    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp
    }

    fn __repr__(&self) -> String {
        format!(
            "DataFrameHeader(timestamp={}, data_type={:?}, dot_num={})",
            self.inner.timestamp, self.inner.data_type as u8, self.inner.dot_num
        )
    }
}

impl From<DataFrameHeader> for PyDataFrameHeader {
    fn from(inner: DataFrameHeader) -> Self {
        Self { inner }
    }
}

#[pyclass(frozen, name = "DataPacket")]
#[derive(Clone)]
pub struct PyDataPacket {
    inner: DataPacket,
}

#[pymethods]
impl PyDataPacket {
    #[staticmethod]
    fn parse(data: &[u8]) -> PyResult<Self> {
        DataPacket::parse(data)
            .map(|inner| Self { inner })
            .map_err(map_err)
    }

    #[staticmethod]
    fn from_points(header: &PyDataFrameHeader, points: Vec<PyPoint>) -> Self {
        let mut header = header.inner;
        header.dot_num = points.len() as u16;
        Self {
            inner: DataPacket {
                header,
                payload: DataPayload::Points(points.iter().map(|p| p.inner).collect()),
            },
        }
    }

    #[staticmethod]
    fn from_imu(header: &PyDataFrameHeader, sample: &PyImuSample) -> Self {
        let mut header = header.inner;
        header.dot_num = 1;
        header.data_type = DataType::Imu;
        Self {
            inner: DataPacket {
                header,
                payload: DataPayload::Imu(sample.inner),
            },
        }
    }

    #[getter]
    fn header(&self) -> PyDataFrameHeader {
        self.inner.header.into()
    }

    #[getter]
    fn kind(&self) -> &'static str {
        match self.inner.payload {
            DataPayload::Points(_) => "points",
            DataPayload::Imu(_) => "imu",
        }
    }

    #[getter]
    fn points(&self) -> Option<Vec<PyPoint>> {
        match &self.inner.payload {
            DataPayload::Points(points) => Some(points.iter().copied().map(PyPoint::from).collect()),
            DataPayload::Imu(_) => None,
        }
    }

    #[getter]
    fn imu(&self) -> Option<PyImuSample> {
        match self.inner.payload {
            DataPayload::Imu(sample) => Some(sample.into()),
            DataPayload::Points(_) => None,
        }
    }

    fn to_bytes(&self) -> PyResult<Vec<u8>> {
        let payload_len = match &self.inner.payload {
            DataPayload::Imu(_) => ImuSample::SIZE,
            DataPayload::Points(points) => self.inner.header.data_type.point_size() * points.len(),
        };
        let mut buf = vec![0u8; DATA_HEADER_SIZE + payload_len];
        DataPacket::build(&self.inner.header, &self.inner.payload, &mut buf).map_err(map_err)?;
        Ok(buf)
    }

    fn __repr__(&self) -> String {
        format!(
            "DataPacket(kind='{}', timestamp={})",
            self.kind(),
            self.inner.header.timestamp
        )
    }
}

impl From<DataPacket> for PyDataPacket {
    fn from(inner: DataPacket) -> Self {
        Self { inner }
    }
}

#[pyclass(frozen, name = "Sample")]
#[derive(Clone)]
pub struct PySample {
    inner: Sample,
}

#[pymethods]
impl PySample {
    #[getter]
    fn kind(&self) -> &'static str {
        match self.inner {
            Sample::Points { .. } => "points",
            Sample::Imu { .. } => "imu",
        }
    }

    #[getter]
    fn header(&self) -> PyDataFrameHeader {
        match &self.inner {
            Sample::Points { header, .. } | Sample::Imu { header, .. } => (*header).into(),
        }
    }

    #[getter]
    fn points(&self) -> Option<Vec<PyPoint>> {
        match &self.inner {
            Sample::Points { points, .. } => Some(points.iter().copied().map(PyPoint::from).collect()),
            Sample::Imu { .. } => None,
        }
    }

    #[getter]
    fn imu(&self) -> Option<PyImuSample> {
        match self.inner {
            Sample::Imu { sample, .. } => Some(sample.into()),
            Sample::Points { .. } => None,
        }
    }

    fn __repr__(&self) -> String {
        format!(
            "Sample(kind='{}', timestamp={})",
            self.kind(),
            self.header().timestamp()
        )
    }
}

impl From<Sample> for PySample {
    fn from(inner: Sample) -> Self {
        Self { inner }
    }
}

pub fn parse_data_type(value: Option<&Bound<'_, PyAny>>) -> PyResult<DataType> {
    let Some(value) = value else {
        return Ok(DataType::PointCloudCartesian32);
    };
    if let Ok(dt) = value.extract::<PyDataType>() {
        return Ok(dt.into());
    }
    if let Ok(raw) = value.extract::<u8>() {
        return DataType::try_from(raw).map_err(map_err);
    }
    if let Ok(name) = value.extract::<&str>() {
        return match name {
            "imu" => Ok(DataType::Imu),
            "cartesian32" | "point_cloud_cartesian32" => Ok(DataType::PointCloudCartesian32),
            "cartesian16" | "point_cloud_cartesian16" => Ok(DataType::PointCloudCartesian16),
            "spherical" | "point_cloud_spherical" => Ok(DataType::PointCloudSpherical),
            other => Err(PyValueError::new_err(format!("unknown data_type: {other}"))),
        };
    }
    Err(PyValueError::new_err(
        "data_type must be DataType, int, or str",
    ))
}
