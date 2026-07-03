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

//! CSV recording of LiDAR point-cloud and IMU data streams.
//!
//! [`CsvRecorder::open`] creates a target folder (if missing) and two CSV
//! files inside it: `points.csv` and `imu.csv`. Each packet received from the
//! data stream is appended as one row per sample. Files are buffered and
//! flushed periodically by the caller and on drop, so a Ctrl-C that is caught
//! by the application still preserves all buffered rows.

use std::fs::{File, create_dir_all};
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::error::Result;
use crate::packet::DataFrameHeader;
use crate::points::{ImuSample, Point};

pub const POINTS_FILE: &str = "points.csv";
pub const IMU_FILE: &str = "imu.csv";

const POINTS_HEADER: &str =
    "timestamp_ns,udp_cnt,frame_cnt,x_m,y_m,z_m,reflectivity,tag,detection_confidence,particle_confidence,adhesion_confidence";
const IMU_HEADER: &str = "timestamp_ns,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z";

/// Writes incoming point-cloud and IMU packets to two CSV files in a folder.
pub struct CsvRecorder {
    points: BufWriter<File>,
    imu: BufWriter<File>,
}

impl CsvRecorder {
    /// Create `dir` (if missing) and open `points.csv` / `imu.csv` inside it,
    /// writing the CSV header rows. Existing files are overwritten.
    pub fn open(dir: impl AsRef<Path>) -> Result<Self> {
        let dir = dir.as_ref();
        create_dir_all(dir)?;
        let points = BufWriter::new(File::create(dir.join(POINTS_FILE))?);
        let imu = BufWriter::new(File::create(dir.join(IMU_FILE))?);
        let mut recorder = Self { points, imu };
        recorder.write_headers()?;
        Ok(recorder)
    }

    fn write_headers(&mut self) -> Result<()> {
        writeln!(self.points, "{POINTS_HEADER}")?;
        writeln!(self.imu, "{IMU_HEADER}")?;
        Ok(())
    }

    /// Append one row per point, tagged with the packet timestamp/counters.
    pub fn write_points(&mut self, header: &DataFrameHeader, points: &[Point]) -> Result<()> {
        for p in points {
            let (x, y, z) = p.coords_m();
            let tag = p.tag();
            writeln!(
                self.points,
                "{},{},{},{},{},{},{},{},{},{},{}",
                header.timestamp,
                header.udp_cnt,
                header.frame_cnt,
                x,
                y,
                z,
                p.reflectivity(),
                tag.raw(),
                tag.detection_confidence(),
                tag.particle_confidence(),
                tag.adhesion_confidence(),
            )?;
        }
        Ok(())
    }

    /// Append one IMU sample row.
    pub fn write_imu(&mut self, header: &DataFrameHeader, imu: &ImuSample) -> Result<()> {
        writeln!(
            self.imu,
            "{},{},{},{},{},{},{}",
            header.timestamp,
            imu.gyro_x,
            imu.gyro_y,
            imu.gyro_z,
            imu.acc_x,
            imu.acc_y,
            imu.acc_z,
        )?;
        Ok(())
    }

    /// Flush both buffered writers to disk.
    pub fn flush(&mut self) -> Result<()> {
        self.points.flush()?;
        self.imu.flush()?;
        Ok(())
    }
}

impl Drop for CsvRecorder {
    fn drop(&mut self) {
        let _ = self.points.flush();
        let _ = self.imu.flush();
    }
}

// ---------------------------------------------------------------------------
// Loading recorded CSVs back in.
//
// The functions below read back the CSVs written by `CsvRecorder` so they can
// be replayed. Rows are returned in file order; recordings are appended in
// timestamp order, so the result is naturally time-sorted.

use std::fs;
use std::io::BufRead;

/// One point loaded from `points.csv`. Coordinates are in the LiDAR body
/// frame (metres), identical to what was written by [`CsvRecorder`].
/// Reflectivity / tag are not loaded because the viewer does not use them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadedPoint {
    pub ts_ns: u64,
    pub x_m: f32,
    pub y_m: f32,
    pub z_m: f32,
}

/// One IMU sample loaded from `imu.csv`, in the same units written by the
/// recorder (gyro in rad/s, acc in g).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadedImu {
    pub ts_ns: u64,
    pub gyro: [f32; 3],
    pub acc: [f32; 3],
}

/// Load every row of a `points.csv` file. The header line is detected by
/// the first field not parsing as a `u64` and skipped. Blank lines are
/// ignored. Returns rows in file order.
pub fn load_points_csv(path: impl AsRef<Path>) -> Result<Vec<LoadedPoint>> {
    let file = fs::File::open(path)?;
    let mut out = Vec::new();
    for (i, raw) in std::io::BufReader::new(file).lines().enumerate() {
        let line = raw?;
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        let f: Vec<&str> = trimmed.split(',').map(|s| s.trim()).collect();
        // timestamp_ns,udp_cnt,frame_cnt,x_m,y_m,z_m,...
        if f.len() < 6 {
            return Err(csv_err(i + 1, "expected at least 6 columns"));
        }
        // Skip a header line (first column literally "timestamp_ns").
        if ts_ns_check_is_header(f[0]) {
            continue;
        }
        let ts_ns = f[0]
            .parse::<u64>()
            .map_err(|_| csv_err(i + 1, "timestamp_ns is not a u64"))?;
        let x_m = parse_f32(f[3], i + 1, "x_m")?;
        let y_m = parse_f32(f[4], i + 1, "y_m")?;
        let z_m = parse_f32(f[5], i + 1, "z_m")?;
        out.push(LoadedPoint { ts_ns, x_m, y_m, z_m });
    }
    Ok(out)
}

/// Load every row of an `imu.csv` file. Header is skipped like above.
pub fn load_imu_csv(path: impl AsRef<Path>) -> Result<Vec<LoadedImu>> {
    let file = fs::File::open(path)?;
    let mut out = Vec::new();
    for (i, raw) in std::io::BufReader::new(file).lines().enumerate() {
        let line = raw?;
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        let f: Vec<&str> = trimmed.split(',').map(|s| s.trim()).collect();
        // timestamp_ns,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z
        if f.len() < 7 {
            return Err(csv_err(i + 1, "expected 7 columns"));
        }
        if ts_ns_check_is_header(f[0]) {
            continue;
        }
        let ts_ns = f[0]
            .parse::<u64>()
            .map_err(|_| csv_err(i + 1, "timestamp_ns is not a u64"))?;
        let gyro = [
            parse_f32(f[1], i + 1, "gyro_x")?,
            parse_f32(f[2], i + 1, "gyro_y")?,
            parse_f32(f[3], i + 1, "gyro_z")?,
        ];
        let acc = [
            parse_f32(f[4], i + 1, "acc_x")?,
            parse_f32(f[5], i + 1, "acc_y")?,
            parse_f32(f[6], i + 1, "acc_z")?,
        ];
        out.push(LoadedImu { ts_ns, gyro, acc });
    }
    Ok(out)
}

fn ts_ns_check_is_header(first_field: &str) -> bool {
    // The header's first column is the literal "timestamp_ns".
    first_field.eq_ignore_ascii_case("timestamp_ns")
}

fn parse_f32(s: &str, line: usize, col: &str) -> Result<f32> {
    s.parse::<f32>()
        .map_err(|_| csv_err(line, format!("{col} is not a number")))
}

fn csv_err(line: usize, msg: impl Into<String>) -> crate::error::LidarError {
    crate::error::LidarError::ParameterParse(format!("csv line {line}: {}", msg.into()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::points::{Cartesian32Point, Tag};
    use crate::protocol::{DataType, TimestampType};
    use std::fs;
    use std::path::PathBuf;

    fn test_dir(name: &str) -> PathBuf {
        let mut p = std::env::temp_dir();
        p.push(format!("lidar_reader_{name}_{}", std::process::id()));
        p
    }

    fn header(ts: u64) -> DataFrameHeader {
        DataFrameHeader {
            version: 0,
            length: 0,
            time_interval: 10,
            dot_num: 1,
            udp_cnt: 7,
            frame_cnt: 3,
            data_type: DataType::PointCloudCartesian32,
            time_type: TimestampType::None,
            timestamp: ts,
        }
    }

    #[test]
    fn writes_headers_and_rows() {
        let dir = test_dir("writes_headers_and_rows");
        let _ = fs::remove_dir_all(&dir);
        {
            let mut r = CsvRecorder::open(&dir).unwrap();
            // tag = 0b00_01_10: detection=0, particle=1, adhesion=2.
            let point = Point::Cartesian32(Cartesian32Point {
                x_mm: 1000,
                y_mm: 2000,
                z_mm: -3000,
                reflectivity: 128,
                tag: Tag(0b0000_0110),
            });
            r.write_points(&header(12345), &[point]).unwrap();
            let imu = ImuSample {
                gyro_x: 0.1,
                gyro_y: 0.2,
                gyro_z: 0.3,
                acc_x: 0.4,
                acc_y: 0.5,
                acc_z: 0.6,
            };
            r.write_imu(&header(12345), &imu).unwrap();
        }

        let points_csv = fs::read_to_string(dir.join(POINTS_FILE)).unwrap();
        let imu_csv = fs::read_to_string(dir.join(IMU_FILE)).unwrap();

        let points_lines: Vec<&str> = points_csv.lines().collect();
        assert_eq!(points_lines.len(), 2, "header + one row");
        assert!(points_lines[0].starts_with("timestamp_ns,udp_cnt,frame_cnt,x_m,y_m,z_m"));
        // 1000mm -> 1 m, 2000mm -> 2 m, -3000mm -> -3 m.
        assert_eq!(
            points_lines[1],
            "12345,7,3,1,2,-3,128,6,0,1,2",
            "exact row: ts,udp,frame,x,y,z,reflectivity,tag,det,part,adh"
        );

        let imu_lines: Vec<&str> = imu_csv.lines().collect();
        assert_eq!(imu_lines.len(), 2, "header + one row");
        assert_eq!(imu_lines[0], IMU_HEADER);
        assert_eq!(imu_lines[1], "12345,0.1,0.2,0.3,0.4,0.5,0.6");

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn open_creates_missing_nested_dir() {
        let root = test_dir("open_creates_missing_nested_dir");
        let nested = root.join("a/b/c");
        let _ = fs::remove_dir_all(&root);
        {
            let mut r = CsvRecorder::open(&nested).unwrap();
            let imu = ImuSample {
                gyro_x: 1.0,
                gyro_y: 0.0,
                gyro_z: 0.0,
                acc_x: 0.0,
                acc_y: 0.0,
                acc_z: 1.0,
            };
            r.write_imu(&header(1), &imu).unwrap();
        }
        assert!(nested.join(IMU_FILE).exists());
        assert!(nested.join(POINTS_FILE).exists());
        let _ = fs::remove_dir_all(&root);
    }

    #[test]
    fn multiple_packets_accumulate_rows() {
        let dir = test_dir("multiple_packets_accumulate_rows");
        let _ = fs::remove_dir_all(&dir);
        {
            let mut r = CsvRecorder::open(&dir).unwrap();
            for ts in 0..3 {
                let pts = vec![
                    Point::Cartesian32(Cartesian32Point {
                        x_mm: 0,
                        y_mm: 0,
                        z_mm: 0,
                        reflectivity: 0,
                        tag: Tag(0),
                    }),
                    Point::Cartesian32(Cartesian32Point {
                        x_mm: 1000,
                        y_mm: 0,
                        z_mm: 0,
                        reflectivity: 1,
                        tag: Tag(0),
                    }),
                ];
                r.write_points(&header(ts), &pts).unwrap();
                r.write_imu(&header(ts), &ImuSample {
                    gyro_x: 0.0,
                    gyro_y: 0.0,
                    gyro_z: 0.0,
                    acc_x: 0.0,
                    acc_y: 0.0,
                    acc_z: 0.0,
                })
                .unwrap();
            }
        }
        let points_lines = fs::read_to_string(dir.join(POINTS_FILE)).unwrap().lines().count();
        let imu_lines = fs::read_to_string(dir.join(IMU_FILE)).unwrap().lines().count();
        assert_eq!(points_lines, 1 + 3 * 2, "header + 3 packets * 2 points");
        assert_eq!(imu_lines, 1 + 3, "header + 3 samples");
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn load_roundtrips_recorded_csvs() {
        let dir = test_dir("load_roundtrips_recorded_csvs");
        let _ = fs::remove_dir_all(&dir);
        {
            let mut r = CsvRecorder::open(&dir).unwrap();
            let pts = vec![
                Point::Cartesian32(Cartesian32Point {
                    x_mm: 1000,
                    y_mm: -2000,
                    z_mm: 3000,
                    reflectivity: 128,
                    tag: Tag(0),
                }),
                Point::Cartesian32(Cartesian32Point {
                    x_mm: 0,
                    y_mm: 0,
                    z_mm: 0,
                    reflectivity: 0,
                    tag: Tag(0),
                }),
            ];
            r.write_points(&header(12345), &pts).unwrap();
            r.write_imu(&header(12345), &ImuSample {
                gyro_x: 0.1,
                gyro_y: 0.2,
                gyro_z: 0.3,
                acc_x: 0.4,
                acc_y: 0.5,
                acc_z: 0.6,
            })
            .unwrap();
        }

        let pts = load_points_csv(dir.join(POINTS_FILE)).unwrap();
        assert_eq!(pts.len(), 2);
        assert_eq!(pts[0].ts_ns, 12345);
        assert!((pts[0].x_m - 1.0).abs() < 1e-3);
        assert!((pts[0].y_m - (-2.0)).abs() < 1e-3);
        assert!((pts[0].z_m - 3.0).abs() < 1e-3);

        let imu = load_imu_csv(dir.join(IMU_FILE)).unwrap();
        assert_eq!(imu.len(), 1);
        assert_eq!(imu[0].ts_ns, 12345);
        assert!((imu[0].gyro[0] - 0.1).abs() < 1e-3);
        assert!((imu[0].acc[2] - 0.6).abs() < 1e-3);

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn load_skips_blank_lines_and_handles_header() {
        let dir = test_dir("load_skips_blank_lines_and_handles_header");
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        // Hand-written file with a blank line to confirm tolerance.
        fs::write(
            dir.join(POINTS_FILE),
            "timestamp_ns,udp_cnt,frame_cnt,x_m,y_m,z_m,reflectivity,tag,a,b,c\n\
             \n\
             10,0,0,1.5,2.5,3.5,0,0,0,0,0\n\
             10,0,0,4.0,5.0,6.0,0,0,0,0,0\n",
        )
        .unwrap();

        let pts = load_points_csv(dir.join(POINTS_FILE)).unwrap();
        assert_eq!(pts.len(), 2);
        assert_eq!(pts[0].ts_ns, 10);
        assert!((pts[0].z_m - 3.5).abs() < 1e-3);
        assert_eq!(pts[1].ts_ns, 10);

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn load_empty_file_returns_empty() {
        let dir = test_dir("load_empty_file_returns_empty");
        let _ = fs::remove_dir_all(&dir);
        fs::create_dir_all(&dir).unwrap();
        fs::write(dir.join(POINTS_FILE), "timestamp_ns,udp_cnt,frame_cnt,x_m,y_m,z_m,reflectivity,tag,a,b,c\n").unwrap();
        let pts = load_points_csv(dir.join(POINTS_FILE)).unwrap();
        assert!(pts.is_empty());
        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn load_missing_file_errors() {
        let err = load_points_csv("/nonexistent/lidar_reader_test_12345/points.csv");
        assert!(err.is_err());
    }
}
