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

//! Point-cloud tokenizer based on a PCA-oriented bounding box.
//!
//! [`encode`] fits a PCA-based oriented bounding box (OBB) around the cloud
//! and uses it as the reference frame. The OBB volume is recursively split
//! into octants `rounds` times, producing `8^rounds` leaf segments. Every
//! leaf segment emits 4 floats: the fraction of cloud points it contains and
//! the centroid of those points (the segment's geometric center when empty).
//!
//! Each centroid is expressed in a normalized frame relative to its own
//! segment: `[-1, -1, -1]` is the segment's left/down/back corner and
//! `[1, 1, 1]` its right/up/front corner, so an empty segment (whose
//! centroid is the geometric center) emits `[0, 0, 0]`. Axes follow the OBB
//! frame: x is the principal component (left/right), y the second (down/up),
//! z the third (back/front).
//!
//! Octants are always emitted in this fixed order:
//!
//! ```text
//! 0 left_down_back    1 right_down_back
//! 2 left_down_front   3 right_down_front
//! 4 left_up_back      5 right_up_back
//! 6 left_up_front     7 right_up_front
//! ```
//!
//! For `rounds > 1` each octant is replaced, in place, by the recursively
//! generated vector of its eight children, preserving the same order. The
//! final vector therefore has `4 * 8^rounds` floats and forms an ordered
//! 8-ary tree.

/// Minimum number of subdivision rounds.
pub const MIN_ROUNDS: u8 = 1;
/// Maximum number of subdivision rounds.
pub const MAX_ROUNDS: u8 = 5;
/// Floats emitted per segment: fraction + centroid (x, y, z).
pub const FLOATS_PER_SEGMENT: usize = 4;

#[derive(Debug, thiserror::Error)]
pub enum EncodeError {
    #[error("cannot encode an empty point cloud")]
    EmptyCloud,
    #[error("rounds must be between 1 and 5, got {0}")]
    InvalidRounds(u8),
}

/// Encodes a point cloud into a fixed-order token vector. See module docs.
pub fn encode(cloud: &[[f32; 3]], rounds: u8) -> Result<Vec<f32>, EncodeError> {
    if cloud.is_empty() {
        return Err(EncodeError::EmptyCloud);
    }
    if !(MIN_ROUNDS..=MAX_ROUNDS).contains(&rounds) {
        return Err(EncodeError::InvalidRounds(rounds));
    }

    let points: Vec<Vec3> = cloud
        .iter()
        .map(|p| [p[0] as f64, p[1] as f64, p[2] as f64])
        .collect();

    let obb = Obb::fit(&points);
    let local: Vec<Vec3> = points.iter().map(|p| obb.to_local(*p)).collect();
    let min = scale(obb.half_extents, -1.0);

    let mut tokens = Vec::with_capacity(FLOATS_PER_SEGMENT * 8usize.pow(rounds as u32));
    encode_box(
        &local,
        min,
        obb.half_extents,
        rounds,
        local.len() as f64,
        &mut tokens,
    );
    Ok(tokens)
}

/// Recursively subdivides the box and appends segment tokens in octant order.
fn encode_box(points: &[Vec3], min: Vec3, max: Vec3, depth: u8, total: f64, out: &mut Vec<f32>) {
    if depth == 0 {
        push_segment(points, min, max, total, out);
        return;
    }

    let mid = midpoint(min, max);
    let mut buckets: [Vec<Vec3>; 8] = std::array::from_fn(|_| Vec::new());
    for p in points {
        buckets[octant_index(p, &mid)].push(*p);
    }

    for (index, bucket) in buckets.iter().enumerate() {
        let (child_min, child_max) = child_bounds(min, mid, max, index);
        encode_box(bucket, child_min, child_max, depth - 1, total, out);
    }
}

/// Appends the 4-float token of one leaf segment.
fn push_segment(points: &[Vec3], min: Vec3, max: Vec3, total: f64, out: &mut Vec<f32>) {
    let fraction = points.len() as f64 / total;

    let centroid = if points.is_empty() {
        // Geometric center of the segment: the origin of the normalized frame.
        [0.0; 3]
    } else {
        let mut sum = [0.0; 3];
        for p in points {
            for axis in 0..3 {
                sum[axis] += p[axis];
            }
        }
        let mean = scale(sum, 1.0 / points.len() as f64);
        normalize_to_segment(mean, min, max)
    };

    out.push(fraction as f32);
    out.extend(centroid.map(|c| c as f32));
}

/// Maps a point inside the segment `[min, max]` to `[-1, 1]^3`, where -1 is
/// the left/down/back corner and +1 the right/up/front corner. Degenerate
/// (zero-thickness) axes map to 0.
fn normalize_to_segment(p: Vec3, min: Vec3, max: Vec3) -> Vec3 {
    let mut out = [0.0; 3];
    for axis in 0..3 {
        let extent = max[axis] - min[axis];
        if extent > 0.0 {
            out[axis] = 2.0 * (p[axis] - min[axis]) / extent - 1.0;
        }
    }
    out
}

/// Bit each axis contributes to the octant index: x -> bit 0, y -> bit 2, z -> bit 1.
/// This yields exactly the fixed order from the module docs.
const AXIS_BITS: [usize; 3] = [1, 4, 2];

/// Octant of a point relative to the box midpoint. Points exactly on the
/// split plane fall on the positive side (right / up / front).
fn octant_index(p: &Vec3, mid: &Vec3) -> usize {
    let mut index = 0;
    for axis in 0..3 {
        if p[axis] >= mid[axis] {
            index |= AXIS_BITS[axis];
        }
    }
    index
}

/// Bounds of the child octant `index` of the box `[min, max]` split at `mid`.
fn child_bounds(min: Vec3, mid: Vec3, max: Vec3, index: usize) -> (Vec3, Vec3) {
    let mut child_min = min;
    let mut child_max = max;
    for axis in 0..3 {
        if index & AXIS_BITS[axis] != 0 {
            child_min[axis] = mid[axis];
        } else {
            child_max[axis] = mid[axis];
        }
    }
    (child_min, child_max)
}

/// PCA-based oriented bounding box used as the encoding reference frame.
#[derive(Debug, Clone, Copy)]
struct Obb {
    center: Vec3,
    /// Orthonormal axes (rows): x = principal component, then y, z. Right-handed.
    axes: Mat3,
    half_extents: Vec3,
}

impl Obb {
    fn fit(points: &[Vec3]) -> Self {
        let (mean, axes) = pca(points);

        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for p in points {
            let d = sub(*p, mean);
            for k in 0..3 {
                let t = dot(d, axes[k]);
                min[k] = min[k].min(t);
                max[k] = max[k].max(t);
            }
        }

        let mut center = mean;
        let mut half_extents = [0.0; 3];
        for k in 0..3 {
            center = add(center, scale(axes[k], (min[k] + max[k]) / 2.0));
            half_extents[k] = (max[k] - min[k]) / 2.0;
        }

        Self {
            center,
            axes,
            half_extents,
        }
    }

    /// Expresses a world point in the box frame (box center = origin).
    fn to_local(&self, p: Vec3) -> Vec3 {
        let d = sub(p, self.center);
        [
            dot(d, self.axes[0]),
            dot(d, self.axes[1]),
            dot(d, self.axes[2]),
        ]
    }
}

/// Centroid and principal axes of the cloud, sorted by decreasing variance.
fn pca(points: &[Vec3]) -> (Vec3, Mat3) {
    let n = points.len() as f64;

    let mut mean = [0.0; 3];
    for p in points {
        for axis in 0..3 {
            mean[axis] += p[axis] / n;
        }
    }

    let mut covariance = [[0.0; 3]; 3];
    for p in points {
        let d = sub(*p, mean);
        for i in 0..3 {
            for j in 0..3 {
                covariance[i][j] += d[i] * d[j] / n;
            }
        }
    }

    let (values, vectors) = jacobi_eigen(covariance);

    let mut order = [0, 1, 2];
    order.sort_by(|&a, &b| values[b].partial_cmp(&values[a]).unwrap());

    let mut axes: Mat3 = order.map(|col| [vectors[0][col], vectors[1][col], vectors[2][col]]);
    for axis in &mut axes {
        fix_sign(axis);
    }
    // Keep the frame right-handed so the result is never mirrored.
    if dot(cross(axes[0], axes[1]), axes[2]) < 0.0 {
        axes[2] = scale(axes[2], -1.0);
    }

    (mean, axes)
}

/// Flips an eigenvector so its dominant component is positive, making the
/// decomposition deterministic.
fn fix_sign(axis: &mut Vec3) {
    let dominant = (0..3)
        .max_by(|&i, &j| axis[i].abs().partial_cmp(&axis[j].abs()).unwrap())
        .unwrap();
    if axis[dominant] < 0.0 {
        *axis = scale(*axis, -1.0);
    }
}

/// Eigen-decomposition of a symmetric 3x3 matrix via Jacobi rotations.
/// Returns the eigenvalues and a matrix whose columns are the eigenvectors.
fn jacobi_eigen(matrix: Mat3) -> ([f64; 3], Mat3) {
    let mut a = matrix;
    let mut v = [[0.0; 3]; 3];
    for i in 0..3 {
        v[i][i] = 1.0;
    }

    // The trace is invariant under rotations; use it as convergence scale.
    let tolerance = 1e-14 * (a[0][0] + a[1][1] + a[2][2]);

    for _ in 0..32 {
        // Largest off-diagonal element.
        let (mut p, mut q, mut off) = (0, 1, 0.0_f64);
        for i in 0..3 {
            for j in (i + 1)..3 {
                if a[i][j].abs() > off {
                    off = a[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if off <= tolerance {
            break;
        }

        let angle = 0.5 * f64::atan2(2.0 * a[p][q], a[q][q] - a[p][p]);
        let (sin, cos) = angle.sin_cos();

        for k in 0..3 {
            let (akp, akq) = (a[k][p], a[k][q]);
            a[k][p] = cos * akp - sin * akq;
            a[k][q] = sin * akp + cos * akq;
        }
        for k in 0..3 {
            let (apk, aqk) = (a[p][k], a[q][k]);
            a[p][k] = cos * apk - sin * aqk;
            a[q][k] = sin * apk + cos * aqk;
        }
        for k in 0..3 {
            let (vkp, vkq) = (v[k][p], v[k][q]);
            v[k][p] = cos * vkp - sin * vkq;
            v[k][q] = sin * vkp + cos * vkq;
        }
    }

    ([a[0][0], a[1][1], a[2][2]], v)
}

type Vec3 = [f64; 3];
type Mat3 = [[f64; 3]; 3];

fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn midpoint(min: Vec3, max: Vec3) -> Vec3 {
    [
        (min[0] + max[0]) / 2.0,
        (min[1] + max[1]) / 2.0,
        (min[2] + max[2]) / 2.0,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fractions(tokens: &[f32]) -> impl Iterator<Item = f32> + '_ {
        tokens.iter().step_by(FLOATS_PER_SEGMENT).copied()
    }

    #[test]
    fn rejects_empty_cloud() {
        assert!(matches!(encode(&[], 1), Err(EncodeError::EmptyCloud)));
    }

    #[test]
    fn rejects_out_of_range_rounds() {
        let cloud = [[0.0, 0.0, 0.0]];
        assert!(matches!(
            encode(&cloud, 0),
            Err(EncodeError::InvalidRounds(0))
        ));
        assert!(matches!(
            encode(&cloud, 6),
            Err(EncodeError::InvalidRounds(6))
        ));
    }

    #[test]
    fn produces_4_times_8_to_the_n_floats() {
        let cloud = [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [2.0, 0.0, 1.0]];
        for rounds in MIN_ROUNDS..=MAX_ROUNDS {
            let tokens = encode(&cloud, rounds).unwrap();
            assert_eq!(tokens.len(), FLOATS_PER_SEGMENT * 8usize.pow(rounds as u32));
        }
    }

    #[test]
    fn fractions_sum_to_one() {
        let cloud = [
            [0.0, 0.0, 0.0],
            [1.0, 2.0, 3.0],
            [-1.0, 0.5, 2.0],
            [4.0, -3.0, 1.0],
        ];
        for rounds in MIN_ROUNDS..=MAX_ROUNDS {
            let tokens = encode(&cloud, rounds).unwrap();
            let sum: f32 = fractions(&tokens).sum();
            assert!((sum - 1.0).abs() < 1e-5, "rounds {rounds}: sum {sum}");
        }
    }

    #[test]
    fn octant_order_matches_specification() {
        let mid = [0.0, 0.0, 0.0];
        let cases = [
            ([-1.0, -1.0, -1.0], 0), // left_down_back
            ([1.0, -1.0, -1.0], 1),  // right_down_back
            ([-1.0, -1.0, 1.0], 2),  // left_down_front
            ([1.0, -1.0, 1.0], 3),   // right_down_front
            ([-1.0, 1.0, -1.0], 4),  // left_up_back
            ([1.0, 1.0, -1.0], 5),   // right_up_back
            ([-1.0, 1.0, 1.0], 6),   // left_up_front
            ([1.0, 1.0, 1.0], 7),    // right_up_front
        ];
        for (point, expected) in cases {
            assert_eq!(octant_index(&point, &mid), expected);
        }
    }

    #[test]
    fn segment_fractions_and_centroids() {
        // Two points on the x axis: the PCA frame is the world frame, the
        // box is [-1, 1] x [0, 0] x [0, 0] centered at (1, 0, 0).
        let cloud = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
        let tokens = encode(&cloud, 1).unwrap();

        // (0,0,0) -> left_up_front (6), (2,0,0) -> right_up_front (7). Each
        // point sits exactly at its segment's left/right corner, so the
        // normalized centroid x is -1 / +1.
        assert_eq!(&tokens[24..28], &[0.5, -1.0, 0.0, 0.0]);
        assert_eq!(&tokens[28..32], &[0.5, 1.0, 0.0, 0.0]);
        // Empty segment (left_down_back) falls back to its geometric center,
        // which is the origin of the normalized per-segment frame.
        assert_eq!(&tokens[0..4], &[0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn centroid_is_normalized_to_its_segment() {
        // Three collinear points; the box is [-1, 1] x [0, 0] x [0, 0].
        // Local coords: -1.0, -0.5 and 1.0 on the x axis.
        let cloud = [[0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [2.0, 0.0, 0.0]];
        let tokens = encode(&cloud, 1).unwrap();

        // The first two share left_up_front (6): local mean x = -0.75 in a
        // segment spanning [-1, 0] -> normalized -0.5.
        assert_eq!(&tokens[24..28], &[2.0 / 3.0, -0.5, 0.0, 0.0]);
        assert_eq!(&tokens[28..32], &[1.0 / 3.0, 1.0, 0.0, 0.0]);
    }

    #[test]
    fn recursion_nests_segments_in_octant_order() {
        // Same geometry as above, one level deeper: each point lands on the
        // up/front edge of its parent octant, so the child index repeats.
        let cloud = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
        let tokens = encode(&cloud, 2).unwrap();

        assert_eq!(tokens.len(), FLOATS_PER_SEGMENT * 64);
        assert_eq!(&tokens[6 * 32 + 24..6 * 32 + 28], &[0.5, -1.0, 0.0, 0.0]);
        assert_eq!(&tokens[7 * 32 + 28..7 * 32 + 32], &[0.5, 1.0, 0.0, 0.0]);
    }

    #[test]
    fn jacobi_recovers_symmetric_matrix() {
        let m = [[4.0, 1.0, 2.0], [1.0, 3.0, 0.5], [2.0, 0.5, 5.0]];
        let (values, vectors) = jacobi_eigen(m);
        for i in 0..3 {
            for j in 0..3 {
                let mut reconstructed = 0.0;
                for k in 0..3 {
                    reconstructed += vectors[i][k] * values[k] * vectors[j][k];
                }
                assert!((reconstructed - m[i][j]).abs() < 1e-10);
            }
        }
    }
}
