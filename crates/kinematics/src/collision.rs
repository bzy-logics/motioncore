//! Self-collision and floor checks for a joint tree with capsule links.
//!
//! Each joint owns one *body*: capsules (radius = the joint's link radius)
//! from its frame origin to the origins of the joints mounted on it and to
//! its tip. Joint frames follow the URDF convention used by the robot
//! profile geometry:
//!
//! `frame(child) = frame(parent) · T(origin) · RPY(rotation) · Joint(q about axis)`
//!
//! Bodies that touch by construction are never tested against each other:
//! a joint and its parent, and joints connected only through bodies of zero
//! length (e.g. the axes of a spherical wrist sharing one point). Bodies of
//! zero length have no volume. Every body not mounted directly on the base
//! must also stay above the floor (z = 0 of the base frame).
//!
//! This is a coarse model (capsules, not CAD) meant to catch gross
//! self-collisions of joint moves. It is no substitute for guarding,
//! reduced speed during commissioning, or a hardware E-stop.

use std::fmt;

/// Joint description, in mm and degrees.
#[derive(Debug, Clone, PartialEq)]
pub struct JointSpec {
    pub id: String,
    /// Index of the parent joint (must come earlier in the list); None = base.
    pub parent: Option<usize>,
    pub origin: [f64; 3],
    pub rotation_deg: [f64; 3],
    /// Unit joint axis.
    pub axis: [f64; 3],
    pub tip: Option<[f64; 3]>,
    pub radius: f64,
    pub prismatic: bool,
}

/// What a check found.
#[derive(Debug, Clone, PartialEq)]
pub struct Collision {
    /// Joint ids of the two bodies; `b` is "floor" for floor contact.
    pub a: String,
    pub b: String,
    /// Gap between surfaces (negative = overlap), mm.
    pub gap: f64,
    /// Joint values at which it happens.
    pub at: Vec<f64>,
}

impl fmt::Display for Collision {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.b == "floor" {
            write!(
                f,
                "the link of {} would reach the floor ({:.1} mm)",
                self.a, self.gap
            )
        } else {
            write!(
                f,
                "the links of {} and {} would collide (gap {:.1} mm)",
                self.a, self.b, self.gap
            )
        }
    }
}

type V3 = [f64; 3];
type M4 = [[f64; 4]; 4];

fn mul(a: &M4, b: &M4) -> M4 {
    let mut o = [[0.0; 4]; 4];
    for (r, row) in o.iter_mut().enumerate() {
        for (c, cell) in row.iter_mut().enumerate() {
            *cell = (0..4).map(|k| a[r][k] * b[k][c]).sum();
        }
    }
    o
}

fn translate(v: V3) -> M4 {
    [
        [1.0, 0.0, 0.0, v[0]],
        [0.0, 1.0, 0.0, v[1]],
        [0.0, 0.0, 1.0, v[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn rotate(u: V3, angle: f64) -> M4 {
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    let [x, y, z] = u;
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn rpy(r: V3) -> M4 {
    let d = std::f64::consts::PI / 180.0;
    mul(
        &rotate([0.0, 0.0, 1.0], r[2] * d),
        &mul(
            &rotate([0.0, 1.0, 0.0], r[1] * d),
            &rotate([1.0, 0.0, 0.0], r[0] * d),
        ),
    )
}

fn apply(m: &M4, p: V3) -> V3 {
    [
        m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2] + m[0][3],
        m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2] + m[1][3],
        m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + m[2][3],
    ]
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

/// Exact minimum distance between segments p1–q1 and p2–q2.
pub fn segment_distance(p1: V3, q1: V3, p2: V3, q2: V3) -> f64 {
    let d1 = sub(q1, p1);
    let d2 = sub(q2, p2);
    let r = sub(p1, p2);
    let a = dot(d1, d1);
    let e = dot(d2, d2);
    let f = dot(d2, r);
    const EPS: f64 = 1e-12;
    let (s, t);
    if a <= EPS && e <= EPS {
        return norm(r);
    }
    if a <= EPS {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = dot(d1, r);
        if e <= EPS {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = dot(d1, d2);
            let denom = a * e - b * b;
            let mut s0 = if denom > EPS {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t0 = (b * s0 + f) / e;
            if t0 < 0.0 {
                t0 = 0.0;
                s0 = (-c / a).clamp(0.0, 1.0);
            } else if t0 > 1.0 {
                t0 = 1.0;
                s0 = ((b - c) / a).clamp(0.0, 1.0);
            }
            s = s0;
            t = t0;
        }
    }
    let c1 = [p1[0] + d1[0] * s, p1[1] + d1[1] * s, p1[2] + d1[2] * s];
    let c2 = [p2[0] + d2[0] * t, p2[1] + d2[1] * t, p2[2] + d2[2] * t];
    norm(sub(c1, c2))
}

/// Capsule model of a joint tree.
#[derive(Debug, Clone)]
pub struct CollisionModel {
    joints: Vec<JointSpec>,
    /// Local segment end points of each body (in its joint frame).
    ends: Vec<Vec<V3>>,
    /// Body pairs that are tested.
    pairs: Vec<(usize, usize)>,
    /// Required surface clearance, mm.
    pub margin: f64,
}

const ZERO_LEN: f64 = 1e-6;

impl CollisionModel {
    /// Build the model. Joints must be ordered parents first.
    pub fn new(joints: Vec<JointSpec>, margin: f64) -> Self {
        let n = joints.len();
        let mut ends: Vec<Vec<V3>> = vec![Vec::new(); n];
        for (i, j) in joints.iter().enumerate() {
            if let Some(p) = j.parent {
                ends[p].push(j.origin);
            }
            if let Some(t) = j.tip {
                ends[i].push(t);
            }
        }
        let has_volume = |i: usize| ends[i].iter().any(|e| norm(*e) > ZERO_LEN);
        // a and b touch by construction if one is an ancestor of the other
        // and every joint strictly between them has no volume.
        let touching = |a: usize, b: usize| -> bool {
            for (lo, hi) in [(a, b), (b, a)] {
                let mut cur = joints[hi].parent;
                let mut between_empty = true;
                while let Some(c) = cur {
                    if c == lo {
                        return between_empty;
                    }
                    if has_volume(c) {
                        between_empty = false;
                    }
                    cur = joints[c].parent;
                }
            }
            false
        };
        let mut pairs = Vec::new();
        for a in 0..n {
            for b in a + 1..n {
                if has_volume(a) && has_volume(b) && !touching(a, b) {
                    pairs.push((a, b));
                }
            }
        }
        Self {
            joints,
            ends,
            pairs,
            margin,
        }
    }

    pub fn joints(&self) -> &[JointSpec] {
        &self.joints
    }

    /// World frame of every joint for joint values `q` (deg / mm).
    pub fn frames(&self, q: &[f64]) -> Vec<M4> {
        let mut out: Vec<M4> = Vec::with_capacity(self.joints.len());
        for (i, j) in self.joints.iter().enumerate() {
            let base = j.parent.map(|p| out[p]).unwrap_or(translate([0.0; 3]));
            let v = q.get(i).copied().unwrap_or(0.0);
            let motion = if j.prismatic {
                translate([j.axis[0] * v, j.axis[1] * v, j.axis[2] * v])
            } else {
                rotate(j.axis, v.to_radians())
            };
            out.push(mul(
                &base,
                &mul(&mul(&translate(j.origin), &rpy(j.rotation_deg)), &motion),
            ));
        }
        out
    }

    fn segments(&self, q: &[f64]) -> Vec<Vec<(V3, V3)>> {
        let f = self.frames(q);
        self.ends
            .iter()
            .enumerate()
            .map(|(i, ends)| {
                let o = apply(&f[i], [0.0; 3]);
                ends.iter()
                    .filter(|e| norm(**e) > ZERO_LEN)
                    .map(|e| (o, apply(&f[i], *e)))
                    .collect()
            })
            .collect()
    }

    /// Smallest surface gap at `q`, and which bodies it is between.
    pub fn clearance(&self, q: &[f64]) -> Collision {
        let segs = self.segments(q);
        let mut worst = Collision {
            a: String::new(),
            b: String::new(),
            gap: f64::INFINITY,
            at: q.to_vec(),
        };
        for &(a, b) in &self.pairs {
            for &(p1, q1) in &segs[a] {
                for &(p2, q2) in &segs[b] {
                    let gap = segment_distance(p1, q1, p2, q2)
                        - self.joints[a].radius
                        - self.joints[b].radius;
                    if gap < worst.gap {
                        worst = Collision {
                            a: self.joints[a].id.clone(),
                            b: self.joints[b].id.clone(),
                            gap,
                            at: q.to_vec(),
                        };
                    }
                }
            }
        }
        for (i, j) in self.joints.iter().enumerate() {
            if j.parent.is_none() {
                continue; // stands on the base
            }
            for &(p, e) in &segs[i] {
                let gap = p[2].min(e[2]) - j.radius;
                if gap < worst.gap {
                    worst = Collision {
                        a: j.id.clone(),
                        b: "floor".into(),
                        gap,
                        at: q.to_vec(),
                    };
                }
            }
        }
        worst
    }

    /// Check moving joint `axis` from `q[axis]` to `to` with the others held
    /// still. Returns the first pose on the way with less than `margin`
    /// clearance. Moves that never get worse than the start are allowed, so
    /// the robot can always back out of a bad pose.
    pub fn check_sweep(&self, q: &[f64], axis: usize, to: f64) -> Result<(), Collision> {
        let from = q[axis];
        let step = if self.joints[axis].prismatic {
            1.0
        } else {
            0.5
        };
        let n = (((to - from).abs() / step).ceil() as usize).clamp(1, 10_000);
        let start = self.clearance(q).gap;
        let mut qs = q.to_vec();
        for k in 1..=n {
            qs[axis] = from + (to - from) * k as f64 / n as f64;
            let c = self.clearance(&qs);
            if c.gap < self.margin && c.gap < start - 1e-6 {
                return Err(c);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The six-axis desktop arm of robot-profiles/dummy-arm-virtual.toml.
    pub(crate) fn dummy() -> CollisionModel {
        let j =
            |id: &str, parent: Option<usize>, origin: V3, rot: V3, tip: Option<V3>, radius: f64| {
                JointSpec {
                    id: id.into(),
                    parent,
                    origin,
                    rotation_deg: rot,
                    axis: [0.0, 0.0, 1.0],
                    tip,
                    radius,
                    prismatic: false,
                }
            };
        CollisionModel::new(
            vec![
                j("j1", None, [0.0; 3], [0.0; 3], None, 30.0),
                j(
                    "j2",
                    Some(0),
                    [35.0, 0.0, 109.0],
                    [0.0, -90.0, -90.0],
                    None,
                    22.0,
                ),
                j(
                    "j3",
                    Some(1),
                    [146.0, 0.0, 0.0],
                    [0.0, 0.0, 90.0],
                    None,
                    20.0,
                ),
                j(
                    "j4",
                    Some(2),
                    [-52.0, -115.0, 0.0],
                    [90.0, 0.0, 0.0],
                    None,
                    18.0,
                ),
                j("j5", Some(3), [0.0; 3], [-90.0, 0.0, 0.0], None, 18.0),
                j(
                    "j6",
                    Some(4),
                    [0.0; 3],
                    [90.0, 0.0, 0.0],
                    Some([0.0, 0.0, 72.0]),
                    15.0,
                ),
            ],
            2.0,
        )
    }

    #[test]
    fn segment_distance_cases() {
        assert!(
            (segment_distance([0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]) - 1.0)
                .abs()
                < 1e-12
        );
        assert!(
            (segment_distance([0.0; 3], [2.0, 0.0, 0.0], [1.0, -1.0, 1.0], [1.0, 1.0, 1.0]) - 1.0)
                .abs()
                < 1e-12
        );
        assert!(
            (segment_distance([0.0; 3], [1.0, 0.0, 0.0], [3.0, 0.0, 0.0], [4.0, 0.0, 0.0]) - 2.0)
                .abs()
                < 1e-12
        );
        assert!(segment_distance([0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3]) < 1e-12);
    }

    #[test]
    fn forward_kinematics_matches_the_dummy_firmware() {
        let m = dummy();
        let tip = |q: &[f64]| apply(&m.frames(q)[5], [0.0, 0.0, 72.0]);
        let up = tip(&[0.0; 6]);
        assert!(
            (up[0] + 17.0).abs() < 1e-9 && up[1].abs() < 1e-9 && (up[2] - 442.0).abs() < 1e-9,
            "{up:?}"
        );
        let rest = tip(&[0.0, -73.0, 180.0, 0.0, 0.0, 0.0]);
        assert!(
            (rest[0] - 89.41).abs() < 0.01 && (rest[2] - 146.74).abs() < 0.01,
            "{rest:?}"
        );
    }

    #[test]
    fn spherical_wrist_bodies_are_not_tested_against_each_other() {
        let m = dummy();
        let ids: Vec<(String, String)> = m
            .pairs
            .iter()
            .map(|&(a, b)| (m.joints[a].id.clone(), m.joints[b].id.clone()))
            .collect();
        // j4, j5 have no volume; j3 (forearm) and j6 (flange) touch through them.
        assert!(
            !ids.iter()
                .any(|(a, b)| a == "j4" || a == "j5" || b == "j4" || b == "j5"),
            "{ids:?}"
        );
        assert!(!ids.contains(&("j3".into(), "j6".into())));
        assert!(ids.contains(&("j2".into(), "j6".into())));
        assert!(ids.contains(&("j1".into(), "j3".into())));
    }

    #[test]
    fn known_poses() {
        let m = dummy();
        assert!(
            m.clearance(&[0.0, 0.0, 90.0, 0.0, 0.0, 0.0]).gap > 50.0,
            "upright is clear"
        );
        let rest = m.clearance(&[0.0, -73.0, 180.0, 0.0, 0.0, 0.0]);
        assert!(rest.gap > m.margin, "rest pose is clear: {rest:?}");
        // The pose the old homing plan ended in: wrist folded into the upper arm.
        let bad = m.clearance(&[-170.0, -73.0, 180.0, -180.0, -120.0, -720.0]);
        assert!(bad.gap < 0.0 && bad.a == "j2" && bad.b == "j6", "{bad:?}");
    }

    #[test]
    fn sweeps_into_collision_are_refused_but_backing_out_is_allowed() {
        let m = dummy();
        // Folded arm, wrist straight: pitching the wrist positive drives the
        // flange into the upper arm; it is refused part-way, not at the end.
        let rest = [0.0, -73.0, 180.0, 0.0, 0.0, 0.0];
        let e = m.check_sweep(&rest, 4, 60.0).unwrap_err();
        assert!(e.gap < m.margin && e.b == "j6", "{e:?}");
        assert!(
            e.at[4] > 0.0 && e.at[4] < 30.0,
            "first contact well before 60: {e:?}"
        );
        assert!(e.to_string().contains("would collide"), "{e}");
        // Pitching the other way is clear.
        m.check_sweep(&rest, 4, -60.0).unwrap();
        // Slightly into the collision: reversing (clearance only improves)
        // is allowed, going deeper is not.
        let into = [0.0, -73.0, 180.0, 0.0, 25.0, 0.0];
        assert!(m.clearance(&into).gap < m.margin);
        m.check_sweep(&into, 4, 0.0).unwrap();
        assert!(m.check_sweep(&into, 4, 40.0).is_err());
    }

    #[test]
    fn floor_contact_is_detected() {
        let m = dummy();
        // Shoulder tipped fully forward with the elbow bent down.
        let c = m.clearance(&[0.0, 90.0, 90.0, 0.0, 0.0, 0.0]);
        assert_eq!(c.b, "floor", "{c:?}");
        assert!(c.gap < 0.0);
    }
}
