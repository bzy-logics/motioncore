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
    /// No checked pose is below the margin, but the gap between two checked
    /// poses near `at` could not be shown to stay above it; `gap` is the
    /// lowest it could be.
    pub between: bool,
}

impl fmt::Display for Collision {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.between {
            let other = if self.b == "floor" { "the floor".to_string() } else { format!("the link of {}", self.b) };
            write!(f, "the link of {} could come within {:.1} mm of {other} between checked poses", self.a, self.gap)
        } else if self.b == "floor" {
            write!(f, "the link of {} would reach the floor ({:.1} mm)", self.a, self.gap)
        } else {
            write!(f, "the links of {} and {} would collide (gap {:.1} mm)", self.a, self.b, self.gap)
        }
    }
}

use crate::math::{apply, dot, mul, norm, rotate, rpy, sub, translate, M4, V3};

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
            let mut s0 = if denom > EPS { ((b * f - c * e) / denom).clamp(0.0, 1.0) } else { 0.0 };
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
    /// Every gap that is measured: the pairs, then each body's floor gap.
    items: Vec<Item>,
    /// Joints whose motion can change each item's gap, with the body they move.
    movers: Vec<Vec<(usize, usize)>>,
    /// `reach[x][i]`: how far the points of body x can be from the axis of
    /// joint i (None when joint i does not move body x).
    reach: Vec<Vec<Option<Reach>>>,
    /// Required surface clearance, mm.
    pub margin: f64,
}

/// One measured gap.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Item {
    Pair(usize, usize),
    Floor(usize),
}

/// Bound on the distance of a body's points from a joint axis, mm: a fixed
/// part from the geometry plus the travel of the prismatic joints between
/// them. For a prismatic joint, the bound is 1 (mm per mm).
#[derive(Debug, Clone, PartialEq)]
struct Reach {
    fixed: f64,
    prismatic: Vec<usize>,
}

/// Outcome of a certified path check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathClearance {
    /// Lower bound on the surface gap anywhere along the path, including
    /// between the checked poses, mm. Infinite when the model tests nothing.
    pub min_gap: f64,
    /// Poses evaluated.
    pub checks: usize,
}

const ZERO_LEN: f64 = 1e-6;

/// Finest resolution of the certified check: a gap that cannot be shown to
/// stay above the margin with steps that move no point more than this is
/// refused, mm.
pub const RESOLUTION_MM: f64 = 0.01;

/// A move that starts closer than the margin (e.g. after a geometry change)
/// may back out: its gap may dip this much below the start, mm, and must
/// not end smaller than it started.
pub const BACK_OUT_MM: f64 = 0.1;

/// Upper bound on the poses one path check evaluates.
const MAX_CHECKS: usize = 500_000;

/// Distance of `v` from the line through the origin along `axis`.
fn axis_distance(v: V3, axis: V3) -> f64 {
    let n = norm(axis);
    if n < 1e-12 {
        return norm(v);
    }
    let u = [axis[0] / n, axis[1] / n, axis[2] / n];
    let along = dot(v, u);
    norm(sub(v, [u[0] * along, u[1] * along, u[2] * along]))
}

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
        // Joints that move body x: x and its ancestors, nearest first.
        let chain = |x: usize| -> Vec<usize> {
            let mut out = vec![x];
            let mut cur = joints[x].parent;
            while let Some(c) = cur {
                out.push(c);
                cur = joints[c].parent;
            }
            out
        };
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
        // Reach of every body from the axis of every joint that moves it.
        // A point p of body x hangs off joint i through the frame origins of
        // the joints in between: |p − axis_i| ≤ (distance of the first origin
        // from the axis) + (lengths of the later origins) + (longest end of
        // x), plus the travel of any prismatic joint on the way.
        let mut reach: Vec<Vec<Option<Reach>>> = vec![vec![None; n]; n];
        for x in 0..n {
            if !has_volume(x) {
                continue;
            }
            let longest = ends[x].iter().map(|e| norm(*e)).fold(0.0, f64::max);
            let up = chain(x);
            for (depth, &i) in up.iter().enumerate() {
                reach[x][i] = Some(if joints[i].prismatic {
                    Reach { fixed: 1.0, prismatic: Vec::new() }
                } else if i == x {
                    Reach { fixed: ends[x].iter().map(|e| axis_distance(*e, joints[i].axis)).fold(0.0, f64::max), prismatic: Vec::new() }
                } else {
                    // Joints below i down to x, nearest to i first.
                    let below: Vec<usize> = up[..depth].iter().rev().copied().collect();
                    let first = axis_distance(joints[below[0]].origin, joints[i].axis);
                    let rest: f64 = below[1..].iter().map(|&k| norm(joints[k].origin)).sum();
                    Reach { fixed: first + rest + longest, prismatic: below.iter().copied().filter(|&k| joints[k].prismatic).collect() }
                });
            }
        }
        let mut items: Vec<Item> = pairs.iter().map(|&(a, b)| Item::Pair(a, b)).collect();
        items.extend((0..n).filter(|&x| joints[x].parent.is_some() && has_volume(x)).map(Item::Floor));
        // A joint that moves both bodies of a pair moves them rigidly
        // together and cannot change their distance.
        let movers = items
            .iter()
            .map(|it| match *it {
                Item::Pair(a, b) => {
                    let (ca, cb) = (chain(a), chain(b));
                    let mut m: Vec<(usize, usize)> = ca.iter().filter(|i| !cb.contains(i)).map(|&i| (i, a)).collect();
                    m.extend(cb.iter().filter(|i| !ca.contains(i)).map(|&i| (i, b)));
                    m
                }
                Item::Floor(x) => chain(x).into_iter().map(|i| (i, x)).collect(),
            })
            .collect();
        Self { joints, ends, items, movers, reach, margin }
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
            let motion = if j.prismatic { translate([j.axis[0] * v, j.axis[1] * v, j.axis[2] * v]) } else { rotate(j.axis, v.to_radians()) };
            out.push(mul(&base, &mul(&mul(&translate(j.origin), &rpy(j.rotation_deg)), &motion)));
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
                ends.iter().filter(|e| norm(**e) > ZERO_LEN).map(|e| (o, apply(&f[i], *e))).collect()
            })
            .collect()
    }

    /// Surface gap of every item at `q`.
    fn gaps(&self, q: &[f64]) -> Vec<f64> {
        let segs = self.segments(q);
        self.items
            .iter()
            .map(|it| match *it {
                Item::Pair(a, b) => {
                    let mut gap = f64::INFINITY;
                    for &(p1, q1) in &segs[a] {
                        for &(p2, q2) in &segs[b] {
                            gap = gap.min(segment_distance(p1, q1, p2, q2) - self.joints[a].radius - self.joints[b].radius);
                        }
                    }
                    gap
                }
                Item::Floor(x) => segs[x].iter().map(|&(p, e)| p[2].min(e[2]) - self.joints[x].radius).fold(f64::INFINITY, f64::min),
            })
            .collect()
    }

    fn report(&self, item: usize, gap: f64, at: Vec<f64>, between: bool) -> Collision {
        let (a, b) = match self.items[item] {
            Item::Pair(a, b) => (self.joints[a].id.clone(), self.joints[b].id.clone()),
            Item::Floor(x) => (self.joints[x].id.clone(), "floor".to_string()),
        };
        Collision { a, b, gap, at, between }
    }

    /// Smallest surface gap at `q`, and which bodies it is between.
    pub fn clearance(&self, q: &[f64]) -> Collision {
        let gaps = self.gaps(q);
        match gaps.iter().enumerate().min_by(|x, y| x.1.total_cmp(y.1)) {
            Some((k, &g)) => self.report(k, g, q.to_vec(), false),
            None => Collision { a: String::new(), b: String::new(), gap: f64::INFINITY, at: q.to_vec(), between: false },
        }
    }

    /// For each item, the joints that can change its gap and how fast: mm
    /// per degree (revolute) or per mm (prismatic) of joint motion, with
    /// every prismatic joint within `travel` of zero.
    fn rates(&self, travel: &[f64]) -> Vec<Vec<(usize, f64)>> {
        self.movers
            .iter()
            .map(|m| {
                m.iter()
                    .map(|&(i, x)| {
                        let r = self.reach[x][i].as_ref().expect("a mover has a reach");
                        let d = r.fixed + r.prismatic.iter().map(|&k| travel.get(k).copied().unwrap_or(0.0)).sum::<f64>();
                        (i, if self.joints[i].prismatic { d } else { d.to_radians() })
                    })
                    .collect()
            })
            .collect()
    }

    /// First pose with less than the margin when joint `axis` moves from
    /// `q[axis]` to `to` with the others held still. See [`Self::check_path`].
    pub fn check_sweep(&self, q: &[f64], axis: usize, to: f64) -> Result<PathClearance, Collision> {
        let mut target = q.to_vec();
        target[axis] = to;
        self.check_path(q, &target)
    }

    /// Certified check of the straight joint-space line from `from` to `to`
    /// (the path of a synchronised MoveJ). See [`Self::check_path_within`].
    pub fn check_path(&self, from: &[f64], to: &[f64]) -> Result<PathClearance, Collision> {
        self.check_path_within(from, to, &[])
    }

    /// Certified check of every pose within `deviation[i]` (deg / mm; missing
    /// = 0) of each pose on the straight joint-space line from `from` to `to`.
    ///
    /// Every gap is a Lipschitz function of the path parameter: a joint can
    /// move a point of a body no faster than the point's distance from the
    /// joint's axis allows, and that distance is bounded once from the
    /// geometry. From each checked pose the check steps no further than the
    /// remaining gap above the margin allows (and never further than 0.5° /
    /// 1 mm of the joint that moves furthest, the fixed step of earlier
    /// versions), so the margin holds between the checked poses too. Far
    /// from contact the steps are long; near it they shrink. A gap that
    /// cannot be shown to stay above the margin at [`RESOLUTION_MM`] is
    /// refused (`between` set).
    ///
    /// A gap that is already below the margin at the start may back out: it
    /// may not dip more than [`BACK_OUT_MM`] below its start and must not end
    /// smaller.
    pub fn check_path_within(&self, from: &[f64], to: &[f64], deviation: &[f64]) -> Result<PathClearance, Collision> {
        let nj = self.joints.len();
        let at = |v: &[f64], i: usize| v.get(i).copied().unwrap_or(0.0);
        let dev = |i: usize| deviation.get(i).copied().unwrap_or(0.0).abs();
        let dq: Vec<f64> = (0..nj).map(|i| at(to, i) - at(from, i)).collect();
        let travel: Vec<f64> = (0..nj).map(|i| at(from, i).abs().max(at(to, i).abs()) + dev(i)).collect();
        let rates = self.rates(&travel);
        // Gap change per unit of the path parameter, and the gap lost to the deviation.
        let lip: Vec<f64> = rates.iter().map(|r| r.iter().map(|&(i, c)| c * dq[i].abs()).sum()).collect();
        let slack: Vec<f64> = rates.iter().map(|r| r.iter().map(|&(i, c)| c * dev(i)).sum()).collect();
        let n = dq.iter().zip(&self.joints).map(|(d, j)| d.abs() / if j.prismatic { 1.0 } else { 0.5 }).fold(0.0, f64::max).ceil().clamp(1.0, 10_000.0);
        let ds_max = 1.0 / n;
        let lip_max = lip.iter().copied().fold(0.0, f64::max);
        let ds_min = if lip_max > 0.0 { (RESOLUTION_MM / lip_max).min(ds_max) } else { ds_max };
        let pose = |s: f64| -> Vec<f64> { (0..nj).map(|i| at(from, i) + dq[i] * s).collect() };
        let effective = |q: &[f64]| -> Vec<f64> { self.gaps(q).iter().zip(&slack).map(|(g, l)| g - l).collect() };

        let start = effective(&pose(0.0));
        let floor: Vec<f64> = start.iter().map(|&e| if e >= self.margin { self.margin } else { e - BACK_OUT_MM }).collect();
        let mut e0 = start.clone();
        let mut s = 0.0;
        let mut checks = 1;
        let mut min_gap = f64::INFINITY;
        while s < 1.0 {
            // Longest step no gap can close below its floor in (by the bound
            // from this pose alone), within the fixed step and the resolution.
            let mut ds = ds_max;
            for k in 0..e0.len() {
                if lip[k] > 0.0 {
                    ds = ds.min((e0[k] - floor[k]) / lip[k]);
                }
            }
            let ds = ds.max(ds_min);
            let s1 = if ds >= 1.0 - s - 1e-12 { 1.0 } else { s + ds };
            let ds = s1 - s;
            let q1 = pose(s1);
            let e1 = effective(&q1);
            checks += 1;
            if let Some(k) = (0..e1.len()).filter(|&k| e1[k] < floor[k]).min_by(|&x, &y| (e1[x] - floor[x]).total_cmp(&(e1[y] - floor[y]))) {
                return Err(self.report(k, e1[k], q1, false));
            }
            // Lowest a gap can reach between the two poses: it falls from
            // each end at most `lip` per unit of s.
            for k in 0..e1.len() {
                let low = ((e0[k] + e1[k] - lip[k] * ds) / 2.0).min(e0[k]).min(e1[k]);
                if low < floor[k] {
                    // Refused. When a pose further on is below the floor,
                    // report that contact rather than the uncertainty.
                    let mut t = s1;
                    while t < 1.0 {
                        t = (t + ds_max).min(1.0);
                        let q = pose(t);
                        let e = effective(&q);
                        if let Some(j) = (0..e.len()).find(|&j| e[j] < floor[j]) {
                            return Err(self.report(j, e[j], q, false));
                        }
                    }
                    return Err(self.report(k, low, pose(s + ds / 2.0), true));
                }
                min_gap = min_gap.min(low);
            }
            if checks >= MAX_CHECKS {
                let k = (0..e1.len()).min_by(|&x, &y| e1[x].total_cmp(&e1[y])).unwrap_or(0);
                return Err(self.report(k, min_gap, q1, true));
            }
            s = s1;
            e0 = e1;
        }
        if let Some(k) = (0..e0.len()).find(|&k| start[k] < self.margin && e0[k] < start[k] - 1e-9) {
            return Err(self.report(k, e0[k], pose(1.0), false));
        }
        Ok(PathClearance { min_gap, checks })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// The six-axis desktop arm of robot-profiles/dummy-arm-virtual.toml.
    pub(crate) fn dummy() -> CollisionModel {
        let j = |id: &str, parent: Option<usize>, origin: V3, rot: V3, tip: Option<V3>, radius: f64| JointSpec {
            id: id.into(),
            parent,
            origin,
            rotation_deg: rot,
            axis: [0.0, 0.0, 1.0],
            tip,
            radius,
            prismatic: false,
        };
        CollisionModel::new(
            vec![
                j("j1", None, [0.0; 3], [0.0; 3], None, 30.0),
                j("j2", Some(0), [35.0, 0.0, 109.0], [0.0, -90.0, -90.0], None, 22.0),
                j("j3", Some(1), [146.0, 0.0, 0.0], [0.0, 0.0, 90.0], None, 20.0),
                j("j4", Some(2), [-52.0, -115.0, 0.0], [90.0, 0.0, 0.0], None, 18.0),
                j("j5", Some(3), [0.0; 3], [-90.0, 0.0, 0.0], None, 18.0),
                j("j6", Some(4), [0.0; 3], [90.0, 0.0, 0.0], Some([0.0, 0.0, 72.0]), 15.0),
            ],
            2.0,
        )
    }

    #[test]
    fn segment_distance_cases() {
        assert!((segment_distance([0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]) - 1.0).abs() < 1e-12);
        assert!((segment_distance([0.0; 3], [2.0, 0.0, 0.0], [1.0, -1.0, 1.0], [1.0, 1.0, 1.0]) - 1.0).abs() < 1e-12);
        assert!((segment_distance([0.0; 3], [1.0, 0.0, 0.0], [3.0, 0.0, 0.0], [4.0, 0.0, 0.0]) - 2.0).abs() < 1e-12);
        assert!(segment_distance([0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3]) < 1e-12);
    }

    #[test]
    fn forward_kinematics_matches_the_dummy_firmware() {
        let m = dummy();
        let tip = |q: &[f64]| apply(&m.frames(q)[5], [0.0, 0.0, 72.0]);
        let up = tip(&[0.0; 6]);
        assert!((up[0] + 17.0).abs() < 1e-9 && up[1].abs() < 1e-9 && (up[2] - 442.0).abs() < 1e-9, "{up:?}");
        let rest = tip(&[0.0, -73.0, 180.0, 0.0, 0.0, 0.0]);
        assert!((rest[0] - 89.41).abs() < 0.01 && (rest[2] - 146.74).abs() < 0.01, "{rest:?}");
    }

    #[test]
    fn spherical_wrist_bodies_are_not_tested_against_each_other() {
        let m = dummy();
        let ids: Vec<(String, String)> = m
            .items
            .iter()
            .filter_map(|it| match *it {
                Item::Pair(a, b) => Some((m.joints[a].id.clone(), m.joints[b].id.clone())),
                Item::Floor(_) => None,
            })
            .collect();
        // j4, j5 have no volume; j3 (forearm) and j6 (flange) touch through them.
        assert!(!ids.iter().any(|(a, b)| a == "j4" || a == "j5" || b == "j4" || b == "j5"), "{ids:?}");
        assert!(!ids.contains(&("j3".into(), "j6".into())));
        assert!(ids.contains(&("j2".into(), "j6".into())));
        assert!(ids.contains(&("j1".into(), "j3".into())));
    }

    #[test]
    fn known_poses() {
        let m = dummy();
        assert!(m.clearance(&[0.0, 0.0, 90.0, 0.0, 0.0, 0.0]).gap > 50.0, "upright is clear");
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
        assert!(e.at[4] > 0.0 && e.at[4] < 30.0, "first contact well before 60: {e:?}");
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
    fn joint_space_paths_are_checked_between_safe_endpoints() {
        let m = dummy();
        let zero = [0.0; 6];
        let work = [35.0, 0.0, 90.0, 30.0, -50.0, 90.0];
        assert!(m.clearance(&zero).gap >= m.margin && m.clearance(&work).gap >= m.margin);
        m.check_path(&zero, &work).unwrap();
        // Both ends are well clear, but the straight joint-space path between
        // them swings the flange through the base column (over 15 mm deep).
        let a = [-120.0, 20.0, 180.0, 130.0, 80.0, -10.0];
        let b = [150.0, -35.0, 150.0, -145.0, 60.0, 465.0];
        assert!(m.clearance(&a).gap > 10.0 && m.clearance(&b).gap > 10.0);
        let e = m.check_path(&a, &b).unwrap_err();
        assert_eq!((e.a.as_str(), e.b.as_str()), ("j1", "j6"), "{e:?}");
        // A single-joint path is the same as a sweep.
        let rest = [0.0, -73.0, 180.0, 0.0, 0.0, 0.0];
        let mut to = rest;
        to[4] = 60.0;
        assert_eq!(m.check_path(&rest, &to).unwrap_err().at, m.check_sweep(&rest, 4, 60.0).unwrap_err().at);
    }

    #[test]
    fn floor_contact_is_detected() {
        let m = dummy();
        // Shoulder tipped fully forward with the elbow bent down.
        let c = m.clearance(&[0.0, 90.0, 90.0, 0.0, 0.0, 0.0]);
        assert_eq!(c.b, "floor", "{c:?}");
        assert!(c.gap < 0.0);
    }

    #[test]
    fn backing_out_may_not_end_closer_than_it_started() {
        let m = dummy();
        let into = [0.0, -73.0, 180.0, 0.0, 25.0, 0.0];
        let start = m.clearance(&into).gap;
        assert!(start < m.margin);
        // A twentieth of a degree deeper stays within the back-out allowance
        // on the way, but ends closer than it started.
        let e = m.check_sweep(&into, 4, 25.05).unwrap_err();
        assert!(!e.between && e.gap < start && e.gap > start - BACK_OUT_MM, "{e:?}");
    }

    /// Two root bodies: a post at the origin and an arm pivoting at x = −2000
    /// at mid-height of the post, whose tip passes `pass` mm from the post's
    /// surface at 0°. A 0.5° step moves the tip 17 mm.
    fn swing(pass: f64) -> CollisionModel {
        let j = |id: &str, origin: V3, tip: V3| JointSpec {
            id: id.into(),
            parent: None,
            origin,
            rotation_deg: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            tip: Some(tip),
            radius: 10.0,
            prismatic: false,
        };
        CollisionModel::new(vec![j("post", [0.0; 3], [0.0, 0.0, 500.0]), j("arm", [-2000.0, 0.0, 250.0], [1980.0 - pass, 0.0, 0.0])], 2.0)
    }

    #[test]
    fn a_contact_between_fixed_samples_is_found() {
        let m = swing(0.5);
        // Every pose on the fixed 0.5° grid of earlier versions is clear ...
        for k in 0..=40 {
            let q = [0.0, -10.25 + 0.5 * k as f64];
            assert!(m.clearance(&q).gap > m.margin, "{q:?}: {:?}", m.clearance(&q));
        }
        // ... but the tip passes 0.5 mm from the post at 0°.
        assert!((m.clearance(&[0.0, 0.0]).gap - 0.5).abs() < 1e-9);
        let e = m.check_path(&[0.0, -10.25], &[0.0, 9.75]).unwrap_err();
        assert!(e.gap < m.margin && e.at[1].abs() < 0.25, "{e:?}");
        assert_eq!((e.a.as_str(), e.b.as_str()), ("post", "arm"));
    }

    #[test]
    fn steps_are_long_far_from_contact_and_short_near_it() {
        let m = dummy();
        let (zero, work) = ([0.0; 6], [35.0, 0.0, 90.0, 30.0, -50.0, 90.0]);
        let r = m.check_path(&zero, &work).unwrap();
        // Fixed step: 90° of the furthest joint at 0.5° is 180 steps, plus the start.
        assert!(r.checks <= 181, "{r:?}");
        assert!(r.min_gap >= m.margin, "{r:?}");
        // A pass 3 mm from contact is accepted, but checked more finely
        // than every 0.5°, and the certified gap is between margin and 3 mm.
        let m = swing(3.0);
        let r = m.check_path(&[0.0, -10.25], &[0.0, 9.75]).unwrap();
        assert!(r.checks > 41, "{r:?}");
        assert!(r.min_gap >= m.margin && r.min_gap <= 3.0, "{r:?}");
    }

    #[test]
    fn a_deviation_from_the_line_uses_up_clearance() {
        let m = swing(3.0);
        let (from, to) = ([0.0, -10.25], [0.0, 9.75]);
        // 0.01° of the 1977 mm arm is 0.35 mm.
        let r = m.check_path_within(&from, &to, &[0.0, 0.01]).unwrap();
        assert!(r.min_gap <= 3.0 - 0.34, "{r:?}");
        // 0.05° is 1.7 mm: under the margin.
        assert!(m.check_path_within(&from, &to, &[0.0, 0.05]).is_err());
    }

    /// A cylindrical robot: a base turning about z, a lift (prismatic, z) on
    /// it, an arm pitching about y at the top of the lift and a hand hanging
    /// from the arm. The lift's travel adds to the base's reach.
    fn cylindrical() -> CollisionModel {
        let j = |id: &str, parent: Option<usize>, origin: V3, axis: V3, tip: Option<V3>, radius: f64, prismatic: bool| JointSpec {
            id: id.into(),
            parent,
            origin,
            rotation_deg: [0.0; 3],
            axis,
            tip,
            radius,
            prismatic,
        };
        CollisionModel::new(
            vec![
                j("base", None, [0.0; 3], [0.0, 0.0, 1.0], None, 40.0, false),
                j("lift", Some(0), [0.0, 0.0, 100.0], [0.0, 0.0, 1.0], None, 30.0, true),
                j("arm", Some(1), [60.0, 0.0, 50.0], [0.0, 1.0, 0.0], None, 20.0, false),
                j("hand", Some(2), [250.0, 0.0, 0.0], [0.0, 0.0, 1.0], Some([0.0, 0.0, -120.0]), 15.0, false),
            ],
            2.0,
        )
    }

    /// Smallest gap on the straight line, sampled every 0.01° / 0.01 mm of
    /// the joint that moves furthest.
    fn dense_min(m: &CollisionModel, from: &[f64], to: &[f64]) -> f64 {
        let n = from.iter().zip(to).map(|(a, b)| ((b - a).abs() / 0.01).ceil()).fold(1.0, f64::max) as usize;
        (0..=n)
            .map(|k| {
                let f = k as f64 / n as f64;
                let q: Vec<f64> = from.iter().zip(to).map(|(a, b)| a + (b - a) * f).collect();
                m.clearance(&q).gap
            })
            .fold(f64::INFINITY, f64::min)
    }

    /// The property: a path the check accepts keeps the margin at every
    /// densely resampled pose, and the reported minimum is a lower bound.
    fn holds(m: &CollisionModel, from: &[f64], to: &[f64]) -> Result<(), proptest::test_runner::TestCaseError> {
        use proptest::prelude::*;
        prop_assume!(m.clearance(from).gap >= m.margin);
        match m.check_path(from, to) {
            Ok(r) => {
                let d = dense_min(m, from, to);
                prop_assert!(d >= m.margin - 1e-9, "accepted, but dense gap {} < margin ({:?})", d, r);
                prop_assert!(d >= r.min_gap - 1e-9, "dense gap {} below the certified {}", d, r.min_gap);
            }
            Err(c) if !c.between => {
                prop_assert!(m.clearance(&c.at).gap < m.margin, "refused at a clear pose: {:?}", c);
            }
            Err(_) => {}
        }
        Ok(())
    }

    mod property {
        use super::*;
        use proptest::prelude::*;

        const ARM: [(f64, f64); 6] = [(-170.0, 170.0), (-73.0, 90.0), (35.0, 180.0), (-180.0, 180.0), (-120.0, 120.0), (-720.0, 720.0)];
        const CYL: [(f64, f64); 4] = [(-180.0, 180.0), (0.0, 300.0), (-120.0, 120.0), (-180.0, 180.0)];

        fn path(limits: &[(f64, f64)], start: &[f64], delta: &[f64], moving: &[bool], reach: f64) -> (Vec<f64>, Vec<f64>) {
            let from: Vec<f64> = limits.iter().zip(start).map(|(&(lo, hi), s)| lo + (hi - lo) * s).collect();
            let to = from
                .iter()
                .zip(limits)
                .zip(delta.iter().zip(moving))
                .map(|((&f, &(lo, hi)), (&d, &mv))| if mv { (f + reach * d).clamp(lo, hi) } else { f })
                .collect();
            (from, to)
        }

        proptest! {
            #![proptest_config(ProptestConfig::with_cases(96))]

            #[test]
            fn six_axis_arm(start in prop::array::uniform6(0.0..1.0f64), delta in prop::array::uniform6(-1.0..1.0f64), moving in prop::array::uniform6(any::<bool>())) {
                let (from, to) = path(&ARM, &start, &delta, &moving, 40.0);
                holds(&dummy(), &from, &to)?;
            }

            #[test]
            fn cylindrical_robot_with_a_prismatic_joint(start in prop::array::uniform4(0.0..1.0f64), delta in prop::array::uniform4(-1.0..1.0f64), moving in prop::array::uniform4(any::<bool>())) {
                let (from, to) = path(&CYL, &start, &delta, &moving, 60.0);
                holds(&cylindrical(), &from, &to)?;
            }

            #[test]
            fn swinging_arm(a in -15.0..15.0f64, b in -15.0..15.0f64, pass in 0.0..6.0f64) {
                holds(&swing(pass), &[0.0, a], &[0.0, b])?;
            }
        }
    }
}
