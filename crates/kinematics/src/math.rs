//! Small 3D math helpers (row-major 4x4 homogeneous matrices; mm, rad).

pub type V3 = [f64; 3];
pub type M4 = [[f64; 4]; 4];

pub fn mul(a: &M4, b: &M4) -> M4 {
    let mut o = [[0.0; 4]; 4];
    for (r, row) in o.iter_mut().enumerate() {
        for (c, cell) in row.iter_mut().enumerate() {
            *cell = (0..4).map(|k| a[r][k] * b[k][c]).sum();
        }
    }
    o
}

pub fn translate(v: V3) -> M4 {
    [
        [1.0, 0.0, 0.0, v[0]],
        [0.0, 1.0, 0.0, v[1]],
        [0.0, 0.0, 1.0, v[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

pub fn rotate(u: V3, angle: f64) -> M4 {
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

pub fn rpy(r: V3) -> M4 {
    let d = std::f64::consts::PI / 180.0;
    mul(
        &rotate([0.0, 0.0, 1.0], r[2] * d),
        &mul(
            &rotate([0.0, 1.0, 0.0], r[1] * d),
            &rotate([1.0, 0.0, 0.0], r[0] * d),
        ),
    )
}

pub fn apply(m: &M4, p: V3) -> V3 {
    [
        m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2] + m[0][3],
        m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2] + m[1][3],
        m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + m[2][3],
    ]
}

pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}
