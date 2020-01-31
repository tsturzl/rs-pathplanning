use geo::{Coordinate, LineString};
use std::f64::{consts::PI, INFINITY};

#[derive(Clone)]
pub enum Mode {
    L,
    S,
    R,
}

type ModeSlice = Option<&'static [Mode; 3]>;
type PlannerResult = (Option<f64>, Option<f64>, Option<f64>, ModeSlice);

fn fmodr(x: f64, y: f64) -> f64 {
    x - y * (x / y).floor()
}

pub fn mod2pi(theta: f64) -> f64 {
    fmodr(theta, 2.0 * PI)
}

pub fn pi_2_pi(angle: f64) -> f64 {
    (angle + PI) % (2.0 * PI) - PI
}

const LSL_MODE: ModeSlice = Some(&[Mode::L, Mode::S, Mode::L]);
pub fn lsl(alpha: f64, beta: f64, d: f64) -> PlannerResult {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let c_ab = (alpha - beta).cos();

    let tmp0 = d + sa - sb;

    let p_squared = 2.0 + (d * d) - (2.0 * c_ab) + (2.0 * d * (sa - sb));

    if p_squared < 0.0 {
        return (None, None, None, LSL_MODE);
    }

    let tmp1 = (cb - ca).atan2(tmp0);
    let t = mod2pi(-alpha + tmp1);
    let p = p_squared.sqrt();
    let q = mod2pi(beta - tmp1);

    (Some(t), Some(p), Some(q), LSL_MODE)
}

const RSR_MODE: ModeSlice = Some(&[Mode::R, Mode::S, Mode::R]);
pub fn rsr(alpha: f64, beta: f64, d: f64) -> PlannerResult {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let c_ab = (alpha - beta).cos();

    let tmp0 = d - sa + sb;
    let p_squared = 2.0 + (d * d) - (2.0 * c_ab) + (2.0 * d * (sb - sa));

    if p_squared < 0.0 {
        return (None, None, None, RSR_MODE);
    }

    let tmp1 = (ca - cb).atan2(tmp0);
    let t = mod2pi(alpha - tmp1);
    let p = p_squared.sqrt();
    let q = mod2pi(-beta + tmp1);

    (Some(t), Some(p), Some(q), RSR_MODE)
}

const LSR_MODE: ModeSlice = Some(&[Mode::L, Mode::S, Mode::R]);
pub fn lsr(alpha: f64, beta: f64, d: f64) -> PlannerResult {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let c_ab = (alpha - beta).cos();

    let p_squared = -2.0 + (d * d) + (2.0 * c_ab) + (2.0 * d * (sa + sb));
    if p_squared < 0.0 {
        return (None, None, None, LSR_MODE);
    }

    let p = p_squared.sqrt();
    let tmp = (-ca - cb).atan2(d + sa + sb) - (-2.0_f64).atan2(p);
    let t = mod2pi(-alpha + tmp);
    let q = mod2pi(-mod2pi(beta) + tmp);

    (Some(t), Some(p), Some(q), LSR_MODE)
}

const RSL_MODE: ModeSlice = Some(&[Mode::R, Mode::S, Mode::L]);
pub fn rsl(alpha: f64, beta: f64, d: f64) -> PlannerResult {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let c_ab = (alpha - beta).cos();

    let p_squared = -2.0 + (d * d) + (2.0 * c_ab) - (2.0 * d * (sa + sb));
    if p_squared < 0.0 {
        return (None, None, None, RSL_MODE);
    }

    let p = p_squared.sqrt();
    let tmp = (ca + cb).atan2(d - sa - sb) - 2.0_f64.atan2(p);
    let t = mod2pi(alpha - tmp);
    let q = mod2pi(beta - tmp);

    (Some(t), Some(p), Some(q), RSL_MODE)
}

const RLR_MODE: ModeSlice = Some(&[Mode::R, Mode::L, Mode::R]);
pub fn rlr(alpha: f64, beta: f64, d: f64) -> PlannerResult {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let c_ab = (alpha - beta).cos();

    let tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0;
    if tmp_rlr.abs() > 1.0 {
        return (None, None, None, RLR_MODE);
    }

    let p = mod2pi(2.0 * PI - tmp_rlr.acos());
    let t = mod2pi(alpha - (ca - cb).atan2(d - sa + sb) + mod2pi(p / 2.0));
    let q = mod2pi(alpha - beta - t + mod2pi(p));

    (Some(t), Some(p), Some(q), RLR_MODE)
}

const LRL_MODE: ModeSlice = Some(&[Mode::L, Mode::R, Mode::L]);
pub fn lrl(alpha: f64, beta: f64, d: f64) -> PlannerResult {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let c_ab = (alpha - beta).cos();

    let tmp_lrl = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (-sa + sb)) / 8.0;
    if tmp_lrl.abs() > 1.0 {
        return (None, None, None, LRL_MODE);
    }

    let p = mod2pi(2.0 * PI - tmp_lrl.acos());
    let t = mod2pi(-alpha - (ca - cb).atan2(d + sa - sb) + p / 2.0);
    let q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p));

    (Some(t), Some(p), Some(q), LRL_MODE)
}

fn interpolate(
    ind: usize,
    length: f64,
    mode: &Mode,
    max_curvature: f64,
    origin_x: f64,
    origin_y: f64,
    origin_yaw: f64,
    path_x: &mut Vec<f64>,
    path_y: &mut Vec<f64>,
    path_yaw: &mut Vec<f64>,
    directions: &mut Vec<isize>,
) {
    if let Mode::S = mode {
        path_x[ind] = origin_x + length / max_curvature * origin_yaw.cos();
        path_y[ind] = origin_y + length / max_curvature * origin_yaw.sin();
        path_yaw[ind] = origin_yaw;
    } else {
        let ldx = length.sin() / max_curvature;
        let mut ldy = 0.0;
        if let Mode::L = mode {
            ldy = (1.0 - length.cos()) / max_curvature;
        } else if let Mode::R = mode {
            ldy = (1.0 - length.cos()) / -max_curvature;
        }
        let gdx = (-origin_yaw).cos() * ldx + (-origin_yaw).sin() * ldy;
        let gdy = -(-origin_yaw).sin() * ldx + (-origin_yaw).cos() * ldy;

        path_x[ind] = origin_x + gdx;
        path_y[ind] = origin_y + gdy;
    }

    if let Mode::L = mode {
        path_yaw[ind] = origin_yaw + length;
    } else if let Mode::R = mode {
        path_yaw[ind] = origin_yaw - length;
    }

    if length > 0.0 {
        directions[ind] = 1;
    } else {
        directions[ind] = -1;
    }
}

pub fn generate_course(
    length: &[f64],
    mode: &[Mode],
    c: f64,
    d_angle: f64,
    px: &mut Vec<f64>,
    py: &mut Vec<f64>,
    pyaw: &mut Vec<f64>,
) {
    for (m, l) in mode.iter().zip(length.iter()) {
        let mut pd = 0.0;

        let mut d = d_angle;

        if let Mode::S = m {
            d = 1.0 * c;
        }

        while pd < (l - d).abs() {
            px.push(px[px.len() - 1] + d / c * pyaw[pyaw.len() - 1].cos());
            py.push(py[py.len() - 1] + d / c * pyaw[pyaw.len() - 1].sin());

            match m {
                Mode::L => pyaw.push(pyaw[pyaw.len() - 1] + d),
                Mode::S => pyaw.push(pyaw[pyaw.len() - 1]),
                Mode::R => pyaw.push(pyaw[pyaw.len() - 1] - d),
            }
            pd += d;
        }

        d = l - pd;

        px.push(px[px.len() - 1] + d / c * pyaw[pyaw.len() - 1].cos());
        py.push(py[py.len() - 1] + d / c * pyaw[pyaw.len() - 1].sin());

        match m {
            Mode::L => pyaw.push(pyaw[pyaw.len() - 1] + d),
            Mode::S => pyaw.push(pyaw[pyaw.len() - 1]),
            Mode::R => pyaw.push(pyaw[pyaw.len() - 1] - d),
        }
        // pd += d; // unused?
    }
}

fn generate_local_course(
    lengths: &[f64],
    mode: &[Mode],
    max_curvature: f64,
    step_size: f64,
    path_x: &mut Vec<f64>,
    path_y: &mut Vec<f64>,
    path_yaw: &mut Vec<f64>,
    directions: &mut Vec<isize>,
) {
    let mut ind = 1;

    if lengths[0] > 0.0 {
        directions[0] = 1
    } else {
        directions[0] = -1
    }

    let mut ll = 0.0;

    let iter = mode.iter().zip(lengths).zip(0..3).map(|a| {
        let b = a.0;
        (b.0, b.1, a.1)
    });

    for (m, l, i) in iter {
        let mut d = -step_size;
        if l > &0.0 {
            d = step_size;
        }

        let (origin_x, origin_y, origin_yaw) = (path_x[ind], path_y[ind], path_yaw[ind]);

        ind -= 1;
        let mut pd = d - ll;
        if i >= 1 && (lengths[i - 1] * lengths[i]) > 0.0 {
            pd = -d - ll;
        }

        while pd.abs() <= l.abs() {
            ind += 1;
            interpolate(
                ind,
                pd,
                m,
                max_curvature,
                origin_x,
                origin_y,
                origin_yaw,
                path_x,
                path_y,
                path_yaw,
                directions,
            );
            pd += d;
        }
        ll = l - pd - d;

        ind += 1;
        interpolate(
            ind,
            *l,
            m,
            max_curvature,
            origin_x,
            origin_y,
            origin_yaw,
            path_x,
            path_y,
            path_yaw,
            directions,
        );
    }

    if path_x.len() <= 1 {
        path_x.clear();
        path_y.clear();
        path_yaw.clear();
        directions.clear();
    }

    let mut last = path_x[path_x.len() - 1];
    while path_x.len() >= 1 && last == 0.0 {
        last = path_x[path_x.len() - 1];
        path_x.pop();
        path_y.pop();
        path_yaw.pop();
        directions.pop();
    }
}

const ALL_PLANNERS: &[fn(f64, f64, f64) -> PlannerResult] = &[lsl, rsr, lsr, rsl, rlr, lrl];
struct Planners {
    i: usize,
    alpha: f64,
    beta: f64,
    d: f64,
}

impl Iterator for Planners {
    type Item = PlannerResult;

    fn next(&mut self) -> Option<PlannerResult> {
        if self.i < ALL_PLANNERS.len() {
            let result = ALL_PLANNERS[self.i](self.alpha, self.beta, self.d);
            self.i += 1;
            Some(result)
        } else {
            None
        }
    }
}
type DubinsPath = (Vec<f64>, Vec<f64>, Vec<f64>, ModeSlice, f64);
type DubinsPathResult = Option<DubinsPath>;

pub struct DubinsConfig {
    pub sx: f64,
    pub sy: f64,
    pub syaw: f64,
    pub ex: f64,
    pub ey: f64,
    pub eyaw: f64,
    pub c: f64,
    pub step_size: f64,
}

pub fn dubins_path_planning_from_origin(
    ex: f64,
    ey: f64,
    eyaw: f64,
    c: f64,
    step_size: f64,
) -> DubinsPathResult {
    let dx = ex;
    let dy = ey;
    let hypot = dx.hypot(dy);
    let d = hypot * c;

    let theta = mod2pi(dy.atan2(dx));
    let alpha = mod2pi(-theta);
    let beta = mod2pi(eyaw - theta);

    let planners = Planners {
        i: 0,
        alpha,
        beta,
        d,
    };

    let mut bcost = INFINITY;
    let (mut bt, mut bp, mut bq, mut bmode) = (None, None, None, None) as PlannerResult;

    for (t, p, q, mode) in planners {
        if let (Some(ot), Some(op), Some(oq)) = (t, p, q) {
            let cost = ot.abs() + op.abs() + oq.abs();

            if bcost > cost {
                bt = Some(ot);
                bp = Some(op);
                bq = Some(oq);
                bmode = mode;

                bcost = cost;
            }
        }
    }

    match (bt, bp, bq, bmode) {
        (Some(bt), Some(bp), Some(bq), Some(bmode)) => {
            let lengths = [bt, bp, bq];
            let total_length: f64 = lengths.iter().sum();
            let n_point = ((total_length / step_size).trunc() as usize) + lengths.len() + 4;
            let (mut px, mut py, mut pyaw) = (
                vec![0.0_f64; n_point],
                vec![0.0_f64; n_point],
                vec![0.0_f64; n_point],
            );
            let mut directions = vec![0_isize; n_point];
            // generate_course(
            //     &[bt, bp, bq],
            //     bmode,
            //     c,
            //     d_angle,
            //     &mut px,
            //     &mut py,
            //     &mut pyaw,
            // );
            generate_local_course(
                &lengths,
                bmode,
                c,
                step_size,
                &mut px,
                &mut py,
                &mut pyaw,
                &mut directions,
            );
            Some((px, py, pyaw, Some(bmode), bcost))
        }
        _ => None,
    }
}

pub fn dubins_path_planning(conf: &DubinsConfig) -> DubinsPathResult {
    let sx = conf.sx;
    let sy = conf.sy;
    let ex = conf.ex - conf.sx;
    let ey = conf.ey - conf.sy;
    let syaw = conf.syaw;
    let eyaw = conf.eyaw;
    let c = conf.c;

    let lex = syaw.cos() * ex + syaw.sin() * ey;
    let ley = -(syaw.sin()) * ex + syaw.cos() * ey;
    let leyaw = eyaw - syaw;

    match dubins_path_planning_from_origin(lex, ley, leyaw, c, conf.step_size) {
        Some((lpx, lpy, lpyaw, mode, clen)) => {
            let px: Vec<f64> = lpx
                .iter()
                .zip(lpy.iter())
                .map(|(x, y)| (-syaw).cos() * x + (-syaw).sin() * y + sx)
                .collect();
            let py: Vec<f64> = lpx
                .iter()
                .zip(lpy.iter())
                .map(|(x, y)| -(-syaw).sin() * x + (-syaw).cos() * y + sy)
                .collect();
            let pyaw: Vec<f64> = lpyaw.iter().map(|iyaw| pi_2_pi(iyaw + syaw)).collect();

            Some((px, py, pyaw, mode, clen))
        }
        None => None,
    }
}

pub fn dubins_path_linestring(conf: &DubinsConfig) -> Option<LineString<f64>> {
    match dubins_path_planning(conf) {
        Some((px, py, _, _, _)) => Some(
            px.into_iter()
                .zip(py.into_iter())
                .map(|(x, y)| Coordinate { x, y })
                .collect(),
        ),
        None => None,
    }
}
