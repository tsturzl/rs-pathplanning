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

pub fn generate_course(
    length: &[f64],
    mode: &[Mode],
    c: f64,
    d_angle: f64,
    px: &mut Vec<f64>,
    py: &mut Vec<f64>,
    pyaw: &mut Vec<f64>,
) {
    // let mut px = vec![0.0_f64];
    // let mut py = vec![0.0_f64];
    // let mut pyaw = vec![0.0_f64];

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
}

pub fn dubins_path_planning_from_origin(conf: &DubinsConfig, d_angle: f64) -> DubinsPathResult {
    let dx = conf.ex;
    let dy = conf.ey;
    let hypot = dx.hypot(dy);
    let d = hypot * conf.c;

    let theta = mod2pi(dy.atan2(dx));
    let alpha = mod2pi(-theta);
    let beta = mod2pi(conf.eyaw - theta);

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
            let (mut px, mut py, mut pyaw) = (vec![0.0_f64], vec![0.0_f64], vec![0.0_f64]);
            generate_course(
                &[bt, bp, bq],
                bmode,
                conf.c,
                d_angle,
                &mut px,
                &mut py,
                &mut pyaw,
            );
            Some((px, py, pyaw, Some(bmode), bcost))
        }
        _ => None,
    }
}

const D_ANGLE: f64 = PI / 72.0; // 2.5 angular degrees
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

    match dubins_path_planning_from_origin(&conf, D_ANGLE) {
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
