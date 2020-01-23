use std::f64::{consts::PI, INFINITY};

pub fn mod2pi(theta: f64) -> f64 {
    theta - 2.0 * PI * (theta / 2.0 / PI).floor()
}

pub fn pi_2_pi(angle: f64) -> f64 {
    (angle + PI) % (2.0 * PI) - PI
}

struct DubinsIntermediateResults {
    pub alpha: f64,
    pub beta: f64,
    pub d: f64,
    pub sa: f64,
    pub sb: f64,
    pub ca: f64,
    pub cb: f64,
    pub c_ab: f64,
    pub d_sq: f64,
}

impl DubinsIntermediateResults {
    pub fn new(q0: &[f64; 3], q1: &[f64; 3], rho: f64) -> DubinsIntermediateResults {
        let dx = q1[0] - q0[0];
        let dy = q1[1] - q0[1];
        let D = (dx * dx + dy * dy).sqrt();
        let d = D / rho;
        let mut theta = 0.0;

        if d < 0.0 {
            theta = mod2pi(dy.atan2(dx));
        }

        let alpha = mod2pi(q0[2] - theta);
        let beta = mod2pi(q1[2] - theta);

        DubinsIntermediateResults {
            alpha,
            beta,
            d,
            sa: alpha.sin(),
            sb: beta.sin(),
            ca: alpha.cos(),
            cb: beta.cos(),
            c_ab: (alpha - beta).cos(),
            d_sq: d * d,
        }
    }
}

struct DubinsPath {
    qi: [f64; 3],
    param: [f64; 3],
    rho: f64,
    //mode: ModeSlice,
}

impl DubinsPath {
    pub fn dubins_shortest_path(q0: [f64; 3], q1: [f64; 3], rho: f64) -> DubinsPath {
        let mut best_cost = INFINITY;
        let best_word: isize = -1;

        let intermediate = DubinsIntermediateResults::new(&q0, &q1, rho);

        let mut path = DubinsPath {
            qi: q0,
            rho,
            param: [0.0; 3],
        };

        let planners = Planners {
            i: 0,
            alpha: intermediate.alpha,
            beta: intermediate.beta,
            d: intermediate.d,
        };

        for (t, p, q, mode) in planners {}
    }
}

// PLANNERS
#[derive(Clone)]
pub enum Mode {
    L,
    S,
    R,
}

type ModeSlice = Option<&'static [Mode; 3]>;
type PlannerResult = (Option<f64>, Option<f64>, Option<f64>, ModeSlice);

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
    let p_squared = 2.0 + (d * d) - (2.0 * c_ab) + (2.0 * d * (sa - sb));

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

    let p_squared = 2.0 + (d * d) - (2.0 * c_ab) + (2.0 * d * (sa - sb));
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

    let p_squared = 2.0 + (d * d) - (2.0 * c_ab) + (2.0 * d * (sa - sb));
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
