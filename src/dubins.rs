use std::f64::{consts::PI, INFINITY};

#[derive(Clone)]
pub enum Mode {
    L,
    S,
    R,
}

type PlannerResult = (Option<f64>, Option<f64>, Option<f64>, Vec<Mode>);

pub fn mod2pi(theta: f64) -> f64 {
    theta - 2.0 * PI * (theta / 2.0 / PI).floor()
}

pub fn pi_2_pi(angle: f64) -> f64 {
    (angle + PI) % (2.0 * PI) - PI
}

pub fn lsl(alpha: f64, beta: f64, d: f64) -> PlannerResult {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let c_ab = (alpha - beta).cos();

    let tmp0 = d + sa - sb;

    let mode = vec![Mode::L, Mode::S, Mode::L];

    let p_squared = 2.0 + (d * d) - (2.0 * c_ab) + (2.0 * d * (sa - sb));

    if p_squared < 0.0 {
        return (None, None, None, mode);
    }

    let tmp1 = (cb - ca).atan2(tmp0);
    let t = mod2pi(-alpha + tmp1);
    let p = p_squared.sqrt();
    let q = mod2pi(beta - tmp1);

    (Some(t), Some(p), Some(q), mode)
}

pub fn rsr(alpha: f64, beta: f64, d: f64) -> PlannerResult {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let c_ab = (alpha - beta).cos();

    let tmp0 = d - sa + sb;
    let mode = vec![Mode::R, Mode::S, Mode::R];
    let p_squared = 2.0 + (d * d) - (2.0 * c_ab) + (2.0 * d * (sb - sa));

    if p_squared < 0.0 {
        return (None, None, None, mode);
    }

    let tmp1 = (ca - cb).atan2(tmp0);
    let t = mod2pi(alpha - tmp1);
    let p = p_squared.sqrt();
    let q = mod2pi(-beta + tmp1);

    (Some(t), Some(p), Some(q), mode)
}

pub fn lsr(alpha: f64, beta: f64, d: f64) -> PlannerResult {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let c_ab = (alpha - beta).cos();

    let mode = vec![Mode::L, Mode::S, Mode::R];
    let p_squared = -2.0 + (d * d) + (2.0 * c_ab) + (2.0 * d * (sa + sb));
    if p_squared < 0.0 {
        return (None, None, None, mode);
    }

    let p = p_squared.sqrt();
    let tmp = (-ca - cb).atan2(d + sa + sb) - (-2.0_f64).atan2(p);
    let t = mod2pi(-alpha + tmp);
    let q = mod2pi(-mod2pi(beta) + tmp);

    (Some(t), Some(p), Some(q), mode)
}

pub fn rsl(alpha: f64, beta: f64, d: f64) -> PlannerResult {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let c_ab = (alpha - beta).cos();

    let mode = vec![Mode::R, Mode::S, Mode::L];
    let p_squared = (d * d) - 2.0 + (2.0 * c_ab) - (2.0 * d * (sa + sb));
    if p_squared < 0.0 {
        return (None, None, None, mode);
    }

    let p = p_squared.sqrt();
    let tmp = (ca + cb).atan2(d - sa - sb) - 2.0_f64.atan2(p);
    let t = mod2pi(alpha - tmp);
    let q = mod2pi(beta - tmp);

    (Some(t), Some(p), Some(q), mode)
}

pub fn rlr(alpha: f64, beta: f64, d: f64) -> PlannerResult {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let c_ab = (alpha - beta).cos();

    let mode = vec![Mode::R, Mode::L, Mode::R];
    let tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0;
    if tmp_rlr.abs() > 1.0 {
        return (None, None, None, mode);
    }

    let p = mod2pi(2.0 * PI - tmp_rlr.acos());
    let t = mod2pi(alpha - (ca - cb).atan2(d - sa + sb) + mod2pi(p / 2.0));
    let q = mod2pi(alpha - beta - t + mod2pi(p));

    (Some(t), Some(p), Some(q), mode)
}

pub fn lrl(alpha: f64, beta: f64, d: f64) -> PlannerResult {
    let sa = alpha.sin();
    let sb = beta.sin();
    let ca = alpha.cos();
    let cb = beta.cos();
    let c_ab = (alpha - beta).cos();

    let mode = vec![Mode::L, Mode::R, Mode::L];
    let tmp_lrl = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (-sa + sb)) / 8.0;
    if tmp_lrl.abs() > 1.0 {
        return (None, None, None, mode);
    }

    let p = mod2pi(2.0 * PI - tmp_lrl.acos());
    let t = mod2pi(-alpha - (ca - cb).atan2(d + sa - sb) + p / 2.0);
    let q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p));

    (Some(t), Some(p), Some(q), mode)
}

pub fn generate_course(
    length: &[f64],
    mode: &[Mode],
    c: f64,
    d_angle: f64,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut px = vec![0.0_f64];
    let mut py = vec![0.0_f64];
    let mut pyaw = vec![0.0_f64];

    for (m, l) in mode.iter().zip(length.iter()) {
        let mut pd = 0.0;

        let mut d = d_angle;
        // This can probably be less dumb
        match m {
            Mode::S => d = 1.0 * c,
            Mode::L => {}
            Mode::R => {}
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

    (px, py, pyaw)
}

fn all_planners(alpha: f64, beta: f64, d: f64) -> [PlannerResult; 6] {
    [
        lsl(alpha, beta, d),
        rsr(alpha, beta, d),
        lsr(alpha, beta, d),
        rsl(alpha, beta, d),
        rlr(alpha, beta, d),
        lrl(alpha, beta, d),
    ]
}

type DubinsPathResult = Option<(Vec<f64>, Vec<f64>, Vec<f64>, Vec<Mode>, f64)>;

pub fn dubins_path_planning_from_origin(
    ex: f64,
    ey: f64,
    eyaw: f64,
    c: f64,
    d_angle: f64,
) -> DubinsPathResult {
    let dx = ex;
    let dy = ey;
    let hypot = dx.hypot(dy);
    let d = hypot * c;

    let theta = mod2pi(dy.atan2(dx));
    let alpha = mod2pi(-theta);
    let beta = mod2pi(eyaw - theta);

    let planners: [PlannerResult; 6] = all_planners(alpha, beta, d);

    let mut bcost = INFINITY;
    let (mut bt, mut bp, mut bq, mut bmode) = (None, None, None, vec![]) as PlannerResult;

    for (t, p, q, mode) in planners.iter() {
        if let (Some(ot), Some(op), Some(oq)) = (t, p, q) {
            let cost = ot.abs() + op.abs() + oq.abs();

            if bcost > cost {
                bt = Some(*ot);
                bp = Some(*op);
                bq = Some(*oq);
                bmode = mode.to_vec();

                bcost = cost;
            }
        }
    }

    match (bt, bp, bq) {
        (Some(bt), Some(bp), Some(bq)) => {
            let (px, py, pyaw) = generate_course(&[bt, bp, bq], &bmode, c, d_angle);
            Some((px, py, pyaw, bmode, bcost))
        }
        _ => None,
    }
}

const D_ANGLE: f64 = PI / 18.0; // 10 angular degrees
pub fn dubins_path_planning(
    sx: f64,
    sy: f64,
    syaw: f64,
    ex: f64,
    ey: f64,
    eyaw: f64,
    c: f64,
) -> DubinsPathResult {
    let ex = ex - sx;
    let ey = ey - sy;

    let lex = syaw.cos() * ex + syaw.sin() * ey;
    let ley = -(syaw.sin()) * ex + syaw.cos() * ey;
    let leyaw = eyaw - syaw;

    match dubins_path_planning_from_origin(lex, ley, leyaw, c, D_ANGLE) {
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
