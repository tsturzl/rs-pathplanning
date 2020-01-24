use std::f64::{consts::PI, INFINITY};

pub enum SegmentType {
    LSeg,
    SSeg,
    RSeg,
}

const DIRDATA: [[SegmentType; 3]; 6] = [
    [SegmentType::LSeg, SegmentType::SSeg, SegmentType::LSeg],
    [SegmentType::LSeg, SegmentType::SSeg, SegmentType::RSeg],
    [SegmentType::RSeg, SegmentType::SSeg, SegmentType::LSeg],
    [SegmentType::RSeg, SegmentType::SSeg, SegmentType::RSeg],
    [SegmentType::RSeg, SegmentType::LSeg, SegmentType::RSeg],
    [SegmentType::LSeg, SegmentType::RSeg, SegmentType::LSeg],
];

pub enum ErrorCode {
    EDUBCOCONFIGS,
    EDUBPARAM,
    EDUBBADRHO,
    EDUBNOPATH,
}

fn fmodr(x: f64, y: f64) -> f64 {
    x - y * (x / y).floor()
}

fn mod2pi(theta: f64) -> f64 {
    fmodr(theta, 2.0 * PI)
}

pub struct DubinsIntermediateResults {
    alpha: f64,
    beta: f64,
    d: f64,
    sa: f64,
    sb: f64,
    ca: f64,
    cb: f64,
    c_ab: f64,
    d_sq: f64,
}

impl DubinsIntermediateResults {
    fn new(q0: Config, q1: Config, rho: f64) -> Result<DubinsIntermediateResults, ErrorCode> {
        if rho <= 0.0 {
            return Err(ErrorCode::EDUBBADRHO);
        }

        let dx = q1[0] - q0[0];
        let dy = q1[1] - q0[1];
        let D = (dx * dx + dy * dy).sqrt();
        let d = D / rho;
        let mut theta = 0.0;

        if d > 0.0 {
            theta = mod2pi(dy.atan2(dx));
        }
        let alpha = mod2pi(q0[2] - theta);
        let beta = mod2pi(q1[2] - theta);

        Ok(DubinsIntermediateResults {
            alpha,
            beta,
            d,
            sa: alpha.sin(),
            sb: beta.sin(),
            ca: alpha.cos(),
            cb: beta.cos(),
            c_ab: (alpha - beta).cos(),
            d_sq: d * d,
        })
    }
}

pub enum DubinsPathType {
    LSL,
    LSR,
    RSL,
    RSR,
    RLR,
    LRL,
}

type Config = [f64; 3];

pub struct DubinsPath {
    qi: Config,
    seg_length: Config,
    rho: f64,
    path_type: Option<DubinsPathType>,
}

impl DubinsPath {
    fn new(q0: Config, q1: Config, rho: f64) -> Result<DubinsPath, ErrorCode> {
        let mut best_cost = INFINITY;
        let best_word: isize = -1;

        match DubinsIntermediateResults::new(q0, q1, rho) {
            Ok(interm) => {
                let mut path = DubinsPath {
                    qi: q0,
                    seg_length: [0.0; 3],
                    rho,
                    path_type: None,
                };

                Err(ErrorCode::EDUBNOPATH)
            }
            Err(err_code) => Err(err_code),
        }
    }
}

// Planners(aka words)
const planners: [fn(&DubinsIntermediateResults, &mut [f64; 3]) -> Result<(), ErrorCode>; 6] =
    [dubins_lsl];

fn dubins_lsl(interm: &DubinsIntermediateResults, out: &mut [f64; 3]) -> Result<(), ErrorCode> {
    let tmp0 = interm.d + interm.sa - interm.sb;
    let p_sq = 2.0 + interm.d_sq - (2.0 * interm.c_ab) + (2.0 * interm.d * (interm.sa - interm.sb));

    if p_sq >= 0.0 {
        let tmp1 = (interm.ca - interm.cb).atan2(tmp0);
        out[0] = mod2pi(interm.alpha - tmp1);
        out[1] = p_sq.sqrt();
        out[2] = mod2pi(tmp1 - interm.beta);
        Ok(())
    } else {
        Err(ErrorCode::EDUBNOPATH)
    }
}

fn dubins_rsr(interm: &DubinsIntermediateResults, out &mut [f64; 3]) -> Result<(), ErrorCode> {
    let tmp0 = interm.d - interm.sa + interm.sb;
    let p_sq = 2.0 + interm.d_sq - (2.0 * interm.c_ab) + (2.0 * interm.d * (interm.sb - interm.sa));
    if p_sq >= 0.0 {
        let tmp1 = (interm.ca - interm.cb).atan2(tmp0);
        out[0] = mod2pi(interm.alpha -tmp1);
        out[1] = p_sq.sqrt();
        out[2] = mod2pi(tmp1 - interm.beta);
    } else {
        Err(ErrorCode::EDUBNOPATH)
    }
}
