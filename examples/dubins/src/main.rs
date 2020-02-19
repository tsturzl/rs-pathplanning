extern crate gnuplot;
extern crate pathplanning;

use gnuplot::{Color, Figure};
use pathplanning::dubins::{dubins_path_planning, DubinsConfig};

fn main() {
    let mut fg = Figure::new();
    fg.set_scale(1.0, 1.0);
    let axes = fg.axes2d();

    let start_x = 1.0;
    let start_y = 1.0;
    let start_yaw = 45.0_f64.to_radians();

    let end_x = -3.0;
    let end_y = -3.0;
    let end_yaw = (-45.0_f64).to_radians();

    let turn_radius = 0.5;

    axes.points(vec![start_x], vec![start_y], &[Color("blue")])
        .points(vec![end_x], vec![end_y], &[Color("blue")]);

    let conf1 = DubinsConfig {
        sx: start_x,
        sy: start_y,
        syaw: start_yaw,
        ex: end_x,
        ey: end_y,
        eyaw: end_yaw,
        step_size: 0.01,
        turn_radius,
    };

    let conf2 = DubinsConfig {
        sx: end_x,
        sy: end_y,
        syaw: end_yaw,
        ex: start_x,
        ey: start_y,
        eyaw: start_yaw,
        step_size: 0.01,
        turn_radius,
    };

    println!("Start planner");
    match dubins_path_planning(&conf1) {
        Some((px, py, _, _, _)) => {
            axes.lines(&px, &py, &[Color("red")]);
        }
        None => println!("Could not generate path"),
    }
    match dubins_path_planning(&conf2) {
        Some((px, py, _, _, _)) => {
            axes.lines(&px, &py, &[Color("red")]);
        }
        None => println!("Could not generate path"),
    }
    fg.show().expect("Plotter should plot");
}
