extern crate gnuplot;
extern crate pathplanning;

use gnuplot::{Color, Figure};
use pathplanning::dubins::{dubins_path_linestring, DubinsConfig};

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

    let curvature = 1.0;

    axes.points(vec![start_x], vec![start_y], &[Color("blue")])
        .points(vec![end_x], vec![end_y], &[Color("blue")]);

    let conf1 = DubinsConfig {
        sx: start_x,
        sy: start_y,
        syaw: start_yaw,
        ex: end_x,
        ey: end_y,
        eyaw: end_yaw,
        c: curvature,
    };

    let conf2 = DubinsConfig {
        sx: end_x,
        sy: end_y,
        syaw: end_yaw,
        ex: start_x,
        ey: start_y,
        eyaw: start_yaw,
        c: curvature,
    };

    println!("Start planner");
    match dubins_path_linestring(&conf1) {
        Some(line) => {
            let (px, py): (Vec<f64>, Vec<f64>) = line.points_iter().map(|p| p.x_y()).unzip();
            axes.lines(&px, &py, &[Color("red")]);
        }
        None => println!("Could not generate path"),
    }
    match dubins_path_linestring(&conf2) {
        Some(line) => {
            let (px, py): (Vec<f64>, Vec<f64>) = line.points_iter().map(|p| p.x_y()).unzip();
            axes.lines(&px, &py, &[Color("red")]);
        }
        None => println!("Could not generate path"),
    }
    fg.show().expect("Plotter should plot");
}
