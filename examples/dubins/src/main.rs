extern crate gnuplot;
extern crate pathplanning;

use gnuplot::{Color, Figure};
use pathplanning::dubins::dubins_path_linestring;

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

    println!("Start planner");
    match dubins_path_linestring(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature,
    ) {
        Some(line) => {
            let (px, py): (Vec<f64>, Vec<f64>) = line.points_iter().map(|p| p.x_y()).unzip();
            axes.lines(&px, &py, &[Color("red")]);
        }
        None => println!("Could not generate path"),
    }
    match dubins_path_linestring(
        end_x, end_y, end_yaw, start_x, start_y, start_yaw, curvature,
    ) {
        Some(line) => {
            let (px, py): (Vec<f64>, Vec<f64>) = line.points_iter().map(|p| p.x_y()).unzip();
            axes.lines(&px, &py, &[Color("red")]);
        }
        None => println!("Could not generate path"),
    }
    fg.show().expect("Plotter should plot");
}
