extern crate gnuplot;
extern crate pathplanning;

use gnuplot::{Color, Figure};
use pathplanning::dubins::dubins_path_planning;

fn main() {
    let mut fg = Figure::new();

    let start_x = 1.0;
    let start_y = 1.0;
    let start_yaw = 45.0_f64.to_radians();

    let end_x = -3.0;
    let end_y = -3.0;
    let end_yaw = (-45.0_f64).to_radians();

    let curvature = 1.0;

    println!("Start planner");
    match dubins_path_planning(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature,
    ) {
        Some((px, py, _pyaw, _mode, _clen)) => {
            let axes = fg.axes2d();
            axes.lines(&px, &py, &[Color("red")]);

            //fg.show().expect("Plotter should plot");
        }
        None => println!("Could not generate path"),
    }
}
