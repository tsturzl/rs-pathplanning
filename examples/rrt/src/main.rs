extern crate gnuplot;
extern crate pathplanning;
extern crate serde;

use geo::{Coordinate, LineString, Point, Polygon, Rect};
use gnuplot::{Color, Figure};
use pathplanning::rrt::{create_circle, Robot, Space, RRT};
use serde::{Deserialize, Serialize};
use std::env;
use std::fs::File;
use std::io::prelude::*;

#[derive(Serialize, Deserialize)]
struct Conf {
    bounds: Vec<(f64, f64)>,
    obstacles: Vec<Vec<(f64, f64)>>,
    path: Vec<(f64, f64)>,
    start: (f64, f64, f64),
    goal: (f64, f64, f64),
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let file_path = &args[1];
    let mut file = File::open(file_path).expect("file should open");
    let mut contents = String::new();
    file.read_to_string(&mut contents)
        .expect("should read file");

    let conf: Conf = serde_json::from_str(&contents).expect("should deserialize");

    let mut fg = Figure::new();

    let obstacle_list: Vec<Polygon<f64>> = conf
        .obstacles
        .into_iter()
        .map(|o| Polygon::new(LineString::from(o), vec![]))
        .collect();

    let bounds = Polygon::new(LineString::from(conf.bounds), vec![]);

    let (bx, by): (Vec<f64>, Vec<f64>) = bounds.exterior().points_iter().map(|p| p.x_y()).unzip();

    let robot = Robot::new(1.8, 3.0, 0.8);
    let space = Space::new(bounds, robot, obstacle_list.clone());

    let buffer_obs = space.get_obs();
    let (bbx, bby): (Vec<f64>, Vec<f64>) = space
        .get_bounds()
        .exterior()
        .points_iter()
        .map(|p| p.x_y())
        .unzip();

    let planner = RRT::new(
        (conf.start.0, conf.start.1).into(),
        conf.start.2,
        (conf.goal.0, conf.goal.1).into(),
        conf.goal.2,
        8000,
        0.1,
        space,
    );

    let axes = fg.axes2d();
    axes.lines(&bx, &by, &[Color("black")])
        .lines(&bbx, &bby, &[Color("blue")])
        .points(
            &[conf.start.0, conf.goal.0],
            &[conf.start.1, conf.goal.1],
            &[Color("green")],
        );

    println!("Start planner");
    match planner.plan() {
        Some(path) => {
            println!("Path generated!");
            let (px, py): (Vec<f64>, Vec<f64>) = path.points_iter().map(|p| p.x_y()).unzip();
            axes.lines(&px, &py, &[Color("green")]);
            println!("Num points: {:?}", px.len());
        }
        None => println!("Unable to generate path"),
    }

    for obs in obstacle_list.iter() {
        let (ox, oy): (Vec<f64>, Vec<f64>) = obs.exterior().points_iter().map(|p| p.x_y()).unzip();
        axes.lines(&ox, &oy, &[Color("red")]);
    }

    for obs in buffer_obs.iter() {
        let (ox, oy): (Vec<f64>, Vec<f64>) = obs.exterior().points_iter().map(|p| p.x_y()).unzip();
        axes.lines(&ox, &oy, &[Color("blue")]);
    }

    fg.show();
}
