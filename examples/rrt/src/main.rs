extern crate gnuplot;
extern crate pathplanning;

use geo::{Coordinate, LineString, Point, Polygon, Rect};
use gnuplot::{Color, Figure};
use pathplanning::rrt::{create_circle, Robot, Space, RRT};

fn main() {
    let mut fg = Figure::new();

    let obstacle_list = vec![
        // Rect::new(Coordinate{x: 0.0, y: 30.0}, Coordinate{x: 10.0, y: 110.0}).into(),
        // Rect::new(Coordinate{x: 0.0, y: 30.0}, Coordinate{x: 90.0, y: 48.0}).into(),
        // Rect::new(Coordinate{x: 80.0, y: 30.0}, Coordinate{x: 110.0, y: 113.0}).into(),
        create_circle(Point::new(50.0, 50.0), 10.0),
        create_circle(Point::new(30.0, 60.0), 20.0),
        create_circle(Point::new(30.0, 80.0), 20.0),
        create_circle(Point::new(30.0, 100.0), 20.0),
        create_circle(Point::new(70.0, 50.0), 20.0),
        create_circle(Point::new(90.0, 50.0), 20.0),
        create_circle(Point::new(80.0, 100.0), 10.0),
    ];

    let bounds = Polygon::new(
        LineString::from(vec![
            (-100.0, -100.0),
            (-100.0, 200.0),
            (200.0, 200.0),
            (200.0, -100.0),
            (-100.0, -100.0),
        ]),
        vec![],
    );

    let (bx, by): (Vec<f64>, Vec<f64>) = bounds.exterior().points_iter().map(|p| p.x_y()).unzip();

    let robot = Robot::new(10.0, 10.0);
    let space = Space::new(bounds, robot, obstacle_list.clone());

    let buffer_obs = space.get_obs();

    let mut planner = RRT::new((-50.0, -50.0).into(), (60.0, 100.0).into(), 5000, space);

    let axes = fg.axes2d();
    axes.lines(&bx, &by, &[Color("black")]);

    println!("Start planner");
    match planner.plan() {
        Some(path) => {
            println!("Path generated!");
            let (px, py): (Vec<f64>, Vec<f64>) = path.points_iter().map(|p| p.x_y()).unzip();
            axes.lines(&px, &py, &[Color("green")]);
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
