extern crate gnuplot;
extern crate pathplanning;

use geo::{Coordinate, LineString, Point, Polygon, Rect};
use gnuplot::{Color, Figure};
use pathplanning::rrt::{create_circle, Robot, Space, RRT};

fn main() {
    let mut fg = Figure::new();

    let obstacle_list = vec![
        Rect::new(
            Coordinate { x: 0.0, y: 3.0 },
            Coordinate { x: 1.0, y: 11.0 },
        )
        .into(),
        Rect::new(Coordinate { x: 0.0, y: 3.0 }, Coordinate { x: 9.0, y: 4.8 }).into(),
        Rect::new(
            Coordinate { x: 8.0, y: 3.0 },
            Coordinate { x: 11.0, y: 11.0 },
        )
        .into(),
        // create_circle(Point::new(5.0, 5.0), 1.0),
        // create_circle(Point::new(3.0, 6.0), 2.0),
        // create_circle(Point::new(3.0, 8.0), 2.0),
        // create_circle(Point::new(3.0, 10.0), 2.0),
        // create_circle(Point::new(7.0, 5.0), 2.0),
        // create_circle(Point::new(9.0, 5.0), 2.0),
        // create_circle(Point::new(80.0, 100.0), 10.0),
    ];

    let bounds = Polygon::new(
        LineString::from(vec![
            (-6.0, -6.0),
            (-6.0, 15.0),
            (15.0, 15.0),
            (15.0, -6.0),
            (-6.0, -6.0),
        ]),
        vec![],
    );

    let (bx, by): (Vec<f64>, Vec<f64>) = bounds.exterior().points_iter().map(|p| p.x_y()).unzip();

    let robot = Robot::new(1.0, 1.0, 0.8);
    let space = Space::new(bounds, robot, obstacle_list.clone());

    let buffer_obs = space.get_obs();
    let (bbx, bby): (Vec<f64>, Vec<f64>) = space
        .get_bounds()
        .exterior()
        .points_iter()
        .map(|p| p.x_y())
        .unzip();

    let planner = RRT::new(
        (5.0, 0.0).into(),
        (-45.0_f64).to_radians(),
        (6.0, 10.0).into(),
        45.0_f64.to_radians(),
        8000,
        0.1,
        space,
    );

    let axes = fg.axes2d();
    axes.lines(&bx, &by, &[Color("black")])
        .lines(&bbx, &bby, &[Color("blue")]);

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
