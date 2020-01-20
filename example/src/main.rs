extern crate gnuplot;
extern crate pathplanning;

use pathplanning::rrt::{Robot, Space, RRT, create_circle};
use geo::{Point, Polygon, LineString, Rect, Coordinate};
use gnuplot::{Figure, Color};

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

    let bounds = Polygon::new(LineString::from(vec![
            (-100.0, -100.0),
            (-100.0, 200.0),
            (200.0, 200.0),
            (200.0, -100.0),
            (-100.0, -100.0),
        ]), vec![]);

    let (bx, by): (Vec<f64>, Vec<f64>) = bounds.exterior().points_iter().map(|p| p.x_y()).unzip();

    let robot = Robot::new(10.0, 10.0);
    let space = Space::new(
        bounds,
        robot,
        obstacle_list.clone()
    );

    let mut planner = RRT::new(
        (0.0, 0.0).into(),
        (60.0, 100.0).into(),
        1000,
        space
    );

    println!("Start planner");
    let path = planner.plan().expect("Planner should find path");

    let (px, py): (Vec<f64>, Vec<f64>) = path.points_iter().map(|p| p.x_y()).unzip();

    let axes = fg.axes2d();
    axes
        .lines(&px, &py, &[Color("green")])
        .lines(&bx, &by, &[Color("black")]);

    for obs in obstacle_list.iter() {
        let (ox, oy): (Vec<f64>, Vec<f64>) = obs.exterior().points_iter().map(|p| p.x_y()).unzip();

        axes.lines(&ox, &oy, &[Color("red")]);
    }

    println!("DONE");
    // fg.show();
}
