extern crate gnuplot;
extern crate pathplanning;

use pathplanning::rrt::{Robot, Space, RRT, create_circle};
use geo::{Point, Polygon, LineString};
use gnuplot::{Figure, Color};

fn main() {

    let mut fg = Figure::new();

    let obstacle_list = vec![
        create_circle(Point::new(50.0, 50.0), 10.0),
        create_circle(Point::new(30.0, 60.0), 20.0),
        create_circle(Point::new(30.0, 80.0), 20.0),
        create_circle(Point::new(30.0, 100.0), 20.0),
        create_circle(Point::new(70.0, 50.0), 20.0),
        create_circle(Point::new(90.0, 50.0), 20.0),
        create_circle(Point::new(80.0, 100.0), 10.0),
    ];

    let bounds = Polygon::new(LineString::from(vec![
            (-250.0, -250.0),
            (-250.0, 500.0),
            (500.0, 500.0),
            (500.0, -250.0),
            (-250.0, -250.0),
        ]), vec![]);

    let (bx, by): (Vec<f64>, Vec<f64>) = bounds.exterior().points_iter().map(|p| p.x_y()).unzip();

    let robot = Robot::new(10.0, 5.0);
    let space = Space::new(
        bounds,
        robot,
        obstacle_list.clone()
    );

    let mut planner = RRT::new(
        (0.0, 0.0).into(),
        (60.0, 100.0).into(),
        100,
        space
    );

    println!("Start planner");
    let path = planner.plan().expect("Planner should find path");

    let (px, py): (Vec<f64>, Vec<f64>) = path.points_iter().map(|p| p.x_y()).unzip();

    let axes = fg.axes2d();
    axes.lines(&px, &py, &[Color("red")]).lines(&bx, &by, &[Color("black")]);

    for obs in obstacle_list.iter() {
        let (ox, oy): (Vec<f64>, Vec<f64>) = obs.exterior().points_iter().map(|p| p.x_y()).unzip();

        axes.lines(&ox, &oy, &[Color("black")]);
    }

    fg.show();
}
