extern crate matplotrust;
extern crate pathplanning;

use pathplanning::rrt::{Robot, Space, RRT, create_circle};
use geo::{Point, Polygon, LineString};
use matplotrust::*;

fn main() {
    let mut figure = Figure::new();

    let obstacle_list = vec![
        create_circle(Point::new(5.0, 5.0), 1.0),
        create_circle(Point::new(3.0, 6.0), 2.0),
        create_circle(Point::new(3.0, 8.0), 2.0),
        create_circle(Point::new(3.0, 10.0), 2.0),
        create_circle(Point::new(7.0, 5.0), 2.0),
        create_circle(Point::new(9.0, 5.0), 2.0),
        create_circle(Point::new(8.0, 10.0), 1.0),
    ];

    let robot = Robot::new(2.0, 4.0);
    let space = Space::new(
        Polygon::new(LineString::from(vec![
            (-25.0, -25.0),
            (-25.0, 50.0),
            (50.0, 50.0),
            (50.0, -25.0),
            (-25.0, -25.0),
        ]), vec![]),
        robot,
        obstacle_list.clone()
    );

    let mut planner = RRT::new(
        (0.0, 0.0).into(),
        (6.0, 10.0).into(),
        200,
        space
    );

    println!("Start planner");
    let path = planner.plan().expect("Planner should find path");

    let (px, py) = path.points_iter().map(|p| p.x_y()).unzip();

    let path_plot = line_plot::<f64, f64>(px, py, None);
    figure.add_plot(path_plot.clone());
    
    obstacle_list.iter()
        .map(|p| p.exterior().points_iter().map(|p| p.x_y()).unzip())
        .map(|o| line_plot::<f64, f64>(o.0, o.1, None))
        .for_each(|l| figure.add_plot(l.clone()));

    print!("{:?}", figure.save("./plot.png", None));
    
    println!("DONE!!");
}
