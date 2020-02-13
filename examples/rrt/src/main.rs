extern crate gnuplot;
extern crate pathplanning;

use geo::{Coordinate, LineString, Point, Polygon, Rect};
use gnuplot::{Color, Figure};
use pathplanning::rrt::{create_circle, Robot, Space, RRT};

fn main() {
    let mut fg = Figure::new();

    let obstacle_list = vec![
        // Rect::new(
        //     Coordinate { x: 0.0, y: 3.0 },
        //     Coordinate { x: 1.0, y: 11.0 },
        // )
        // .into(),
        // Rect::new(Coordinate { x: 0.0, y: 3.0 }, Coordinate { x: 9.0, y: 4.8 }).into(),
        // Rect::new(
        //     Coordinate { x: 8.0, y: 3.0 },
        //     Coordinate { x: 11.0, y: 11.0 },
        // )
        // .into(),
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
            (-12.9812903405, -5.3352307953),
            (-14.8101818991, -36.9866127437),
            (-2.1226197326, -40.2883956985),
            (8.1645969508, -39.491414008),
            (9.193307710300001, -30.610791779299998),
            (2.5637750657, -27.992153711300002),
            (2.220866071, -20.2500852688),
            (10.907825213999999, -19.3392469529),
            (18.108857109200002, -19.2253793374),
            (28.396036237399997, -16.8344165342),
            (33.5396007061, -10.5724371445),
            (36.7400181034, -2.8303720359),
            (31.9393221802, 1.6098962018),
            (32.5108029928, 8.2134021791),
            (35.8255396833, 10.718192783300001),
            (36.3969428579, 32.1226069065),
            (49.0843907668, 31.667265667800002),
            (59.3715045293, 32.2366046315),
            (72.51616242760001, 31.4397473243),
            (73.0875480373, 43.8497302878),
            (48.2842106667, 42.5971493499),
            (26.9098974127, 42.36933878279999),
            (11.2506460775, 39.1814246752),
            (12.5080204024, 1.6098392684),
            (12.1651258205, -5.4490852279),
            (-12.9812903405, -5.3352307953),
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
        (5.0, -35.0).into(),
        0.0_f64.to_radians(),
        (70.0, 40.0).into(),
        180.0_f64.to_radians(),
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
