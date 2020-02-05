use criterion::{black_box, criterion_group, criterion_main, Criterion};
use geo::{LineString, Point, Polygon};
use pathplanning::{dubins, rrt};

fn bench_plan_one(c: &mut Criterion) {
    c.bench_function("RRT::plan_one", |b| {
        let obstacle_list = vec![
            // Rect::new(Coordinate{x: 0.0, y: 30.0}, Coordinate{x: 10.0, y: 110.0}).into(),
            // Rect::new(Coordinate{x: 0.0, y: 30.0}, Coordinate{x: 90.0, y: 48.0}).into(),
            // Rect::new(Coordinate{x: 80.0, y: 30.0}, Coordinate{x: 110.0, y: 113.0}).into(),
            rrt::create_circle(Point::new(5.0, 5.0), 1.0),
            rrt::create_circle(Point::new(3.0, 6.0), 2.0),
            rrt::create_circle(Point::new(3.0, 8.0), 2.0),
            rrt::create_circle(Point::new(3.0, 10.0), 2.0),
            rrt::create_circle(Point::new(7.0, 5.0), 2.0),
            rrt::create_circle(Point::new(9.0, 5.0), 2.0),
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
        let robot = rrt::Robot::new(1.0, 1.0, 0.8);
        let space = rrt::Space::new(bounds, robot, obstacle_list.clone());

        let planner = rrt::RRT::new(
            (-5.0, -5.0).into(),
            (-45.0_f64).to_radians(),
            (6.0, 10.0).into(),
            45.0_f64.to_radians(),
            8000,
            0.1,
            space,
        );

        b.iter(|| planner.plan_one());
    });
}

fn bench_plan_10(c: &mut Criterion) {
    c.bench_function("RRT::plan_one", |b| {
        let obstacle_list = vec![
            // Rect::new(Coordinate{x: 0.0, y: 30.0}, Coordinate{x: 10.0, y: 110.0}).into(),
            // Rect::new(Coordinate{x: 0.0, y: 30.0}, Coordinate{x: 90.0, y: 48.0}).into(),
            // Rect::new(Coordinate{x: 80.0, y: 30.0}, Coordinate{x: 110.0, y: 113.0}).into(),
            rrt::create_circle(Point::new(5.0, 5.0), 1.0),
            rrt::create_circle(Point::new(3.0, 6.0), 2.0),
            rrt::create_circle(Point::new(3.0, 8.0), 2.0),
            rrt::create_circle(Point::new(3.0, 10.0), 2.0),
            rrt::create_circle(Point::new(7.0, 5.0), 2.0),
            rrt::create_circle(Point::new(9.0, 5.0), 2.0),
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
        let robot = rrt::Robot::new(1.0, 1.0, 0.8);
        let space = rrt::Space::new(bounds, robot, obstacle_list.clone());

        let planner = rrt::RRT::new(
            (-5.0, -5.0).into(),
            (-45.0_f64).to_radians(),
            (6.0, 10.0).into(),
            45.0_f64.to_radians(),
            10,
            0.1,
            space,
        );

        b.iter(|| planner.plan());
    });
}

fn bench_dubins(c: &mut Criterion) {
    c.bench_function("Dubins::dubins_path_planning", |b| {
        let conf = dubins::DubinsConfig {
            sx: 1.0,
            sy: 1.0,
            syaw: 45.0_f64.to_radians(),
            ex: -3.0,
            ey: -3.0,
            eyaw: (-45.0_f64).to_radians(),
            c: 1.0,
            step_size: 0.1,
        };

        b.iter(|| dubins::dubins_path_planning(&conf))
    });
}

criterion_group!(benches, bench_plan_one, bench_plan_10, bench_dubins);
criterion_main!(benches);
