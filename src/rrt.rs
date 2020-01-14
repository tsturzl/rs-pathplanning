use geo::{LineString, Polygon, Rect, Coordinate};
use geo::algorithm::{intersects::Intersects, contains::Contains};
use std::f64::consts::PI;

pub struct Robot {
    width: f64,
    height: f64,
    turning_radius: f64,
}

impl Robot{
    pub fn new(width: f64, height: f64, turning_radius: f64) -> Robot {
        Robot {
            width,
            height,
            turning_radius,
        }
    }

    pub fn turning_radius(&self) -> f64 {
        self.turning_radius
    }

    pub fn create_poly(&self, coord: Coordinate<f64>) -> Polygon<f64> {
        let (x, y) = coord.x_y();
        Rect::new(
            coord,
            Coordinate::<f64> { x: x + self.width, y: y + self.height},
        ).into()
    }
}

// Maybe implement a proper circle type?
pub fn create_circle(center: Coordinate<f64>, radius: f64) -> Polygon<f64> {
    let (cx, cy) = center.x_y();
    let circum = 2.0 * PI * radius;
    let n = (circum / 10.0).ceil();
    let mut points = Vec::<(f64, f64)>::new();
    for _x in 0..n as usize + 1 {
        let x = _x as f64;
        
        points.push( (((2.0 * PI / n * x).cos() * radius) + cx, ((2.0 * PI / n *x).sin() * radius) + cy) )
    }

    Polygon::new(LineString::from(points), vec![])
}

pub struct Space {
    bounds: Polygon<f64>,
    robot: Robot,
    obstacles: Vec<Polygon<f64>>,
}

impl Space {
    pub fn new(bounds: Polygon<f64>, robot: Robot, obstacles: Vec<Polygon<f64>>) -> Space {
        Space {
            bounds,
            robot,
            obstacles,
        }
    }

    pub fn verify(&self, line: &LineString<f64>) -> bool {
        let line_points = line.to_owned().into_points();
        let last_point = line_points[line_points.len()];
        let robot_poly = self.robot.create_poly(last_point.into());

        if (self.bounds.contains(line) && self.bounds.contains(&robot_poly)) == false {
            false
        } else {
            let intersected = self.obstacles.iter()
                .map(|obs| (line.intersects(obs) && robot_poly.intersects(obs)))
                .fold(false, |acc, x| acc && x);
            !intersected
        }
    }
}

pub struct Node<'b> {
    coord: Coordinate<f64>,
    heading: f64,
    children: Vec<&'b Node<'b>>,
} 

pub trait DrawLine {
    fn new(node: &Node) -> Self;
    fn draw(&self) -> &LineString<f64>;
}

pub struct RRT<'a, T: DrawLine> {
    start: (Coordinate<f64>, f64),
    goal: (Coordinate<f64>, f64),
    space: Space,
    root: Node<'a>,
    line_type: T,
}

impl<'a, T: DrawLine> RRT<'a, T> {
    pub fn new(&self, start: (Coordinate<f64>, f64), goal: (Coordinate<f64>, f64), space: Space, line_type: T) -> RRT<'a, T> {
        RRT::<'a, T> {
            start,
            goal,
            space,
            line_type,
            root: Node::<'a> { coord: start.0, heading: start.1, children: vec![]},
        }
    }
}
