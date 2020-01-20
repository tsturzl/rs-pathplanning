use geo::{LineString, Polygon, Rect, Coordinate, Point};
use geo::algorithm::{intersects::Intersects, contains::Contains, euclidean_distance::EuclideanDistance};
use std::f64::consts::PI;
use std::rc::Rc;
use rand::{thread_rng, Rng};

pub struct Robot {
    width: f64,
    height: f64,
}

impl Robot{
    pub fn new(width: f64, height: f64) -> Robot {
        Robot {
            width,
            height,
        }
    }

    pub fn create_poly(&self, point: Point<f64>) -> Polygon<f64> {
        let (x, y) = point.x_y();
        Rect::new(
            point,
            Point::new (x + self.width, y + self.height),
        ).into()
    }
}

// Maybe this will create a more elegant way: https://github.com/georust/geo/issues/414
pub fn create_circle(center: Point<f64>, radius: f64) -> Polygon<f64> {
    let (cx, cy) = center.x_y();
    let circum = 2.0 * PI * radius;
    let n = circum.ceil();
    let mut points = Vec::<(f64, f64)>::new();
    for _x in 0..(n + 1.0) as usize {
        let x = _x as f64;
        
        points.push( (((2.0 * PI / n * x).cos() * radius) + cx, ((2.0 * PI / n *x).sin() * radius) + cy) )
    }

    Polygon::new(LineString::from(points), vec![])
}

pub struct Space {
    bounds: Polygon<f64>,
    robot: Robot,
    obstacles: Vec<Polygon<f64>>,
    minx: f64,
    maxx: f64,
    miny: f64,
    maxy: f64,
}

impl Space {
    pub fn new(bounds: Polygon<f64>, robot: Robot, obstacles: Vec<Polygon<f64>>) -> Space {
        let ext = bounds.exterior();
        let points: Vec<Point<f64>> = ext.points_iter().collect();
        let (fx, fy) = points[0].x_y();
        let mut minx = fx;
        let mut maxx = fx;
        let mut miny = fy;
        let mut maxy = fy;

        for point in points {
            let (x, y) = point.x_y();
            if x < minx {
                minx = x;
            }
            if x > maxx {
                maxx = x;
            }

            if y < miny {
                miny = y;
            }
            if y > maxy {
                maxy = y;
            }
        }

        Space {
            bounds,
            robot,
            obstacles,
            minx,
            maxx,
            miny,
            maxy,
        }
    }

    pub fn verify(&self, line: &LineString<f64>) -> bool {
        let last_point = line.points_iter().last().expect("Linestring should have a last point");
        let robot_poly = self.robot.create_poly(last_point);

        if !(self.bounds.contains(line) && self.bounds.contains(&robot_poly)) {
            false
        } else {
            let intersected = self.obstacles.iter()
                .map(|obs| (line.intersects(obs) && robot_poly.intersects(obs)))
                .fold(false, |acc, x| acc && x);
            !intersected
        }
    }

    pub fn rand_point(&self) -> Point<f64> {
        let mut rng = thread_rng();

        let randx: f64 = rng.gen_range(self.minx, self.maxx);
        let randy: f64 = rng.gen_range(self.miny, self.maxy);

        Point::new(randx, randy)
    }
}

pub struct Node {
    point: Point<f64>,
    parent: Option<Rc<Node>>,
} 

impl Node {
    pub fn get_parent(&self) -> Option<Rc<Node>> {
        self.parent.clone()
    }

    pub fn get_point(&self) -> &Point<f64> {
        &self.point
    }

    pub fn get_coord(&self) -> Coordinate<f64> {
        self.point.into()
    }
}

pub fn create_line(from: Rc<Node>, to: Rc<Node>) -> LineString<f64> {
    LineString(vec![
        from.get_coord(),
        to.get_coord(),
    ])
}


pub struct RRT {
    goal: Coordinate<f64>,
    max_iter: usize,
    space: Space,
    nodes: Vec<Rc<Node>>, // note: root node is the first item in this vector
}

impl RRT {
    pub fn new(start: Coordinate<f64>, goal: Coordinate<f64>, max_iter: usize, space: Space) -> RRT {
        let root = Rc::new(Node { point: start.into(), parent: None});
        RRT {
            goal,
            max_iter,
            space,
            nodes: vec![root],
        }
    }
    
    // a KD-Tree could speed this up
    pub fn get_nearest_node(&self, point: &Point<f64>) -> Rc<Node> {
        let result: (Rc<Node>, f64) = self.nodes.iter()
            .map(|node| {
                (node.clone(), point.euclidean_distance(node.get_point()))
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).expect("Node lengths(<f64>) should compare."))
            .expect("Should get the closest node");
        result.0.clone()
    }

    pub fn get_random_node(&self) -> Rc<Node> {
        let point = self.space.rand_point();
        let nearest_node = self.get_nearest_node(&point);

        Rc::new(Node { point, parent: Some(nearest_node.clone())})
    }

    pub fn verify_node(&self, node: Rc<Node>) -> bool {
        match node.clone().get_parent() {
            Some(parent) => {
                let line = create_line(parent, node.clone());
                self.space.verify(&line)
            },
            None => false,
        }
    }

    pub fn check_finish(&self, node: Rc<Node>) -> Option<Rc<Node>> {
        let goal_node = Rc::new(Node { point: self.goal.into(), parent: Some(node.clone())});

        if self.verify_node(goal_node.clone()) {
            Some(goal_node.clone())
        } else {
            None
        }
    }
    
    pub fn finalize(&self, goal_node: Rc<Node>) -> LineString<f64> {
        let mut points = vec![];
        points.push(goal_node.clone().get_coord());

        let mut parent = goal_node.get_parent();

        while let Some(next_parent) = parent.clone() {
            points.push(next_parent.get_coord());
            parent = next_parent.get_parent();
        }

        points.reverse();

        LineString(points)
    }

    pub fn plan(&mut self) -> Option<LineString<f64>> {
        for _i in 0..self.max_iter {
            let rnd_node = self.get_random_node();

            if self.verify_node(rnd_node.clone()) {
                self.nodes.push(rnd_node.clone());

                if let Some(goal_node) = self.check_finish(rnd_node.clone()) {
                    return Some(self.finalize(goal_node.clone()))
                }
            }
        }

        None
    }
}
