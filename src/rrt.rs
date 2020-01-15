use geo::{LineString, Polygon, Rect, Coordinate, Point};
use geo::algorithm::{intersects::Intersects, contains::Contains, euclidean_distance::EuclideanDistance};
use std::f64::consts::PI;
use std::sync::Arc;
use rand::{thread_rng, Rng};

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

    pub fn create_poly(&self, point: Coordinate<f64>) -> Polygon<f64> {
        let (x, y) = point.x_y();
        Rect::new(
            point,
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
        let ext = self.bounds.exterior();
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

        let mut rng = thread_rng();

        let randx: f64 = rng.gen_range(minx, maxx);
        let randy: f64 = rng.gen_range(miny, maxy);

        Point::new(randx, randy)
    }
}

pub struct Node {
    point: Point<f64>,
    children: Vec<Arc<Node>>,
    cost: f64,
} 

impl Node {
    pub fn add_child(&mut self, node: Arc<Node>) {
        self.children.push(node.clone());
    }

    pub fn get_point(&self) -> &Point<f64> {
        &self.point
    }

    pub fn get_coord(&self) -> Coordinate<f64> {
        self.point.into()
    }


    pub fn get_cost(&self) -> f64 {
        self.cost
    }

    pub fn set_cost(&mut self, cost: f64) {
        self.cost = cost;
    }
}

pub fn create_line(from: &Node, to: &Node) -> LineString<f64> {
    LineString(vec![
        from.get_coord(),
        to.get_coord(),
    ])
}


#[allow(dead_code)]
pub struct RRT {
    start: Coordinate<f64>,
    goal: Coordinate<f64>,
    space: Space,
    root: Arc<Node>,
    nodes: Vec<Arc<Node>>,
}

impl RRT {
    pub fn new(start: Coordinate<f64>, goal: Coordinate<f64>, space: Space) -> RRT {
        let root = Arc::new(Node { point: start.into(), children: vec![], cost: 0.0});
        let mut rrt = RRT {
            start,
            goal,
            space,
            root: root.clone(),
            nodes: vec![],
        };

        rrt.nodes.push(rrt.root.clone());

        rrt
    }
    
    // The new_node shouldn't be added to the RRT's `nodes` vec yet
    // a KD-Tree could speed this up
    pub fn get_nearest_node(&self, new_node: &Node) -> Arc<Node> {
        let new_point = new_node.get_point();
        let result: (Arc<Node>, f64) = self.nodes.iter()
            .map(|_node| {
                let node = _node.clone();
                let p = node.get_point();
                (node.clone(), new_point.euclidean_distance(p))
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).expect("Node lengths(<f64>) should compare."))
            .expect("Should get the closest node");
        result.0
    }

    pub fn create_node(&self) -> Node {
        let point = self.space.rand_point();
        let node = Node { point, children: vec![], cost: 0.0};

        node
    }
}
