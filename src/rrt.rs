use geo::{LineString, Polygon, Rect, Coordinate, Point};
use geo::algorithm::{intersects::Intersects, contains::Contains, euclidean_distance::EuclideanDistance};
use std::f64::consts::PI;
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

    pub fn create_poly(&self, point: Point<f64>) -> Polygon<f64> {
        let (x, y) = point.x_y();
        Rect::new(
            point,
            Point::new (x + self.width, y + self.height),
        ).into()
    }
}

// Maybe implement a proper circle type?
pub fn create_circle(center: Point<f64>, radius: f64) -> Polygon<f64> {
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
    children: Vec<Box<Node>>,
    cost: f64,
} 

impl Node {
    pub fn add_child(&mut self, node: Box<Node>) {
        self.children.push(node);
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

pub fn create_line(from: Box<Node>, to: Box<Node>) -> LineString<f64> {
    LineString(vec![
        from.get_coord(),
        to.get_coord(),
    ])
}


#[allow(dead_code)]
pub struct RRT {
    start: Coordinate<f64>,
    goal: Coordinate<f64>,
    max_iter: usize,
    goal_sample_rate: usize,
    space: Space,
    nodes: Vec<Box<Node>>, // note: root node is the first item in this vector
}

impl RRT {
    pub fn new(start: Coordinate<f64>, goal: Coordinate<f64>, max_iter: usize, space: Space) -> RRT {
        let root = Box::new(Node { point: start.into(), children: vec![], cost: 0.0});
        let rrt = RRT {
            start,
            goal,
            max_iter,
            goal_sample_rate: 5,
            space,
            nodes: vec![root],
        };

        rrt
    }
    
    // The new_node shouldn't be added to the RRT's `nodes` vec yet
    // a KD-Tree could speed this up
    pub fn get_nearest_node(&mut self, new_node: Box<Node>) -> Box<Node> {
        let new_point = new_node.get_point();
        let result: (Box<Node>, f64) = self.nodes.into_iter()
            .map(|node| {
                let p = node.get_point();
                (node, new_point.euclidean_distance(p))
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).expect("Node lengths(<f64>) should compare."))
            .expect("Should get the closest node");
        result.0
    }

    pub fn get_random_node(&self) -> Box<Node> {

        let mut rng = thread_rng();

        if rng.gen_range(0, 100) > self.goal_sample_rate {

        let point = self.space.rand_point();
        let node = Node { point, children: vec![], cost: 0.0};

        Box::new(node)
        } else {
            Box::new(Node { point: self.goal.into(), children: vec![], cost: 0.0})
        }
    }

    pub fn verify_node(&self, from_node: Box<Node>, to_node: Box<Node>) -> bool {
        let line = create_line(from_node, to_node);
        self.space.verify(&line)
    }

    #[allow(unused)]
    pub fn plan(&mut self) {
        for i in 0..self.max_iter {
            let rnd_node = self.get_random_node();
            let nearest_node = self.get_nearest_node(rnd_node);

            if self.verify_node(nearest_node, rnd_node) {
                self.nodes.push(rnd_node);
                let new_node = self.nodes[self.nodes.len()];

                nearest_node.add_child(new_node);
            }
        }
    }
}
