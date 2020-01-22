use geo::algorithm::{
    contains::Contains, euclidean_distance::EuclideanDistance, euclidean_length::EuclideanLength,
    intersects::Intersects,
};
use geo::{Coordinate, LineString, MultiPolygon, Point, Polygon};
use geo_offset::Offset;
use rand::{rngs::ThreadRng, thread_rng, Rng};
use std::f64::consts::PI;
use std::rc::Rc;

#[allow(dead_code)]
pub struct Robot {
    width: f64,
    height: f64,
}

impl Robot {
    pub fn new(width: f64, height: f64) -> Robot {
        Robot { width, height }
    }

    pub fn get_width(&self) -> f64 {
        self.width
    }
}

// Maybe this will create a more elegant way: https://github.com/georust/geo/issues/414
pub fn create_circle(center: Point<f64>, radius: f64) -> Polygon<f64> {
    let (cx, cy) = center.x_y();
    let circum = 2.0 * PI * radius;
    let n = (circum / 10.0).ceil();
    let mut points = Vec::<(f64, f64)>::new();
    for _x in 0..(n + 1.0) as usize {
        let x = _x as f64;

        points.push((
            ((2.0 * PI / n * x).cos() * radius) + cx,
            ((2.0 * PI / n * x).sin() * radius) + cy,
        ))
    }

    Polygon::new(LineString::from(points), vec![])
}

fn buffer_poly(poly: &Polygon<f64>, buffer: f64) -> MultiPolygon<f64> {
    poly.offset(buffer).expect("polygon to set a buffer")
}

pub struct Space {
    bounds: Polygon<f64>,
    // robot: Robot,
    obstacles: Vec<Polygon<f64>>,
    rng: ThreadRng,
    minx: f64,
    maxx: f64,
    miny: f64,
    maxy: f64,
}

impl Space {
    pub fn new(bounds: Polygon<f64>, robot: Robot, obstacle_list: Vec<Polygon<f64>>) -> Space {
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

        let width = robot.get_width() / 2.0;

        let obstacles = obstacle_list
            .iter()
            .map(|o| buffer_poly(o, width))
            .map(|p| -> Vec<Polygon<f64>> { p.into_iter().collect() })
            .flatten()
            .collect();

        Space {
            rng: thread_rng(),
            bounds,
            // robot,
            obstacles,
            minx,
            maxx,
            miny,
            maxy,
        }
    }

    //pub fn create_buffered_line(&self, from: Rc<Node>, to: Rc<Node>) -> Polygon<f64> {}

    pub fn verify(&self, line: &LineString<f64>) -> bool {
        if !self.bounds.contains(line) {
            // println!("Verify: Out of bounds");
            false
        } else {
            let intersected = self
                .obstacles
                .iter()
                .map(|obs| line.intersects(obs))
                .any(|n| n);
            // println!("Verify result:{:?}", intersected);
            !intersected
        }
    }

    pub fn rand_point(&mut self) -> Point<f64> {
        let randx: f64 = self.rng.gen_range(self.minx, self.maxx);
        let randy: f64 = self.rng.gen_range(self.miny, self.maxy);

        Point::new(randx, randy)
    }

    pub fn get_obs(&self) -> Vec<Polygon<f64>> {
        self.obstacles.clone()
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
    LineString(vec![from.get_coord(), to.get_coord()])
}

pub struct RRT {
    goal: Coordinate<f64>,
    max_iter: usize,
    max_dist: f64,
    space: Space,
    nodes: Vec<Rc<Node>>, // note: root node is the first item in this vector
}

impl RRT {
    pub fn new(
        start: Coordinate<f64>,
        goal: Coordinate<f64>,
        max_iter: usize,
        space: Space,
    ) -> RRT {
        let root = Rc::new(Node {
            point: start.into(),
            parent: None,
        });
        RRT {
            goal,
            max_iter,
            max_dist: 1000.0,
            space,
            nodes: vec![root],
        }
    }

    // a KD-Tree could speed this up
    pub fn get_nearest_node(&self, point: &Point<f64>) -> Option<(Rc<Node>, f64)> {
        self.nodes
            .iter()
            .filter_map(|node| {
                let dist = point.euclidean_distance(node.get_point());

                if dist <= self.max_dist {
                    Some((node.clone(), dist))
                } else {
                    None
                }
            })
            .min_by(|a, b| {
                a.1.partial_cmp(&b.1)
                    .expect("Node lengths(<f64>) should compare.")
            })
    }

    pub fn get_random_node(&mut self) -> Option<Rc<Node>> {
        let point = self.space.rand_point();
        match self.get_nearest_node(&point) {
            Some((nearest_node, dist)) => {
                if dist <= self.max_dist {
                    Some(Rc::new(Node {
                        point,
                        parent: Some(nearest_node),
                    }))
                } else {
                    None
                }
            }
            None => None,
        }
    }

    pub fn verify_node(&self, node: Rc<Node>) -> bool {
        match node.clone().get_parent() {
            Some(parent) => {
                let line = create_line(parent, node);

                line.euclidean_length() <= self.max_dist && self.space.verify(&line)
            }
            None => false,
        }
    }

    pub fn check_finish(&self, node: Rc<Node>) -> Option<Rc<Node>> {
        let point = self.goal.into();
        let goal_node = Rc::new(Node {
            point,
            parent: Some(node),
        });

        if self.verify_node(goal_node.clone()) {
            Some(goal_node)
        } else {
            None
        }
    }

    pub fn optimize(&self, nodes: Vec<Rc<Node>>) -> LineString<f64> {
        let mut optimized = vec![];

        let len = nodes.len() - 1;
        for i in 0..len {
            let line = create_line(nodes[i].clone(), nodes[len].clone());
            optimized.push(nodes[i].get_coord());
            if self.space.verify(&line) {
                optimized.push(nodes[len].get_coord());
                break;
            }
        }

        optimized.reverse();

        println!("optimized: {:?}", nodes.len() - optimized.len());

        LineString(optimized)
    }

    pub fn finalize(&self, goal_node: Rc<Node>) -> LineString<f64> {
        let mut nodes = vec![];
        nodes.push(goal_node.clone());

        let mut parent = goal_node.get_parent();

        while let Some(next_parent) = parent.clone() {
            nodes.push(next_parent.clone());
            parent = next_parent.get_parent();
        }

        self.optimize(nodes)
    }

    pub fn old_finalize(&self, goal_node: Rc<Node>) -> LineString<f64> {
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
        let mut results: Vec<LineString<f64>> = vec![];
        let mut unopt_results: Vec<LineString<f64>> = vec![];
        for _i in 0..self.max_iter {
            // println!("iter: {:?}", _i);
            if let Some(rnd_node) = self.get_random_node() {
                if self.verify_node(rnd_node.clone()) {
                    self.nodes.push(rnd_node.clone());

                    if let Some(goal_node) = self.check_finish(rnd_node.clone()) {
                        results.push(self.finalize(goal_node.clone()));
                        unopt_results.push(self.old_finalize(goal_node.clone()));
                    }
                }
            }
        }

        // println!("Paths: {:?}", results.len());
        if results.is_empty() {
            None
        } else {
            results.into_iter().min_by(|a, b| {
                let a_len = a.euclidean_length();
                let b_len = b.euclidean_length();
                a_len
                    .partial_cmp(&b_len)
                    .expect("Compare length of results")
            })
        }
    }
}
