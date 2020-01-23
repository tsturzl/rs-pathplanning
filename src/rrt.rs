use crate::dubins::{dubins_path_linestring, dubins_path_planning};
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
    let n = (circum / 1.0).ceil();
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
            bounds: buffer_poly(&bounds, -width)
                .into_iter()
                .last()
                .expect("should provide dialated polygon"),
            // robot,
            obstacles,
            minx,
            maxx,
            miny,
            maxy,
        }
    }

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

    pub fn get_bounds(&self) -> Polygon<f64> {
        self.bounds.clone()
    }
}

pub struct Node {
    point: Point<f64>,
    parent: Option<Rc<Node>>,
    // angle from self to parent, or the heading angle(eg the start point)
    yaw: f64,
}

impl Node {
    pub fn new(point: Point<f64>, parent: Rc<Node>) -> Node {
        Node {
            point,
            parent: Some(parent.clone()),
            yaw: compute_yaw(&point, parent.get_point()),
        }
    }

    pub fn new_root(point: Point<f64>, yaw: f64) -> Node {
        Node {
            point,
            parent: None,
            yaw,
        }
    }

    pub fn new_goal(point: Point<f64>, parent: Rc<Node>, yaw: f64) -> Node {
        Node {
            point,
            parent: Some(parent),
            yaw,
        }
    }

    pub fn get_parent(&self) -> Option<Rc<Node>> {
        self.parent.clone()
    }

    pub fn get_above(&self) -> NodeIter {
        NodeIter {
            curr: self.parent.clone(),
        }
    }

    pub fn get_point(&self) -> &Point<f64> {
        &self.point
    }

    pub fn get_coord(&self) -> Coordinate<f64> {
        self.point.into()
    }

    pub fn get_yaw(&self) -> f64 {
        self.yaw
    }
}

// Iterate nodes back to root
pub struct NodeIter {
    curr: Option<Rc<Node>>,
}

impl Iterator for NodeIter {
    type Item = Rc<Node>;

    fn next(&mut self) -> Option<Rc<Node>> {
        match self.curr.clone() {
            Some(current) => {
                self.curr = current.get_parent();
                Some(current)
            }
            None => None,
        }
    }
}

fn compute_yaw(from: &Point<f64>, to: &Point<f64>) -> f64 {
    let (fx, fy) = from.x_y();
    let (tx, ty) = to.x_y();
    (ty - fy).atan2(tx - fx)
}

pub fn create_line(from: Rc<Node>, to: Rc<Node>) -> LineString<f64> {
    let (sx, sy) = from.get_point().x_y();
    let syaw = from.get_yaw();
    let (ex, ey) = to.get_point().x_y();
    let eyaw = to.get_yaw();
    dubins_path_linestring(sx, sy, syaw, ex, ey, eyaw, 1.0).expect("Should generate a dubins line")
}

pub fn line_to_origin(node: Rc<Node>) -> LineString<f64> {
    let node_iter = NodeIter { curr: Some(node) };
    node_iter
        .map(|node| match node.get_parent() {
            Some(parent) => {
                let (sx, sy) = node.get_coord().x_y();
                let syaw = node.get_yaw();
                let (ex, ey) = parent.get_coord().x_y();
                let eyaw = parent.get_yaw();

                match dubins_path_planning(sx, sy, syaw, ex, ey, eyaw, 0.8) {
                    Some((px, py, _, _, _)) => px.into_iter().zip(py.into_iter()).collect(),
                    _ => vec![(sx, sy)],
                }
            }
            None => vec![node.get_coord().x_y()],
        })
        .flatten()
        .collect()
}

pub struct RRT {
    goal: Coordinate<f64>,
    goal_yaw: f64,
    max_iter: usize,
    max_dist: f64,
    space: Space,
    nodes: Vec<Rc<Node>>, // note: root node is the first item in this vector
}

impl RRT {
    pub fn new(
        start: Coordinate<f64>,
        start_yaw: f64,
        goal: Coordinate<f64>,
        goal_yaw: f64,
        max_iter: usize,
        space: Space,
    ) -> RRT {
        let root = Rc::new(Node::new_root(start.into(), start_yaw));
        RRT {
            goal,
            goal_yaw,
            max_iter,
            max_dist: std::f64::INFINITY,
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

                if dist <= self.max_dist && dist >= 2.0 {
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
                    Some(Rc::new(Node::new(point, nearest_node)))
                } else {
                    None
                }
            }
            None => None,
        }
    }

    pub fn verify_node(&self, node: Rc<Node>) -> bool {
        // match node.clone().get_parent() {
        //     Some(parent) => {
        //         let line = create_line(parent, node);

        //         line.euclidean_length() <= self.max_dist && self.space.verify(&line)
        //     }
        //     None => false,
        // }

        let line = line_to_origin(node);
        self.space.verify(&line)
    }

    pub fn check_finish(&self, node: Rc<Node>) -> Option<LineString<f64>> {
        let point = self.goal.into();
        let goal_node = Rc::new(Node::new_goal(point, node, self.goal_yaw));
        let line = self.finalize(goal_node);

        if self.space.verify(&line) {
            Some(line)
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

        // println!("optimized: {:?}", nodes.len() - optimized.len());

        LineString(optimized)
    }

    //pub fn new_finalize(&self, goal_node: Rc<Node>) -> LineString<f64> {}

    pub fn finalize(&self, goal_node: Rc<Node>) -> LineString<f64> {
        let node_iter = NodeIter {
            curr: Some(goal_node),
        };
        node_iter
            .map(|node| match node.get_parent() {
                Some(parent) => {
                    let (sx, sy) = node.get_coord().x_y();
                    let syaw = node.get_yaw();
                    let (ex, ey) = parent.get_coord().x_y();
                    let eyaw = parent.get_yaw();

                    match dubins_path_planning(sx, sy, syaw, ex, ey, eyaw, 0.8) {
                        Some((px, py, _, _, _)) => px.into_iter().zip(py.into_iter()).collect(),
                        _ => panic!("WTF"),
                    }
                }
                None => vec![],
            })
            .flatten()
            .collect()
    }

    pub fn old_finalize(&self, goal_node: Rc<Node>) -> LineString<f64> {
        println!("Finalizing");
        let mut points = vec![];
        let mut parent = Some(goal_node);

        while let Some(current) = parent.clone() {
            //points.push(next_parent.get_coord());
            parent = current.get_parent();
            if let Some(p) = parent.clone() {
                let (sx, sy) = current.get_point().x_y();
                let syaw = current.get_yaw();
                let (ex, ey) = p.get_point().x_y();
                let eyaw = p.get_yaw();

                let (px, py, _, _, _) = dubins_path_planning(sx, sy, syaw, ex, ey, eyaw, 1.0)
                    .expect("should create dubins segment");

                let mut new_points: Vec<Coordinate<f64>> = px
                    .into_iter()
                    .zip(py.into_iter())
                    .map(|(x, y)| Coordinate { x, y })
                    .collect();

                points.append(&mut new_points);
            }
        }

        //points.reverse();

        LineString(points)
    }

    pub fn plan(&mut self) -> Option<LineString<f64>> {
        let mut results: Vec<LineString<f64>> = vec![];
        for _i in 0..self.max_iter {
            // println!("iter: {:?}", _i);
            if let Some(rnd_node) = self.get_random_node() {
                if self.verify_node(rnd_node.clone()) {
                    self.nodes.push(rnd_node.clone());

                    if let Some(finish) = self.check_finish(rnd_node.clone()) {
                        results.push(finish);
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
