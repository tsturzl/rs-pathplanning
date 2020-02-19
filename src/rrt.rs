use crate::dubins::{dubins_path_planning, DubinsConfig};
use geo::algorithm::{
    contains::Contains, euclidean_distance::EuclideanDistance, euclidean_length::EuclideanLength,
    intersects::Intersects,
};
use geo::{Coordinate, LineString, Point, Polygon};
use geo_offset::Offset;
use rand::{thread_rng, Rng};
use rayon::prelude::*;
use rstar::{PointDistance, RTree, RTreeObject, AABB};
use std::f64::consts::PI;
use std::sync::{Arc, Mutex};

const RECURSION_LIMIT: usize = 16;

#[allow(dead_code)]
pub struct Robot {
    width: f64,
    height: f64,
    // turn radius
    max_steer: f64,
}

impl Robot {
    pub fn new(width: f64, height: f64, max_steer: f64) -> Robot {
        Robot {
            width,
            height,
            max_steer,
        }
    }

    pub fn get_width(&self) -> f64 {
        self.width
    }

    pub fn get_steer(&self) -> f64 {
        self.max_steer
    }
}

// Maybe this will create a more elegant way: https://github.com/georust/geo/issues/414
pub fn create_circle(center: Point<f64>, radius: f64) -> Polygon<f64> {
    let (cx, cy) = center.x_y();
    let circum = 2.0 * PI * radius;
    let n = (circum / 1.0).ceil();
    let i = (n + 1.0) as usize;
    let points: Vec<(f64, f64)> = (0..i)
        .map(|x| {
            let x = x as f64;

            (
                ((2.0 * PI / n * x).cos() * radius) + cx,
                ((2.0 * PI / n * x).sin() * radius) + cy,
            )
        })
        .collect();

    Polygon::new(points.into(), vec![])
}

fn buffer_poly(poly: &Polygon<f64>, buffer: f64) -> Polygon<f64> {
    poly.offset(buffer)
        .expect("polygon to set a buffer")
        .into_iter()
        .last()
        .expect("should get buffered polygon")
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
    pub fn new(bounds: Polygon<f64>, robot: Robot, obstacle_list: Vec<Polygon<f64>>) -> Space {
        let width = robot.get_width() / 2.0;
        let bounds = buffer_poly(&bounds, -width);
        let points: Vec<Point<f64>> = bounds.exterior().points_iter().collect();
        let (fx, fy) = points[0].x_y();
        let mut minx = fx;
        let mut maxx = fx;
        let mut miny = fy;
        let mut maxy = fy;

        points.iter().for_each(|point| {
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
        });

        let obstacles = obstacle_list
            .iter()
            .map(|o| buffer_poly(o, width))
            .collect();

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

    pub fn rand_point(&self) -> Point<f64> {
        let mut rng = thread_rng();

        let randx: f64 = rng.gen_range(self.minx, self.maxx);
        let randy: f64 = rng.gen_range(self.miny, self.maxy);

        Point::new(randx, randy)
    }

    pub fn get_steer(&self) -> f64 {
        self.robot.get_steer()
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
    parent: Option<Arc<Node>>,
    // angle from self to parent, or the heading angle(eg the start point)
    yaw: f64,
}

impl Node {
    pub fn new(point: Point<f64>, parent: Arc<Node>) -> Node {
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

    pub fn new_goal(point: Point<f64>, parent: Arc<Node>, yaw: f64) -> Node {
        Node {
            point,
            parent: Some(parent),
            yaw,
        }
    }

    pub fn get_parent(&self) -> Option<Arc<Node>> {
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

struct NodeEnvelope {
    inner: Arc<Node>,
}

impl NodeEnvelope {
    pub fn new(inner: Arc<Node>) -> NodeEnvelope {
        NodeEnvelope { inner }
    }

    pub fn get_inner(&self) -> Arc<Node> {
        self.inner.clone()
    }
}

impl RTreeObject for NodeEnvelope {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        let (x, y) = self.inner.get_point().x_y();
        AABB::from_point([x, y])
    }
}

impl PointDistance for NodeEnvelope {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let p1 = Point::from((point[0], point[1]));
        let p2 = self.inner.get_point();

        p1.euclidean_distance(p2)
    }
}

// Iterate nodes back to root
pub struct NodeIter {
    curr: Option<Arc<Node>>,
}

impl Iterator for NodeIter {
    type Item = Arc<Node>;

    fn next(&mut self) -> Option<Arc<Node>> {
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

// fn create_line(from: Arc<Node>, to: Arc<Node>, c: f64) -> LineString<f64> {
//     let (sx, sy) = from.get_point().x_y();
//     let syaw = from.get_yaw();
//     let (ex, ey) = to.get_point().x_y();
//     let eyaw = to.get_yaw();
//     let conf = DubinsConfig {
//         sx,
//         sy,
//         syaw,
//         ex,
//         ey,
//         eyaw,
//         c,
//         step_size: STEP_SIZE,
//     };
//     dubins_path_linestring(&conf).expect("Should generate a dubins line")
// }

pub fn line_to_origin(node: Arc<Node>, turn_radius: f64, step_size: f64) -> LineString<f64> {
    let node_iter = NodeIter { curr: Some(node) };
    let l: Vec<(f64, f64)> = node_iter
        .par_bridge()
        .map(|node| match node.get_parent() {
            Some(parent) => {
                let (sx, sy) = node.get_coord().x_y();
                let syaw = node.get_yaw();
                let (ex, ey) = parent.get_coord().x_y();
                let eyaw = parent.get_yaw();
                let conf = DubinsConfig {
                    sx,
                    sy,
                    syaw,
                    ex,
                    ey,
                    eyaw,
                    turn_radius,
                    step_size,
                };
                match dubins_path_planning(&conf) {
                    Some((px, py, _, _, _)) => px.into_iter().zip(py.into_iter()).collect(),
                    _ => vec![(sx, sy)],
                }
            }
            None => vec![node.get_coord().x_y()],
        })
        .flatten()
        .collect();
    l.into()
}

type SpacialTree = Arc<Mutex<RTree<NodeEnvelope>>>;

pub struct RRT {
    goal: Coordinate<f64>,
    goal_yaw: f64,
    max_iter: usize,
    step_size: f64,
    space: Space,
    spatial: SpacialTree,
}

impl RRT {
    pub fn new(
        start: Coordinate<f64>,
        start_yaw: f64,
        goal: Coordinate<f64>,
        goal_yaw: f64,
        max_iter: usize,
        step_size: f64,
        space: Space,
    ) -> RRT {
        let root = Arc::new(Node::new_root(start.into(), start_yaw));
        let spatial = Arc::new(Mutex::new(RTree::new()));
        spatial.lock().unwrap().insert(NodeEnvelope::new(root));
        RRT {
            goal,
            goal_yaw,
            max_iter,
            step_size,
            space,
            spatial,
        }
    }

    // a KD-Tree could speed this up
    // pub fn get_nearest_node(&self, point: &Point<f64>) -> Option<(Arc<Node>, f64)> {
    //     self.nodes
    //         .lock()
    //         .unwrap()
    //         .iter()
    //         .filter_map(|node| {
    //             let dist = point.euclidean_distance(node.get_point());

    //             if dist <= self.max_dist && dist >= 2.0 {
    //                 Some((node.clone(), dist))
    //             } else {
    //                 None
    //             }
    //         })
    //         .min_by(|a, b| {
    //             a.1.partial_cmp(&b.1)
    //                 .expect("Node lengths(<f64>) should compare.")
    //         })
    // }

    pub fn get_nearest_node(&self, point: &Point<f64>) -> Option<Arc<Node>> {
        let (x, y) = point.x_y();

        match self
            .spatial
            .clone()
            .lock()
            .unwrap()
            .nearest_neighbor(&[x, y])
        {
            Some(node) => Some(node.get_inner()),
            None => None,
        }
    }

    // pub fn get_random_node(&self) -> Option<Arc<Node>> {
    //     let point = self.space.rand_point();
    //     match self.get_nearest_node(&point) {
    //         Some((nearest_node, dist)) => {
    //             if dist <= self.max_dist {
    //                 Some(Arc::new(Node::new(point, nearest_node)))
    //             } else {
    //                 None
    //             }
    //         }
    //         None => None,
    //     }
    // }
    pub fn get_random_node(&self) -> Option<Arc<Node>> {
        let point = self.space.rand_point();
        match self.get_nearest_node(&point) {
            Some(nearest_node) => Some(Arc::new(Node::new(point, nearest_node))),
            None => None,
        }
    }

    pub fn verify_node(&self, node: Arc<Node>) -> bool {
        // match node.clone().get_parent() {
        //     Some(parent) => {
        //         let line = create_line(parent, node);

        //         line.euclidean_length() <= self.max_dist && self.space.verify(&line)
        //     }
        //     None => false,
        // }

        let line = line_to_origin(node, self.space.get_steer(), self.step_size);
        self.space.verify(&line)
    }

    pub fn check_finish(&self, node: Arc<Node>) -> Option<LineString<f64>> {
        let point = self.goal.into();
        let goal_node = Arc::new(Node::new_goal(point, node, self.goal_yaw));
        let line = self.finalize(goal_node);

        if self.space.verify(&line) {
            Some(line)
        } else {
            None
        }
    }

    // pub fn optimize(&self, nodes: Vec<Arc<Node>>) -> LineString<f64> {
    //     let mut optimized = vec![];

    //     let len = nodes.len() - 1;
    //     for i in 0..len {
    //         let line = create_line(nodes[i].clone(), nodes[len].clone(), self.space.get_steer());
    //         optimized.push(nodes[i].get_coord());
    //         if self.space.verify(&line) {
    //             optimized.push(nodes[len].get_coord());
    //             break;
    //         }
    //     }

    //     optimized.reverse();

    //     // println!("optimized: {:?}", nodes.len() - optimized.len());

    //     LineString(optimized)
    // }

    //pub fn new_finalize(&self, goal_node: Arc<Node>) -> LineString<f64> {}

    // Trim out unneeded nodes in the tree to reduce length
    pub fn optimize(&self, node: Arc<Node>, i: usize) -> Option<Arc<Node>> {
        if i >= RECURSION_LIMIT {
            return None;
        }

        let node_iter = NodeIter {
            curr: Some(node.clone()),
        };
        let nodes_vec: Vec<Arc<Node>> = node_iter.collect();

        for to_node in nodes_vec.into_iter().rev() {
            let new_node = Arc::new(Node::new(node.get_coord().into(), to_node.clone()));

            let line = line_to_origin(new_node.clone(), self.space.get_steer(), self.step_size);
            if self.space.verify(&line) {
                match self.optimize(to_node, i + 1) {
                    Some(to_node) => {
                        return Some(Arc::new(Node::new(node.get_coord().into(), to_node)))
                    }
                    None => return Some(new_node),
                }
            }
        }
        None
    }

    pub fn optimize_from_goal(&self, goal_node: Arc<Node>) -> Arc<Node> {
        match goal_node.get_parent() {
            Some(parent) => match self.optimize(parent, 0) {
                Some(n) => Arc::new(Node::new_goal(
                    goal_node.get_coord().into(),
                    n,
                    self.goal_yaw,
                )),
                None => goal_node,
            },
            None => goal_node,
        }
    }

    pub fn finalize(&self, goal_node: Arc<Node>) -> LineString<f64> {
        let node_iter = NodeIter {
            curr: Some(self.optimize_from_goal(goal_node)),
        };

        let mut l: Vec<(f64, f64)> = node_iter
            .par_bridge()
            .map(|node| match node.get_parent() {
                Some(parent) => {
                    let (sx, sy) = node.get_coord().x_y();
                    let syaw = node.get_yaw();
                    let (ex, ey) = parent.get_coord().x_y();
                    let eyaw = parent.get_yaw();

                    let conf = DubinsConfig {
                        sx,
                        sy,
                        syaw,
                        ex,
                        ey,
                        eyaw,
                        turn_radius: self.space.get_steer(),
                        step_size: self.step_size,
                    };
                    match dubins_path_planning(&conf) {
                        Some((px, py, _, _, _)) => px.into_iter().zip(py.into_iter()).collect(),
                        _ => panic!("Should plan dubins curve"),
                    }
                }
                None => vec![],
            })
            .flatten()
            .collect();

        // these lines get drawn back to the root node, we need to flip it
        l.reverse();
        l.into()
    }

    // pub fn old_finalize(&self, goal_node: Arc<Node>) -> LineString<f64> {
    //     println!("Finalizing");
    //     let mut points = vec![];
    //     let mut parent = Some(goal_node);

    //     while let Some(current) = parent.clone() {
    //         //points.push(next_parent.get_coord());
    //         parent = current.get_parent();
    //         if let Some(p) = parent.clone() {
    //             let (sx, sy) = current.get_point().x_y();
    //             let syaw = current.get_yaw();
    //             let (ex, ey) = p.get_point().x_y();
    //             let eyaw = p.get_yaw();

    //             let conf = DubinsConfig {
    //                 sx,
    //                 sy,
    //                 syaw,
    //                 ex,
    //                 ey,
    //                 eyaw,
    //                 c: 0.8,
    //             };
    //             let (px, py, _, _, _) =
    //                 dubins_path_planning(&conf).expect("should create dubins segment");

    //             let mut new_points: Vec<Coordinate<f64>> = px
    //                 .into_iter()
    //                 .zip(py.into_iter())
    //                 .map(|(x, y)| Coordinate { x, y })
    //                 .collect();

    //             points.append(&mut new_points);
    //         }
    //     }

    //     //points.reverse();

    //     LineString(points)
    // }

    pub fn plan_one(&self) -> Option<LineString<f64>> {
        if let Some(rnd_node) = self.get_random_node() {
            if self.verify_node(rnd_node.clone()) {
                self.spatial
                    .lock()
                    .unwrap()
                    .insert(NodeEnvelope::new(rnd_node.clone()));

                if let Some(finish) = self.check_finish(rnd_node) {
                    return Some(finish);
                }
            }
        }
        None
    }

    pub fn plan(&self) -> Option<LineString<f64>> {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(4)
            // .stack_size(32 * 1024 * 1024)
            .build()
            .unwrap();

        pool.install(|| {
            (0..self.max_iter)
                .into_par_iter()
                .map(|_| self.plan_one())
                .filter_map(|r| r)
                .min_by(|a, b| {
                    let a_cost = a.euclidean_length();
                    let b_cost = b.euclidean_length();
                    a_cost
                        .partial_cmp(&b_cost)
                        .expect("should compared route costs")
                })
        })
    }
}
