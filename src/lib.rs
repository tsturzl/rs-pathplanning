extern crate geo;
extern crate geo_offset;
extern crate rand;
extern crate rayon;

use geo::{LineString, Polygon};
use pyo3::exceptions;
use pyo3::prelude::*;
use pyo3::wrap_pyfunction;

pub mod dubins;
pub mod rrt;

// Python bindings

#[pyclass]
struct SpaceConf {
    bounds: Vec<(f64, f64)>,
    obstacles: Vec<Vec<(f64, f64)>>,
}

#[pymethods]
impl SpaceConf {
    #[new]
    fn new(bounds: Vec<(f64, f64)>, obstacles: Vec<Vec<(f64, f64)>>) -> Self {
        SpaceConf { bounds, obstacles }
    }
}

#[pyclass(module = "path_planning")]
struct RobotConf {
    width: f64,
    height: f64,
    max_steer: f64,
}

#[pymethods]
impl RobotConf {
    #[new]
    fn new(width: f64, height: f64, max_steer: f64) -> Self {
        RobotConf {
            width,
            height,
            max_steer,
        }
    }
}

#[pyclass(module = "path_planning")]
struct RRTDubinsPlanner {
    start: (f64, f64),
    start_yaw: f64,
    goal: (f64, f64),
    goal_yaw: f64,
    max_iter: usize,
    space: SpaceConf,
    robot: RobotConf,
}

#[pymethods]
impl RRTDubinsPlanner {
    #[new]
    fn new(
        start: (f64, f64),
        start_yaw: f64,
        goal: (f64, f64),
        goal_yaw: f64,
        max_iter: usize,
        space: SpaceConf,
        robot: RobotConf,
    ) -> Self {
        RRTDubinsPlanner {
            start,
            start_yaw,
            goal,
            goal_yaw,
            max_iter,
            space,
            robot,
        }
    }

    fn plan(&self, py: Python<'_>) -> PyResult<Vec<(f64, f64)>> {
        let robot = rrt::Robot::new(self.robot.width, self.robot.height, self.robot.max_steer);
        let bounds = Polygon::new(LineString::from(self.space.bounds), vec![]);
        let obs: Vec<Polygon<f64>> = self
            .space
            .obstacles
            .iter()
            .map(|o| Polygon::new(LineString::from(*o), vec![]))
            .collect();
        let space = rrt::Space::new(bounds, robot, obs);
        let planner = rrt::RRT::new(
            self.start.into(),
            self.start_yaw,
            self.goal.into(),
            self.goal_yaw,
            self.max_iter,
            space,
        );

        let result = py.allow_threads(move || planner.plan());

        match result {
            Some(path) => Ok(path.points_iter().map(|p| p.x_y()).collect()),
            None => Err(exceptions::Exception::py_err("Could not generate path")),
        }
    }
}

#[pyfunction]
fn plan(
    start: (f64, f64),
    start_yaw: f64,
    goal: (f64, f64),
    goal_yaw: f64,
    max_iter: usize,
    space: SpaceConf,
    robot: RobotConf,
) -> PyResult<Vec<(f64, f64)>> {
    let robot = rrt::Robot::new(robot.width, robot.height, robot.max_steer);
    let bounds = Polygon::new(LineString::from(space.bounds), vec![]);
    let obs: Vec<Polygon<f64>> = space
        .obstacles
        .iter()
        .map(|o| Polygon::new(LineString::from(*o), vec![]))
        .collect();
    let space = rrt::Space::new(bounds, robot, obs);
    let planner = rrt::RRT::new(
        start.into(),
        start_yaw,
        goal.into(),
        goal_yaw,
        max_iter,
        space,
    );

    let result = py.allow_threads(move || planner.plan());

    match result {
        Some(path) => Ok(path.points_iter().map(|p| p.x_y()).collect()),
        None => Err(exceptions::Exception::py_err("Could not generate path")),
    }
}

#[pymodule]
fn path_planning(_py: Python<'_>, m: &PyModule) -> PyResult<()> {
    m.add_class::<RobotConf>()?;
    m.add_class::<SpaceConf>()?;
    m.add_class::<RRTDubinsPlanner>()?;

    Ok(())
}
