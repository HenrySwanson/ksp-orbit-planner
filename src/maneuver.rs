use kiss3d::nalgebra as na;
use na::Vector3;

pub enum Maneuver {
    WaitTime(f64),
    Boost(Vector3<f64>), // parent inertial reference frame
}
