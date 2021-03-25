use kiss3d::nalgebra as na;
use na::{Rotation3, Vector3};

use std::f64::consts::PI;

#[derive(Debug)]
pub struct Orbit {
    // encodes the orientation of the orbit: it moves the xy plane to the orbital
    // plane, and x to point towards periapsis
    rotation: Rotation3<f64>,
    energy: f64,
    ang_mom: f64,
    mu: f64, // TODO: BodyInfo instead?
}

impl Orbit {
    pub fn from_cartesian(position: &Vector3<f64>, velocity: &Vector3<f64>, mu: f64) -> Self {
        // Compute some physical quantities for the orbit.
        let r = position.norm();
        let energy = velocity.norm_squared() / 2.0 - mu / r;
        let ang_mom = position.cross(&velocity);

        // LRL vector = v x h / mu - r/|r|
        let lrl = velocity.cross(&ang_mom) / mu - position / r;

        // We want to rotate this orbit into a standard frame. Unfortunately, this
        // might be ambiguous, if either angular momentum or the LRL vector are too
        // close to zero. So we use a particularly cautious method.
        let rotation = always_find_rotation(&ang_mom, &lrl, 1e-20);

        Orbit {
            rotation,
            energy,
            ang_mom: ang_mom.norm(),
            mu,
        }
    }

    pub fn rotation(&self) -> Rotation3<f64> {
        self.rotation
    }

    pub fn energy(&self) -> f64 {
        self.energy
    }

    pub fn angular_momentum(&self) -> f64 {
        self.ang_mom
    }

    pub fn eccentricity(&self) -> f64 {
        // 1 = e^2 - 2 E (h/mu)^2
        let mut e_squared = 1.0 + 2.0 * self.energy * (self.ang_mom / self.mu).powi(2);

        // If we're just barely below zero, round up to zero.
        if -1e-9 < e_squared && e_squared < 0.0 {
            e_squared = 0.0;
        }

        if e_squared < 0.0 {
            panic!(
                "Illegal orbit configuration: E = {}, h = {}, e^2 computed as {}",
                self.energy, self.ang_mom, e_squared
            )
        }
        e_squared.sqrt()
    }

    pub fn semimajor_axis(&self) -> f64 {
        if self.energy == 0.0 {
            f64::INFINITY
        } else {
            -self.mu / (2.0 * self.energy)
        }
    }

    pub fn semilatus_rectum(&self) -> f64 {
        self.ang_mom.powi(2) / self.mu
    }

    pub fn periapsis(&self) -> f64 {
        // the periapsis is a(1-e), but when e = 1 that's got problems
        // a(1-e) = a(1-e^2)/(1+e) = h^2 / mu (1+e)
        self.ang_mom.powi(2) / self.mu / (1.0 + self.eccentricity())
    }

    pub fn apoapsis(&self) -> f64 {
        2.0 * self.semimajor_axis() - self.periapsis()
    }

    pub fn get_position_at_theta(&self, theta: f64) -> Option<Vector3<f64>> {
        let denominator = 1.0 + self.eccentricity() * theta.cos();
        if denominator <= 0.0 {
            // Happens when we're hyperbolic/parabolic, and we're larger than max anomaly
            return None;
        }

        if self.ang_mom == 0.0 {
            // Radial orbits
            return None;
        }

        let radius = self.semilatus_rectum() / denominator;
        let position = radius * Vector3::new(theta.cos(), theta.sin(), 0.0);

        Some(self.rotation() * position)
    }
}

fn reject(u: &Vector3<f64>, v: &Vector3<f64>) -> Vector3<f64> {
    // Computes the vector rejection of u from v. v must be non-zero.
    let v_hat = v.normalize();
    let u_proj_v = u.dot(&v_hat) * v_hat;
    u - u_proj_v

    // TODO this could also be done by computing v x (u x v) and normalizing
}

/// Returns a rotation R that sends the z- and x- axes to point in the specified directions.
/// The orthogonality of new_z and new_x is not checked.
/// If new_z or new_x is sufficiently close to zero, then some semi-canonical choices will
/// be made. Unfortunately, the hairy ball theorem prevents us from doing so in a completely
/// canonical way.
///
/// The specific choices we make are:
/// - if new_z is small:
///   - R(z) will point as much along the z-axis as possible, while remaining perpendicular
///     to R(x) = new_x
///   - if this is ill-defined (new_x ~= z), then R(z) = y
/// - similarly, if new_x is small:
///   - R(x) will point as much along the x-axis as possible, while remaining perpendicular
///     to R(z) = new_z
///   - if this is ill-defined (new_z ~= x), then R(x) = -y
/// - if both new_z and new_x are small, then this returns the identity
pub fn always_find_rotation(
    new_z: &Vector3<f64>,
    new_x: &Vector3<f64>,
    tolerance: f64,
) -> Rotation3<f64> {
    let z_large_enough = new_z.norm() >= tolerance;
    let x_large_enough = new_x.norm() >= tolerance;

    let (new_z, new_x) = match (z_large_enough, x_large_enough) {
        // Both are good; the easy case
        (true, true) => (new_z.clone(), new_x.clone()),
        // z is too small
        (false, true) => {
            // Rejecting the z axis from new_x gives us the most-z-like vector
            // that's perpendicular to new_x. If it's too small, we just pick
            // our fallback choice.
            let mut best_new_z = reject(&Vector3::z(), &new_x);
            if best_new_z.norm() < tolerance {
                best_new_z = Vector3::y();
            };
            (best_new_z, new_x.clone())
        }
        // x is too small
        (true, false) => {
            // Same thing as above, with z and x switched.
            let mut best_new_x = reject(&Vector3::x(), &new_z);
            if best_new_x.norm() < tolerance {
                best_new_x = -Vector3::y();
            };
            (new_z.clone(), best_new_x)
        }
        // Easy case, early return here.
        (false, false) => return Rotation3::identity(),
    };

    // Unfortunately, the Rotation::face_towards call takes new-z and new-y as arguments,
    // so we prepend a 90-degree rotation around z (e.g., one taking x to y).
    let mut rotation = Rotation3::face_towards(&new_z, &new_x);
    rotation = rotation * Rotation3::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
    rotation.renormalize();
    rotation
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::consts;

    // TODO pull out into common?
    macro_rules! assert_very_large {
        ($exp:expr) => {
            approx::assert_relative_eq!($exp.recip(), 0.0)
        };
    }

    #[test]
    fn test_rotation_code() {
        fn test_rotation(r: Rotation3<f64>, expected_z: &Vector3<f64>, expected_x: &Vector3<f64>) {
            approx::assert_relative_eq!(
                r * Vector3::z(),
                expected_z.normalize(),
                max_relative = 1e-15
            );
            approx::assert_relative_eq!(
                r * Vector3::x(),
                expected_x.normalize(),
                max_relative = 1e-15
            );
        }

        // Common vectors
        let u = Vector3::new(1.0, 2.0, 3.0);
        let v = Vector3::new(2.0, 2.0, -2.0);

        // Normal
        test_rotation(always_find_rotation(&u, &v, 1e-20), &u, &v);

        // new-z is too small
        test_rotation(
            always_find_rotation(&Vector3::zeros(), &v, 1e-20),
            &Vector3::new(1.0, 1.0, 2.0),
            &v,
        );

        // new-z is too small, and new-x points along z
        // TODO should we treat new-x = kz and new-x = -kz differently?
        test_rotation(
            always_find_rotation(&Vector3::zeros(), &Vector3::z(), 1e-20),
            &Vector3::y(),
            &Vector3::z(),
        );

        // new-x is too small
        test_rotation(
            always_find_rotation(&u, &Vector3::zeros(), 1e-20),
            &u,
            &Vector3::new(13.0, -2.0, -3.0),
        );

        // new-x is too small, and new-z points along x
        test_rotation(
            always_find_rotation(&Vector3::x(), &Vector3::zeros(), 1e-20),
            &Vector3::x(),
            &-Vector3::y(),
        );

        // both are small
        test_rotation(
            always_find_rotation(&Vector3::zeros(), &Vector3::zeros(), 1e-20),
            &Vector3::z(),
            &Vector3::x(),
        );
    }

    #[test]
    fn test_orbit_quantities() {
        let mu = consts::KERBOL_MU;
        let radius = consts::KERBIN_ORBIT_RADIUS;
        let circ_velocity = consts::get_circular_velocity(radius, mu);

        let make_orbit = |p_dir, v_dir, multiplier| {
            Orbit::from_cartesian(&(radius * p_dir), &(circ_velocity * multiplier * v_dir), mu)
        };

        // Circular orbit
        let orbit = make_orbit(Vector3::y(), Vector3::z(), 1.0);
        approx::assert_relative_eq!(orbit.energy(), -mu / 2.0 / radius);
        approx::assert_relative_eq!(orbit.angular_momentum(), radius * circ_velocity);
        approx::assert_relative_eq!(orbit.eccentricity(), 0.0);
        approx::assert_relative_eq!(orbit.semimajor_axis(), radius);
        approx::assert_relative_eq!(orbit.semilatus_rectum(), radius);
        approx::assert_relative_eq!(orbit.periapsis(), radius);
        approx::assert_relative_eq!(orbit.apoapsis(), radius);

        // Parabolic orbit: escape velocity = mu/r * sqrt(2)
        let sqrt_2 = std::f64::consts::SQRT_2;
        let orbit = make_orbit(Vector3::z(), Vector3::x(), sqrt_2);
        approx::assert_relative_eq!(orbit.energy(), 0.0, epsilon = 1e-5);
        approx::assert_relative_eq!(orbit.angular_momentum(), radius * sqrt_2 * circ_velocity);
        approx::assert_relative_eq!(orbit.eccentricity(), 1.0);
        assert_very_large!(orbit.semimajor_axis());
        approx::assert_relative_eq!(orbit.semilatus_rectum(), 2.0 * radius);
        approx::assert_relative_eq!(orbit.periapsis(), radius);
        assert_very_large!(orbit.apoapsis());

        // Radial orbit
        let orbit = make_orbit(Vector3::z(), Vector3::x(), 0.0);
        approx::assert_relative_eq!(orbit.energy(), -mu / radius);
        approx::assert_relative_eq!(orbit.angular_momentum(), 0.0);
        approx::assert_relative_eq!(orbit.eccentricity(), 1.0);
        approx::assert_relative_eq!(orbit.semimajor_axis(), radius / 2.0);
        approx::assert_relative_eq!(orbit.semilatus_rectum(), 0.0);
        approx::assert_relative_eq!(orbit.periapsis(), 0.0);
        approx::assert_relative_eq!(orbit.apoapsis(), radius);

        // Elliptic orbit
        let orbit = make_orbit(Vector3::y(), Vector3::x(), 1.2);
        assert!(orbit.energy() < 0.0);
        assert!(orbit.eccentricity() < 1.0);
        assert!(orbit.semimajor_axis() > radius);
        approx::assert_relative_eq!(orbit.periapsis(), radius);
        assert!(orbit.apoapsis() > radius);

        // Elliptic orbit but the other way
        let orbit = make_orbit(Vector3::x(), Vector3::z(), 0.8);
        assert!(orbit.energy() < 0.0);
        assert!(orbit.eccentricity() < 1.0);
        assert!(orbit.semimajor_axis() < radius);
        assert!(orbit.periapsis() < radius);
        approx::assert_relative_eq!(orbit.apoapsis(), radius);

        // Hyperbolic orbit
        let orbit = make_orbit(Vector3::y(), Vector3::x(), 1.5);
        assert!(orbit.energy() > 0.0);
        assert!(orbit.eccentricity() > 1.0);
        assert!(orbit.semimajor_axis() < 0.0);
        approx::assert_relative_eq!(orbit.periapsis(), radius);
        assert!(orbit.apoapsis() < -radius); // negative but also < periapsis
    }
}
