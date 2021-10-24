use nalgebra::{Rotation3, Vector3};

use std::f64::consts::PI;

use crate::math::anomaly;
use crate::math::geometry::{always_find_rotation, directed_angle};
use crate::math::stumpff::stumpff_G;

use super::body::BodyID;
use super::state::CartesianState;

#[derive(Debug, Clone)]
pub struct Orbit {
    // encodes the orientation of the orbit: it moves the xy plane to the orbital
    // plane, and x to point towards periapsis
    rotation: Rotation3<f64>,
    energy: f64,
    ang_mom: f64,
    mu: f64,
}

// TODO: re-evaluate if we need this
#[derive(Debug, Clone)]
pub struct OrbitPatch {
    pub orbit: Orbit,
    pub start_anomaly: f64,
    pub end_anomaly: Option<f64>,
    pub parent_id: BodyID,
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

    pub fn from_kepler(a: f64, ecc: f64, incl: f64, lan: f64, argp: f64, mu: f64) -> Self {
        // The energy depends only on the semimajor axis, and the angular momentum
        // includes the eccentricity.
        let energy = -mu / 2.0 / a;
        let h_sq = mu * a * (1.0 - ecc * ecc);

        // We have an orbit in the xy plane where the periapsis is pointed along the x-axis.
        // So first, we rotate it around z until the periapsis is at argp away from the x-axis
        // (which will now be the ascending node). We then rotate around x to get the inclination,
        // and then one final turn around z to get the correct longitude of the AN.
        let rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), lan)
            * Rotation3::from_axis_angle(&Vector3::x_axis(), incl)
            * Rotation3::from_axis_angle(&Vector3::z_axis(), argp);

        Orbit {
            rotation,
            energy,
            ang_mom: h_sq.sqrt(),
            mu,
        }
    }

    pub fn rotation(&self) -> Rotation3<f64> {
        self.rotation
    }

    // TODO these should return unit vectors
    pub fn periapse_vector(&self) -> Vector3<f64> {
        self.rotation() * Vector3::x()
    }

    pub fn normal_vector(&self) -> Vector3<f64> {
        self.rotation() * Vector3::z()
    }

    pub fn asc_node_vector(&self) -> Vector3<f64> {
        // TODO: the ambiguity here makes me think i might wanna just store the angles
        let v = Vector3::z().cross(&self.normal_vector());
        v.try_normalize(1e-20)
            .unwrap_or_else(|| self.periapse_vector())
    }

    pub fn inclination(&self) -> f64 {
        // Inclination is the angle the normal makes with z
        self.normal_vector().angle(&Vector3::z())
    }

    pub fn long_asc_node(&self) -> f64 {
        // Longitude of ascending node is the directed angle from x to the ascending node
        directed_angle(&Vector3::x(), &self.asc_node_vector(), &Vector3::z())
    }

    pub fn arg_periapse(&self) -> f64 {
        // Argument of periapsis is the directed angle from the ascending node to the periapsis
        directed_angle(
            &self.asc_node_vector(),
            &self.periapse_vector(),
            &self.normal_vector(),
        )
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

    pub fn period(&self) -> Option<f64> {
        if self.energy < 0.0 {
            Some(2.0 * PI * (self.semimajor_axis().powi(3) / self.mu).sqrt())
        } else {
            None
        }
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

    pub fn get_state_at_theta(&self, theta: f64) -> (Vector3<f64>, Vector3<f64>) {
        // Taken from https://www.mathworks.com/matlabcentral/fileexchange/35455-convert-keplerian-orbital-elements-to-a-state-vector
        let p = self.semilatus_rectum();
        let ecc = self.eccentricity();
        let mu = self.mu;
        let rotation = self.rotation();

        let radius = p / (1.0 + ecc * theta.cos());
        let position = radius * Vector3::new(theta.cos(), theta.sin(), 0.0);
        let velocity = (mu / p).sqrt() * Vector3::new(-theta.sin(), ecc + theta.cos(), 0.0);

        (rotation * position, rotation * velocity)
    }

    pub fn true_to_universal(&self, true_anomaly: f64) -> f64 {
        // TODO what about radial orbits?
        let eccentricity = self.eccentricity();
        let energy = self.energy();
        if eccentricity < 1.0 {
            let ecc = anomaly::true_to_eccentric(true_anomaly, eccentricity);
            anomaly::eccentric_to_universal(ecc, energy)
        } else if eccentricity > 1.0 {
            let hyp = anomaly::true_to_hyperbolic(true_anomaly, eccentricity);
            anomaly::hyperbolic_to_universal(hyp, energy)
        } else {
            let para = anomaly::true_to_parabolic(true_anomaly);
            anomaly::parabolic_to_universal(para, self.ang_mom, self.mu)
        }
    }

    pub fn universal_to_true(&self, universal_anomaly: f64) -> f64 {
        let eccentricity = self.eccentricity();
        let energy = self.energy();
        if eccentricity < 1.0 {
            let ecc = anomaly::universal_to_eccentric(universal_anomaly, energy);
            anomaly::eccentric_to_true(ecc, eccentricity)
        } else if eccentricity > 1.0 {
            let hyp = anomaly::universal_to_hyperbolic(universal_anomaly, energy);
            anomaly::hyperbolic_to_true(hyp, eccentricity)
        } else {
            let para = anomaly::universal_to_parabolic(universal_anomaly, self.ang_mom, self.mu);
            anomaly::parabolic_to_true(para)
        }
    }

    #[allow(non_snake_case)]
    pub fn get_state_native_frame(&self, s: f64) -> CartesianState {
        // Note: this function is only exposed because it makes rendering faster. Would be
        // nice to have an alternative... :\

        let beta = -2.0 * self.energy;
        let mu = self.mu;
        let G: [f64; 4] = stumpff_G(beta, s);

        // Get the position at s
        let x = self.periapsis() - mu * G[2];
        let y = self.ang_mom * G[1];
        let r = (x * x + y * y).sqrt();
        let vx = -mu / r * G[1];
        let vy = self.ang_mom / r * G[0];

        let position = Vector3::new(x, y, 0.0);
        let velocity = Vector3::new(vx, vy, 0.0);

        CartesianState::new(position, velocity, self.mu)
    }

    pub fn get_state(&self, s: f64) -> CartesianState {
        let native_state = self.get_state_native_frame(s);

        let position = self.rotation * native_state.position();
        let velocity = self.rotation * native_state.velocity();

        CartesianState::new(position, velocity, self.mu)
    }

    #[allow(non_snake_case)]
    pub fn get_time_since_periapsis(&self, s: f64) -> f64 {
        // Identical to the one in CartesianState, but we can simplify a bit
        // because \dot{r}_p = 0
        let r_p = self.periapsis();
        let beta = -2.0 * self.energy;
        let mu = self.mu;
        let G = stumpff_G(beta, s);

        r_p * G[1] + mu * G[3]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use approx::assert_relative_eq;

    use crate::consts;

    // TODO pull out into common?
    macro_rules! assert_very_large {
        ($exp:expr) => {
            approx::assert_relative_eq!($exp.recip(), 0.0)
        };
    }

    // TODO: test creation of orbit from cartesian and from kepler

    #[test]
    fn test_orbit_quantities() {
        let mu = consts::KERBOL_MU;
        let radius = consts::KERBIN_ORBIT_RADIUS;
        let circ_velocity = consts::get_circular_velocity(radius, mu);
        let period = 2.0 * PI * radius / circ_velocity;

        let make_orbit = |p_dir, v_dir, multiplier| {
            Orbit::from_cartesian(&(radius * p_dir), &(circ_velocity * multiplier * v_dir), mu)
        };

        // Circular orbit
        let orbit = make_orbit(Vector3::y(), Vector3::z(), 1.0);
        assert_relative_eq!(orbit.energy(), -mu / 2.0 / radius);
        assert_relative_eq!(orbit.angular_momentum(), radius * circ_velocity);
        assert_relative_eq!(orbit.eccentricity(), 0.0);
        assert_relative_eq!(orbit.semimajor_axis(), radius);
        assert_relative_eq!(orbit.semilatus_rectum(), radius);
        assert_relative_eq!(orbit.periapsis(), radius);
        assert_relative_eq!(orbit.apoapsis(), radius);
        assert_relative_eq!(orbit.period().unwrap(), period, max_relative = 1e-15);

        // Parabolic orbit: escape velocity = mu/r * sqrt(2)
        let sqrt_2 = std::f64::consts::SQRT_2;
        let orbit = make_orbit(Vector3::z(), Vector3::x(), sqrt_2);
        assert_relative_eq!(orbit.energy(), 0.0, epsilon = 1e-5);
        assert_relative_eq!(orbit.angular_momentum(), radius * sqrt_2 * circ_velocity);
        assert_relative_eq!(orbit.eccentricity(), 1.0);
        assert_very_large!(orbit.semimajor_axis());
        assert_relative_eq!(orbit.semilatus_rectum(), 2.0 * radius);
        assert_relative_eq!(orbit.periapsis(), radius);
        assert_very_large!(orbit.apoapsis());

        // Radial orbit
        let orbit = make_orbit(Vector3::z(), Vector3::x(), 0.0);
        assert_relative_eq!(orbit.energy(), -mu / radius);
        assert_relative_eq!(orbit.angular_momentum(), 0.0);
        assert_relative_eq!(orbit.eccentricity(), 1.0);
        assert_relative_eq!(orbit.semimajor_axis(), radius / 2.0);
        assert_relative_eq!(orbit.semilatus_rectum(), 0.0);
        assert_relative_eq!(orbit.periapsis(), 0.0);
        assert_relative_eq!(orbit.apoapsis(), radius);
        assert_relative_eq!(
            orbit.period().unwrap(),
            period / f64::sqrt(8.0),
            max_relative = 1e-15
        );

        // Elliptic orbit
        let orbit = make_orbit(Vector3::y(), Vector3::x(), 1.2);
        assert!(orbit.energy() < 0.0);
        assert!(orbit.eccentricity() < 1.0);
        assert!(orbit.semimajor_axis() > radius);
        assert_relative_eq!(orbit.periapsis(), radius);
        assert!(orbit.apoapsis() > radius);
        assert!(orbit.period().is_some());

        // Elliptic orbit but the other way
        let orbit = make_orbit(Vector3::x(), Vector3::z(), 0.8);
        assert!(orbit.energy() < 0.0);
        assert!(orbit.eccentricity() < 1.0);
        assert!(orbit.semimajor_axis() < radius);
        assert!(orbit.periapsis() < radius);
        assert_relative_eq!(orbit.apoapsis(), radius);
        assert!(orbit.period().is_some());

        // Hyperbolic orbit
        let orbit = make_orbit(Vector3::y(), Vector3::x(), 1.5);
        assert!(orbit.energy() > 0.0);
        assert!(orbit.eccentricity() > 1.0);
        assert!(orbit.semimajor_axis() < 0.0);
        assert_relative_eq!(orbit.periapsis(), radius);
        assert!(orbit.apoapsis() < -radius); // negative but also < periapsis
        assert!(orbit.period().is_none());
    }

    #[test]
    fn test_orbit_angles() {
        let mu = consts::KERBOL_MU;
        let radius = consts::KERBIN_ORBIT_RADIUS;

        // First try an ellipse with moderate inclination, so we don't have any degeneracy
        let orbit = Orbit::from_kepler(radius, 0.3, PI / 10.0, PI / 6.0, PI / 4.0, mu);
        assert_relative_eq!(orbit.inclination(), PI / 10.0);
        assert_relative_eq!(orbit.long_asc_node(), PI / 6.0);
        assert_relative_eq!(orbit.arg_periapse(), PI / 4.0);

        // Now an ellipse with degenerate inclinations. For 0 inclination, we assume the ascending
        // node is the periapsis.
        let orbit = Orbit::from_kepler(radius, 0.3, 0.0, PI / 6.0, PI / 4.0, mu);
        assert_relative_eq!(orbit.inclination(), 0.0);
        assert_relative_eq!(orbit.long_asc_node(), PI * 5.0 / 12.0); // pi/6 + pi/4
        assert_relative_eq!(orbit.arg_periapse(), 0.0);

        // I'd test 180 inclination too, but sin(PI) != 0, so normalize actually succeeds lol
    }
}
