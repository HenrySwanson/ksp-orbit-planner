use std::f64::consts::PI;

use nalgebra::{Rotation3, Unit, Vector3};

use crate::math::geometry::{always_find_rotation, directed_angle};

mod state_methods;
mod timed_orbit;

pub use timed_orbit::TimedOrbit;

// Newton's gravitational constant, in N m^2 / kg^2
pub const NEWTON_G: f64 = 6.6743015e-11;

#[derive(Debug, Clone, Copy)]
pub struct PointMass(f64);

#[derive(Debug, Clone, Copy)]
pub struct Orbit<P, S> {
    primary: P,
    secondary: S,
    /// Encodes the orientation of the orbit: it moves the xy plane to the
    /// orbital plane, and x to point towards periapsis.
    rotation: Rotation3<f64>,
    /// (semimajor axis)^-1. It's easier to use this instead of a directly,
    /// because in parabolic orbits, a = infty.
    /// TODO: use apsis instead? might result in no more invalid regions...
    alpha: f64,
    /// Semi-latus rectum
    slr: f64,
}

pub trait HasMass {
    fn mu(&self) -> f64;

    fn mass(&self) -> f64 {
        self.mu() / NEWTON_G
    }
}

impl PointMass {
    pub fn with_mu(mu: f64) -> Self {
        Self(mu)
    }

    pub fn mu(&self) -> f64 {
        self.0
    }
}

impl HasMass for PointMass {
    fn mu(&self) -> f64 {
        self.0
    }
}

impl<T> HasMass for &T
where
    T: HasMass,
{
    fn mu(&self) -> f64 {
        (*self).mu()
    }
}

pub type BareOrbit = Orbit<(), ()>;
pub type PhysicalOrbit = Orbit<PointMass, ()>;

impl<P, S> Orbit<P, S> {
    pub fn primary(&self) -> &P {
        &self.primary
    }

    pub fn secondary(&self) -> &S {
        &self.secondary
    }

    pub fn as_ref(&self) -> Orbit<&P, &S> {
        Orbit {
            primary: &self.primary,
            secondary: &self.secondary,
            rotation: self.rotation,
            alpha: self.alpha,
            slr: self.slr,
        }
    }

    pub fn with_primary<P2>(self, new_primary: P2) -> Orbit<P2, S> {
        self.map_primary(|_| new_primary)
    }

    pub fn with_secondary<S2>(self, new_secondary: S2) -> Orbit<P, S2> {
        self.map_secondary(|_| new_secondary)
    }

    pub fn map_primary<P2>(self, primary_fn: impl FnOnce(P) -> P2) -> Orbit<P2, S> {
        self.map(primary_fn, std::convert::identity)
    }

    pub fn map_secondary<S2>(self, secondary_fn: impl FnOnce(S) -> S2) -> Orbit<P, S2> {
        self.map(std::convert::identity, secondary_fn)
    }

    pub fn to_bare(&self) -> BareOrbit {
        Orbit {
            primary: (),
            secondary: (),
            rotation: self.rotation,
            alpha: self.alpha,
            slr: self.slr,
        }
    }

    fn map<P2, S2>(
        self,
        primary_fn: impl FnOnce(P) -> P2,
        secondary_fn: impl FnOnce(S) -> S2,
    ) -> Orbit<P2, S2> {
        Orbit {
            primary: primary_fn(self.primary),
            secondary: secondary_fn(self.secondary),
            rotation: self.rotation,
            alpha: self.alpha,
            slr: self.slr,
        }
    }

    pub fn from_kepler(
        primary: P,
        secondary: S,
        a: f64,
        ecc: f64,
        incl: f64,
        lan: f64,
        argp: f64,
    ) -> Self {
        Orbit {
            primary,
            secondary,
            rotation: rotation_from_angles(incl, lan, argp),
            alpha: a.recip(),
            slr: a * (1.0 - ecc * ecc),
        }
    }

    // -- Axes and vectors --

    pub fn rotation(&self) -> Rotation3<f64> {
        self.rotation
    }

    pub fn periapse_vector(&self) -> Unit<Vector3<f64>> {
        self.rotation() * Vector3::x_axis()
    }

    pub fn normal_vector(&self) -> Unit<Vector3<f64>> {
        self.rotation() * Vector3::z_axis()
    }

    pub fn asc_node_vector(&self) -> Unit<Vector3<f64>> {
        // TODO: the ambiguity here makes me think i might wanna just store the angles
        let v = Vector3::z().cross(&self.normal_vector());
        Unit::try_new(v, 1e-20).unwrap_or_else(|| self.periapse_vector())
    }

    // -- Orbital elements --

    pub fn semimajor_axis(&self) -> f64 {
        self.alpha.recip()
    }

    pub fn eccentricity(&self) -> f64 {
        // l = a(1-e^2), so e^2 = 1 - l/a
        let e_squared = 1.0 - self.slr * self.alpha;

        if e_squared >= 0.0 {
            e_squared.sqrt()
        } else if e_squared > -1e-9 {
            // If we're just barely below zero, round up to zero.
            0.0
        } else {
            panic!(
                "Illegal orbit configuration: alpha = {}, slr = {}, e^2 computed as {}",
                self.alpha, self.slr, e_squared
            )
        }
    }

    pub fn inclination(&self) -> f64 {
        // Inclination is the angle the normal makes with z
        self.normal_vector().angle(&Vector3::z())
    }

    pub fn long_asc_node(&self) -> f64 {
        // Longitude of ascending node is the directed angle from x to the ascending
        // node
        directed_angle(&Vector3::x(), &self.asc_node_vector(), &Vector3::z())
    }

    pub fn arg_periapse(&self) -> f64 {
        // Argument of periapsis is the directed angle from the ascending node to the
        // periapsis
        directed_angle(
            &self.asc_node_vector(),
            &self.periapse_vector(),
            &self.normal_vector(),
        )
    }

    // -- Other geometric characteristics --

    pub fn is_closed(&self) -> bool {
        self.alpha > 0.0
    }

    pub fn semilatus_rectum(&self) -> f64 {
        self.slr
    }

    pub fn periapsis(&self) -> f64 {
        // the periapsis is a(1-e), but when e = 1 that's got problems
        // a(1-e) = a(1-e^2)/(1+e) = l / (1+e)
        self.slr / (1.0 + self.eccentricity())
    }

    pub fn apoapsis(&self) -> Option<f64> {
        if self.is_closed() {
            Some(2.0 * self.semimajor_axis() - self.periapsis())
        } else {
            None
        }
    }
}

impl<P: HasMass, S> Orbit<P, S> {
    pub fn to_physical(&self) -> PhysicalOrbit {
        Orbit {
            primary: PointMass(self.primary.mu()),
            secondary: (),
            rotation: self.rotation,
            alpha: self.alpha,
            slr: self.slr,
        }
    }

    pub fn from_cartesian(
        primary: P,
        secondary: S,
        position: &Vector3<f64>,
        velocity: &Vector3<f64>,
    ) -> Self {
        // Compute some physical quantities for the orbit.
        let mu = primary.mu();
        let r = position.norm();
        let energy = velocity.norm_squared() / 2.0 - mu / r;
        let ang_mom = position.cross(velocity);

        // LRL vector = v x h / mu - r/|r|
        let lrl = velocity.cross(&ang_mom) / mu - position / r;

        // We want to rotate this orbit into a standard frame. Unfortunately, this
        // might be ambiguous, if either angular momentum or the LRL vector are too
        // close to zero. So we use a particularly cautious method.
        let rotation = always_find_rotation(&ang_mom, &lrl, 1e-20);

        Self {
            primary,
            secondary,
            rotation,
            alpha: -2.0 * energy / mu,
            slr: ang_mom.norm_squared() / mu,
        }
    }

    // -- Physical orbital characteristics --

    pub fn energy(&self) -> f64 {
        // -2E = mu / a
        -self.primary.mu() * self.alpha / 2.0
    }

    pub fn beta(&self) -> f64 {
        self.primary.mu() * self.alpha
    }

    pub fn angular_momentum(&self) -> f64 {
        // l = h^2/mu
        (self.slr * self.primary.mu()).sqrt()
    }

    pub fn period(&self) -> Option<f64> {
        if self.is_closed() {
            Some(2.0 * PI * (self.semimajor_axis().powi(3) / self.primary.mu()).sqrt())
        } else {
            None
        }
    }

    pub fn periapsis_velocity(&self) -> f64 {
        // Since h = r cross v, which are perpendicular at apeses
        self.angular_momentum() / self.periapsis()
    }

    pub fn apoapsis_velocity(&self) -> Option<f64> {
        self.apoapsis().map(|r_a| self.angular_momentum() / r_a)
    }

    pub fn excess_velocity(&self) -> Option<f64> {
        if self.is_closed() {
            None
        } else {
            Some((2.0 * self.energy()).sqrt())
        }
    }
}

impl<P: HasMass, S: HasMass> Orbit<P, S> {
    pub fn soi_radius(&self) -> f64 {
        let mu_1 = self.primary.mu();
        let mu_2 = self.secondary.mu();

        let sma = self.semimajor_axis();
        assert!(
            sma > 0.0,
            "SOI radius approximation only works with elliptical orbits"
        );

        sma * (mu_2 / mu_1).powf(0.4)
    }
}

fn rotation_from_angles(incl: f64, lan: f64, argp: f64) -> Rotation3<f64> {
    // We have an orbit in the xy plane where the periapsis is pointed along the
    // x-axis. So first, we rotate it around z until the periapsis is at argp
    // away from the x-axis (which will now be the ascending node). We then
    // rotate around x to get the inclination, and then one final turn around z
    // to get the correct longitude of the AN.
    Rotation3::from_axis_angle(&Vector3::z_axis(), lan)
        * Rotation3::from_axis_angle(&Vector3::x_axis(), incl)
        * Rotation3::from_axis_angle(&Vector3::z_axis(), argp)
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;
    use crate::testing_utils::assert_very_large;

    #[test]
    fn test_orbit_shape() {
        let make_orbit = |alpha, slr| Orbit {
            primary: (),
            secondary: (),
            rotation: Rotation3::identity(),
            alpha,
            slr,
        };

        // Circular orbit
        let orbit = make_orbit(0.1, 10.0);
        assert!(orbit.is_closed());
        assert_relative_eq!(orbit.semimajor_axis(), 10.0);
        assert_relative_eq!(orbit.eccentricity(), 0.0);
        assert_relative_eq!(orbit.semilatus_rectum(), 10.0);
        assert_relative_eq!(orbit.periapsis(), 10.0);
        assert_relative_eq!(orbit.apoapsis().unwrap(), 10.0);

        // Parabolic orbit
        let orbit = make_orbit(0.0, 10.0);
        assert!(!orbit.is_closed());
        assert_very_large!(orbit.semimajor_axis());
        assert_relative_eq!(orbit.eccentricity(), 1.0);
        assert_relative_eq!(orbit.semilatus_rectum(), 10.0);
        assert_relative_eq!(orbit.periapsis(), 5.0);
        assert_eq!(orbit.apoapsis(), None);

        // Radial orbit
        let orbit = make_orbit(0.1, 0.0);
        assert!(orbit.is_closed());
        assert_relative_eq!(orbit.semimajor_axis(), 10.0);
        assert_relative_eq!(orbit.eccentricity(), 1.0);
        assert_relative_eq!(orbit.semilatus_rectum(), 0.0);
        assert_relative_eq!(orbit.periapsis(), 0.0);
        assert_relative_eq!(orbit.apoapsis().unwrap(), 20.0);

        // Elliptic orbit, e = 3/5
        // l = a(1-e^2)
        let orbit = make_orbit(0.1, 6.4);
        assert!(orbit.is_closed());
        assert_relative_eq!(orbit.semimajor_axis(), 10.0);
        assert_relative_eq!(orbit.eccentricity(), 0.6);
        assert_relative_eq!(orbit.semilatus_rectum(), 6.4);
        assert_relative_eq!(orbit.periapsis(), 4.0);
        assert_relative_eq!(orbit.apoapsis().unwrap(), 16.0);

        // Hyperbolic orbit, e = 5/3
        let orbit = make_orbit(-1.0 / 9.0, 16.0);
        assert!(!orbit.is_closed());
        assert_relative_eq!(orbit.semimajor_axis(), -9.0);
        assert_relative_eq!(orbit.eccentricity(), 5.0 / 3.0);
        assert_relative_eq!(orbit.semilatus_rectum(), 16.0);
        assert_relative_eq!(orbit.periapsis(), 6.0);
        assert_eq!(orbit.apoapsis(), None);
    }

    #[test]
    fn test_orbit_angles() {
        // Ellipse with eccentricity 3/5 and SMA = 10
        let make_ellipse_orbit = |rotation| Orbit {
            primary: (),
            secondary: (),
            rotation,
            alpha: 0.1,
            slr: 6.4,
        };

        // For making nice numbers that are easy to understand
        fn rotation_from_angles_degrees(incl: f64, lan: f64, argp: f64) -> Rotation3<f64> {
            rotation_from_angles(incl.to_radians(), lan.to_radians(), argp.to_radians())
        }

        // Orbit in the standard orientation
        let orbit = make_ellipse_orbit(Rotation3::identity());
        assert_relative_eq!(orbit.inclination(), 0.0);
        assert_relative_eq!(orbit.long_asc_node(), 0.0);
        assert_relative_eq!(orbit.arg_periapse(), 0.0);

        // Just some arbitrary orientation
        let orbit = make_ellipse_orbit(rotation_from_angles_degrees(15.0, 30.0, 45.0));
        assert_relative_eq!(orbit.inclination().to_degrees(), 15.0, max_relative = 1e-15);
        assert_relative_eq!(
            orbit.long_asc_node().to_degrees(),
            30.0,
            max_relative = 1e-15
        );
        assert_relative_eq!(
            orbit.arg_periapse().to_degrees(),
            45.0,
            max_relative = 1e-15
        );

        // TODO: more tests here...
        // including a test of rotation_from_angles!
    }

    #[test]
    fn test_kepler_constructor() {
        // TODO: more test cases! make sure to cover the edge cases

        let data = [
            // Some standard elliptical orbit
            (100.0, 0.4, 10.0_f64, 130.0_f64, 25.0_f64),
        ];

        for (a, ecc, incl, lan, argp) in data {
            let orbit = Orbit::from_kepler(
                (),
                (),
                a,
                ecc,
                incl.to_radians(),
                lan.to_radians(),
                argp.to_radians(),
            );
            assert_relative_eq!(orbit.semimajor_axis(), a);
            assert_relative_eq!(orbit.eccentricity(), ecc);
            assert_relative_eq!(orbit.inclination().to_degrees(), incl, max_relative = 1e-10);
            assert_relative_eq!(
                orbit.long_asc_node().to_degrees(),
                lan,
                max_relative = 1e-10
            );
            assert_relative_eq!(
                orbit.arg_periapse().to_degrees(),
                argp,
                max_relative = 1e-10
            );
        }
    }

    // TODO: reduce this test a bit
    #[test]
    fn test_cartesian_constructor() {
        let mu = crate::consts::KERBOL_MU;
        let radius = crate::consts::KERBIN_ORBIT_RADIUS;
        let circ_velocity = (mu / radius).sqrt();

        let make_orbit = |p_dir, v_dir, multiplier| {
            Orbit::from_cartesian(
                PointMass::with_mu(mu),
                (),
                &(radius * p_dir),
                &(circ_velocity * multiplier * v_dir),
            )
        };

        // Circular orbit
        let orbit = make_orbit(Vector3::y(), Vector3::z(), 1.0);
        assert_relative_eq!(orbit.semimajor_axis(), radius, max_relative = 1e-15);
        assert_relative_eq!(orbit.eccentricity(), 0.0, epsilon = 1e-7);
        assert_relative_eq!(orbit.inclination(), PI / 2.0);
        assert_relative_eq!(orbit.long_asc_node(), PI / 2.0);
        assert_relative_eq!(orbit.arg_periapse(), 0.0);

        // Parabolic orbit: escape velocity = mu/r * sqrt(2)
        let sqrt_2 = std::f64::consts::SQRT_2;
        let orbit = make_orbit(Vector3::z(), Vector3::x(), sqrt_2);
        assert_very_large!(orbit.semimajor_axis());
        assert_relative_eq!(orbit.eccentricity(), 1.0);
        assert_relative_eq!(orbit.inclination(), PI / 2.0);
        assert_relative_eq!(orbit.long_asc_node(), PI);
        assert_relative_eq!(orbit.arg_periapse(), PI / 2.0);

        // Radial orbit
        let orbit = make_orbit(Vector3::z(), Vector3::x(), 0.0);
        assert_relative_eq!(orbit.semimajor_axis(), radius / 2.0, max_relative = 1e-15);
        assert_relative_eq!(orbit.eccentricity(), 1.0);
        // TODO: double check why these are they way they are!
        assert_relative_eq!(orbit.inclination(), PI / 2.0);
        assert_relative_eq!(orbit.long_asc_node(), PI);
        assert_relative_eq!(orbit.arg_periapse(), 3.0 * PI / 2.0);

        // Elliptic orbit
        let orbit = make_orbit(Vector3::y(), Vector3::x(), 1.2);
        assert!(orbit.semimajor_axis() > radius);
        assert!(orbit.eccentricity() < 1.0);
        assert_relative_eq!(orbit.inclination(), PI);
        assert_relative_eq!(orbit.long_asc_node(), PI / 2.0);
        assert_relative_eq!(orbit.arg_periapse(), 0.0);

        // Elliptic orbit but the other way
        let orbit = make_orbit(Vector3::y(), Vector3::x(), 0.8);
        assert!(orbit.semimajor_axis() < radius);
        assert!(orbit.eccentricity() < 1.0);
        assert_relative_eq!(orbit.inclination(), PI);
        assert_relative_eq!(orbit.long_asc_node(), 3.0 * PI / 2.0);
        assert_relative_eq!(orbit.arg_periapse(), 0.0);

        // Hyperbolic orbit
        let orbit = make_orbit(Vector3::x(), Vector3::z(), 1.5);
        assert!(orbit.semimajor_axis() < 0.0);
        assert!(orbit.eccentricity() > 1.0);
        assert_relative_eq!(orbit.inclination(), PI / 2.0);
        assert_relative_eq!(orbit.long_asc_node(), 0.0);
        assert_relative_eq!(orbit.arg_periapse(), 2.0 * PI);
    }

    #[test]
    fn test_physical_quantities() {
        use crate::consts::{KERBIN_MU, KERBIN_ORBIT_RADIUS, KERBOL_MU};

        let kerbin_orbit = Orbit::from_kepler(
            PointMass(KERBOL_MU),
            PointMass(KERBIN_MU),
            KERBIN_ORBIT_RADIUS,
            0.0,
            0.0,
            0.0,
            0.0,
        );
        // Energy in a circular orbit is: -mu/2r
        assert_relative_eq!(
            kerbin_orbit.energy(),
            -KERBOL_MU / 2.0 / KERBIN_ORBIT_RADIUS
        );
        // Angular momentum in a circular orbit is: sqrt(mu * r)
        assert_relative_eq!(
            kerbin_orbit.angular_momentum(),
            (KERBOL_MU * KERBIN_ORBIT_RADIUS).sqrt()
        );
        assert_relative_eq!(kerbin_orbit.period().unwrap(), 9_203_545.0, epsilon = 1.0);
        assert_relative_eq!(kerbin_orbit.periapsis_velocity(), 9_285.0, epsilon = 1.0);
        assert_relative_eq!(
            kerbin_orbit.apoapsis_velocity().unwrap(),
            9_285.0,
            epsilon = 1.0
        );
        assert_relative_eq!(kerbin_orbit.soi_radius(), 84_159_286.0, epsilon = 1.0);
    }
}
