use nalgebra::{Point3, Vector3};

use super::event::EventPoint;
use super::orbit::Orbit;

use crate::math::geometry::directed_angle;
use crate::math::root_finding::{bisection, find_root_bracket, newton_plus_bisection, Bracket};
use crate::math::stumpff::stumpff_G;

const NUM_ITERATIONS_DELTA_T: usize = 200;
const NUM_ITERATIONS_SOI_ENCOUNTER: usize = 1000;

// TODO pub these fields
#[derive(Debug, Clone)]
pub struct CartesianState {
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    parent_mu: f64,
    // TODO: maybe track time that we couldn't account for in advance_s?
}

impl CartesianState {
    pub fn new(position: Vector3<f64>, velocity: Vector3<f64>, parent_mu: f64) -> Self {
        CartesianState {
            position,
            velocity,
            parent_mu,
        }
    }

    pub fn get_position(&self) -> Vector3<f64> {
        self.position
    }

    pub fn get_velocity(&self) -> Vector3<f64> {
        self.velocity
    }

    pub fn get_mu(&self) -> f64 {
        self.parent_mu
    }

    pub fn get_orbit(&self) -> Orbit {
        Orbit::from_cartesian(&self.get_position(), &self.get_velocity(), self.get_mu())
    }

    pub fn get_universal_anomaly(&self) -> f64 {
        // TODO make this work for radial orbits too!
        let orbit = self.get_orbit();
        let my_theta = self.get_theta();
        orbit.true_to_universal(my_theta)
    }

    fn get_energy(&self) -> f64 {
        // KE = 1/2 v^2, PE = - mu/r
        self.velocity.norm_squared() / 2.0 - self.parent_mu / self.position.norm()
    }

    #[allow(non_snake_case)]
    pub fn update_s(&mut self, delta_s: f64) -> f64 {
        let beta = -2.0 * self.get_energy();
        let mu = self.parent_mu;
        let G: [f64; 4] = stumpff_G(beta, delta_s);

        let r_0 = self.position.norm();
        let r_dot_0 = self.position.dot(&self.velocity) / r_0;

        let f = 1.0 - mu / r_0 * G[2];
        let g = r_0 * G[1] + r_0 * r_dot_0 * G[2];

        let new_position = f * self.position + g * self.velocity;
        let new_r = new_position.norm();

        let f_dot = -mu / r_0 / new_r * G[1];
        let g_dot = r_0 / new_r * (G[0] + r_dot_0 * G[1]);

        let new_velocity = f_dot * self.position + g_dot * self.velocity;
        let delta_t = r_0 * G[1] + r_0 * r_dot_0 * G[2] + mu * G[3];

        self.position = new_position;
        self.velocity = new_velocity;

        // Return the elapsed time, in case anyone's interested
        delta_t
    }

    #[allow(non_snake_case)]
    pub fn update_t(&mut self, delta_t: f64) {
        // We find the delta_s corresponding to the given delta_t, and advance using that.
        // Since ds/dt = 1/r, s and t are monotonically related, so there's a unique solution.
        if delta_t == 0.0 {
            return;
        }

        // Grab some constants
        let beta = -2.0 * self.get_energy();
        let mu = self.parent_mu;
        let r_0 = self.position.norm();
        let r_dot_0 = self.position.dot(&self.velocity) / r_0;

        // We want to find a root of this function, which is monotonically increasing:
        //  r_0 * G_1(β, s) + r_0 * r_dot_0 * G_2(β, s) + mu * G_3(β, s) - delta_t
        let f_and_f_prime = |s: f64| {
            let G = stumpff_G(beta, s);
            let f = r_0 * G[1] + r_0 * r_dot_0 * G[2] + mu * G[3] - delta_t;
            let f_prime = r_0 * G[0] + r_0 * r_dot_0 * G[1] + mu * G[2];
            (f, f_prime)
        };

        // TODO if these fail, we need to log the parameters somewhere :\
        let bracket = find_root_bracket(
            |x| f_and_f_prime(x).0,
            delta_t / r_0,
            delta_t / r_0,
            NUM_ITERATIONS_DELTA_T,
        );
        let root = newton_plus_bisection(f_and_f_prime, bracket, NUM_ITERATIONS_DELTA_T);

        self.update_s(root);
    }

    // TODO: name this better >:(
    pub fn clone_update_t(&self, time: f64) -> Self {
        let mut copy = self.clone();
        copy.update_t(time);
        copy
    }

    #[allow(non_snake_case)]
    pub fn delta_s_to_t(&self, delta_s: f64) -> f64 {
        let r_0 = self.position.norm();
        let r_dot_0 = self.position.dot(&self.velocity) / r_0;
        let beta = -2.0 * self.get_energy();
        let mu = self.parent_mu;
        let G = stumpff_G(beta, delta_s);

        r_0 * G[1] + r_0 * r_dot_0 * G[2] + mu * G[3]
    }

    pub fn get_theta(&self) -> f64 {
        let orbit = self.get_orbit();
        let x_vec = orbit.periapse_vector();
        let z_vec = orbit.normal_vector();
        directed_angle(&x_vec, &self.position, &z_vec)
    }

    fn get_s_until_radius(&self, radius: f64) -> Option<f64> {
        // TODO this doesn't work well for radial orbits, try to adjust it so that it does
        let orbit = self.get_orbit();
        let current_s = self.get_universal_anomaly();

        // Since (h^2/mu) / (1 + e cos theta) = r, we can invert that to get
        // a desired theta, which will always be in the first or second quadrant
        let target_cos_theta = (orbit.semilatus_rectum() / radius - 1.0) / orbit.eccentricity();
        if target_cos_theta.abs() > 1.0 {
            return None;
        }

        let target_theta = target_cos_theta.acos();
        let target_s = orbit.true_to_universal(target_theta);
        let delta_s = target_s - current_s;

        Some(delta_s)
    }

    pub fn get_t_until_radius(&self, radius: f64) -> Option<f64> {
        self.get_s_until_radius(radius)
            .map(|s| self.delta_s_to_t(s))
    }

    pub fn find_soi_escape_event(&self, soi_radius: f64, current_time: f64) -> Option<EventPoint> {
        let delta_s = match self.get_s_until_radius(soi_radius) {
            Some(s) => s,
            None => return None,
        };

        let delta_t = self.delta_s_to_t(delta_s);

        let target_s = self.get_universal_anomaly() + delta_s;
        let new_state = self.get_orbit().get_state(target_s);
        let event_pt = EventPoint {
            time: current_time + delta_t,
            anomaly: target_s,
            location: Point3::from(new_state.get_position()),
        };

        Some(event_pt)
    }

    pub fn find_soi_encounter_event(
        &self,
        planet_state: &CartesianState,
        soi_radius: f64,
        current_time: f64,
        window: f64,
    ) -> Option<EventPoint> {
        let self_orbit = self.get_orbit();
        let planet_orbit = planet_state.get_orbit();

        // Quick check: if one orbit is much smaller than the other, then we can skip the rest
        let pe_ap_check = |o1: &Orbit, o2: &Orbit| {
            o1.apoapsis() > 0.0 && o1.apoapsis() + soi_radius < o2.periapsis()
        };
        if pe_ap_check(&self_orbit, &planet_orbit) || pe_ap_check(&planet_orbit, &self_orbit) {
            return None;
        }

        // Method for checking distance between bodies
        let check_distance = |time| {
            let new_self_pos = self.clone_update_t(time).get_position();
            let new_planet_pos = planet_state.clone_update_t(time).get_position();
            (new_self_pos - new_planet_pos).norm()
        };

        // TODO have better algorithm than this!

        // We know their maximum relative velocity
        let self_max_velocity =
            self.parent_mu * (1.0 + self_orbit.eccentricity()) / self_orbit.angular_momentum();
        let planet_max_velocity =
            self.parent_mu * (1.0 + planet_orbit.eccentricity()) / planet_orbit.angular_momentum();
        let max_rel_velocity = self_max_velocity.abs() + planet_max_velocity.abs();
        let current_distance = check_distance(0.0);

        // If we can't possibly catch up, then return no event.
        if current_distance - soi_radius > max_rel_velocity * window {
            return None;
        }

        // Just step by step look for an intersection
        let num_slices = 1000;
        let slice_duration = window / (num_slices as f64);
        let mut encounter_time = None;
        for i in 0..num_slices {
            let t = i as f64 * slice_duration;
            if check_distance(t) < soi_radius {
                encounter_time = Some(t);
                break;
            }
        }

        // If we didn't get close enough, return no event
        let t2 = match encounter_time {
            Some(t) => t,
            None => return None,
        };

        // Edge case: if t2 is zero, we may have just exited an SOI, and shouldn't
        // return an event, since we'd then get into a loop
        if t2 == 0.0 {
            return None;
        }

        // Now we narrow in on the point using binary search.
        let t1 = t2 - slice_duration;
        let entry_time = bisection(
            |t| check_distance(t) - soi_radius,
            Bracket::new(t1, t2),
            NUM_ITERATIONS_SOI_ENCOUNTER,
        );

        // Lastly, figure out anomaly and position at that point
        // TODO: new s can end up less than old s if we're not careful :\
        let new_state = self.clone_update_t(entry_time);
        Some(EventPoint {
            time: current_time + entry_time,
            anomaly: new_state.get_universal_anomaly(),
            location: Point3::from(new_state.get_position()),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::consts::{
        get_circular_velocity, get_period, KERBIN_ORBIT_PERIOD, KERBIN_ORBIT_RADIUS, KERBOL_MU,
    };

    use std::f64::consts::PI;

    // TODO use approx::assert_relative_eq!

    fn assert_close(expected: f64, actual: f64, tolerance: f64) {
        let difference = actual - expected;
        if difference.abs() >= tolerance * expected.abs() {
            panic!("Floats expected to be close: {} vs. {}", actual, expected);
        }
    }

    // We'll count this as a success if the difference between the vectors is small,
    // relative to the length of the expected vector.
    fn assert_vectors_close(expected: &Vector3<f64>, actual: &Vector3<f64>, tolerance: f64) {
        let difference = actual - expected;
        if difference.norm() >= tolerance * expected.norm() {
            panic!(
                "Vectors were not as close as expected!\n
                Expected: {}\n
                Received: {}\n
                Difference: {}\n
                Relative difference: {:e}",
                expected,
                actual,
                difference,
                difference.norm() / expected.norm(),
            );
        }
    }

    #[test]
    fn test_kerbin() {
        // Build Kerbin and see if the orbit simulation is right.
        // Kerbin's orbit is perfectly circular.
        let initial_position = Vector3::x() * KERBIN_ORBIT_RADIUS;
        let initial_velocity = Vector3::y() * get_circular_velocity(KERBIN_ORBIT_RADIUS, KERBOL_MU);

        let mut state = CartesianState::new(initial_position, initial_velocity, KERBOL_MU);
        let mut elapsed_time = 0.0;

        // Advance for one full orbit.
        // This is a circular orbit, so s is proportional to theta. Specifically,
        // s = theta / sqrt(beta).
        let beta = -2.0 * state.get_energy();
        let s = 2.0 * PI / beta.sqrt();
        elapsed_time += state.update_s(s);

        // We expect these to be extremely close, since we got s from the orbit itself
        assert_vectors_close(&initial_position, &state.get_position(), 1e-14);
        assert_vectors_close(&initial_velocity, &state.get_velocity(), 1e-14);

        // Time is a little fuzzier, because the velocity constant (and thus this orbit) isn't
        // perfect.
        let computed_period = get_period(KERBIN_ORBIT_RADIUS, KERBOL_MU);
        println!(
            "Periods:\n\
            \t(defined)   {}\n\
            \t(computed)  {}\n\
            \t(simulated) {}",
            KERBIN_ORBIT_PERIOD, computed_period, elapsed_time,
        );
        assert_close(KERBIN_ORBIT_PERIOD, elapsed_time, 1e-6);
    }

    #[test]
    fn test_perfect_circle() {
        // Build a perfectly circular orbit, and test what it looks like as we move
        // incrementally around.
        let radius = 10.0 * KERBIN_ORBIT_RADIUS; // some number
        let velocity = (KERBOL_MU / radius).sqrt(); // v^2 = mu (2/r - 1/a) = mu / r

        let initial_position = Vector3::x() * radius;
        let initial_velocity = Vector3::z() * velocity;
        let mut state = CartesianState::new(initial_position, initial_velocity, KERBOL_MU);
        let mut elapsed_time = 0.0;

        // Compute s for a whole orbit. Since r doesn't change, s varies linearly with t.
        let beta = -2.0 * state.get_energy();
        let s = 2.0 * PI / beta.sqrt();

        let num_points = 1000;
        for i in 0..num_points {
            let num_points = num_points as f64;

            elapsed_time += state.update_s(s / num_points);

            let theta = 2.0 * PI * (i + 1) as f64 / num_points;
            let expected = radius * Vector3::new(theta.cos(), 0.0, theta.sin());
            assert_vectors_close(&expected, &state.get_position(), 1e-14);
        }

        // Check that the expected amount of time has elapsed. Not sure why we get slightly
        // less precision here, but whatever. Maybe it's just adding something to itself
        // a thousand times.
        let computed_period = 2.0 * PI * radius / velocity;
        assert_close(computed_period, elapsed_time, 1e-12);
    }
}
