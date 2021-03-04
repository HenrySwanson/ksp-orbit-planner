use kiss3d::nalgebra as na;
use na::Vector3;

use crate::stumpff::stumpff_G;

pub struct State {
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    time: f64,
    mu: f64, // TODO: move this elsewhere
}

#[allow(non_snake_case)]
impl State {
    pub fn new(position: Vector3<f64>, velocity: Vector3<f64>, time: f64, mu: f64) -> Self {
        State {
            position,
            velocity,
            time,
            mu,
        }
    }

    pub fn get_position(&self) -> &Vector3<f64> {
        &self.position
    }

    pub fn get_velocity(&self) -> &Vector3<f64> {
        &self.velocity
    }

    pub fn get_energy(&self) -> f64 {
        // KE = 1/2 v^2, PE = - mu/r
        self.velocity.norm_squared() / 2.0 - self.mu / self.position.norm()
    }

    pub fn advance_s(&mut self, delta_s: f64) {
        let beta = -2.0 * self.get_energy();
        let mu = self.mu;
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
        self.time += delta_t;
    }
}
