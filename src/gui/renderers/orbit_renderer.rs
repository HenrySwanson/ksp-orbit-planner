use kiss3d::camera::Camera;
use kiss3d::renderer::{LineRenderer, Renderer};

use nalgebra::{Isometry3, Point3, Vector3};

use std::f64::consts::PI;

use crate::universe::{Orbit, OrbitPatch};

use super::utils::{draw_path, path_iter_parametric};

struct OrbitData {
    pub orbit: Orbit,
    pub start_anomaly: f64,
    pub end_anomaly: Option<f64>,
    pub color: Point3<f32>,
    pub transform: Isometry3<f32>,
}

pub struct OrbitRenderer {
    line_renderer: LineRenderer,
    orbits: Vec<OrbitData>,
}

impl OrbitRenderer {
    pub fn new() -> Self {
        OrbitRenderer {
            line_renderer: LineRenderer::new(),
            orbits: vec![],
        }
    }

    pub fn add_orbit(&mut self, orbit: OrbitPatch, color: Point3<f32>, transform: Isometry3<f32>) {
        let orbit = OrbitData {
            orbit: orbit.orbit,
            start_anomaly: orbit.start_anomaly,
            end_anomaly: orbit.end_anomaly,
            color,
            transform,
        };
        self.orbits.push(orbit);
    }

    fn load_orbit_into_renderer(line_renderer: &mut LineRenderer, orbit: &OrbitData) {
        // Find the starting and ending anomalies
        let start_s = orbit.start_anomaly;
        let end_s = match orbit.end_anomaly {
            Some(s) => s,
            None => {
                let beta = -2.0 * orbit.orbit.energy();
                if beta > 0.0 {
                    // Since this is an ellipse, the eccentric anomaly makes sense.
                    // We want E to increase by 2pi, and s = E / sqrt(beta)
                    start_s + 2.0 * PI / beta.sqrt()
                } else {
                    start_s + 1.0 // TODO: or whatever
                }
            }
        };
        assert!(end_s >= start_s);

        // Get some points around the orbit
        let f = |s| {
            let v = orbit.orbit.get_state(s).get_position();
            let v: Vector3<f32> = nalgebra::convert(v);
            orbit.transform * Point3::from(v)
        };
        draw_path(
            line_renderer,
            path_iter_parametric(f, start_s, end_s, 180),
            &orbit.color,
        );
    }
}

impl Renderer for OrbitRenderer {
    fn render(&mut self, pass: usize, camera: &mut dyn Camera) {
        for orbit in self.orbits.iter() {
            OrbitRenderer::load_orbit_into_renderer(&mut self.line_renderer, orbit);
        }
        self.line_renderer.render(pass, camera);
        self.orbits.clear();
    }
}
