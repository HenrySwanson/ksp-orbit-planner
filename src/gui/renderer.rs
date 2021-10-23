use kiss3d::camera::Camera;
use kiss3d::renderer::Renderer;

use nalgebra::{Isometry3, Point3};

use crate::orrery::OrbitPatch;

use super::renderers::{OrbitRenderer, SphereRenderer};

pub struct CompoundRenderer {
    sphere_renderer: SphereRenderer,
    orbit_renderer: OrbitRenderer,
}

impl CompoundRenderer {
    pub fn new() -> Self {
        CompoundRenderer {
            sphere_renderer: SphereRenderer::new(),
            orbit_renderer: OrbitRenderer::new(),
        }
    }

    pub fn draw_soi(&mut self, center: Point3<f32>, radius: f32, color: Point3<f32>) {
        self.sphere_renderer.add_sphere(center, radius, color);
    }

    pub fn draw_orbit(&mut self, orbit: OrbitPatch, color: Point3<f32>, transform: Isometry3<f32>) {
        self.orbit_renderer.add_orbit(orbit, color, transform);
    }
}

impl Renderer for CompoundRenderer {
    fn render(&mut self, pass: usize, camera: &mut dyn Camera) {
        self.sphere_renderer.render(pass, camera);
        self.orbit_renderer.render(pass, camera);
    }
}
