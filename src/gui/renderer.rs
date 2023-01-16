use kiss3d::camera::Camera;
use kiss3d::renderer::{LineRenderer, Renderer};
use nalgebra::{Isometry3, Point3, Unit, Vector3};

use super::renderers::{OrbitRenderer, SphereRenderer};
use super::OrbitPatch;

pub struct CompoundRenderer {
    sphere_renderer: SphereRenderer,
    orbit_renderer: OrbitRenderer,
    line_renderer: LineRenderer,
}

impl CompoundRenderer {
    pub fn new() -> Self {
        CompoundRenderer {
            sphere_renderer: SphereRenderer::new(),
            orbit_renderer: OrbitRenderer::new(),
            line_renderer: LineRenderer::new(),
        }
    }

    pub fn draw_grid(&mut self) {
        let num_squares = 20;
        let square_size = 1.0e9;
        let color = Point3::new(0.5, 0.5, 0.5);

        let max_coord = square_size * (num_squares as f32);
        for i in (-num_squares)..(num_squares + 1) {
            let coord = square_size * (i as f32);
            // horizontal
            self.line_renderer.draw_line(
                Point3::new(-max_coord, coord, 0.0),
                Point3::new(max_coord, coord, 0.0),
                color,
            );
            // vertical
            self.line_renderer.draw_line(
                Point3::new(coord, -max_coord, 0.0),
                Point3::new(coord, max_coord, 0.0),
                color,
            );
        }
    }

    pub fn draw_axes(
        &mut self,
        axes: &[(Unit<Vector3<f32>>, Point3<f32>)],
        axis_length: f32,
        transform: Isometry3<f32>,
    ) {
        let origin = transform * Point3::origin();
        for (v, color) in axes {
            let end_pt = origin + axis_length * (transform * v.into_inner());
            self.line_renderer.draw_line(origin, end_pt, *color);
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
        self.line_renderer.render(pass, camera);
    }
}
