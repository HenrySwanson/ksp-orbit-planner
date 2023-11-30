use kiss3d::camera::Camera;
use kiss3d::renderer::{LineRenderer, Renderer};
use nalgebra::{Isometry3, Point3, Unit, Vector3};

use self::marker_renderer::MarkerRenderer;
use self::orbit_renderer::OrbitRenderer;
use self::sphere_renderer::SphereRenderer;

mod marker_renderer;
mod orbit_renderer;
mod sphere_renderer;
mod utils;

pub use orbit_renderer::OrbitPatch;

pub struct CompoundRenderer {
    sphere_renderer: SphereRenderer,
    orbit_renderer: OrbitRenderer,
    line_renderer: LineRenderer,
    marker_renderer: MarkerRenderer,
}

impl CompoundRenderer {
    pub fn new() -> Self {
        CompoundRenderer {
            sphere_renderer: SphereRenderer::new(),
            orbit_renderer: OrbitRenderer::new(),
            line_renderer: LineRenderer::new(),
            marker_renderer: MarkerRenderer::new(),
        }
    }

    pub fn draw_grid(&mut self, camera_distance: f32) {
        const NUM_SQUARES: i32 = 40;

        // We use the camera distance to determine the grid spacing.
        // If G is the grid subdivision, and R is a corrective factor, if the camera is
        // between G^k and G^(k+1) away, we draw a bright grid with spacing
        // R*G^(k+1) and a dimmer one with spacing G^k, getting dimmer as we get
        // closer to R*G^(k+1).
        const GRID_SUBDIV: i32 = 10;
        const GRID_SUBDIV_FLOAT: f32 = GRID_SUBDIV as f32;
        const CORRECTIVE: f32 = 0.4;

        // Figure out which interval we're in and how far along
        let log_distance = camera_distance.log(GRID_SUBDIV_FLOAT);
        let k = camera_distance.log(GRID_SUBDIV_FLOAT).floor();
        let interp = log_distance - k;

        // Determine square size and color
        let square_size = CORRECTIVE * GRID_SUBDIV_FLOAT.powf(k);
        let color = Point3::new(0.5, 0.5, 0.5);
        let dim_color = color * (1.0 - interp);

        // Draw the squares
        let max_coord = square_size * (NUM_SQUARES as f32);
        for i in (-NUM_SQUARES)..(NUM_SQUARES + 1) {
            let coord = square_size * (i as f32);
            let color = if i % GRID_SUBDIV == 0 {
                color
            } else {
                dim_color
            };

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

    pub fn draw_marker(&mut self, center: Point3<f32>, height: f32, color: Point3<f32>) {
        self.marker_renderer.add_marker(center, height, color);
    }
}

impl Renderer for CompoundRenderer {
    fn render(&mut self, pass: usize, camera: &mut dyn Camera) {
        self.sphere_renderer.render(pass, camera);
        self.orbit_renderer.render(pass, camera);
        self.line_renderer.render(pass, camera);
        self.marker_renderer.render(pass, camera);
    }
}
