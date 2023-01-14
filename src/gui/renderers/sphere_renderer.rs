use kiss3d::camera::Camera;
use kiss3d::renderer::{LineRenderer, Renderer};
use nalgebra::{Point3, Vector3};

use super::utils::{draw_path, path_iter_parametric};

struct SphereData {
    pub center: Point3<f32>,
    pub radius: f32,
    pub color: Point3<f32>,
}

pub struct SphereRenderer {
    line_renderer: LineRenderer,
    spheres: Vec<SphereData>,
}

impl SphereRenderer {
    pub fn new() -> Self {
        SphereRenderer {
            line_renderer: LineRenderer::new(),
            spheres: vec![],
        }
    }

    pub fn add_sphere(&mut self, center: Point3<f32>, radius: f32, color: Point3<f32>) {
        let sphere = SphereData {
            center,
            radius,
            color,
        };
        self.spheres.push(sphere);
    }

    fn load_sphere_into_renderer(
        line_renderer: &mut LineRenderer,
        camera: &dyn Camera,
        sphere: &SphereData,
    ) {
        // Transform the screen x and y vectors into whatever frame we're currently
        // focused on
        let camera_transform = camera.view_transform().inverse();
        let x_vec = camera_transform.transform_vector(&Vector3::x()).normalize();
        let y_vec = camera_transform.transform_vector(&Vector3::y()).normalize();

        let f = |theta: f32| {
            let v = x_vec * theta.cos() + y_vec * theta.sin();
            sphere.center + sphere.radius * v
        };
        draw_path(
            line_renderer,
            path_iter_parametric(f, 0.0, std::f32::consts::TAU, 100),
            &sphere.color,
        );
    }
}

impl Renderer for SphereRenderer {
    fn render(&mut self, pass: usize, camera: &mut dyn Camera) {
        for sphere in self.spheres.iter() {
            SphereRenderer::load_sphere_into_renderer(&mut self.line_renderer, camera, sphere);
        }
        self.line_renderer.render(pass, camera);
        self.spheres.clear();
    }
}
