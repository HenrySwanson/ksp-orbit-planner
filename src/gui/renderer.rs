use kiss3d::camera::Camera;
use kiss3d::renderer::{LineRenderer, Renderer};

use nalgebra::{Point3, Vector3};

struct SphereData {
    pub center: Point3<f32>,
    pub radius: f32,
    pub color: Point3<f32>,
}

struct SphereRenderer {
    line_renderer: LineRenderer,
    spheres: Vec<SphereData>,
}

pub struct CompoundRenderer {
    sphere_renderer: SphereRenderer,
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
        // Transform the screen x and y vectors into whatever frame we're currently focused on
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

impl CompoundRenderer {
    pub fn new() -> Self {
        CompoundRenderer {
            sphere_renderer: SphereRenderer::new(),
        }
    }

    pub fn draw_soi(&mut self, center: Point3<f32>, radius: f32, color: Point3<f32>) {
        self.sphere_renderer.add_sphere(center, radius, color);
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

impl Renderer for CompoundRenderer {
    fn render(&mut self, pass: usize, camera: &mut dyn Camera) {
        self.sphere_renderer.render(pass, camera);
    }
}

fn path_iter_parametric<F, S>(
    f: F,
    t_start: S,
    t_end: S,
    num_segments: usize,
) -> impl Iterator<Item = Point3<f32>>
where
    F: Fn(S) -> Point3<f32>,
    S: nalgebra::RealField + simba::scalar::SupersetOf<usize>,
{
    assert!(
        num_segments >= 1,
        "Must have at least one segment, num_segments was {}",
        num_segments
    );
    let convert = nalgebra::convert::<usize, S>;
    (0..=num_segments)
        .map(move |i| convert(i) / convert(num_segments))
        // u ranges from 0 to 1 (inclusive)
        .map(move |u| t_start + u * (t_end - t_start))
        .map(f)
}

fn draw_path<I: Iterator<Item = Point3<f32>>>(
    line_renderer: &mut LineRenderer,
    points: I,
    color: &Point3<f32>,
) {
    let mut prev_pt = None;
    for pt in points {
        if let Some(prev_pt) = prev_pt {
            line_renderer.draw_line(prev_pt, pt, color.clone());
        }
        prev_pt = Some(pt);
    }
}
