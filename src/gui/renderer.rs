use kiss3d::camera::Camera;
use kiss3d::renderer::{LineRenderer, Renderer};

use nalgebra::{Isometry3, Point3, Vector3};

use std::f64::consts::PI;

use crate::universe::{Orbit, OrbitPatch};

struct SphereData {
    pub center: Point3<f32>,
    pub radius: f32,
    pub color: Point3<f32>,
}

struct SphereRenderer {
    line_renderer: LineRenderer,
    spheres: Vec<SphereData>,
}

struct OrbitData {
    pub orbit: Orbit,
    pub start_anomaly: f64,
    pub end_anomaly: Option<f64>,
    pub color: Point3<f32>,
    pub transform: Isometry3<f32>,
}

struct OrbitRenderer {
    line_renderer: LineRenderer,
    orbits: Vec<OrbitData>,
}

pub struct CompoundRenderer {
    sphere_renderer: SphereRenderer,
    orbit_renderer: OrbitRenderer,
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

impl Renderer for SphereRenderer {
    fn render(&mut self, pass: usize, camera: &mut dyn Camera) {
        for sphere in self.spheres.iter() {
            SphereRenderer::load_sphere_into_renderer(&mut self.line_renderer, camera, sphere);
        }
        self.line_renderer.render(pass, camera);
        self.spheres.clear();
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

impl Renderer for CompoundRenderer {
    fn render(&mut self, pass: usize, camera: &mut dyn Camera) {
        self.sphere_renderer.render(pass, camera);
        self.orbit_renderer.render(pass, camera);
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
