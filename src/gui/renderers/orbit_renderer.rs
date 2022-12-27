use kiss3d::camera::Camera;
use kiss3d::context::Context;
use kiss3d::renderer::Renderer;
use kiss3d::resource::{
    AllocationType, BufferType, Effect, GPUVec, ShaderAttribute, ShaderUniform,
};

use nalgebra::{Isometry3, Matrix4, Point3, Vector3};

use std::f64::consts::PI;

use crate::orrery::OrbitPatch;

use super::utils::path_iter_parametric;

struct OrbitData {
    // Orbit path, stored as (pt, color, pt, color)
    // Evaluated in the orbit's natural space (z normal, x periapsis)
    orbit_lines: GPUVec<Point3<f32>>,
    // Transform from the orbit's natural space to focus space
    transform: Matrix4<f32>,
}

pub struct OrbitRenderer {
    // OpenGL stuff
    shader: Effect,
    pos: ShaderAttribute<Point3<f32>>,
    color: ShaderAttribute<Point3<f32>>,
    model: ShaderUniform<Matrix4<f32>>,
    view: ShaderUniform<Matrix4<f32>>,
    proj: ShaderUniform<Matrix4<f32>>,
    line_width: f32,
    // Data storage
    orbits: Vec<OrbitData>,
}

impl OrbitRenderer {
    pub fn new() -> Self {
        let mut shader = Effect::new_from_str(VERTEX_SRC, FRAGMENT_SRC);

        shader.use_program();

        OrbitRenderer {
            pos: shader
                .get_attrib::<Point3<f32>>("position")
                .expect("Failed to get shader attribute."),
            color: shader
                .get_attrib::<Point3<f32>>("color")
                .expect("Failed to get shader attribute."),
            model: shader
                .get_uniform::<Matrix4<f32>>("model")
                .expect("Failed to get shader uniform."),
            view: shader
                .get_uniform::<Matrix4<f32>>("view")
                .expect("Failed to get shader uniform."),
            proj: shader
                .get_uniform::<Matrix4<f32>>("proj")
                .expect("Failed to get shader uniform."),
            shader,
            line_width: 1.0,
            orbits: vec![],
        }
    }

    pub fn add_orbit(&mut self, orbit: OrbitPatch, color: Point3<f32>, transform: Isometry3<f32>) {
        // Collect points and put them into the GPUVec
        let points: Vec<_> = OrbitRenderer::get_orbit_points(&orbit).collect();
        let mut data = Vec::with_capacity(4 * points.len());
        for pts in points.windows(2) {
            data.push(pts[0]);
            data.push(color);
            data.push(pts[1]);
            data.push(color);
        }

        // The transform we're given is from the parent body's space to focusspace, but we want
        // to start out in the orbit's native space.
        let transform2: Isometry3<f32> = nalgebra::convert(orbit.orbit.rotation());
        let total_transform = transform * transform2;

        let orbit_data = OrbitData {
            orbit_lines: GPUVec::new(data, BufferType::Array, AllocationType::StreamDraw),
            transform: total_transform.to_homogeneous(),
        };

        self.orbits.push(orbit_data);
    }

    /// Returns a sequence of points tracing out the orbit's path, evaluated in the orbit's native
    /// frame.
    fn get_orbit_points(orbit: &OrbitPatch) -> impl Iterator<Item = Point3<f32>> + '_ {
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
        let f = move |s| {
            let v = orbit.orbit.get_state_native_frame(s).position();
            let v: Vector3<f32> = nalgebra::convert(v);
            Point3::from(v)
        };

        path_iter_parametric(f, start_s, end_s, 180)
    }
}

impl Renderer for OrbitRenderer {
    fn render(&mut self, pass: usize, camera: &mut dyn Camera) {
        if self.orbits.is_empty() {
            return;
        }

        self.shader.use_program();
        self.pos.enable();
        self.color.enable();

        camera.upload(pass, &mut self.proj, &mut self.view);

        for orbit in self.orbits.iter_mut() {
            self.pos.bind_sub_buffer(&mut orbit.orbit_lines, 1, 0);
            self.color.bind_sub_buffer(&mut orbit.orbit_lines, 1, 1);

            self.model.upload(&orbit.transform);

            let ctxt = Context::get();
            ctxt.draw_arrays(Context::LINES, 0, (orbit.orbit_lines.len() / 2) as i32);
            ctxt.line_width(self.line_width);
        }

        self.pos.disable();
        self.color.disable();

        // TODO keep the GPUVecs around each loop?
        self.orbits.clear();
    }
}

// TODO add model matrix

/// Vertex shader used by the material to display line.
static VERTEX_SRC: &str = "#version 100
    attribute vec3 position;
    attribute vec3 color;
    varying   vec3 vColor;
    uniform   mat4 model;
    uniform   mat4 proj;
    uniform   mat4 view;
    void main() {
        gl_Position = proj * view * model * vec4(position, 1.0);
        vColor = color;
    }";

/// Fragment shader used by the material to display line.
static FRAGMENT_SRC: &str = "#version 100
#ifdef GL_FRAGMENT_PRECISION_HIGH
   precision highp float;
#else
   precision mediump float;
#endif

    varying vec3 vColor;
    void main() {
        gl_FragColor = vec4(vColor, 1.0);
    }";
