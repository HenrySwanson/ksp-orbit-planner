use kiss3d::camera::Camera;
use kiss3d::context::Context;
use kiss3d::renderer::Renderer;
use kiss3d::resource::{
    AllocationType, BufferType, Effect, GPUVec, ShaderAttribute, ShaderUniform,
};

use nalgebra::{Isometry3, Matrix4, Point3, Vector3};

use std::f64::consts::PI;

use crate::universe::OrbitPatch;

use super::utils::path_iter_parametric;

pub struct OrbitRenderer {
    // OpenGL stuff
    shader: Effect,
    pos: ShaderAttribute<Point3<f32>>,
    color: ShaderAttribute<Point3<f32>>,
    view: ShaderUniform<Matrix4<f32>>,
    proj: ShaderUniform<Matrix4<f32>>,
    line_width: f32,
    // Data storage (a, color, b, color)
    orbit_lines: GPUVec<Point3<f32>>,
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
            view: shader
                .get_uniform::<Matrix4<f32>>("view")
                .expect("Failed to get shader uniform."),
            proj: shader
                .get_uniform::<Matrix4<f32>>("proj")
                .expect("Failed to get shader uniform."),
            shader: shader,
            line_width: 1.0,
            orbit_lines: GPUVec::new(Vec::new(), BufferType::Array, AllocationType::StreamDraw),
        }
    }

    pub fn add_orbit(&mut self, orbit: OrbitPatch, color: Point3<f32>, transform: Isometry3<f32>) {
        // Don't bother storing the orbit itself, just load data into the GPUVec
        let data = match self.orbit_lines.data_mut() {
            Some(x) => x,
            None => return,
        };

        // TODO: would be easier if we could use GL_LINE_STRIP
        let mut prev_pt = None;
        for pt in OrbitRenderer::get_orbit_points(orbit, transform) {
            if let Some(prev_pt) = prev_pt {
                data.push(prev_pt);
                data.push(color);
                data.push(pt);
                data.push(color);
            }
            prev_pt = Some(pt);
        }
    }

    fn get_orbit_points(
        orbit: OrbitPatch,
        transform: Isometry3<f32>,
    ) -> impl Iterator<Item = Point3<f32>> {
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
            let v = orbit.orbit.get_state(s).get_position();
            let v: Vector3<f32> = nalgebra::convert(v);
            transform * Point3::from(v)
        };

        path_iter_parametric(f, start_s, end_s, 180)
    }
}

impl Renderer for OrbitRenderer {
    fn render(&mut self, pass: usize, camera: &mut dyn Camera) {
        if self.orbit_lines.len() == 0 {
            return;
        }

        self.shader.use_program();
        self.pos.enable();
        self.color.enable();

        camera.upload(pass, &mut self.proj, &mut self.view);

        self.pos.bind_sub_buffer(&mut self.orbit_lines, 1, 0);
        self.color.bind_sub_buffer(&mut self.orbit_lines, 1, 1);

        let ctxt = Context::get();
        ctxt.draw_arrays(Context::LINES, 0, (self.orbit_lines.len() / 2) as i32);
        ctxt.line_width(self.line_width);

        self.pos.disable();
        self.color.disable();

        for data in self.orbit_lines.data_mut().iter_mut() {
            data.clear()
        }
    }
}

// TODO add model matrix

/// Vertex shader used by the material to display line.
static VERTEX_SRC: &'static str = "#version 100
    attribute vec3 position;
    attribute vec3 color;
    varying   vec3 vColor;
    uniform   mat4 proj;
    uniform   mat4 view;
    void main() {
        gl_Position = proj * view * vec4(position, 1.0);
        vColor = color;
    }";

/// Fragment shader used by the material to display line.
static FRAGMENT_SRC: &'static str = "#version 100
#ifdef GL_FRAGMENT_PRECISION_HIGH
   precision highp float;
#else
   precision mediump float;
#endif

    varying vec3 vColor;
    void main() {
        gl_FragColor = vec4(vColor, 1.0);
    }";
