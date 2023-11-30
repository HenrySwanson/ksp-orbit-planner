use kiss3d::camera::Camera;
use kiss3d::context::Context;
use kiss3d::renderer::Renderer;
use kiss3d::resource::{
    AllocationType, BufferType, Effect, GPUVec, ShaderAttribute, ShaderUniform,
};
use nalgebra::Point3;

// TODO: would it be a better idea to render a rectangle, and then use various
// fragment shaders to draw circles / textures, etc?

pub enum MarkerType {
    Square,
    Circle,
}

/// Represents a marker to be drawn on-screen.
struct Marker {
    pub mtype: MarkerType,
    pub center: Point3<f32>,
    // Height in NDC
    pub height: f32,
    pub color: Point3<f32>,
}

pub struct MarkerRenderer {
    // OpenGL stuff
    shader: Effect,
    offset: ShaderAttribute<Point3<f32>>,
    center: ShaderUniform<Point3<f32>>,
    height: ShaderUniform<f32>,
    color: ShaderUniform<Point3<f32>>,
    screen_aspect: ShaderUniform<f32>,
    // Data storage
    markers: Vec<Marker>,
}

impl MarkerRenderer {
    pub fn new() -> Self {
        let mut shader = Effect::new_from_str(VERTEX_SRC, FRAGMENT_SRC);

        shader.use_program();

        MarkerRenderer {
            offset: shader
                .get_attrib::<Point3<f32>>("offset")
                .expect("Failed to get shader attribute."),
            center: shader
                .get_uniform::<Point3<f32>>("center")
                .expect("Failed to get shader uniform."),
            height: shader
                .get_uniform::<f32>("height")
                .expect("Failed to get shader uniform."),
            color: shader
                .get_uniform::<Point3<f32>>("color")
                .expect("Failed to get shader uniform."),
            screen_aspect: shader
                .get_uniform::<f32>("screen_aspect")
                .expect("Failed to get shader uniform."),
            shader,
            markers: vec![],
        }
    }

    pub fn add_marker(
        &mut self,
        mtype: MarkerType,
        center: Point3<f32>,
        height: f32,
        color: Point3<f32>,
    ) {
        let marker = Marker {
            mtype,
            center,
            height,
            color,
        };
        self.markers.push(marker);
    }

    fn gen_body_marker_triangles(n: usize) -> Vec<Point3<f32>> {
        use std::f32::consts::TAU;

        // Generate a circle with a fan of triangles
        (0..n)
            .flat_map(move |i| {
                let theta1 = (i as f32) / (n as f32) * TAU;
                let theta2 = ((i + 1) as f32) / (n as f32) * TAU;
                [
                    Point3::origin(),
                    Point3::new(theta1.cos(), theta1.sin(), 0.0),
                    Point3::new(theta2.cos(), theta2.sin(), 0.0),
                ]
            })
            .collect()
    }

    fn gen_ship_marker_triangles() -> Vec<Point3<f32>> {
        // make a square with two triangles, oriented CCW
        let ul = Point3::new(-1.0, 1.0, 0.0);
        let ur = Point3::new(1.0, 1.0, 0.0);
        let dl = Point3::new(-1.0, -1.0, 0.0);
        let dr = Point3::new(1.0, -1.0, 0.0);
        vec![ul, dr, ur, ul, dl, dr]
    }
}

impl Renderer for MarkerRenderer {
    fn render(&mut self, _: usize, camera: &mut dyn Camera) {
        if self.markers.is_empty() {
            return;
        }

        // Generate the GPUVecs for the marker shapes
        let mut circle_triangles = GPUVec::new(
            Self::gen_body_marker_triangles(16),
            BufferType::Array,
            AllocationType::StaticDraw,
        );
        let mut square_triangles = GPUVec::new(
            Self::gen_ship_marker_triangles(),
            BufferType::Array,
            AllocationType::StaticDraw,
        );

        // Deduce the aspect ratio of the window -- it's the inverse of the aspect ratio
        // caused by the camera
        let vp_transform = camera.transformation();
        let aspect = {
            let inv_transform = camera.inverse_transformation();
            let o_world = inv_transform.transform_point(&Point3::new(0.0, 0.0, 1.0));
            let x_world = inv_transform.transform_point(&Point3::new(1.0, 0.0, 1.0));
            let y_world = inv_transform.transform_point(&Point3::new(0.0, 1.0, 1.0));

            (x_world - o_world).norm() / (y_world - o_world).norm()
        };

        self.shader.use_program();
        self.offset.enable();

        self.screen_aspect.upload(&aspect);

        for marker in self.markers.iter() {
            // Compute the center of the marker in screen space
            let center = vp_transform * marker.center.to_homogeneous();
            let center = Point3::from(center.xyz() / center.w);

            self.offset.bind(match marker.mtype {
                MarkerType::Square => &mut square_triangles,
                MarkerType::Circle => &mut circle_triangles,
            });
            self.center.upload(&center);
            self.height.upload(&marker.height);
            self.color.upload(&marker.color);

            let ctxt = Context::get();
            ctxt.draw_arrays(Context::TRIANGLES, 0, circle_triangles.len() as i32);
        }

        self.offset.disable();

        self.markers.clear();
    }
}

/// Vertex shader used by the material to display line.
static VERTEX_SRC: &str = "#version 100
    attribute vec3 offset;
    uniform   vec3 center;
    uniform   float height;
    uniform   float screen_aspect;

    void main() {
        // Markers fit in [-1, 1], so we divide height by 2.
        // Then we divide x by screen aspect to pre-compensate for the x-stretching
        // that NDC -> screen space does
        vec3 offset2 = (height / 2.0) * offset / vec3(screen_aspect, 1, 1);
        gl_Position = vec4(center + offset2, 1.0);
    }";

/// Fragment shader used by the material to display line.
static FRAGMENT_SRC: &str = "#version 100
#ifdef GL_FRAGMENT_PRECISION_HIGH
   precision highp float;
#else
   precision mediump float;
#endif

    uniform vec3 color;
    void main() {
        gl_FragColor = vec4(color, 1.0);
    }";
