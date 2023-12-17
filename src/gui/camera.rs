use std::f32::consts::PI;

use kiss3d::camera::Camera;
use kiss3d::event::{Action, Key, MouseButton, WindowEvent};
use kiss3d::resource::ShaderUniform;
use kiss3d::window::Canvas;
use nalgebra::{Isometry3, Matrix4, Perspective3, Point3, Vector2, Vector3};

const KEY_CAMERA_MOVE_UP: Key = Key::W;
const KEY_CAMERA_MOVE_DOWN: Key = Key::S;
const KEY_CAMERA_MOVE_LEFT: Key = Key::A;
const KEY_CAMERA_MOVE_RIGHT: Key = Key::D;
const KEY_CAMERA_ZOOM_IN: Key = Key::Equals;
const KEY_CAMERA_ZOOM_OUT: Key = Key::Minus;

const KEY_ANGLE_STEP: f32 = 0.1;
const KEY_ZOOM_STEP: f32 = 1.2;

// This camera is a close cousin of ArcBall. Like ArcBall, this camera can be
// click-and-dragged to adjust its pitch and yaw, and scrolled to zoom in and
// out. However, since we're operating over very large length scales, our zooms
// must also adjust the clipping planes. Since ArcBall doesn't expose those
// parameters, this prevents us from simply wrapping ArcBall.
//
// Our possible camera positions are more restricted than ArcBall. The camera
// always points at the origin, and uses the z-axis as up. This is because we
// translate the universe so that the origin is at the object we're "focused"
// on.
pub struct ZoomableCamera {
    // -- position --
    theta: f32,  // azimuthal angle
    phi: f32,    // polar angle
    radius: f32, // distance from origin
    // -- perspective --
    width: u32,
    height: u32,
    fovy: f32,
    // -- other --
    last_cursor_pos: Vector2<f32>,
    // -- knobs to fiddle with --
    theta_step: f32,
    phi_step: f32,
    scroll_ratio: f32,
    phi_limit: f32,
    radius_limits: (f32, f32),
    z_near_multiplier: f32,
    z_far_multipler: f32,
}

impl ZoomableCamera {
    // TODO: more parameters
    pub fn new(radius: f32) -> Self {
        ZoomableCamera {
            theta: 0.0,
            phi: PI / 2.0,
            radius,
            width: 800,
            height: 600,
            fovy: PI / 4.0,
            last_cursor_pos: Vector2::zeros(),
            theta_step: 0.005,
            phi_step: 0.005,
            scroll_ratio: 1.5,
            phi_limit: 0.001,
            radius_limits: (1.0, 2.5e11),
            z_near_multiplier: 0.1,
            z_far_multipler: 1024.0,
        }
    }

    fn projection(&self) -> Perspective3<f32> {
        Perspective3::new(
            self.width as f32 / self.height as f32,
            self.fovy,
            self.radius * self.z_near_multiplier,
            self.radius * self.z_far_multipler,
        )
    }

    fn projection_matrix(&self) -> Matrix4<f32> {
        self.projection().into_inner()
    }

    fn view_matrix(&self) -> Matrix4<f32> {
        self.view_transform().to_homogeneous()
    }

    pub fn set_min_distance(&mut self, min_dist: f32) {
        self.radius_limits.0 = min_dist;
        self.radius = nalgebra::clamp(self.radius, self.radius_limits.0, self.radius_limits.1);
    }

    pub fn distance(&self) -> f32 {
        self.radius
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    pub fn fovy(&self) -> f32 {
        self.fovy
    }

    pub fn rotate(&mut self, dtheta: f32, dphi: f32) {
        self.theta = (self.theta + dtheta) % (2.0 * PI);
        self.phi = nalgebra::clamp(self.phi + dphi, self.phi_limit, PI - self.phi_limit);
    }

    pub fn zoom(&mut self, factor: f32) {
        self.radius = nalgebra::clamp(
            self.radius * factor,
            self.radius_limits.0,
            self.radius_limits.1,
        );
    }
}

impl Camera for ZoomableCamera {
    fn handle_event(&mut self, canvas: &Canvas, event: &WindowEvent) {
        match *event {
            WindowEvent::CursorPos(x, y, _) => {
                let curr_pos = Vector2::new(x as f32, y as f32);

                if canvas.get_mouse_button(MouseButton::Button1) == Action::Press {
                    // Rotate the opposite direction as the mouse moves (drag right == camera glides
                    // left)
                    let dpos = curr_pos - self.last_cursor_pos;
                    self.rotate(-dpos.x * self.theta_step, -dpos.y * self.phi_step);
                }

                self.last_cursor_pos = curr_pos;
            }
            WindowEvent::Scroll(_, off, _) => {
                // scroll up == zoom in
                if off < 0.0 {
                    self.zoom(self.scroll_ratio);
                } else if off > 0.0 {
                    self.zoom(self.scroll_ratio.recip())
                }
            }
            WindowEvent::FramebufferSize(w, h) => {
                self.width = w;
                self.height = h;
            }
            WindowEvent::Key(KEY_CAMERA_MOVE_UP, Action::Press, _) => {
                self.rotate(0.0, -KEY_ANGLE_STEP)
            }
            WindowEvent::Key(KEY_CAMERA_MOVE_DOWN, Action::Press, _) => {
                self.rotate(0.0, KEY_ANGLE_STEP)
            }
            WindowEvent::Key(KEY_CAMERA_MOVE_LEFT, Action::Press, _) => {
                self.rotate(-KEY_ANGLE_STEP, 0.0)
            }
            WindowEvent::Key(KEY_CAMERA_MOVE_RIGHT, Action::Press, _) => {
                self.rotate(KEY_ANGLE_STEP, 0.0)
            }
            WindowEvent::Key(KEY_CAMERA_ZOOM_IN, Action::Press, _) => {
                self.zoom(KEY_ZOOM_STEP.recip())
            }
            WindowEvent::Key(KEY_CAMERA_ZOOM_OUT, Action::Press, _) => self.zoom(KEY_ZOOM_STEP),
            _ => {}
        }
    }

    fn eye(&self) -> Point3<f32> {
        Point3::new(
            self.radius * self.theta.cos() * self.phi.sin(),
            self.radius * self.theta.sin() * self.phi.sin(),
            self.radius * self.phi.cos(),
        )
    }

    fn view_transform(&self) -> Isometry3<f32> {
        Isometry3::look_at_rh(&self.eye(), &Point3::origin(), &Vector3::z())
    }

    fn transformation(&self) -> Matrix4<f32> {
        self.projection_matrix() * self.view_matrix()
    }

    fn inverse_transformation(&self) -> Matrix4<f32> {
        self.transformation().try_inverse().unwrap()
    }

    fn clip_planes(&self) -> (f32, f32) {
        (self.projection().znear(), self.projection().zfar())
    }

    fn update(&mut self, _canvas: &Canvas) {}

    fn upload(
        &self,
        _: usize,
        proj: &mut ShaderUniform<Matrix4<f32>>,
        view: &mut ShaderUniform<Matrix4<f32>>,
    ) {
        proj.upload(&self.projection_matrix());
        view.upload(&self.view_matrix());
    }
}
