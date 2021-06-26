use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;

use kiss3d::nalgebra as na;
use na::{Point3, Translation3, Vector3};

use std::collections::HashMap;
use std::f64::consts::PI;

use super::camera::CustomCamera;

use crate::universe::{BodyID, BodyState, Frame, Orbit, ShipID, Universe};

pub struct Path {
    nodes: Vec<Point3<f32>>,
    frame: Frame,
    color: Point3<f32>,
}

pub enum FocusPoint {
    Body(BodyID),
    // TODO add ship focus once you add ship frames...
}

pub struct Scene {
    universe: Universe,
    body_spheres: HashMap<BodyID, SceneNode>,
    ship_objects: HashMap<ShipID, SceneNode>,
    paths: Vec<Path>,

    window: Window,
    camera: CustomCamera,
    camera_focus_order: Vec<FocusPoint>,
    camera_focus_idx: usize,

    timestep: f64,
    paused: bool,
}

impl Scene {
    pub fn new(mut window: Window, universe: Universe) -> Self {
        // Set up camera
        let camera = CustomCamera::new(2.0e9);
        let camera_focus_idx: usize = 0;
        // TODO sort focus points in a more systematic way
        let mut camera_focus_order: Vec<FocusPoint> =
            universe.body_ids().map(|x| FocusPoint::Body(*x)).collect();
        camera_focus_order.sort_by_key(|f| match f {
            FocusPoint::Body(x) => *x,
        });

        let mut paths = vec![];

        // Collect planetary bodies
        let mut body_spheres = HashMap::with_capacity(universe.bodies.len());
        for (id, body) in universe.bodies.iter() {
            // Make the sphere that represents the body
            let mut sphere = window.add_sphere(body.info.radius);
            let color = &body.info.color;
            sphere.set_color(color.x, color.y, color.z);
            body_spheres.insert(*id, sphere);

            // Compute the path to draw for the orbit, if relevant
            let (parent_id, state) = match &body.state {
                BodyState::FixedAtOrigin => continue,
                BodyState::Orbiting { parent_id, state } => (parent_id, state),
            };
            let orbit = state.get_orbit();
            paths.push(Path {
                nodes: get_path_for_orbit(&orbit, 100),
                frame: Frame::BodyInertial(*parent_id),
                color: body.info.color,
            });

            // TODO remove, or somehow make optional
            // Make axes that show the planet's orbits orientation
            let make_axis_path = |v, color| -> Path {
                let v: Vector3<f32> = na::convert(v);
                let v = 2.0 * body.info.radius * v;
                let pt: Point3<f32> = Point3::from(v);
                Path {
                    nodes: vec![Point3::origin(), pt],
                    frame: Frame::BodyInertial(*id),
                    color,
                }
            };
            paths.push(make_axis_path(
                orbit.periapse_vector(),
                Point3::new(1.0, 0.0, 0.0),
            ));
            paths.push(make_axis_path(
                orbit.asc_node_vector(),
                Point3::new(0.0, 1.0, 0.0),
            ));
            paths.push(make_axis_path(
                orbit.normal_vector(),
                Point3::new(0.0, 0.0, 1.0),
            ));
        }

        // Collect ships
        let mut ship_objects = HashMap::with_capacity(universe.ships.len());
        for (id, ship) in universe.ships.iter() {
            // Make the cube that represents the ship
            let mut cube = window.add_cube(1e6, 1e6, 1e6);
            cube.set_color(1.0, 1.0, 1.0);
            ship_objects.insert(*id, cube);

            // Compute the path to draw for the orbit
            let nodes = get_path_for_orbit(&ship.state.get_orbit(), 100);
            paths.push(Path {
                nodes,
                frame: Frame::BodyInertial(ship.parent_id),
                color: Point3::new(1.0, 1.0, 1.0),
            });
        }

        // We can't query the fps, so let's just set it
        window.set_framerate_limit(Some(60));

        Scene {
            universe,
            body_spheres,
            ship_objects,
            paths,
            window,
            camera,
            camera_focus_order,
            camera_focus_idx,
            timestep: 21600.0 / 60.0, // one Kerbin-day
            paused: true,
        }
    }

    pub fn draw_loop(&mut self) {
        loop {
            self.process_user_input();
            self.update_state();
            self.update_scene_objects();

            // This step is when kiss3d detects when the window is exited
            if !self.render_scene() {
                break;
            };
        }
    }

    pub fn process_user_input(&mut self) {
        // Process events
        let num_bodies = self.camera_focus_order.len();
        for event in self.window.events().iter() {
            match event.value {
                WindowEvent::Key(Key::E, Action::Press, _) => {
                    self.camera_focus_idx = (self.camera_focus_idx + 1) % num_bodies;
                }
                WindowEvent::Key(Key::Q, Action::Press, _) => {
                    self.camera_focus_idx = (self.camera_focus_idx + num_bodies - 1) % num_bodies;
                }
                WindowEvent::Key(Key::Period, Action::Press, _) => {
                    self.timestep *= 2.0;
                    println!("Timestep is {} s / s", (60.0 * self.timestep).round())
                }
                WindowEvent::Key(Key::Comma, Action::Press, _) => {
                    self.timestep /= 2.0;
                    println!("Timestep is {} s / s", (60.0 * self.timestep).round())
                }
                WindowEvent::Key(Key::R, Action::Press, _) => {
                    self.timestep *= -1.0;
                    self.paused = false;
                }
                WindowEvent::Key(Key::P, Action::Press, _) => {
                    self.paused = !self.paused;
                }
                _ => {}
            }
        }
    }

    pub fn update_state(&mut self) {
        if !self.paused {
            self.universe.advance_t(self.timestep);
        }
    }

    pub fn update_scene_objects(&mut self) {
        let camera_frame = self.camera_frame();
        for (id, sphere) in self.body_spheres.iter_mut() {
            let position = convert_f32(self.universe.get_body_position(*id, camera_frame));
            sphere.set_local_translation(Translation3::from(position.coords));
        }

        for (id, cube) in self.ship_objects.iter_mut() {
            let position = convert_f32(self.universe.get_ship_position(*id, camera_frame));
            cube.set_local_translation(Translation3::from(position.coords));
        }
    }

    pub fn render_scene(&mut self) -> bool {
        // Draw grid
        draw_grid(&mut self.window, 20, 1.0e9, &Point3::new(0.5, 0.5, 0.5));

        // Draw paths
        for path in self.paths.iter() {
            // Get the reference position for the path you're about to draw
            let transform = self
                .universe
                .convert_frames(path.frame, self.camera_frame());
            let transform: na::Isometry3<f32> = na::convert(transform);
            let transformed_path: Vec<_> = path
                .nodes
                .iter()
                .map(|p| transform.transform_point(p))
                .collect();
            draw_path(&mut self.window, &transformed_path, &path.color);
        }

        // Draw text
        let focused_body = match self.camera_frame() {
            Frame::Root => panic!("shouldn't happen, bad hack bit me"),
            Frame::BodyInertial(id) => &self.universe.bodies[&id],
        };
        let (radius, speed) = match &focused_body.state {
            BodyState::FixedAtOrigin => (0.0, 0.0),
            BodyState::Orbiting { state, .. } => {
                (state.get_position().norm(), state.get_velocity().norm())
            }
        };
        let (sma, ecc, incl, lan, argp) = match focused_body.get_orbit() {
            None => (0.0, 0.0, 0.0, 0.0, 0.0),
            Some(o) => (
                o.semimajor_axis(),
                o.eccentricity(),
                o.inclination().to_degrees(),
                o.long_asc_node().to_degrees(),
                o.arg_periapse().to_degrees(),
            ),
        };

        let body_text = format!(
            "Focused on: {}
State:
    Radius: {:.0} m
    Speed: {:.0} m/s
Orbit:
    SMA: {:.0}
    Eccentricity: {:.3}
    Inclination: {:.3}
    LAN: {:.1}
    Arg PE: {:.1}",
            focused_body.info.name, radius, speed, sma, ecc, incl, lan, argp
        );

        self.window.draw_text(
            &body_text,
            &na::Point2::origin(),
            80.0,
            &kiss3d::text::Font::default(),
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Render
        return self.window.render_with_camera(&mut self.camera);
    }

    fn camera_frame(&self) -> Frame {
        match self.camera_focus_order[self.camera_focus_idx] {
            FocusPoint::Body(id) => Frame::BodyInertial(id),
        }
    }
}

// Helpful for avoiding ambiguous typing
fn convert_f32(p: Point3<f64>) -> Point3<f32> {
    na::convert(p)
}

fn draw_grid(window: &mut Window, num_squares: i32, square_size: f32, color: &Point3<f32>) {
    let max_coord = square_size * (num_squares as f32);
    for i in (-num_squares)..(num_squares + 1) {
        let coord = square_size * (i as f32);
        // horizontal
        window.draw_line(
            &Point3::new(-max_coord, coord, 0.0),
            &Point3::new(max_coord, coord, 0.0),
            &color,
        );
        // vertical
        window.draw_line(
            &Point3::new(coord, -max_coord, 0.0),
            &Point3::new(coord, max_coord, 0.0),
            &color,
        );
    }
}

fn draw_loop(window: &mut Window, pts: &[Point3<f32>], color: &Point3<f32>) {
    draw_path(window, pts, color);
    let length = pts.len();
    if length > 2 {
        window.draw_line(&pts[length - 1], &pts[0], color);
    }
}

fn draw_path(window: &mut Window, pts: &[Point3<f32>], color: &Point3<f32>) {
    for pair in pts.windows(2) {
        window.draw_line(&pair[0], &pair[1], &color);
    }
}

fn get_path_for_orbit(orbit: &Orbit, num_points: usize) -> Vec<Point3<f32>> {
    let mut pts = vec![];
    for i in 0..=num_points {
        let theta = -PI + (2.0 * PI) * (i as f64) / (num_points as f64);
        let pt = match orbit.get_position_at_theta(theta) {
            Some(v) => Point3::from(v),
            None => continue,
        };
        pts.push(na::convert(pt));
    }
    pts
}
