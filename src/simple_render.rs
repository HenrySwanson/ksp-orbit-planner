use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;

use kiss3d::nalgebra as na;
use na::{Point3, Translation3, Vector3};

use std::collections::HashMap;
use std::f64::consts::PI;

use crate::camera::CustomCamera;
use crate::orbit::Orbit;
use crate::state::State;
use crate::universe::{BodyID, Frame, ShipID, Universe};

pub struct Scene {
    universe: Universe,
    body_spheres: HashMap<BodyID, SceneNode>,
    orbit_paths: HashMap<BodyID, Vec<Vector3<f32>>>,

    ship_objects: HashMap<ShipID, SceneNode>,
    ship_paths: HashMap<ShipID, Vec<Vector3<f32>>>,

    window: Window,
    camera: CustomCamera,
    camera_focus_order: Vec<BodyID>,
    camera_focus_idx: usize,

    timestep: f64,
    paused: bool,
}

impl Scene {
    pub fn new(mut window: Window, universe: Universe) -> Self {
        // Set up camera
        let camera = CustomCamera::new(2.0e9);
        let camera_focus_idx: usize = 0;
        let mut camera_focus_order: Vec<BodyID> = universe.body_ids().copied().collect();
        camera_focus_order.sort();

        // Collect planetary bodies
        let mut body_spheres = HashMap::with_capacity(universe.bodies.len());
        let mut orbit_paths = HashMap::with_capacity(universe.bodies.len());
        for (id, body) in universe.bodies.iter() {
            // Make the sphere that represents the body
            let mut sphere = window.add_sphere(body.info.radius);
            let color = &body.info.color;
            sphere.set_color(color.x, color.y, color.z);

            // Compute the path to draw for the orbit
            let path: Vec<Vector3<f32>> = match body.state.get_orbit() {
                Some(orbit) => {
                    (0..=100_usize)
                        // start at -180 so that open orbits work right
                        .map(|i| -PI + (2.0 * PI) * (i as f64) / 100.0)
                        .map(|theta| orbit.get_position_at_theta(theta))
                        .filter_map(|option_v| option_v.map(na::convert))
                        .collect()
                }
                None => vec![],
            };

            body_spheres.insert(*id, sphere);
            orbit_paths.insert(*id, path);
        }

        // Collect ships
        let mut ship_objects = HashMap::with_capacity(universe.ships.len());
        let mut ship_paths = HashMap::with_capacity(universe.ships.len());
        for (id, ship) in universe.ships.iter() {
            // Make the cube that represents the ship
            let mut cube = window.add_cube(1e6, 1e6, 1e6);
            cube.set_color(1.0, 1.0, 1.0);

            // Compute the path to draw for the orbit
            let orbit = Orbit::from_cartesian(
                ship.state.get_position(),
                ship.state.get_velocity(),
                ship.state.get_mu(),
            );
            let path: Vec<Vector3<f32>> = (0..=100_usize)
                // start at -180 so that open orbits work right
                .map(|i| -PI + (2.0 * PI) * (i as f64) / 100.0)
                .map(|theta| orbit.get_position_at_theta(theta))
                .filter_map(|option_v| option_v.map(na::convert))
                .collect();

            ship_objects.insert(*id, cube);
            ship_paths.insert(*id, path);
        }

        // We can't query the fps, so let's just set it
        window.set_framerate_limit(Some(60));

        Scene {
            universe,
            body_spheres,
            orbit_paths,
            ship_objects,
            ship_paths,
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
            for body in self.universe.bodies.values_mut() {
                body.state.advance_t(self.timestep);
            }

            for ship in self.universe.ships.values_mut() {
                ship.state.advance_t(self.timestep);
            }
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

        // Draw orbits
        for (id, body) in self.universe.bodies.iter() {
            let parent_id = match &body.state {
                State::FixedAtOrigin => continue,
                State::Orbiting(parent_id, _) => *parent_id,
            };
            // Get the parent's position within the camera frame
            let parent_position = convert_f32(
                self.universe
                    .get_body_position(parent_id, self.camera_frame()),
            );
            let orbit_path: Vec<_> = self.orbit_paths[id]
                .iter()
                .map(|x| parent_position + x)
                .collect();
            draw_path(&mut self.window, &orbit_path, &body.info.color);
        }

        // Draw orbits for ships
        for (id, ship) in self.universe.ships.iter() {
            let parent_id = ship.parent_id;
            // Get the parent's position within the camera frame
            let parent_position = convert_f32(
                self.universe
                    .get_body_position(parent_id, self.camera_frame()),
            );
            let orbit_path: Vec<_> = self.ship_paths[id]
                .iter()
                .map(|x| parent_position + x)
                .collect();
            draw_path(&mut self.window, &orbit_path, &Point3::new(1.0, 1.0, 1.0));
        }

        // Draw text
        let focused_body = match self.camera_frame() {
            Frame::Root => panic!("shouldn't happen, bad hack bit me"),
            Frame::BodyInertial(id) => &self.universe.bodies[&id],
        };
        let orbit = focused_body.state.get_orbit();
        let o = orbit.as_ref();
        let body_text = format!(
            "Focused on: {}
State:
    Radius: {:.0} m
    Velocity: {:.0} m/s
Orbit:
    SMA: {:.0}
    Eccentricity: {:.3}
    Inclination: {:.3}
    LAN: {:.1}
    Arg PE: {:.1}",
            focused_body.info.name,
            focused_body.state.get_position().0.coords.norm(),
            focused_body.state.get_velocity().0.norm(),
            o.map_or(0.0, |o| o.semimajor_axis()),
            o.map_or(0.0, |o| o.eccentricity()),
            o.map_or(0.0, |o| o.inclination().to_degrees()),
            o.map_or(0.0, |o| o.long_asc_node().to_degrees()),
            o.map_or(0.0, |o| o.arg_periapse().to_degrees()),
        );

        // TODO remove...
        let mut asc_node: Vector3<f32> =
            na::convert(o.map_or(Vector3::zeros(), |o| o.asc_node_vector()));
        asc_node *= focused_body.info.radius * 2.0;
        draw_path(
            &mut self.window,
            &[Point3::origin(), Point3::origin() + asc_node],
            &Point3::new(1.0, 0.0, 0.0),
        );
        let mut periapse_dir: Vector3<f32> =
            na::convert(o.map_or(Vector3::zeros(), |o| o.periapse_vector()));
        periapse_dir *= focused_body.info.radius * 2.0;
        draw_path(
            &mut self.window,
            &[Point3::origin(), Point3::origin() + periapse_dir],
            &Point3::new(0.0, 1.0, 0.0),
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
        Frame::BodyInertial(self.camera_focus_order[self.camera_focus_idx])
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
