use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;

use nalgebra::{Point3, Translation3, Vector3};

use std::collections::HashMap;
use std::f64::consts::PI;

use super::camera::CustomCamera;

use crate::universe::{BodyID, Frame, FrameTransform, Orbit, ShipID, Universe};

const TEST_SHIP_SIZE: f32 = 1e6;

pub struct Path {
    nodes: Vec<Point3<f32>>,
    frame: Frame,
    color: Point3<f32>,
}

#[derive(Debug, Clone, Copy)]
pub enum FocusPoint {
    Body(BodyID),
    Ship(ShipID),
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
    ship_camera_inertial: bool,

    timestep: f64,
    paused: bool,
}

// TODO sort focus points in a more systematic way
fn get_focus_points(universe: &Universe) -> Vec<FocusPoint> {
    let mut body_ids: Vec<_> = universe.body_ids().copied().collect();
    body_ids.sort();
    let mut ships: Vec<_> = universe.ships().collect();
    ships.sort_by_key(|s| s.id);

    let mut focus_pts = vec![];
    for body_id in body_ids.into_iter() {
        focus_pts.push(FocusPoint::Body(body_id));
        // Now put in all ships orbiting that body
        for ship in ships.iter() {
            if ship.get_parent_id() == body_id {
                focus_pts.push(FocusPoint::Ship(ship.id));
            }
        }
    }

    focus_pts
}

impl Scene {
    pub fn new(mut window: Window, universe: Universe) -> Self {
        // Set up camera
        let camera = CustomCamera::new(2.0e9);
        let camera_focus_idx: usize = 0;
        let camera_focus_order = get_focus_points(&universe);
        let ship_camera_inertial = true;

        let mut paths = vec![];

        // Collect planetary bodies
        let mut body_spheres = HashMap::new();
        for body in universe.bodies() {
            let body_info = body.info();

            // Make the sphere that represents the body
            let mut sphere = window.add_sphere(body_info.radius);
            let color = &body_info.color;
            sphere.set_color(color.x, color.y, color.z);
            body_spheres.insert(body.id, sphere);

            // Compute the path to draw for the orbit, if relevant
            let orbit = match body.get_orbit() {
                Some(orbit) => orbit,
                None => continue,
            };
            let parent_id = body.get_parent_id().unwrap();

            paths.push(Path {
                nodes: get_path_for_orbit(&orbit, 100),
                frame: Frame::BodyInertial(parent_id),
                color: body_info.color,
            });

            // TODO remove, or somehow make optional
            // Make axes that show the planet's orbits orientation
            let make_axis_path = |v, color| -> Path {
                let v: Vector3<f32> = nalgebra::convert(v);
                let v = 2.0 * body_info.radius * v;
                let pt: Point3<f32> = Point3::from(v);
                Path {
                    nodes: vec![Point3::origin(), pt],
                    frame: Frame::BodyInertial(body.id),
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
        let mut ship_objects = HashMap::new();
        for ship in universe.ships() {
            // Make the cube that represents the ship
            let mut cube = window.add_cube(TEST_SHIP_SIZE, TEST_SHIP_SIZE, TEST_SHIP_SIZE);
            cube.set_color(1.0, 1.0, 1.0);
            ship_objects.insert(ship.id, cube);

            // Compute the path to draw for the orbit
            let nodes = get_path_for_orbit(&ship.get_orbit(), 100);
            paths.push(Path {
                nodes,
                frame: Frame::BodyInertial(ship.get_parent_id()),
                color: Point3::new(1.0, 1.0, 1.0),
            });
        }

        // We can't query the fps, so let's just set it
        window.set_framerate_limit(Some(60));

        let mut scene = Scene {
            universe,
            body_spheres,
            ship_objects,
            paths,
            window,
            camera,
            camera_focus_order,
            camera_focus_idx,
            ship_camera_inertial,
            timestep: 21600.0 / 60.0, // one Kerbin-day
            paused: true,
        };

        scene.reset_min_distance();
        scene
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
                    self.reset_min_distance();
                }
                WindowEvent::Key(Key::Q, Action::Press, _) => {
                    self.camera_focus_idx = (self.camera_focus_idx + num_bodies - 1) % num_bodies;
                    self.reset_min_distance();
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
                WindowEvent::Key(Key::C, Action::Press, _) => {
                    self.ship_camera_inertial = !self.ship_camera_inertial;
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
        // TODO apply rotations too!
        let camera_frame = self.focused_object_frame();
        for (id, sphere) in self.body_spheres.iter_mut() {
            let state = self.universe.get_body(*id).state();
            set_object_position(sphere, state.get_position(camera_frame));
        }

        for (id, cube) in self.ship_objects.iter_mut() {
            let state = self.universe.get_ship(*id).state();
            set_object_position(cube, state.get_position(camera_frame));
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
                .convert_frames(path.frame, self.focused_object_frame());
            let transform: FrameTransform<f32> = nalgebra::convert(transform);
            let transformed_path: Vec<_> = path
                .nodes
                .iter()
                .map(|p| transform.convert_point(p))
                .collect();
            draw_path(&mut self.window, &transformed_path, &path.color);
        }

        // Draw text
        self.window.draw_text(
            &self.orbit_summary_text(),
            &nalgebra::Point2::origin(),
            80.0,
            &kiss3d::text::Font::default(),
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Draw sphere of influence
        let soi_body_id = match self.focused_object() {
            FocusPoint::Body(id) => id,
            FocusPoint::Ship(id) => self.universe.get_ship(id).get_parent_id(),
        };
        self.render_soi(soi_body_id);

        // Render and return bool
        self.window.render_with_camera(&mut self.camera)
    }

    fn render_soi(&mut self, id: BodyID) {
        if let Some(soi_radius) = self.universe.get_soi_radius(id) {
            // Transform the screen x and y vectors into whatever frame we're currently focused on
            use crate::kiss3d::camera::Camera;
            let camera_transform = self.camera.view_transform().inverse();
            let x_vec = camera_transform.transform_vector(&Vector3::x()).normalize();
            let y_vec = camera_transform.transform_vector(&Vector3::y()).normalize();

            // Get the position of the body in our current frame (may be non-zero if we're
            // focused on a ship).
            let body_pt = self
                .universe
                .convert_frames(Frame::BodyInertial(id), self.focused_object_frame())
                .convert_point(&Point3::origin());

            let pts: Vec<_> = (0..100)
                .map(|i| 2.0 * std::f32::consts::PI * (i as f32) / 100.0)
                .map(|theta| x_vec * theta.cos() + y_vec * theta.sin())
                .map(|v| convert_f32(body_pt) + (soi_radius as f32) * v)
                .collect();

            let body_color = self.universe.get_body(id).info().color;
            let soi_color = Point3::from(body_color.coords * 0.5);
            draw_loop(&mut self.window, &pts, &soi_color);
        }
    }

    fn orbit_summary_text(&self) -> String {
        let (name, state, orbit, frame) = match self.focused_object() {
            FocusPoint::Body(id) => {
                let body = self.universe.get_body(id);
                let frame = match body.get_parent_id() {
                    Some(id) => Frame::BodyInertial(id),
                    None => Frame::Root,
                };
                (
                    body.info().name.to_owned(),
                    body.state(),
                    body.get_orbit(),
                    frame,
                )
            }
            FocusPoint::Ship(id) => {
                let ship = self.universe.get_ship(id);
                let name = if self.ship_camera_inertial {
                    "<Ship> (inertial)"
                } else {
                    "<Ship> (orbital)"
                };

                (
                    name.to_owned(),
                    ship.state(),
                    Some(ship.get_orbit()),
                    Frame::BodyInertial(ship.get_parent_id()),
                )
            }
        };

        // Orbit needs to be handled specially, because the Sun has no orbit
        let orbit_text = match orbit {
            Some(o) => {
                format!(
                    "SMA: {:.0}
Eccentricity: {:.3}
Inclination: {:.3}
LAN: {:.1}
Arg PE: {:.1}",
                    o.semimajor_axis(),
                    o.eccentricity(),
                    o.inclination().to_degrees(),
                    o.long_asc_node().to_degrees(),
                    o.arg_periapse().to_degrees(),
                )
            }
            None => String::from("N/A"),
        };

        format!(
            "Focused on: {}
State:
    Radius: {:.0} m
    Speed: {:.0} m/s
Orbit:
    {}",
            name,
            state.get_position(frame).coords.norm(),
            state.get_velocity(frame).norm(),
            orbit_text
        )
    }

    fn focused_object(&self) -> FocusPoint {
        self.camera_focus_order[self.camera_focus_idx]
    }

    fn focused_object_frame(&self) -> Frame {
        match self.focused_object() {
            FocusPoint::Body(id) => Frame::BodyInertial(id),
            FocusPoint::Ship(id) => match self.ship_camera_inertial {
                true => Frame::ShipInertial(id),
                false => Frame::ShipOrbital(id),
            },
        }
    }

    fn reset_min_distance(&mut self) {
        let dist = match self.focused_object() {
            FocusPoint::Body(id) => self.universe.get_body(id).info().radius * 2.0,
            FocusPoint::Ship(_) => TEST_SHIP_SIZE * 2.0,
        };
        self.camera.set_min_distance(dist);
    }
}

// Helpful for avoiding ambiguous typing
fn convert_f32(p: Point3<f64>) -> Point3<f32> {
    nalgebra::convert(p)
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
        pts.push(nalgebra::convert(pt));
    }
    pts
}

fn set_object_position(obj: &mut SceneNode, position: Point3<f64>) {
    obj.set_local_translation(Translation3::from(convert_f32(position).coords));
}
