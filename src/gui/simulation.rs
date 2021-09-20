use kiss3d::event::{Action, Event, EventManager, Key, WindowEvent};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;

use nalgebra::{Point3, Translation3, Vector3};

use std::collections::HashMap;
use std::f64::consts::PI;
use std::time::Instant;

use super::camera::CustomCamera;

use crate::universe::{BodyID, Frame, FrameTransform, OrbitPatch, ShipID, Universe};

const TEST_SHIP_SIZE: f32 = 1e6;

// Key config, all in one place
const KEY_PREV_FOCUS: Key = Key::Q;
const KEY_NEXT_FOCUS: Key = Key::E;
const KEY_SPEED_UP: Key = Key::Period;
const KEY_SLOW_DOWN: Key = Key::Comma;
const KEY_REWIND: Key = Key::R;
const KEY_PAUSE: Key = Key::P;
const KEY_CAMERA_SWAP: Key = Key::C;

#[derive(Debug, Clone, Copy)]
pub enum FocusPoint {
    Body(BodyID),
    Ship(ShipID),
}

pub struct Path {
    nodes: Vec<Point3<f32>>,
    frame: Frame,
    color: Point3<f32>,
}

pub struct FpsCounter {
    instant: Instant,
    counter: usize,
    window_size_millis: usize,
    previous_fps: f64,
}

pub struct CameraFocus {
    focus_points: Vec<FocusPoint>,
    focus_idx: usize,
}

pub struct Simulation {
    // Object state
    universe: Universe,
    body_spheres: HashMap<BodyID, SceneNode>,
    ship_objects: HashMap<ShipID, SceneNode>,
    paths: Vec<Path>,
    // Timestep
    timestep: f64,
    paused: bool,
    fps_counter: FpsCounter,
    // Camera
    camera: CustomCamera,
    camera_focus: CameraFocus,
    ship_camera_inertial: bool,
}

impl FpsCounter {
    pub fn new(window_size_millis: usize) -> Self {
        FpsCounter {
            instant: Instant::now(),
            counter: 0,
            previous_fps: 0.0,
            window_size_millis,
        }
    }

    pub fn reset(&mut self) {
        self.instant = Instant::now();
        self.counter = 0;
    }

    pub fn value(&self) -> f64 {
        self.previous_fps
    }
    pub fn increment(&mut self) {
        self.counter += 1;

        let elapsed = self.instant.elapsed();
        if elapsed.as_millis() > self.window_size_millis as u128 {
            self.previous_fps = (1000 * self.counter) as f64 / elapsed.as_millis() as f64;
            self.reset();
        }
    }
}

impl CameraFocus {
    pub fn new(universe: &Universe) -> Self {
        // TODO sort focus points in a more systematic way
        let mut bodies: Vec<_> = universe.bodies().collect();
        bodies.sort_by_key(|b| b.id());
        let mut ships: Vec<_> = universe.ships().collect();
        ships.sort_by_key(|s| s.id());

        let mut focus_points = vec![];
        for body in bodies.into_iter() {
            focus_points.push(FocusPoint::Body(body.id()));
            // Now put in all ships orbiting that body
            for ship in ships.iter() {
                if ship.get_parent_id() == body.id() {
                    focus_points.push(FocusPoint::Ship(ship.id()));
                }
            }
        }

        CameraFocus {
            focus_points,
            focus_idx: 0,
        }
    }

    pub fn next(&mut self) {
        let num_bodies = self.focus_points.len();
        self.focus_idx = (self.focus_idx + 1) % num_bodies;
    }

    pub fn prev(&mut self) {
        let num_bodies = self.focus_points.len();
        self.focus_idx = (self.focus_idx + num_bodies - 1) % num_bodies;
    }

    pub fn point(&self) -> FocusPoint {
        self.focus_points[self.focus_idx]
    }
}

impl Simulation {
    pub fn new(universe: Universe, window: &mut Window) -> Self {
        // Set up camera
        // TODO figure out what distance to put the camera...
        let camera = CustomCamera::new(2.0e9);
        let camera_focus = CameraFocus::new(&universe);
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
            body_spheres.insert(body.id(), sphere);

            // TODO remove, or somehow make optional
            // Make axes that show the planet's orbits orientation
            let orbit = match body.get_orbit() {
                Some(orbit) => orbit,
                None => continue,
            };

            let make_axis_path = |v, color| -> Path {
                let v: Vector3<f32> = nalgebra::convert(v);
                let v = 2.0 * body_info.radius * v;
                let pt: Point3<f32> = Point3::from(v);
                Path {
                    nodes: vec![Point3::origin(), pt],
                    frame: Frame::BodyInertial(body.id()),
                    color,
                }
            };
            paths.push(make_axis_path(
                orbit.orbit.periapse_vector(),
                Point3::new(1.0, 0.0, 0.0),
            ));
            paths.push(make_axis_path(
                orbit.orbit.asc_node_vector(),
                Point3::new(0.0, 1.0, 0.0),
            ));
            paths.push(make_axis_path(
                orbit.orbit.normal_vector(),
                Point3::new(0.0, 0.0, 1.0),
            ));
        }

        // Collect ships
        let mut ship_objects = HashMap::new();
        for ship in universe.ships() {
            // Make the cube that represents the ship
            let mut cube = window.add_cube(TEST_SHIP_SIZE, TEST_SHIP_SIZE, TEST_SHIP_SIZE);
            cube.set_color(1.0, 1.0, 1.0);
            ship_objects.insert(ship.id(), cube);
        }

        Simulation {
            universe,
            body_spheres,
            ship_objects,
            paths,
            timestep: 21600.0 / 60.0, // one Kerbin-day
            paused: true,
            fps_counter: FpsCounter::new(1000),
            camera,
            camera_focus,
            ship_camera_inertial,
        }
    }

    pub fn render_loop(&mut self, window: &mut Window) {
        self.fps_counter.reset();

        loop {
            self.process_user_input(window.events());
            self.update_state();
            // This step is when kiss3d detects when the window is exited
            // TODO create "RenderContext" object that can be passed down
            if !self.render_scene(window) {
                break;
            };

            self.fps_counter.increment();
        }
    }

    fn process_user_input(&mut self, mut events: EventManager) {
        // Process events
        for event in events.iter() {
            self.process_event(event);
        }
    }

    fn process_event(&mut self, event: Event) {
        match event.value {
            WindowEvent::Key(KEY_NEXT_FOCUS, Action::Press, _) => {
                self.camera_focus.next();
                self.fix_camera_zoom();
            }
            WindowEvent::Key(KEY_PREV_FOCUS, Action::Press, _) => {
                self.camera_focus.prev();
                self.fix_camera_zoom();
            }
            WindowEvent::Key(KEY_SPEED_UP, Action::Press, _) => {
                self.timestep *= 2.0;
                println!("Timestep is {} s / s", (60.0 * self.timestep).round())
            }
            WindowEvent::Key(KEY_SLOW_DOWN, Action::Press, _) => {
                self.timestep /= 2.0;
                println!("Timestep is {} s / s", (60.0 * self.timestep).round())
            }
            WindowEvent::Key(KEY_REWIND, Action::Press, _) => {
                self.timestep *= -1.0;
                self.paused = false;
            }
            WindowEvent::Key(KEY_PAUSE, Action::Press, _) => {
                self.paused = !self.paused;
            }
            WindowEvent::Key(KEY_CAMERA_SWAP, Action::Press, _) => {
                self.ship_camera_inertial = !self.ship_camera_inertial;
            }
            _ => {}
        }
    }

    fn update_state(&mut self) {
        if !self.paused {
            // Update the universe, then move scene objects to the right places
            self.universe.update_time(self.timestep);
        }
        // TODO: should be able to put it inside this branch but apparently not
        self.update_scene_objects();
    }

    fn fix_camera_zoom(&mut self) {
        let dist = match self.camera_focus.point() {
            FocusPoint::Body(id) => self.universe.get_body(id).info().radius * 2.0,
            FocusPoint::Ship(_) => TEST_SHIP_SIZE * 2.0,
        };
        self.camera.set_min_distance(dist);
    }

    fn focused_object_frame(&self) -> Frame {
        match self.camera_focus.point() {
            FocusPoint::Body(id) => Frame::BodyInertial(id),
            FocusPoint::Ship(id) => match self.ship_camera_inertial {
                true => Frame::ShipInertial(id),
                false => Frame::ShipOrbital(id),
            },
        }
    }

    // the big boy
    fn render_scene(&mut self, window: &mut Window) -> bool {
        // Draw grid
        draw_grid(window, 20, 1.0e9, &Point3::new(0.5, 0.5, 0.5));

        // Draw orbits
        for body in self.universe.bodies() {
            let orbit = match body.get_orbit() {
                Some(o) => o,
                None => continue,
            };
            let color = body.info().color;
            self.render_orbit_patch(window, &orbit, &color);
        }
        for ship in self.universe.ships() {
            let orbit = ship.get_orbit();
            let color = Point3::new(1.0, 1.0, 1.0);
            self.render_orbit_patch(window, &orbit, &color);
        }

        // Draw paths
        for path in self.paths.iter() {
            self.draw_path_object(window, path);
        }

        // Draw sphere of influence
        let soi_body_id = match self.camera_focus.point() {
            FocusPoint::Body(id) => id,
            FocusPoint::Ship(id) => self.universe.get_ship(id).get_parent_id(),
        };
        self.draw_soi(window, soi_body_id);

        // Draw text
        use nalgebra::Point2;
        let default_font = kiss3d::text::Font::default();
        let text_color = Point3::new(1.0, 1.0, 1.0);
        window.draw_text(
            &self.orbit_summary_text(),
            &Point2::origin(),
            60.0,
            &default_font,
            &text_color,
        );
        window.draw_text(
            &self.time_summary_text(),
            // no idea why i have to multiply by 2.0, but there it is
            &Point2::new(window.width() as f32 * 2.0 - 600.0, 0.0),
            60.0,
            &default_font,
            &text_color,
        );

        // Render and return bool
        window.render_with_camera(&mut self.camera)
    }

    fn update_scene_objects(&mut self) {
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

    fn render_orbit_patch(&self, window: &mut Window, orbit: &OrbitPatch, color: &Point3<f32>) {
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
        let delta_s = end_s - start_s;
        assert!(delta_s >= 0.0);

        // Get the transform into the focused frame
        let transform = self.universe.orrery.convert_frames(
            Frame::BodyInertial(orbit.parent_id),
            self.focused_object_frame(),
        );

        // Get some points around the orbit
        let num_segments = 180;
        let pts = (0..num_segments + 1)
            .map(|i| start_s + (i as f64) / (num_segments as f64) * delta_s)
            .map(|s| orbit.orbit.get_state(s).get_position())
            .map(|v| transform.convert_point(&Point3::from(v)))
            .map(convert_f32);

        draw_path_raw(window, pts, &color);
    }

    fn draw_path_object(&self, window: &mut Window, path: &Path) {
        // Transform points into the right frame before drawing them
        let transform = self
            .universe
            .orrery
            .convert_frames(path.frame, self.focused_object_frame());
        let transform: FrameTransform<f32> = nalgebra::convert(transform);

        draw_path_raw(
            window,
            path.nodes.iter().map(|p| transform.convert_point(p)),
            &path.color,
        );
    }

    fn draw_soi(&self, window: &mut Window, id: BodyID) {
        let soi_radius = match self.universe.orrery.get_soi_radius(id) {
            Some(r) => r,
            None => return, // early return if Sun
        };

        // Transform the screen x and y vectors into whatever frame we're currently focused on
        use crate::kiss3d::camera::Camera;
        let camera_transform = self.camera.view_transform().inverse();
        let x_vec = camera_transform.transform_vector(&Vector3::x()).normalize();
        let y_vec = camera_transform.transform_vector(&Vector3::y()).normalize();

        // Get the position of the body in our current frame (may be non-zero if we're
        // focused on a ship).
        let body_pt = self
            .universe
            .orrery
            .convert_frames(Frame::BodyInertial(id), self.focused_object_frame())
            .convert_point(&Point3::origin());

        let num_pts = 100;
        let pts_iter = (0..num_pts + 1)
            .map(|i| 2.0 * std::f32::consts::PI * (i as f32) / (num_pts as f32))
            .map(|theta| x_vec * theta.cos() + y_vec * theta.sin())
            .map(|v| convert_f32(body_pt) + (soi_radius as f32) * v);

        // Make an okayish SOI color
        let body_color = self.universe.get_body(id).info().color;
        let soi_color = Point3::from(body_color.coords * 0.5);
        draw_path_raw(window, pts_iter, &soi_color);
    }

    fn orbit_summary_text(&self) -> String {
        let (name, state, orbit, frame) = match self.camera_focus.point() {
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
                    "{}
    SMA: {:.0}
    Eccentricity: {:.3}
    Inclination: {:.3}
    LAN: {:.1}
    Arg PE: {:.1}",
                    self.universe.get_body(o.parent_id).info().name,
                    o.orbit.semimajor_axis(),
                    o.orbit.eccentricity(),
                    o.orbit.inclination().to_degrees(),
                    o.orbit.long_asc_node().to_degrees(),
                    o.orbit.arg_periapse().to_degrees(),
                )
            }
            None => String::from("N/A"),
        };

        format!(
            "Focused on: {}
State:
    Radius: {:.0} m
    Speed: {:.0} m/s
Orbiting: {}",
            name,
            state.get_position(frame).coords.norm(),
            state.get_velocity(frame).norm(),
            orbit_text
        )
    }

    fn time_summary_text(&self) -> String {
        format!(
            "Time: {}
Timestep: {} s/frame
FPS: {:.0}",
            format_seconds(self.universe.orrery.get_time()),
            self.timestep,
            self.fps_counter.value(),
        )
    }
}

// Helpful for avoiding ambiguous typing
fn convert_f32(p: Point3<f64>) -> Point3<f32> {
    nalgebra::convert(p)
}

fn set_object_position(obj: &mut SceneNode, position: Point3<f64>) {
    obj.set_local_translation(Translation3::from(convert_f32(position).coords));
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

fn draw_path_raw<I: Iterator<Item = Point3<f32>>>(
    window: &mut Window,
    points: I,
    color: &Point3<f32>,
) {
    let mut prev_pt = None;
    for pt in points {
        if let Some(prev_pt) = prev_pt {
            window.draw_line(&prev_pt, &pt, color);
        }
        prev_pt = Some(pt);
    }
}

fn format_seconds(seconds: f64) -> String {
    let mut total_seconds = seconds as u64;
    let n_minutes = 60;
    let n_hours = n_minutes * 60;
    let n_days = n_hours * 24;
    let n_years = 365 * n_days;

    macro_rules! count_and_remainder {
        ($variable:ident, $divisor:expr) => {
            let $variable = total_seconds / $divisor;
            total_seconds %= $divisor;
        };
    }

    count_and_remainder!(years, n_years);
    count_and_remainder!(days, n_days);
    count_and_remainder!(hours, n_hours);
    count_and_remainder!(minutes, n_minutes);

    format!(
        "{}y, {}d, {:02}:{:02}:{:02}",
        years, days, hours, minutes, total_seconds
    )
}
