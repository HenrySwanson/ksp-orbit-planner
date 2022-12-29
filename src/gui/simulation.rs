use kiss3d::camera::Camera;
use kiss3d::event::{Action, Event, EventManager, Key, WindowEvent};
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::renderer::Renderer;
use kiss3d::scene::SceneNode;
use kiss3d::window::{State, Window};

use nalgebra::{Isometry3, Point3, Translation3, Vector3};

use std::collections::HashMap;
use std::time::Instant;

use super::camera::ZoomableCamera;
use super::renderer::CompoundRenderer;

use crate::model::Timeline;
use crate::orrery::{Body, BodyID, BodyState, Frame, OrbitPatch, Orrery, Ship, ShipID};

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

pub struct FpsCounter {
    instant: Instant,
    counter: usize,
    window_size_millis: usize,
    previous_fps: f64,
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

pub struct CameraFocus {
    focus_points: Vec<FocusPoint>,
    focus_idx: usize,
}

impl CameraFocus {
    pub fn new(orrery: &Orrery) -> Self {
        // TODO sort focus points in a more systematic way
        let mut bodies: Vec<_> = orrery.bodies().collect();
        bodies.sort_by_key(|b| b.id);
        let mut ships: Vec<_> = orrery.ships().collect();
        ships.sort_by_key(|s| s.id);

        let mut focus_points = vec![];
        for body in bodies.into_iter() {
            focus_points.push(FocusPoint::Body(body.id));
            // Now put in all ships orbiting that body
            for ship in ships.iter() {
                if ship.parent_id == body.id {
                    focus_points.push(FocusPoint::Ship(ship.id));
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

pub struct Simulation {
    // Object state
    timeline: Timeline,
    orrery: Orrery,
    time: f64,
    body_spheres: HashMap<BodyID, SceneNode>,
    ship_objects: HashMap<ShipID, SceneNode>,
    // Timestep
    timestep: f64,
    paused: bool,
    fps_counter: FpsCounter,
    // Camera
    camera: ZoomableCamera,
    camera_focus: CameraFocus,
    ship_camera_inertial: bool,
    // Misc
    renderer: CompoundRenderer,
}

impl Simulation {
    pub fn new(timeline: Timeline, window: &mut Window) -> Self {
        let orrery = timeline
            .get_orrery_at(0.0)
            .expect("Timeline starts after 0");

        // Set up camera
        // TODO figure out what distance to put the camera...
        let camera = ZoomableCamera::new(2.0e9);
        let camera_focus = CameraFocus::new(&orrery);
        let ship_camera_inertial = true;

        // Create objects for bodies
        let mut body_spheres = HashMap::new();
        for body in orrery.bodies() {
            let sphere = Simulation::create_body_object(window, body);
            body_spheres.insert(body.id, sphere);
        }

        // Create objects for ships
        let mut ship_objects = HashMap::new();
        for ship in orrery.ships() {
            let cube = Simulation::create_ship_object(window, ship);
            ship_objects.insert(ship.id, cube);
        }

        let mut simulation = Simulation {
            timeline,
            orrery,
            time: 0.0,
            body_spheres,
            ship_objects,
            timestep: 21600.0 / 60.0, // one Kerbin-day
            paused: true,
            fps_counter: FpsCounter::new(1000),
            camera,
            camera_focus,
            ship_camera_inertial,
            renderer: CompoundRenderer::new(),
        };
        simulation.update_scene_objects();

        simulation
    }

    fn create_body_object(window: &mut Window, body: &Body) -> SceneNode {
        // Make the sphere that represents the body
        let mut sphere = window.add_sphere(body.info.radius);
        let color = &body.info.color;
        sphere.set_color(color.x, color.y, color.z);
        sphere
    }

    fn create_ship_object(window: &mut Window, _: &Ship) -> SceneNode {
        // Make the cube that represents the ship
        let mut cube = window.add_cube(TEST_SHIP_SIZE, TEST_SHIP_SIZE, TEST_SHIP_SIZE);
        cube.set_color(1.0, 1.0, 1.0);
        cube
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
                self.update_scene_objects();
            }
            WindowEvent::Key(KEY_PREV_FOCUS, Action::Press, _) => {
                self.camera_focus.prev();
                self.fix_camera_zoom();
                self.update_scene_objects();
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
                self.update_scene_objects();
            }
            _ => {}
        }
    }

    fn update_state(&mut self) {
        if !self.paused {
            // Update the universe, then move scene objects to the right places
            self.time = f64::max(self.time + self.timestep, 0.0);
            self.timeline.extend_end_time(self.time);
            self.orrery = self
                .timeline
                .get_orrery_at(self.time)
                .expect("TODO implement model extension");
            self.update_scene_objects();
        }
    }

    fn fix_camera_zoom(&mut self) {
        let dist = match self.camera_focus.point() {
            FocusPoint::Body(id) => self.orrery.get_body(id).info.radius * 2.0,
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

    fn transform_to_focus_space(&self, frame: Frame) -> Isometry3<f32> {
        let transform = self
            .orrery
            .convert_frames(frame, self.focused_object_frame());
        nalgebra::convert(*transform.isometry())
    }

    // the big boy
    fn prerender_scene(&mut self, window: &mut Window) {
        // Draw grid
        draw_grid(window, 20, 1.0e9, &Point3::new(0.5, 0.5, 0.5));

        // Draw orbital axes
        for body in self.orrery.bodies() {
            self.draw_orbital_axes(window, body);
        }

        // Draw text
        use nalgebra::Point2;
        let default_font = kiss3d::text::Font::default();
        let text_color = Point3::new(1.0, 1.0, 1.0);
        window.draw_text(
            &self.left_hand_text(),
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
    }

    fn update_scene_objects(&mut self) {
        // does some nice conversions
        fn set_position_helper(obj: &mut SceneNode, position: Point3<f64>) {
            let position: Point3<f32> = nalgebra::convert(position);
            obj.set_local_translation(Translation3::from(position));
        }

        // TODO apply rotations too!
        let camera_frame = self.focused_object_frame();
        for (id, sphere) in self.body_spheres.iter_mut() {
            let state = self.orrery.get_body_state(*id);
            let position = state.get_position(camera_frame);
            set_position_helper(sphere, position);
        }

        for (id, cube) in self.ship_objects.iter_mut() {
            let state = self.orrery.get_ship_state(*id);
            let position = state.get_position(camera_frame);
            set_position_helper(cube, position);
        }
    }

    fn draw_orbital_axes(&self, window: &mut Window, body: &Body) {
        // TODO: this renders the axes at the center of the body; I think we probably want center
        // of the orbit instead. But only do that if you're doing this only for the focused body.
        let orbit = match &body.state {
            BodyState::FixedAtOrigin => return,
            BodyState::Orbiting(odata) => odata.state.to_orbit_patch(odata.parent_id),
        };

        let axis_length = 2.0 * body.info.radius;
        let transform = self.transform_to_focus_space(Frame::BodyInertial(body.id));
        let origin = transform * Point3::origin();

        // Draws a ray starting at the origin of the body, and proceeding in the given direction.
        // Length and v are separated because one's f32 and the other's f64. Oh well.
        let mut draw_ray = |v: Vector3<f64>, length: f32, color: Point3<f32>| {
            let v: Vector3<f32> = nalgebra::convert(v);
            let end_pt = origin + length * (transform * v.normalize());
            window.draw_line(&origin, &end_pt, &color);
        };

        draw_ray(
            orbit.orbit.periapse_vector(),
            axis_length,
            Point3::new(1.0, 0.0, 0.0),
        );
        draw_ray(
            orbit.orbit.asc_node_vector(),
            axis_length,
            Point3::new(0.0, 1.0, 0.0),
        );
        draw_ray(
            orbit.orbit.normal_vector(),
            axis_length,
            Point3::new(0.0, 0.0, 1.0),
        );
    }

    fn left_hand_text(&self) -> String {
        let (state, frame) = match self.camera_focus.point() {
            FocusPoint::Body(id) => {
                let frame = match self.orrery.get_body(id).parent_id() {
                    Some(id) => Frame::BodyInertial(id),
                    None => Frame::Root,
                };
                (self.orrery.get_body_state(id), frame)
            }
            FocusPoint::Ship(id) => {
                let frame = Frame::BodyInertial(self.orrery.get_ship(id).parent_id);
                (self.orrery.get_ship_state(id), frame)
            }
        };

        format!(
            "Focused on: {}
State:
    Radius: {:.0} m
    Speed: {:.0} m/s
Orbiting: {}",
            self.focused_body_name(),
            state.get_position(frame).coords.norm(),
            state.get_velocity(frame).norm(),
            self.orbit_summary_text(),
        )
    }

    fn focused_body_name(&self) -> String {
        match self.camera_focus.point() {
            FocusPoint::Body(id) => {
                let body = self.orrery.get_body(id);
                body.info.name.to_owned()
            }
            FocusPoint::Ship(_) => {
                format!(
                    "<Ship> ({})",
                    if self.ship_camera_inertial {
                        "inertial"
                    } else {
                        "orbital"
                    }
                )
            }
        }
    }

    fn orbit_summary_text(&self) -> String {
        let orbit = match self.camera_focus.point() {
            FocusPoint::Body(id) => match &self.orrery.get_body(id).state {
                BodyState::FixedAtOrigin => return String::from("N/A"),
                BodyState::Orbiting(odata) => odata.state.to_orbit_patch(odata.parent_id),
            },
            FocusPoint::Ship(id) => {
                let ship = self.orrery.get_ship(id);
                ship.state.to_orbit_patch(ship.parent_id)
            }
        };

        let parent_body = self.orrery.get_body(orbit.parent_id);
        let orbit = orbit.orbit;

        // Indentation is intentional
        format!(
            "{}
    SMA: {:.0}
    Eccentricity: {:.3}
    Inclination: {:.3}
    LAN: {:.1}
    Arg PE: {:.1}",
            parent_body.info.name,
            orbit.semimajor_axis(),
            orbit.eccentricity(),
            orbit.inclination().to_degrees(),
            orbit.long_asc_node().to_degrees(),
            orbit.arg_periapse().to_degrees(),
        )
    }

    fn time_summary_text(&self) -> String {
        format!(
            "Time: {}
Timestep: {} s/frame
FPS: {:.0}",
            format_seconds(self.time),
            self.timestep,
            self.fps_counter.value(),
        )
    }

    fn prep_soi(&mut self) {
        let soi_id = match self.camera_focus.point() {
            FocusPoint::Body(id) => id,
            FocusPoint::Ship(id) => self.orrery.get_ship(id).parent_id,
        };

        let soi_radius = match self.orrery.get_soi_radius(soi_id) {
            Some(r) => r,
            None => return, // early return if Sun
        };

        // The SOI body is located at the origin in its own frame, which might not be the focus
        // frame (for example, if we are focused on a ship).
        let body_pt = self.transform_to_focus_space(Frame::BodyInertial(soi_id)) * Point3::origin();

        // Make an okayish SOI color by dimming the body color.
        let body_color = self.orrery.get_body(soi_id).info.color;
        let soi_color = Point3::from(body_color.coords * 0.5);

        self.renderer
            .draw_soi(body_pt, soi_radius as f32, soi_color);
    }

    fn prep_orbits(&mut self) {
        for body in self.orrery.bodies() {
            let orbit = match &body.state {
                BodyState::FixedAtOrigin => continue,
                BodyState::Orbiting(odata) => odata.state.to_orbit_patch(odata.parent_id),
            };
            let color = body.info.color;
            let frame = Frame::BodyInertial(orbit.parent_id);
            self.renderer
                .draw_orbit(orbit, color, self.transform_to_focus_space(frame));
        }

        for ship in self.orrery.ships() {
            let orbit = ship.state.to_orbit_patch(ship.parent_id);
            let color = Point3::new(1.0, 1.0, 1.0);
            let frame = Frame::BodyInertial(orbit.parent_id);
            self.renderer
                .draw_orbit(orbit, color, self.transform_to_focus_space(frame));
        }
    }
}

impl State for Simulation {
    fn cameras_and_effect_and_renderer(
        &mut self,
    ) -> (
        Option<&mut dyn Camera>,
        Option<&mut dyn PlanarCamera>,
        Option<&mut dyn Renderer>,
        Option<&mut dyn PostProcessingEffect>,
    ) {
        self.prep_soi();
        self.prep_orbits();
        (Some(&mut self.camera), None, Some(&mut self.renderer), None)
    }

    fn step(&mut self, window: &mut Window) {
        self.process_user_input(window.events());
        self.update_state();
        self.prerender_scene(window);
        self.fps_counter.increment();
    }
}

fn draw_grid(window: &mut Window, num_squares: i32, square_size: f32, color: &Point3<f32>) {
    let max_coord = square_size * (num_squares as f32);
    for i in (-num_squares)..(num_squares + 1) {
        let coord = square_size * (i as f32);
        // horizontal
        window.draw_line(
            &Point3::new(-max_coord, coord, 0.0),
            &Point3::new(max_coord, coord, 0.0),
            color,
        );
        // vertical
        window.draw_line(
            &Point3::new(coord, -max_coord, 0.0),
            &Point3::new(coord, max_coord, 0.0),
            color,
        );
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

// TODO: delete this once the timeline migration is done!
impl crate::orrery::CartesianState {
    fn to_orbit_patch(&self, parent_id: BodyID) -> OrbitPatch {
        let orbit = self.get_orbit();
        let start_anomaly = self.get_universal_anomaly();

        OrbitPatch {
            orbit,
            start_anomaly,
            end_anomaly: None,
            parent_id,
        }
    }
}
