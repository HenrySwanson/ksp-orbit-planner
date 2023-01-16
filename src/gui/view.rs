use std::collections::HashMap;

use kiss3d::camera::Camera;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::renderer::Renderer;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use nalgebra::{Isometry3, Point3, Translation3, Unit, Vector3};

use super::camera::ZoomableCamera;
use super::controller::Controller;
use super::renderer::CompoundRenderer;
use super::OrbitPatch;
use crate::astro::orbit::Orbit;
use crate::model::orrery::{Body, BodyID, Frame, Orrery, Ship, ShipID};
use crate::model::timeline::Timeline;

const TEST_SHIP_SIZE: f32 = 1e6;

pub struct View {
    // Object state
    timeline: Timeline,
    orrery: Orrery,
    time: f64,
    body_spheres: HashMap<BodyID, SceneNode>,
    ship_objects: HashMap<ShipID, SceneNode>,
    // Camera
    camera: ZoomableCamera,
    camera_focus: CameraFocus,
    ship_camera_inertial: bool,
    // Misc
    renderer: CompoundRenderer,
}

#[derive(Debug, Clone, Copy)]
pub enum FocusPoint {
    Body(BodyID),
    Ship(ShipID),
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
                if ship.parent_id() == body.id {
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

impl View {
    pub fn new(timeline: Timeline, window: &mut Window) -> Self {
        let orrery = timeline
            .get_orrery_at(0.0)
            .expect("Timeline starts after 0")
            .clone();

        // Set up camera
        // TODO figure out what distance to put the camera...
        let camera = ZoomableCamera::new(2.0e9);
        let camera_focus = CameraFocus::new(&orrery);
        let ship_camera_inertial = true;

        // Create objects for bodies
        let mut body_spheres = HashMap::new();
        for body in orrery.bodies() {
            let sphere = Self::create_body_object(window, &body);
            body_spheres.insert(body.id, sphere);
        }

        // Create objects for ships
        let mut ship_objects = HashMap::new();
        for ship in orrery.ships() {
            let cube = Self::create_ship_object(window, ship);
            ship_objects.insert(ship.id, cube);
        }

        let mut simulation = Self {
            timeline,
            orrery,
            time: 0.0,
            body_spheres,
            ship_objects,
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

    pub fn update_state_by(&mut self, timestep: f64) {
        // Update the universe, then move scene objects to the right places
        self.time = f64::max(self.time + timestep, 0.0);
        self.timeline.extend_end_time(self.time);
        self.orrery = self
            .timeline
            .get_orrery_at(self.time)
            .expect("TODO implement model extension")
            .clone();
        self.update_scene_objects();
    }

    pub fn camera_focus_next(&mut self) {
        self.camera_focus.next();
        self.fix_camera_zoom();
        self.update_scene_objects();
    }

    pub fn camera_focus_prev(&mut self) {
        self.camera_focus.prev();
        self.fix_camera_zoom();
        self.update_scene_objects();
    }

    pub fn camera_inertial_toggle(&mut self) {
        self.ship_camera_inertial = !self.ship_camera_inertial;
        self.fix_camera_zoom();
        self.update_scene_objects();
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
            .convert_frames(frame, self.focused_object_frame(), self.time);
        nalgebra::convert(*transform.isometry())
    }

    // the big boy
    pub fn prerender_scene(&mut self, window: &mut Window, controller: &Controller) {
        // Draw grid
        draw_grid(window, 20, 1.0e9, &Point3::new(0.5, 0.5, 0.5));

        // Draw orbital axes
        for orbit in self.orrery.body_orbits() {
            self.draw_orbital_axes(window, orbit.orbit());
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
            &self.time_summary_text(controller.timestep, controller.fps_counter.value()),
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
            let state = self.orrery.get_body_state(*id, self.time);
            let position = state.get_position(camera_frame, self.time);
            set_position_helper(sphere, position);
        }

        for (id, cube) in self.ship_objects.iter_mut() {
            let state = self.orrery.get_ship_state(*id, self.time);
            let position = state.get_position(camera_frame, self.time);
            set_position_helper(cube, position);
        }
    }

    fn draw_orbital_axes(&self, window: &mut Window, orbit: &Orbit<Body, Body>) {
        // TODO: this renders the axes at the center of the body; I think we probably
        // want center of the orbit instead. But only do that if you're doing
        // this only for the focused body.
        let body = orbit.secondary();
        let axis_length = 2.0 * body.info.radius;
        let transform = self.transform_to_focus_space(Frame::BodyInertial(body.id));
        let origin = transform * Point3::origin();

        // Draws a ray starting at the origin of the body, and proceeding in the given
        // direction. Length and v are separated because one's f32 and the
        // other's f64. Oh well.
        let mut draw_ray = |v: Unit<Vector3<f64>>, length: f32, color: Point3<f32>| {
            let v: Vector3<f32> = nalgebra::convert(v.into_inner());
            let end_pt = origin + length * (transform * v);
            window.draw_line(&origin, &end_pt, &color);
        };

        draw_ray(
            orbit.periapse_vector(),
            axis_length,
            Point3::new(1.0, 0.0, 0.0),
        );
        draw_ray(
            orbit.asc_node_vector(),
            axis_length,
            Point3::new(0.0, 1.0, 0.0),
        );
        draw_ray(
            orbit.normal_vector(),
            axis_length,
            Point3::new(0.0, 0.0, 1.0),
        );
    }

    fn left_hand_text(&self) -> String {
        let (state, frame) = match self.camera_focus.point() {
            FocusPoint::Body(id) => {
                let frame = match self.orrery.get_parent(id) {
                    Some(id) => Frame::BodyInertial(id),
                    None => Frame::Root,
                };
                (self.orrery.get_body_state(id, self.time), frame)
            }
            FocusPoint::Ship(id) => {
                let frame = Frame::BodyInertial(self.orrery.get_ship(id).parent_id());
                (self.orrery.get_ship_state(id, self.time), frame)
            }
        };

        format!(
            "Focused on: {}
State:
    Radius: {:.0} m
    Speed: {:.0} m/s
Orbiting: {}",
            self.focused_body_name(),
            state.get_position(frame, self.time).coords.norm(),
            state.get_velocity(frame, self.time).norm(),
            self.orbit_summary_text(),
        )
    }

    fn focused_body_name(&self) -> String {
        match self.camera_focus.point() {
            FocusPoint::Body(id) => {
                let body = self.orrery.get_body(id);
                body.info.name
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
            FocusPoint::Body(id) => match &self.orrery.orbit_of_body(id) {
                None => return String::from("N/A"),
                Some(orbit) => orbit.clone().with_secondary(()),
            },
            FocusPoint::Ship(id) => self.orrery.orbit_of_ship(id).with_secondary(()),
        };

        let parent_body = self.orrery.get_body(orbit.orbit().primary().id);
        let orbit = orbit.orbit();

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

    fn time_summary_text(&self, timestep: f64, fps: f64) -> String {
        format!(
            "Time: {}
Timestep: {} s/frame
FPS: {:.0}",
            format_seconds(self.time),
            timestep,
            fps,
        )
    }

    pub fn cameras_and_effect_and_renderer(
        &mut self,
    ) -> (
        Option<&mut dyn Camera>,
        Option<&mut dyn PlanarCamera>,
        Option<&mut dyn Renderer>,
        Option<&mut dyn PostProcessingEffect>,
    ) {
        // TODO: move these into the actual render
        self.prep_soi();
        self.prep_orbits();
        (Some(&mut self.camera), None, Some(&mut self.renderer), None)
    }

    fn prep_soi(&mut self) {
        let soi_id = match self.camera_focus.point() {
            FocusPoint::Body(id) => id,
            FocusPoint::Ship(id) => self.orrery.get_ship(id).parent_id(),
        };

        let soi_radius = match self.orrery.get_soi_radius(soi_id) {
            Some(r) => r,
            None => return, // early return if Sun
        };

        // The SOI body is located at the origin in its own frame, which might not be
        // the focus frame (for example, if we are focused on a ship).
        let body_pt = self.transform_to_focus_space(Frame::BodyInertial(soi_id)) * Point3::origin();

        // Make an okayish SOI color by dimming the body color.
        let body_color = self.orrery.get_body(soi_id).info.color;
        let soi_color = Point3::from(body_color.coords * 0.5);

        self.renderer
            .draw_soi(body_pt, soi_radius as f32, soi_color);
    }

    fn prep_orbits(&mut self) {
        for orbit in self.orrery.body_orbits() {
            let secondary = orbit.orbit().secondary();

            let color = secondary.info.color;
            let frame = Frame::BodyInertial(orbit.orbit().primary().id);
            self.renderer.draw_orbit(
                OrbitPatch::new(&orbit, self.time),
                color,
                self.transform_to_focus_space(frame),
            );
        }

        for ship in self.orrery.ships() {
            let orbit = self.orrery.orbit_of_ship(ship.id).with_secondary(());

            let color = Point3::new(1.0, 1.0, 1.0);
            let frame = Frame::BodyInertial(orbit.orbit().primary().id);
            self.renderer.draw_orbit(
                OrbitPatch::new(&orbit, self.time),
                color,
                self.transform_to_focus_space(frame),
            );
        }
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
