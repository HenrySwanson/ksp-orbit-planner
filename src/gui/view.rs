use std::collections::HashMap;

use kiss3d::camera::Camera;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::renderer::Renderer;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use nalgebra::{Isometry3, Point3, Translation3, Unit};

use super::camera::ZoomableCamera;
use super::controller::Controller;
use super::renderers::{CompoundRenderer, OrbitPatch};
use crate::astro::orbit::BareOrbit;
use crate::gui::renderers::MarkerType;
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
        let start_time = timeline.start_time();
        let orrery = timeline.get_orrery_at(start_time).unwrap().clone();

        // Set up camera
        // Initial distance doesn't matter, since we're about to call fix_camera_zoom
        let camera = ZoomableCamera::new(1.0);
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
            time: start_time,
            body_spheres,
            ship_objects,
            camera,
            camera_focus,
            ship_camera_inertial,
            renderer: CompoundRenderer::new(),
        };
        simulation.fix_camera_zoom();
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
            .expect("Lookup before universe start")
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
        // Draw a bunch of stuff
        self.renderer.draw_grid(self.camera.distance());
        self.draw_orbits();
        self.draw_orbital_axes();
        self.draw_soi();
        self.draw_markers();

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
            &self.time_summary_text(controller.timestep(), controller.fps()),
            // no idea why i have to multiply by 2.0, but there it is
            &Point2::new(window.width() as f32 * 2.0 - 600.0, 0.0),
            60.0,
            &default_font,
            &text_color,
        );
    }

    fn draw_orbits(&mut self) {
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

    fn draw_orbital_axes(&mut self) {
        // TODO: this renders the axes at the center of the body; I think we probably
        // want center of the orbit instead. But only do that if you're doing
        // this only for the focused body.
        for orbit in self.orrery.body_orbits() {
            let orbit = orbit.orbit();
            let body = orbit.secondary();

            let axes = [
                (orbit.periapse_vector(), Point3::new(1.0, 0.0, 0.0)),
                (orbit.asc_node_vector(), Point3::new(0.0, 1.0, 0.0)),
                (orbit.normal_vector(), Point3::new(0.0, 0.0, 1.0)),
            ]
            .map(|(v, color)| {
                let v = Unit::new_unchecked(nalgebra::convert(v.into_inner()));
                (v, color)
            });

            self.renderer.draw_axes(
                &axes,
                2.0 * body.info.radius,
                self.transform_to_focus_space(Frame::BodyInertial(body.id)),
            );
        }
    }

    fn draw_soi(&mut self) {
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

    fn draw_markers(&mut self) {
        // We draw the marker if we're far enough away that the body is too
        // small to see, but not if we're far enough away that the orbit is too
        // small.

        // These sizes are in pixels
        const MARKER_SIZE: f32 = 18.0;
        const BODY_CUTOFF: f32 = 3.0;
        const ORBIT_CUTOFF: f32 = MARKER_SIZE;

        // Figure out the ratio of pixel size to worldspace lengths.
        // That's determined from the camera distance, the field of view,
        // and the window size.
        let pixel_size_ndc = 2.0 / self.camera.height() as f32;
        let pixel_size_worldspace = {
            // half of the screen height, in worldspace
            let half_height = self.camera.distance() * (self.camera.fovy() / 2.0).tan();
            half_height * 2.0 / self.camera.height() as f32
        };

        let should_draw = |radius: f32, orbit: BareOrbit| -> bool {
            // Figure out the apparent size of objects in screenspace
            let apparent_body_radius = radius / pixel_size_worldspace;
            let apparent_orbit_apoapsis = match orbit.apoapsis() {
                Some(a) => a as f32 / pixel_size_worldspace,
                None => return true, // always draw markers when orbit is open
            };

            // Draw marker if body is too small, unless orbit is also too small
            apparent_body_radius < BODY_CUTOFF && apparent_orbit_apoapsis > ORBIT_CUTOFF
        };

        for orbit in self.orrery.body_orbits() {
            let orbit = orbit.orbit();
            let body = orbit.secondary();

            if !should_draw(body.info.radius, orbit.to_bare()) {
                continue;
            }

            let body_pt =
                self.transform_to_focus_space(Frame::BodyInertial(body.id)) * Point3::origin();

            self.renderer.draw_marker(
                MarkerType::Circle,
                body_pt,
                MARKER_SIZE * pixel_size_ndc,
                body.info.color,
            );
        }
        for ship in self.orrery.ships() {
            if !should_draw(TEST_SHIP_SIZE / 2.0, ship.orbit.orbit().to_bare()) {
                continue;
            }

            let ship_pt =
                self.transform_to_focus_space(Frame::ShipInertial(ship.id)) * Point3::origin();

            self.renderer.draw_marker(
                MarkerType::Square,
                ship_pt,
                MARKER_SIZE * pixel_size_ndc,
                Point3::new(1.0, 1.0, 1.0),
            );
        }
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
        (Some(&mut self.camera), None, Some(&mut self.renderer), None)
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
