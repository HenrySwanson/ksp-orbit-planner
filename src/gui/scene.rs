use kiss3d::scene::SceneNode;
use kiss3d::window::Window;

use nalgebra::{Point3, Translation3, Vector3};

use std::collections::HashMap;
use std::f64::consts::PI;

use crate::universe::{BodyID, Frame, FrameTransform, Orbit, ShipID, Universe};

use super::camera::CustomCamera;

const TEST_SHIP_SIZE: f32 = 1e6;

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

pub struct Scene {
    body_spheres: HashMap<BodyID, SceneNode>,
    ship_objects: HashMap<ShipID, SceneNode>,
    paths: Vec<Path>,

    camera: CustomCamera,
    camera_focus_order: Vec<FocusPoint>,
    camera_focus_idx: usize,
    ship_camera_inertial: bool,
}

impl Scene {
    pub fn new(window: &mut Window, universe: &Universe) -> Self {
        // We can't query the fps, so let's just set it
        window.set_framerate_limit(Some(60));

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

        Scene {
            body_spheres,
            ship_objects,
            paths,
            camera,
            camera_focus_order,
            camera_focus_idx,
            ship_camera_inertial,
        }
    }

    pub fn reset_min_distance(&mut self, universe: &Universe) {
        let dist = match self.focused_object() {
            FocusPoint::Body(id) => universe.get_body(id).info().radius * 2.0,
            FocusPoint::Ship(_) => TEST_SHIP_SIZE * 2.0,
        };
        self.camera.set_min_distance(dist);
    }

    pub fn next_focus(&mut self) {
        let num_bodies = self.camera_focus_order.len();
        self.camera_focus_idx = (self.camera_focus_idx + 1) % num_bodies;
    }

    pub fn prev_focus(&mut self) {
        let num_bodies = self.camera_focus_order.len();
        self.camera_focus_idx = (self.camera_focus_idx + num_bodies - 1) % num_bodies;
    }

    pub fn switch_inertial_camera(&mut self) {
        self.ship_camera_inertial = !self.ship_camera_inertial;
    }

    pub fn render(&mut self, window: &mut Window, universe: &Universe) -> bool {
        // Move scene objects to the right places
        self.update_scene_objects(universe);

        // Draw grid
        draw_grid(window, 20, 1.0e9, &Point3::new(0.5, 0.5, 0.5));

        // Draw paths
        // TODO this is hard to inline, because passing a reference into the contents
        // of self.paths to a function with &mut self doesn't work
        for path in self.paths.iter() {
            self.draw_path_object(window, universe, path);
        }

        // Draw sphere of influence
        let soi_body_id = match self.focused_object() {
            FocusPoint::Body(id) => id,
            FocusPoint::Ship(id) => universe.get_ship(id).get_parent_id(),
        };
        self.draw_soi(window, universe, soi_body_id);

        // Draw text
        window.draw_text(
            &self.orbit_summary_text(universe),
            &nalgebra::Point2::origin(),
            80.0,
            &kiss3d::text::Font::default(),
            &Point3::new(1.0, 1.0, 1.0),
        );

        // Render and return bool
        window.render_with_camera(&mut self.camera)
    }

    fn update_scene_objects(&mut self, universe: &Universe) {
        // TODO apply rotations too!
        let camera_frame = self.focused_object_frame();
        for (id, sphere) in self.body_spheres.iter_mut() {
            let state = universe.get_body(*id).state();
            set_object_position(sphere, state.get_position(camera_frame));
        }

        for (id, cube) in self.ship_objects.iter_mut() {
            let state = universe.get_ship(*id).state();
            set_object_position(cube, state.get_position(camera_frame));
        }
    }

    fn draw_path_object(&self, window: &mut Window, universe: &Universe, path: &Path) {
        // Transform points into the right frame before drawing them
        let transform = universe.convert_frames(path.frame, self.focused_object_frame());
        let transform: FrameTransform<f32> = nalgebra::convert(transform);

        draw_path_raw(
            window,
            path.nodes.iter().map(|p| transform.convert_point(p)),
            &path.color,
        );
    }

    fn draw_soi(&self, window: &mut Window, universe: &Universe, id: BodyID) {
        let soi_radius = match universe.get_soi_radius(id) {
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
        let body_pt = universe
            .convert_frames(Frame::BodyInertial(id), self.focused_object_frame())
            .convert_point(&Point3::origin());

        let num_pts = 100;
        let pts_iter = (0..num_pts + 1)
            .map(|i| 2.0 * std::f32::consts::PI * (i as f32) / (num_pts as f32))
            .map(|theta| x_vec * theta.cos() + y_vec * theta.sin())
            .map(|v| convert_f32(body_pt) + (soi_radius as f32) * v);

        // Make an okayish SOI color
        let body_color = universe.get_body(id).info().color;
        let soi_color = Point3::from(body_color.coords * 0.5);
        draw_path_raw(window, pts_iter, &soi_color);
    }

    fn orbit_summary_text(&self, universe: &Universe) -> String {
        let (name, state, orbit, frame) = match self.focused_object() {
            FocusPoint::Body(id) => {
                let body = universe.get_body(id);
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
                let ship = universe.get_ship(id);
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

// Helpful for avoiding ambiguous typing
fn convert_f32(p: Point3<f64>) -> Point3<f32> {
    nalgebra::convert(p)
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
