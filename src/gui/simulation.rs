use kiss3d::event::{Action, Key, WindowEvent};
use kiss3d::window::Window;

use std::time::Instant;

use super::scene::Scene;

use crate::universe::Universe;

// Key config, all in one place
const KEY_PREV_FOCUS: Key = Key::Q;
const KEY_NEXT_FOCUS: Key = Key::E;
const KEY_SPEED_UP: Key = Key::Period;
const KEY_SLOW_DOWN: Key = Key::Comma;
const KEY_REWIND: Key = Key::R;
const KEY_PAUSE: Key = Key::P;
const KEY_CAMERA_SWAP: Key = Key::C;

pub struct RenderContext {
    pub timestep: f64,
    pub fps: f64,
}

pub struct Simulation {
    universe: Universe,
    window: Window,
    scene: Scene,
    timestep: f64,
    paused: bool,
    fps_last_time: Instant,
    fps_num_frames: usize,
    fps_current: f64,
}

impl Simulation {
    pub fn new(mut window: Window, universe: Universe) -> Self {
        let mut scene = Scene::new(&mut window, &universe);
        scene.reset_min_distance(&universe);

        Simulation {
            universe,
            window,
            scene,
            timestep: 21600.0 / 60.0, // one Kerbin-day
            paused: true,
            fps_last_time: Instant::now(),
            fps_num_frames: 0,
            fps_current: 0.0,
        }
    }

    pub fn render_loop(&mut self) {
        self.fps_last_time = Instant::now();

        loop {
            // Compute FPS
            let elapsed = self.fps_last_time.elapsed();
            if elapsed.as_secs() > 1 {
                self.fps_current = 1000.0 * self.fps_num_frames as f64 / elapsed.as_millis() as f64;
                self.fps_last_time = Instant::now();
                self.fps_num_frames = 0;
            }

            self.process_user_input();
            self.update_state();
            // This step is when kiss3d detects when the window is exited
            // TODO create "RenderContext" object that can be passed down
            if !self.render_scene() {
                break;
            };

            self.fps_num_frames += 1;
        }
    }

    fn process_user_input(&mut self) {
        // Process events
        for event in self.window.events().iter() {
            match event.value {
                WindowEvent::Key(KEY_NEXT_FOCUS, Action::Press, _) => {
                    self.scene.next_focus();
                    self.scene.reset_min_distance(&self.universe);
                }
                WindowEvent::Key(KEY_PREV_FOCUS, Action::Press, _) => {
                    self.scene.prev_focus();
                    self.scene.reset_min_distance(&self.universe);
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
                    // TODO it's kinda silly to have a function for this.
                    // IMO we should be able to pass the unprocessed events down to the
                    // Scene objects
                    self.scene.switch_inertial_camera();
                }
                _ => {}
            }
        }
    }

    fn update_state(&mut self) {
        if !self.paused {
            self.universe.update_time(self.timestep);
        }
    }

    fn render_scene(&mut self) -> bool {
        let ctx = RenderContext {
            timestep: self.timestep,
            fps: self.fps_current,
        };
        self.scene.render(&mut self.window, &self.universe, ctx)
    }
}
