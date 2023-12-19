use std::time::Instant;

use kiss3d::event::{Action, Event, Key, WindowEvent};

use super::view::View;

// Key config, all in one place
const KEY_PREV_FOCUS: Key = Key::Q;
const KEY_NEXT_FOCUS: Key = Key::E;
const KEY_SPEED_UP: Key = Key::Period;
const KEY_SLOW_DOWN: Key = Key::Comma;
const KEY_REWIND: Key = Key::R;
const KEY_TOGGLE_PAUSE: Key = Key::Space;
const KEY_CAMERA_SWAP: Key = Key::C;

pub struct Controller {
    timestep: f64,
    paused: bool,
    // TODO: i think this belongs in the view or similar
    fps_counter: FpsCounter,
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

impl Controller {
    pub fn new() -> Self {
        Controller {
            timestep: 21600.0 / 60.0, // one Kerbin-day
            paused: true,
            fps_counter: FpsCounter::new(1000),
        }
    }

    pub fn process_event(&mut self, event: Event, view: &mut View) {
        match event.value {
            WindowEvent::Key(KEY_NEXT_FOCUS, Action::Press, _) => {
                view.camera_focus_next();
            }
            WindowEvent::Key(KEY_PREV_FOCUS, Action::Press, _) => {
                view.camera_focus_prev();
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
            WindowEvent::Key(KEY_TOGGLE_PAUSE, Action::Press, _) => {
                self.paused = !self.paused;
            }
            WindowEvent::Key(KEY_CAMERA_SWAP, Action::Press, _) => {
                view.camera_inertial_toggle();
            }
            _ => {}
        }
    }

    pub fn is_paused(&self) -> bool {
        self.paused
    }

    pub fn timestep(&self) -> f64 {
        self.timestep
    }

    pub fn fps(&self) -> f64 {
        self.fps_counter.value()
    }

    pub fn increment_frame_counter(&mut self) {
        self.fps_counter.increment()
    }
}
