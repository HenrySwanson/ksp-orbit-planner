use kiss3d::camera::Camera;
use kiss3d::event::EventManager;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::renderer::Renderer;
use kiss3d::window::{State, Window};

use self::controller::Controller;
use self::view::View;
use crate::model::timeline::Timeline;

mod camera;
mod controller;
mod renderers;
mod view;

pub struct Simulation {
    view: View,
    controller: Controller,
}

impl Simulation {
    pub fn new(timeline: Timeline, window: &mut Window) -> Self {
        Self {
            view: View::new(timeline, window),
            controller: Controller::new(),
        }
    }

    fn process_user_input(&mut self, mut events: EventManager) {
        // Process events
        for event in events.iter() {
            self.controller.process_event(event, &mut self.view);
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
        self.view.cameras_and_effect_and_renderer()
    }

    fn step(&mut self, window: &mut Window) {
        self.process_user_input(window.events());
        if !self.controller.paused {
            self.view.update_state_by(self.controller.timestep);
        }
        self.view.prerender_scene(window, &self.controller);
        self.controller.fps_counter.increment();
    }
}
