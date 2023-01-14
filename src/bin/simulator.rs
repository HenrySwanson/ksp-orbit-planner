use kiss3d::light::Light;
use kiss3d::window::Window;
use nalgebra::Vector3;
use rust_ksp::file::read_file;
use rust_ksp::gui::Simulation;
use rust_ksp::model::orrery::BodyID;
use rust_ksp::model::timeline::Timeline;

fn main() {
    let mut window = Window::new("KSP Orbit Simulator");
    window.set_light(Light::StickToCamera);
    window.set_framerate_limit(Some(60));

    let mut orrery = read_file("ksp-bodies.txt");
    orrery.add_ship(
        Vector3::x() * 6000000.0,
        Vector3::y() * 1000.0,
        0.0,
        BodyID(4),
    );

    let simulation = Simulation::new(Timeline::new(orrery), &mut window);
    window.render_loop(simulation);
}
