use kiss3d::light::Light;
use kiss3d::window::Window;
use nalgebra::Vector3;
use rust_ksp::file::read_file;
use rust_ksp::gui::Simulation;

use rust_ksp::model::Timeline;
use rust_ksp::orrery::BodyID;

/// This particular scenario is one I've been using for a really long time.
/// It goes like this:
/// - 0d: Ship proceedes in moderately elliptical trajectory (e = 0.7, SMA = 20M)
/// - 13d: Encounters Mun and switches to smaller orbit (e = 0.73, SMA = 8.7M)
/// - 14d: Immediate re-encounter, enlarges orbit again (e = 0.7, SMA = 21M)
/// - 22d: Tightens orbit to small and narrow (e = 0.83, SMA = 7.3M)
/// - 31d: Re-enlarges orbit (e = 0.69, SMA = 17M)
/// - 45d: Just grazes Mun, slight modification of orbit (e = 0.66, SMA = 14M)
/// - 49d: Bounces off the Mun (e = 0.74, SMA = 9.5M)
/// - 55d: Clips through the Mun and drops almost into Kerbin (e = 0.92, SMA = 6.7M)
/// - 58d: Bounces off the Mun, and enters a hyperbolic orbit (e = 1.95, SMA = -12M)
/// - 60d: Escapes Kerbin's orbit, and starts orbiting the Sun (e = 0.097, SMA = 15B)

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
