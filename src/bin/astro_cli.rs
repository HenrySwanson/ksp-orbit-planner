use clap::Parser;
use rust_ksp::file::read_file;

#[derive(Debug, Parser)]
struct Args {
    name: String,
}

fn main() {
    let args = Args::parse();

    let orrery = read_file("ksp-bodies.txt");
    for orbit in orrery.body_orbits() {
        let body = orbit.secondary();
        if body.info.name.to_lowercase() != args.name.to_lowercase() {
            continue;
        }

        // Order is the same as on the KSP wiki for ease of sanity-checking
        println!("Orbital characteristics for {}", body.info.name);
        println!("- Semi-major axis: {}", orbit.semimajor_axis());
        println!("- Apoapsis: {:?}", orbit.apoapsis());
        println!("- Periapsis: {}", orbit.periapsis());
        println!("- Orbital eccentricity: {}", orbit.eccentricity());
        println!(
            "- Orbital inclination: {}",
            orbit.inclination().to_degrees()
        );

        println!(
            "- Argument of periapsis: {}",
            orbit.arg_periapse().to_degrees()
        );
        println!("- LAN: {}", orbit.long_asc_node().to_degrees());
        println!("- Sidereal orbital period: {:?}", orbit.period());
        println!(
            "- Minimum orbital velocity: {:?}",
            orbit.apoapsis_velocity()
        );
        println!("- Maximum orbital velocity: {}", orbit.periapsis_velocity());
        println!("- SOI Radius: {:?}", orbit.soi_radius());
        println!();
    }
}
