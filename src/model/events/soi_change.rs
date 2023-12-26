use std::f64::consts::{FRAC_PI_2, PI};
use std::f64::INFINITY;

use nalgebra::{Point3, Vector3};

use super::intervals::BoundingBox;
use super::{Event, EventData, EventPoint, SOIChange};
use crate::astro::orbit::{HasMass, TimedOrbit};
use crate::math::root_finding::{bisection, Bracket};
use crate::math::stumpff::stumpff_G;
use crate::model::events::intervals::Interval;
use crate::model::events::SearchResult;
use crate::model::orrery::{BodyID, Orrery, ShipID};

const NUM_ITERATIONS_SOI_ENCOUNTER: usize = 1000;

// TODO maybe these should be folded into UpcomingEvents? IDK
pub fn search_for_soi_escape(orrery: &Orrery, ship_id: ShipID) -> SearchResult {
    let ship_orbit = orrery.orbit_of_ship(ship_id);

    let current_body = ship_orbit.orbit().primary().id;
    let current_body_orbit = match orrery.orbit_of_body(current_body) {
        Some(o) => o,
        // We can never escape the Sun
        None => return SearchResult::Never,
    };
    let soi_radius = current_body_orbit.orbit().soi_radius();

    let parent_body = current_body_orbit.orbit().primary().id;

    let escape_s = match ship_orbit.orbit().get_s_at_radius(soi_radius) {
        Some(s) => s,
        None => return SearchResult::Never,
    };
    let escape_time = ship_orbit.time_at_s(escape_s);
    let new_state = ship_orbit.orbit().get_state_at_universal_anomaly(escape_s);

    let event = Event {
        ship_id,
        data: EventData::ExitingSOI(SOIChange {
            old: current_body,
            new: parent_body,
        }),
        point: EventPoint {
            time: escape_time,
            anomaly: escape_s,
            location: Point3::from(new_state.position()),
        },
    };

    SearchResult::Found(event)
}

pub fn search_for_soi_encounter(
    orrery: &Orrery,
    ship_id: ShipID,
    target_id: BodyID,
    start_time: f64,
    end_time: f64,
) -> SearchResult {
    // We perform a lot of preflight checks. First, is the window the right way
    // around?
    assert!(
        start_time <= end_time,
        "Reversed window: {} > {}",
        start_time,
        end_time
    );

    let ship_orbit = orrery.orbit_of_ship(ship_id);
    let parent_id = ship_orbit.orbit().primary().id;

    // Check whether this body and ship are co-orbiting. If not, no encounter.
    let target_orbit = match orrery.orbit_of_body(target_id) {
        Some(o) => o,
        // Can't encounter the Sun, since you can't leave it
        None => return SearchResult::Never,
    };
    if target_orbit.orbit().primary().id != parent_id {
        return SearchResult::Never;
    }

    // Everything seems good, let's start looking for intersections
    let soi_radius = target_orbit.orbit().soi_radius();

    // Quick check: if one orbit is much smaller than the other, then there's no
    // chance of intersection, so we can skip the rest of the search.
    let ship_interval = get_apsis_interval(&ship_orbit);
    let target_interval = get_apsis_interval(&target_orbit);
    if ship_interval.separated_by(&target_interval, soi_radius) {
        return SearchResult::Never;
    }

    // Great, preliminary checks pass! Now for the hard part.

    // We do a whole lot of interval analysis here. We start with the original time
    // interval, and compute the bounding box of the ship and of the target. If
    // those boxes are sufficiently far apart, then we can discard the interval,
    // since we know no encounter can occur. However, if the boxes are too
    // close, we divide the interval in two, and try again.

    // We push the intervals onto the stack so that the earliest one is on top.
    let mut interval_stack = vec![Interval::new(start_time, end_time)];
    let stopping_threshold = 10.0; // stop when delta t is this small

    // Method for checking distance between ship and target
    let check_distance = |t| {
        let new_ship_pos = ship_orbit.state_at_time(t).position();
        let new_target_pos = target_orbit.state_at_time(t).position();
        (new_ship_pos - new_target_pos).norm()
    };

    let encounter_interval = loop {
        let time_interval = match interval_stack.pop() {
            Some(i) => i,
            // If we exhaust the stack, we never found a region where encounter was possible
            None => return SearchResult::NotFound(end_time),
        };

        // Check the bounding boxes
        let ship_bbox = get_bbox(&ship_orbit, time_interval.lo(), time_interval.hi());
        let target_bbox = get_bbox(&target_orbit, time_interval.lo(), time_interval.hi());
        if ship_bbox.separated_by(&target_bbox, soi_radius) {
            // No chance of intersection? Discard this region
            continue;
        }

        // Is this region small enough that we should bail out?
        // TODO: use Krawczyk–Moore Existence instead!
        if time_interval.width() < stopping_threshold {
            // The bounding box is an over-estimate, let's check that we're really within
            // the target's SOI.
            if check_distance(time_interval.hi()) > soi_radius {
                continue;
            } else {
                break time_interval;
            }
        }

        // Otherwise, subdivide and push it on the stack, second one first
        let (first, second) = time_interval.bisect();
        interval_stack.push(second);
        interval_stack.push(first);
    };

    println!(
        "Suspected encounter window for {:?} and {:?}: {}",
        ship_id, target_id, encounter_interval
    );

    let entry_time = bisection(
        |t| check_distance(t) - soi_radius,
        Bracket::new(encounter_interval.lo(), encounter_interval.hi()),
        NUM_ITERATIONS_SOI_ENCOUNTER,
    );

    // Lastly, figure out anomaly and position at that point
    let new_state = ship_orbit.state_at_time(entry_time);

    let event = Event {
        ship_id,
        data: EventData::EnteringSOI(SOIChange {
            old: parent_id,
            new: target_id,
        }),
        point: EventPoint {
            time: entry_time,
            anomaly: new_state.get_universal_anomaly(),
            location: Point3::from(new_state.position()),
        },
    };
    SearchResult::Found(event)
}

fn get_apsis_interval<P, S>(timed_orbit: &TimedOrbit<P, S>) -> Interval {
    let lo = timed_orbit.orbit().periapsis();
    let hi = timed_orbit.orbit().apoapsis().unwrap_or(INFINITY);
    Interval::new(lo, hi)
}

fn get_bbox<P: HasMass, S>(
    timed_orbit: &TimedOrbit<P, S>,
    start_time: f64,
    end_time: f64,
) -> BoundingBox {
    let time_interval = Interval::new(start_time, end_time);
    let s_interval = time_interval.monotone_map(|t| timed_orbit.s_at_time(t));

    // Get some constants
    let beta = timed_orbit.orbit().beta();
    let r_p = timed_orbit.orbit().periapsis();
    let mu = timed_orbit.orbit().primary().mu();
    let h = timed_orbit.orbit().angular_momentum();

    // We want to compute some bounds on x cdot u. We start by getting bounds on the
    // Stumpff functions
    let g1_interval = g1_inclusion(beta, s_interval);
    let g2_interval = g2_inclusion(beta, s_interval);

    // Now we're ready for the rest
    let unit_vectors = [Vector3::x(), Vector3::y(), Vector3::z()];
    let rotated_unit_vectors =
        unit_vectors.map(|u| timed_orbit.orbit().rotation().inverse_transform_vector(&u));
    let intervals = rotated_unit_vectors.map(|u| {
        g2_interval.monotone_map(|g2| (r_p - mu * g2) * u.x)
            + g1_interval.monotone_map(|g1| h * g1 * u.y)
    });

    BoundingBox(intervals)
}

fn g1_inclusion(beta: f64, s_interval: Interval) -> Interval {
    let mut output = s_interval.monotone_map(|s| stumpff_G(beta, s)[1]);

    if beta > 0.0 {
        // We want to see whether s sqrt(beta) contains a point of the form (2n +/- 1/2)
        // pi. We'll rescale the interval so that we're looking for integers of
        // the form 4n +/- 1
        let test_interval = s_interval.monotone_map(|s| s * beta.sqrt() / FRAC_PI_2);

        if test_interval.contains_integer_with_mod_constraint(4, 3) {
            // We cross a minimum
            output.include(-1.0 / beta.sqrt())
        }
        if test_interval.contains_integer_with_mod_constraint(4, 1) {
            // We cross a maximum
            output.include(1.0 / beta.sqrt())
        }
    }

    output
}

fn g2_inclusion(beta: f64, s_interval: Interval) -> Interval {
    let mut output = s_interval.monotone_map(|s| stumpff_G(beta, s)[2]);

    if beta > 0.0 {
        // We want to see whether s sqrt(beta) contains an integer multiple of pi
        // We'll rescale the interval so that we're looking for integers
        let test_interval = s_interval.monotone_map(|s| s * beta.sqrt() / PI);

        if test_interval.contains_integer_with_mod_constraint(2, 0) {
            // We cross a minimum
            output.include(0.0)
        }
        if test_interval.contains_integer_with_mod_constraint(2, 1) {
            // We cross a maximum
            output.include(2.0 / beta)
        }
    }

    output
}
