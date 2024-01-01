use std::f64::consts::{FRAC_PI_2, PI};
use std::f64::INFINITY;

use nalgebra::{Point3, Vector3};

use super::{Event, EventData, EventPoint, SOIChange};
use crate::astro::orbit::{HasMass, TimedOrbit};
use crate::math::root_finding::{bisection, Bracket};
use crate::math::stumpff::stumpff_G;
use crate::model::events::intervals::Interval;
use crate::model::events::SearchResult;
use crate::model::orrery::{Body, BodyID, Orrery, ShipID};

const NUM_ITERATIONS_SOI_ENCOUNTER: usize = 1000;

// TODO maybe these should be folded into UpcomingEvents? IDK
pub fn search_for_soi_escape(orrery: &Orrery, ship_id: ShipID) -> SearchResult {
    let ship_orbit = orrery.orbit_of_ship(ship_id);

    let current_body = ship_orbit.primary().id;
    let current_body_orbit = match orrery.orbit_of_body(current_body) {
        Some(o) => *o.orbit(),
        // We can never escape the Sun
        None => return SearchResult::Never,
    };
    let soi_radius = current_body_orbit.soi_radius();

    let parent_body = current_body_orbit.primary().id;

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
    let parent_id = ship_orbit.primary().id;

    // Check whether this body and ship are co-orbiting. If not, no encounter.
    let target_orbit = match orrery.orbit_of_body(target_id) {
        Some(o) => o,
        // Can't encounter the Sun, since you can't leave it
        None => return SearchResult::Never,
    };
    if target_orbit.primary().id != parent_id {
        return SearchResult::Never;
    }

    // Everything seems good, let's start looking for intersections!
    let soi_radius = target_orbit.orbit().soi_radius();
    let soi_radius_sq = soi_radius * soi_radius;

    // Quick check: if one orbit is much smaller than the other, then there's no
    // chance of intersection, so we can skip the rest of the search forever.
    let ship_interval = get_apsis_interval(&ship_orbit);
    let target_interval = get_apsis_interval(&target_orbit).widen(soi_radius);
    if ship_interval.intersect(&target_interval).is_none() {
        return SearchResult::Never;
    }

    // Great, preliminary checks pass! Now for the hard part.

    // We want to find a solution of d(t) = soi_radius, where d is the distance
    // between the two bodies. However, dealing with the square root in the
    // distance function is nasty, so instead, we'll instead look for a root of
    // f(t), where f(t) = d^2(t) - soi_radius^2.
    //
    // There are many ways of root-finding, but since we're not sure whether a root
    // exists, and we're interested in the smallest root (not just any root), we
    // have to get fancy. We'll use the Krawczyk-Moore test to test for roots,
    // which involves a lot of interval arithmetic.

    // We maintain a stack of intervals to search, sorted so that the earliest one
    // is on top
    let mut interval_stack = vec![Interval::new(start_time, end_time)];
    let encounter_helper = SoiEncounterHelper {
        ship_orbit,
        target_orbit,
    };

    let encounter_interval = loop {
        let time_interval = match interval_stack.pop() {
            Some(i) => i,
            // If we exhaust the stack, we never found a region where encounter was possible
            None => return SearchResult::NotFound(end_time),
        };

        // Check whether f(t) could hit zero (i.e., d^2(t) = soi_radius_sq)
        let diff_distance_sq = encounter_helper.get_distance_squared_inclusion(time_interval);
        if !diff_distance_sq.contains(soi_radius_sq) {
            // No chance of intersection? Discard this region
            continue;
        }

        // Next, we apply the Krawczyk-Moore operator, using y = m(X) and Y = 1/f'(y)
        let y = time_interval.midpoint();
        let f_of_y = encounter_helper.get_distance_squared(y) - soi_radius_sq;
        let f_prime_of_y = encounter_helper.get_der_distance_squared(y);
        #[allow(non_snake_case)]
        let Y = 1.0 / f_prime_of_y;
        let f_prime_inclusion = encounter_helper.get_der_distance_squared_inclusion(time_interval);
        let possible_contraction = 1.0 - Y * f_prime_inclusion;
        let krawczyk = (y - Y * f_of_y) + possible_contraction * (time_interval - y);

        // Sanity check; our inclusion function should always be good
        debug_assert!(
            f_prime_inclusion.contains(f_prime_of_y),
            "Inclusion function for time interval {} not an inclusion! {} does not contain {}",
            time_interval,
            f_prime_inclusion,
            f_prime_of_y
        );

        // Not part of the normal test, but useful for us: if the derivative is always
        // positive, we can discard this interval -- it corresponds to an SOI exit,
        // which is not a root we're interested in.
        if f_prime_inclusion.lo() >= 0.0 {
            continue;
        }

        // Check the conditions. If this doesn't overlap at all, no roots! Discard this
        // region.
        let intersection = match time_interval.intersect(&krawczyk) {
            Some(i) => i,
            None => continue,
        };

        // If K(X, y, Y) is a subset of X, and r < 1, then we have a unique solution,
        // and Newton's method will converge to it! Break out.
        if krawczyk.is_subset_of(&time_interval) && possible_contraction.norm() < 1.0 {
            // Check that this is an SOI encounter, not escape
            debug_assert!(f_prime_of_y < 0.0);
            break intersection;
        }

        // Otherwise, we don't know what's going on, and we should try a smaller
        // interval. If the intersection is sufficiently small, we'll use that,
        // otherwise we'll try bisection.
        if intersection.width() < time_interval.width() / 2.0 {
            interval_stack.push(intersection);
        } else {
            // Otherwise, bisect and push it on the stack, second one first
            let (first, second) = time_interval.bisect();
            interval_stack.push(second);
            interval_stack.push(first);
        }
    };

    // println!(
    //     "Found encounter window for {:?} and {:?}: {}",
    //     ship_id, target_id, encounter_interval
    // );

    // TODO: we could use newton here, but the encounter at 45d is very sensitive to
    // initial conditions, and changing to newton changes the timing of that
    // encounter by 30s, and that throws off everything else
    let entry_time = bisection(
        |time| encounter_helper.get_distance_squared(time) - soi_radius_sq,
        Bracket::new(encounter_interval.lo(), encounter_interval.hi()),
        NUM_ITERATIONS_SOI_ENCOUNTER,
    );

    // Lastly, figure out anomaly and position at that point
    let new_anomaly = encounter_helper.ship_orbit.s_at_time(entry_time);
    let new_state = encounter_helper
        .ship_orbit
        .orbit()
        .get_state_at_universal_anomaly(new_anomaly);

    let event = Event {
        ship_id,
        data: EventData::EnteringSOI(SOIChange {
            old: parent_id,
            new: target_id,
        }),
        point: EventPoint {
            time: entry_time,
            anomaly: new_anomaly,
            location: Point3::from(new_state.position()),
        },
    };
    SearchResult::Found(event)
}

/// Helper struct for solving an SOI encounter instance
struct SoiEncounterHelper<'orr> {
    ship_orbit: TimedOrbit<&'orr Body, ShipID>,
    target_orbit: TimedOrbit<&'orr Body, &'orr Body>,
}

impl SoiEncounterHelper<'_> {
    fn get_distance_squared(&self, time: f64) -> f64 {
        let ship_position = self.ship_orbit.state_at_time(time).position();
        let target_position = self.target_orbit.state_at_time(time).position();
        (ship_position - target_position).norm_squared()
    }

    fn get_distance_squared_inclusion(&self, time_interval: Interval) -> Interval {
        // Compute the bounding boxes
        let ship_bbox = get_bbox(&self.ship_orbit, time_interval);
        let target_bbox = get_bbox(&self.target_orbit, time_interval);
        let displacement = bbox_sub(ship_bbox, target_bbox);
        bbox_dot(displacement, displacement)
    }

    fn get_der_distance_squared(&self, time: f64) -> f64 {
        // d/dt (x dot x) = 2 x dot dx/dt
        let ship_state = self.ship_orbit.state_at_time(time);
        let target_state = self.target_orbit.state_at_time(time);
        let displacement = ship_state.position() - target_state.position();
        let rel_velocity = ship_state.velocity() - target_state.velocity();
        2.0 * displacement.dot(&rel_velocity)
    }

    fn get_der_distance_squared_inclusion(&self, time_interval: Interval) -> Interval {
        // Since d/dt (x dot x) = 2 x dot dx/dt, we'll compute those two quantities
        // as intervals first, then dot them together manually
        let ship_bbox = get_bbox(&self.ship_orbit, time_interval);
        let target_bbox = get_bbox(&self.target_orbit, time_interval);
        let ship_vel_bbox = get_velocity_bbox(&self.ship_orbit, time_interval);
        let target_vel_bbox = get_velocity_bbox(&self.target_orbit, time_interval);

        let displacement = bbox_sub(ship_bbox, target_bbox);
        let rel_velocity = bbox_sub(ship_vel_bbox, target_vel_bbox);
        2.0 * bbox_dot(displacement, rel_velocity)
    }
}

fn get_apsis_interval<P, S>(timed_orbit: &TimedOrbit<P, S>) -> Interval {
    let lo = timed_orbit.orbit().periapsis();
    let hi = timed_orbit.orbit().apoapsis().unwrap_or(INFINITY);
    Interval::new(lo, hi)
}

fn get_bbox<P: HasMass, S>(
    timed_orbit: &TimedOrbit<P, S>,
    time_interval: Interval,
) -> [Interval; 3] {
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
    rotated_unit_vectors.map(|u| (r_p - mu * g2_interval) * u.x + h * g1_interval * u.y)
}

fn get_velocity_bbox<P: HasMass, S>(
    timed_orbit: &TimedOrbit<P, S>,
    time_interval: Interval,
) -> [Interval; 3] {
    let s_interval = time_interval.monotone_map(|t| timed_orbit.s_at_time(t));

    // Get some constants
    let beta = timed_orbit.orbit().beta();
    let r_p = timed_orbit.orbit().periapsis();
    let mu = timed_orbit.orbit().primary().mu();
    let h = timed_orbit.orbit().angular_momentum();
    let ecc = timed_orbit.orbit().eccentricity();

    // We want to compute some bounds on v cdot u. We start by getting bounds on the
    // Stumpff functions
    let g0_interval: Interval = g0_inclusion(beta, s_interval);
    let g1_interval = g1_inclusion(beta, s_interval);
    let g2_interval = g2_inclusion(beta, s_interval);

    let r_interval = r_p + mu * ecc * g2_interval;
    assert!(
        !r_interval.contains(0.0),
        "radius too close to zero: {}",
        r_interval
    );
    let r_inv_interval = r_interval.monotone_map(|r| r.recip());

    // Now we're ready for the rest
    let unit_vectors = [Vector3::x(), Vector3::y(), Vector3::z()];
    let rotated_unit_vectors =
        unit_vectors.map(|u| timed_orbit.orbit().rotation().inverse_transform_vector(&u));
    rotated_unit_vectors.map(|u| {
        // Factor out 1/r so that it's only used once (reduces interval width)
        let tmp = -mu * g1_interval * u.x + h * g0_interval * u.y;
        tmp * r_inv_interval
    })
}

fn g0_inclusion(beta: f64, s_interval: Interval) -> Interval {
    let mut output = s_interval.monotone_map(|s| stumpff_G(beta, s)[0]);

    if beta > 0.0 {
        // We want to see whether s sqrt(beta) contains an extremum of cosine, i.e., an
        // integer multiple of pi.
        // We'll rescale the interval so that we're looking for integers
        let test_interval = s_interval * beta.sqrt() / PI;

        if test_interval.contains_integer_with_mod_constraint(2, 0) {
            // We cross a maximum
            output.include(1.0)
        }
        if test_interval.contains_integer_with_mod_constraint(2, 1) {
            // We cross a minimum
            output.include(-1.0)
        }
    }

    output
}

fn g1_inclusion(beta: f64, s_interval: Interval) -> Interval {
    let mut output = s_interval.monotone_map(|s| stumpff_G(beta, s)[1]);

    if beta > 0.0 {
        // We want to see whether s sqrt(beta) contains a root of sine, i.e., a point of
        // the form (2n +/- 1/2) pi.
        // We'll rescale the interval so that we're looking for integers of the form
        // 4n +/- 1
        let test_interval = s_interval * beta.sqrt() / FRAC_PI_2;

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
        // We want to see whether s sqrt(beta) contains an extremum of cosine, i.e., an
        // integer multiple of pi.
        // We'll rescale the interval so that we're looking for integers
        let test_interval = s_interval * beta.sqrt() / PI;

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

fn bbox_sub(a: [Interval; 3], b: [Interval; 3]) -> [Interval; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn bbox_dot(a: [Interval; 3], b: [Interval; 3]) -> Interval {
    a[0] * b[0] + a[1] * b[1] + a[2] + b[2]
}
