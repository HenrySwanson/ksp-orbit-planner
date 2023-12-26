use approx::{assert_abs_diff_eq, assert_relative_eq};
use itertools::{EitherOrBoth, Itertools};
use nalgebra::Vector3;
use rust_ksp::file::read_file;
use rust_ksp::model::events::{EventData, SOIChange};
use rust_ksp::model::orrery::BodyID;
use rust_ksp::model::timeline::Timeline;

const KERBIN: BodyID = BodyID(4);
const MUN: BodyID = BodyID(5);
const KERBOL: BodyID = BodyID(0);

const ENCOUNTER_MUN: EventData = {
    EventData::EnteringSOI(SOIChange {
        old: KERBIN,
        new: MUN,
    })
};
const ESCAPE_MUN: EventData = {
    EventData::ExitingSOI(SOIChange {
        old: MUN,
        new: KERBIN,
    })
};
const ESCAPE_KERBIN: EventData = {
    EventData::ExitingSOI(SOIChange {
        old: KERBIN,
        new: KERBOL,
    })
};

/// This particular scenario is one I've been using for a really long time.
/// It goes like this:
/// - 0d: Ship proceedes in moderately elliptical trajectory (e = 0.7, SMA =
///   20M)
/// - 13d: Encounters Mun and switches to smaller orbit (e = 0.73, SMA = 8.7M)
/// - 14d: Immediate re-encounter, enlarges orbit again (e = 0.7, SMA = 21M)
/// - 22d: Tightens orbit to small and narrow (e = 0.83, SMA = 7.3M)
/// - 31d: Re-enlarges orbit (e = 0.69, SMA = 17M)
/// - 45d: Just grazes Mun, slight modification of orbit (e = 0.66, SMA = 14M)
/// - 49d: Bounces off the Mun (e = 0.74, SMA = 9.5M)
/// - 55d: Clips through the Mun and drops almost into Kerbin (e = 0.92, SMA =
///   6.7M)
/// - 58d: Bounces off the Mun, and enters a hyperbolic orbit (e = 1.95, SMA =
///   -12M)
/// - 60d: Escapes Kerbin's orbit, and starts orbiting the Sun (e = 0.097, SMA =
///   15B)
#[test]
fn test_favorite_scenario() {
    let expected_events = vec![
        // 13d
        (1167224.3810535548, ENCOUNTER_MUN),
        (1176541.0255763677, ESCAPE_MUN),
        // 14d
        (1288753.3454258977, ENCOUNTER_MUN),
        (1298160.1769034935, ESCAPE_MUN),
        // 22d
        (1903256.1219919417, ENCOUNTER_MUN),
        (1913118.229924371, ESCAPE_MUN),
        // 31d
        (2727926.8998953775, ENCOUNTER_MUN),
        (2737612.195569021, ESCAPE_MUN),
        // 45d
        (3891256.0700824754, ENCOUNTER_MUN),
        (3896079.039262741, ESCAPE_MUN),
        // 49d
        (4263267.2755853385, ENCOUNTER_MUN),
        (4272708.9974184595, ESCAPE_MUN),
        // 55d
        (4792775.67788562, ENCOUNTER_MUN),
        (4803927.607622115, ESCAPE_MUN),
        // 59d
        (5066549.0475938115, ENCOUNTER_MUN),
        (5075614.547186604, ESCAPE_MUN),
        // 60d
        (5199986.65163866, ESCAPE_KERBIN),
    ];
    let num_days_to_simulate = 65;

    let mut orrery = read_file("ksp-bodies.txt");
    orrery.add_ship(Vector3::x() * 6000000.0, Vector3::y() * 1000.0, 0.0, KERBIN);

    let mut timeline = Timeline::new(orrery, 0.0);

    // TODO: fix the extend time to work with just one extension,
    // this is atrocious :)
    for day in 0..num_days_to_simulate {
        timeline.extend_until(day as f64 * 86400.0)
    }

    for tup in expected_events.into_iter().zip_longest(timeline.events()) {
        let ((expected_time, expected_data), actual) = match tup {
            EitherOrBoth::Both(expected, actual) => (expected, actual),
            EitherOrBoth::Left(expected) => {
                panic!("Expected event {:?}, but none was found", expected)
            }
            EitherOrBoth::Right(actual) => {
                panic!("Did not expect event, but found one anyways: {:?}", actual)
            }
        };

        assert_eq!(expected_data, actual.data);
        assert_abs_diff_eq!(expected_time, actual.point.time, epsilon = 0.01);
    }
}
