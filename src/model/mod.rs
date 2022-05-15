use crate::orrery::Orrery;
use crate::events::Event;

pub struct Timeline {
    // Invariants:
    //   - The `start_time` of each segment are sorted in ascending order
    segments: Vec<Segment>,
}

struct Segment {
    start_time: f64,
    start_state: Orrery,
}

impl Timeline {
    pub fn new(orrery: Orrery) -> Self {
        Self {
            segments: vec![
                Segment {
                    start_time: 0.0,
                    start_state: orrery,
                }
            ]
        }
    }

    pub fn get_orrery_at(&self, time: f64) -> Option<Orrery> {
        // Get the first segment to strictly precede this time.
        let next_segment_idx = self.segments.partition_point(|s| s.start_time <= time);
        // If that's the first segment, then this time is before the model starts.
        if next_segment_idx == 0 {
            return None;
        }

        let segment_idx = next_segment_idx - 1;
        let segment = &self.segments[segment_idx];
        assert!(time - segment.start_time >= 0.0);

        let mut orrery = segment.start_state.clone();
        orrery.update_time(time - segment.start_time);

        Some(orrery)
    }
}
