use crate::counters::Timer;
use std::fmt::{Display, Formatter, Result};

/// Performance counters related to collision detection.
#[derive(Default, Clone, Copy)]
pub struct CollisionDetectionCounters {
    /// Number of contact pairs detected.
    pub ncontact_pairs: usize,
    /// Time spent for the broad-phase of the collision detection.
    pub broad_phase_time: Timer,
    /// Time spent for the narrow-phase of the collision detection.
    pub narrow_phase_time: Timer,
}

impl CollisionDetectionCounters {
    /// Creates a new counter initialized to zero.
    pub fn new() -> Self {
        CollisionDetectionCounters {
            ncontact_pairs: 0,
            broad_phase_time: Timer::new(),
            narrow_phase_time: Timer::new(),
        }
    }

    /// Resets all the coounters and timers.
    pub fn reset(&mut self) {
        self.ncontact_pairs = 0;
        self.broad_phase_time.reset();
        self.narrow_phase_time.reset();
    }
}

impl Display for CollisionDetectionCounters {
    fn fmt(&self, f: &mut Formatter) -> Result {
        writeln!(f, "Number of contact pairs: {}", self.ncontact_pairs)?;
        writeln!(f, "Broad-phase time: {}", self.broad_phase_time)?;
        writeln!(f, "Narrow-phase time: {}", self.narrow_phase_time)
    }
}
