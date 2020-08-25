use crate::counters::Timer;
use std::fmt::{Display, Formatter, Result};

/// Performance counters related to constraints resolution.
#[derive(Default, Clone, Copy)]
pub struct SolverCounters {
    /// Number of constraints generated.
    pub nconstraints: usize,
    /// Number of contacts found.
    pub ncontacts: usize,
    /// Time spent for the resolution of the constraints (force computation).
    pub velocity_resolution_time: Timer,
    /// Time spent for the assembly of all the velocity constraints.
    pub velocity_assembly_time: Timer,
    /// Time spent for the update of the velocity of the bodies.
    pub velocity_update_time: Timer,
    /// Time spent for the assembly of all the position constraints.
    pub position_assembly_time: Timer,
    /// Time spent for the update of the position of the bodies.
    pub position_resolution_time: Timer,
}

impl SolverCounters {
    /// Creates a new counter initialized to zero.
    pub fn new() -> Self {
        SolverCounters {
            nconstraints: 0,
            ncontacts: 0,
            velocity_assembly_time: Timer::new(),
            velocity_resolution_time: Timer::new(),
            velocity_update_time: Timer::new(),
            position_assembly_time: Timer::new(),
            position_resolution_time: Timer::new(),
        }
    }

    /// Reset all the counters to zero.
    pub fn reset(&mut self) {
        self.nconstraints = 0;
        self.ncontacts = 0;
        self.velocity_resolution_time.reset();
        self.velocity_assembly_time.reset();
        self.velocity_update_time.reset();
        self.position_assembly_time.reset();
        self.position_resolution_time.reset();
    }
}

impl Display for SolverCounters {
    fn fmt(&self, f: &mut Formatter) -> Result {
        writeln!(f, "Number of contacts: {}", self.ncontacts)?;
        writeln!(f, "Number of constraints: {}", self.nconstraints)?;
        writeln!(f, "Velocity assembly time: {}", self.velocity_assembly_time)?;
        writeln!(
            f,
            "Velocity resolution time: {}",
            self.velocity_resolution_time
        )?;
        writeln!(f, "Velocity update time: {}", self.velocity_update_time)?;
        writeln!(f, "Position assembly time: {}", self.position_assembly_time)?;
        writeln!(
            f,
            "Position resolution time: {}",
            self.position_resolution_time
        )
    }
}
