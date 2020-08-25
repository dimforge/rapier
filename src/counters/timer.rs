use std::fmt::{Display, Error, Formatter};

/// A timer.
#[derive(Copy, Clone, Debug, Default)]
pub struct Timer {
    time: f64,
    start: Option<f64>,
}

impl Timer {
    /// Creates a new timer initialized to zero and not started.
    pub fn new() -> Self {
        Timer {
            time: 0.0,
            start: None,
        }
    }

    /// Resets the timer to 0.
    pub fn reset(&mut self) {
        self.time = 0.0
    }

    /// Start the timer.
    pub fn start(&mut self) {
        self.time = 0.0;
        self.start = Some(instant::now());
    }

    /// Pause the timer.
    pub fn pause(&mut self) {
        if let Some(start) = self.start {
            self.time += instant::now() - start;
        }
        self.start = None;
    }

    /// Resume the timer.
    pub fn resume(&mut self) {
        self.start = Some(instant::now());
    }

    /// The measured time between the last `.start()` and `.pause()` calls.
    pub fn time(&self) -> f64 {
        self.time
    }
}

impl Display for Timer {
    fn fmt(&self, f: &mut Formatter) -> Result<(), Error> {
        write!(f, "{}s", self.time)
    }
}
