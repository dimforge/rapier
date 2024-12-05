use std::{
    fmt::{Display, Error, Formatter},
    time::Duration,
};

/// A timer.
#[derive(Copy, Clone, Debug, Default)]
pub struct Timer {
    time: Duration,
    #[allow(dead_code)] // The field isn’t used if the `profiler` feature isn’t enabled.
    start: Option<std::time::Instant>,
}

impl Timer {
    /// Creates a new timer initialized to zero and not started.
    pub fn new() -> Self {
        Timer {
            time: Duration::from_secs(0),
            start: None,
        }
    }

    /// Resets the timer to 0.
    pub fn reset(&mut self) {
        self.time = Duration::from_secs(0)
    }

    /// Start the timer.
    pub fn start(&mut self) {
        #[cfg(feature = "profiler")]
        {
            self.time = Duration::from_secs(0);
            self.start = Some(web_time::Instant::now());
        }
    }

    /// Pause the timer.
    pub fn pause(&mut self) {
        #[cfg(feature = "profiler")]
        {
            if let Some(start) = self.start {
                self.time += web_time::Instant::now().duration_since(start);
            }
            self.start = None;
        }
    }

    /// Resume the timer.
    pub fn resume(&mut self) {
        #[cfg(feature = "profiler")]
        {
            self.start = Some(web_time::Instant::now());
        }
    }

    /// The measured time between the last `.start()` and `.pause()` calls.
    pub fn time(&self) -> f64 {
        self.time.as_secs_f64() * 1000.0
    }
}

impl Display for Timer {
    fn fmt(&self, f: &mut Formatter) -> Result<(), Error> {
        write!(f, "{}s", self.time.as_secs_f32())
    }
}
