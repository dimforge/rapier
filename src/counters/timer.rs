use core::{
    fmt::{Display, Error, Formatter},
    time::Duration,
};

#[cfg(feature = "profiler")]
use web_time::Instant;

#[cfg(feature = "profiler")]
#[derive(Copy, Clone, Debug)]
enum TimerState {
    Disabled,
    Stopped,
    Running(Instant),
}

/// A timer.
#[derive(Copy, Clone, Debug)]
pub struct Timer {
    time: Duration,
    #[cfg(feature = "profiler")]
    state: TimerState,
}

impl Timer {
    /// Creates a new timer initialized to zero and not started.
    pub fn new() -> Self {
        Timer {
            time: Duration::from_secs(0),
            #[cfg(feature = "profiler")]
            state: TimerState::Stopped,
        }
    }

    pub(crate) fn set_enabled(&mut self, enabled: bool) {
        #[cfg(feature = "profiler")]
        {
            if !enabled {
                // Discard an open interval without reading the clock. Re-enabling must
                // never charge time spent with profiling disabled to the next pause.
                self.state = TimerState::Disabled;
            } else if matches!(self.state, TimerState::Disabled) {
                self.state = TimerState::Stopped;
            }
        }
        #[cfg(not(feature = "profiler"))]
        let _ = enabled;
    }

    #[cfg(feature = "profiler")]
    pub(crate) fn is_enabled(&self) -> bool {
        !matches!(self.state, TimerState::Disabled)
    }

    /// Resets the timer to 0.
    pub fn reset(&mut self) {
        self.time = Duration::from_secs(0)
    }

    /// Start the timer.
    pub fn start(&mut self) {
        #[cfg(feature = "profiler")]
        {
            if !self.is_enabled() {
                return;
            }
            self.time = Duration::from_secs(0);
            self.state = TimerState::Running(Instant::now());
        }
    }

    /// Pause the timer.
    pub fn pause(&mut self) {
        #[cfg(feature = "profiler")]
        {
            if let TimerState::Running(start) = self.state {
                self.time += Instant::now().duration_since(start);
                self.state = TimerState::Stopped;
            }
        }
    }

    /// Resume the timer.
    pub fn resume(&mut self) {
        #[cfg(feature = "profiler")]
        {
            if self.is_enabled() {
                self.state = TimerState::Running(Instant::now());
            }
        }
    }

    /// The measured time between the last `.start()` and `.pause()` calls.
    pub fn time(&self) -> Duration {
        self.time
    }

    /// The measured time in milliseconds between the last `.start()` and `.pause()` calls.
    pub fn time_ms(&self) -> f64 {
        self.time.as_secs_f64() * 1000.0
    }
}

impl Default for Timer {
    fn default() -> Self {
        Self::new()
    }
}

impl Display for Timer {
    fn fmt(&self, f: &mut Formatter) -> Result<(), Error> {
        write!(f, "{}ms", self.time_ms())
    }
}
