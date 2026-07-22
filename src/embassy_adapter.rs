use embassy_time::Instant;
use phoenix::phoenix::ServiceClock;

pub struct EmbassyClock;

impl ServiceClock for EmbassyClock {
    fn now_ms(&mut self) -> u64 {
        Instant::now().as_millis()
    }
}
