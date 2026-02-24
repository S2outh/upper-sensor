use core::future::Future;
use core::time::Duration;

use embassy_stm32::gpio::Output;
use embassy_time::{Duration as EmbassyDuration, Instant, with_timeout};
use phoenix::phoenix::{LiftoffControl, ServiceClock};
use phoenix::winmon::TimeoutTimer;

pub struct EmbassyTimer;

impl TimeoutTimer for EmbassyTimer {
    async fn timeout<F: Future>(
        &mut self,
        duration: Duration,
        fut: F,
    ) -> Result<F::Output, ()> {
        with_timeout(to_embassy_duration(duration), fut)
            .await
            .map_err(|_| ())
    }
}

pub struct EmbassyClock;

impl ServiceClock for EmbassyClock {
    fn now_ms(&mut self) -> u64 {
        Instant::now().as_ticks() / 1000
    }
}

pub struct LiftoffPin<'d>(pub Output<'d>);

impl<'d> LiftoffControl for LiftoffPin<'d> {
    fn set_liftoff(&mut self, active: bool) {
        if active {
            self.0.set_high();
        } else {
            self.0.set_low();
        }
    }
}

fn to_embassy_duration(duration: Duration) -> EmbassyDuration {
    EmbassyDuration::from_micros(duration.as_micros() as u64)
}
