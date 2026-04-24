use defmt::error;
use embassy_futures::select::{Either, select};
use embassy_stm32::can::{BufferedFdCanReceiver, BufferedFdCanSender, frame::FdFrame};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};

use embassy_time::Instant;
use south_common::{
    chell::{ChellValue, fd_compat_chell_union, match_value},
    definitions::internal_msgs,
    types::{Telecommand, Timesync},
};

use crate::{TCSender, TMReceiver};

// Timesync stuff
static TIMESYNC_REQUEST: Signal<ThreadModeRawMutex, (u8, Instant)> = Signal::new();
const TIMESYNC_PRIORITY: u8 = 0;
type TimesyncContainer = fd_compat_chell_union!(internal_msgs::TimesyncAnswer);

// tm sending task
#[embassy_executor::task]
pub async fn can_sender_thread(mut can_sender: BufferedFdCanSender, tm_channel: TMReceiver) {
    loop {
        match select(tm_channel.receive(), TIMESYNC_REQUEST.wait()).await {
            // Sending telemetry
            Either::First(container) => {
                let frame = FdFrame::new_standard(container.id(), container.fd_bytes()).unwrap();
                can_sender.write(frame).await;
            }
            // Sending Timesync answer
            Either::Second((request_id, local_instant_recv)) => {
                let diff = super::TIME_REF.load(core::sync::atomic::Ordering::Acquire);
                if diff == 0 {
                    continue;
                }
                let priority = TIMESYNC_PRIORITY;
                let unix_time_recv = diff + local_instant_recv.as_micros();
                let unix_time_snd = diff + Instant::now().as_micros();
                let msg = Timesync {
                    request_id,
                    priority,
                    unix_time_recv,
                    unix_time_snd,
                };
                let container =
                    TimesyncContainer::new(&internal_msgs::TimesyncAnswer, &msg).unwrap();
                let frame = FdFrame::new_standard(container.id(), container.fd_bytes()).unwrap();

                can_sender.write(frame).await;
            }
        }
    }
}

// tc receiving task
#[embassy_executor::task]
pub async fn can_receiver_thread(can_receiver: BufferedFdCanReceiver, tc_channel: TCSender) {
    loop {
        match can_receiver.receive().await {
            Ok(envelope) => {
                if let embedded_can::Id::Standard(id) = envelope.frame.id() {
                    match_value!(internal_msgs::from_id(id.as_raw()).unwrap(), {
                        internal_msgs::Telecommand => {
                            match Telecommand::read(envelope.frame.data()) {
                                Ok(cmd) => tc_channel.send(cmd.1).await,
                                Err(_) => error!("error parsing tc"),
                            }
                        },
                        internal_msgs::TimesyncRequest => {
                            if let Some(request_id) = envelope.frame.data().get(0) {
                                TIMESYNC_REQUEST.signal((*request_id, envelope.ts));
                            }
                        },
                    });
                } else {
                    defmt::unreachable!()
                };
            }
            Err(e) => error!("error in frame! {}", e),
        }
    }
}
