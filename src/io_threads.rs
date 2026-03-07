
use embassy_stm32::{
    can::{
        BufferedFdCanReceiver, BufferedFdCanSender,
        frame::FdFrame,
    },
};
use embassy_sync::{channel::{DynamicReceiver, DynamicSender}};
use defmt::error;

use south_common::{types::Telecommand, tmtc_system::TMValue};

use crate::UpperSensorTMContainer;

// tm sending task
#[embassy_executor::task]
pub async fn tm_thread(
    mut can_sender: BufferedFdCanSender,
    tm_channel: DynamicReceiver<'static, UpperSensorTMContainer>,
) {
    loop {
        let container = tm_channel.receive().await;
        match FdFrame::new_standard(container.id(), container.bytes()) {
            Ok(frame) => {
                can_sender.write(frame).await;
            }
            Err(e) => error!("error constructing can message: {}", e),
        }
    }
}

// tc receiving task
#[embassy_executor::task]
pub async fn tc_thread(
    can_receiver: BufferedFdCanReceiver,
    tc_channel: DynamicSender<'static, Telecommand>,
) {
    loop {
        match can_receiver.receive().await {
            Ok(envelope) => match Telecommand::read(envelope.frame.data()) {
                Ok(cmd) => tc_channel.send(cmd.1).await,
                Err(_) => error!("error parsing tc"),
            },
            Err(e) => error!("error in frame! {}", e),
        }
    }
}
