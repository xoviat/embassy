use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Global event channel for passing events from C callback to Rust async code
/// Size of 8 events should be sufficient for most cases
pub(crate) static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, stm32wb_hci::Event, 8> = Channel::new();

/// Receive the next HCI event (async)
pub async fn read_event() -> stm32wb_hci::Event {
    EVENT_CHANNEL.receive().await
}

/// Try to send an event to the channel (non-blocking, for use in C callbacks)
pub(crate) fn try_send_event(event: stm32wb_hci::Event) -> Result<(), ()> {
    EVENT_CHANNEL.try_send(event).map_err(|_| ())
}
