use core::ops::Range;
use smoltcp::phy::{DeviceCapabilities, Medium};
use smoltcp::time::Instant as SmolInstant;
use smoltcp::Result;

use super::{Packet, PacketBox};

pub trait Phy {
    fn is_link_up(&mut self) -> bool;
    fn capabilities(&self) -> DeviceCapabilities;

    fn can_transmit(&mut self) -> bool;
    fn transmit(&mut self, pkt: PacketBox, r: Range<usize>);
    fn can_receive(&mut self) -> bool;
    fn receive<'a>(&mut self) -> (PacketBox, Range<usize>);
}

pub struct Device {
    pub phy: &'static mut dyn Phy,
    caps: DeviceCapabilities,
}

impl Device {
    pub fn new(phy: &'static mut dyn Phy) -> Self {
        Self {
            caps: phy.capabilities(),
            phy,
        }
    }
}
impl<'a> smoltcp::phy::Device<'a> for Device {
    type RxToken = RxToken;
    type TxToken = TxToken<'a>;

    fn receive(&'a mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        if !self.phy.can_receive() {
            return None;
        }

        let tx_pkt = PacketBox::new(Packet::new())?;
        let (rx_pkt, rx_range) = self.phy.receive();
        let rx_token = RxToken {
            pkt: rx_pkt,
            range: rx_range,
        };
        let tx_token = TxToken {
            phy: self.phy,
            pkt: tx_pkt,
        };

        Some((rx_token, tx_token))
    }

    /// Construct a transmit token.
    fn transmit(&'a mut self) -> Option<Self::TxToken> {
        if !self.phy.can_transmit() {
            return None;
        }

        let tx_pkt = PacketBox::new(Packet::new())?;
        Some(TxToken {
            phy: self.phy,
            pkt: tx_pkt,
        })
    }

    /// Get a description of device capabilities.
    fn capabilities(&self) -> DeviceCapabilities {
        self.caps.clone()
    }
}

pub struct RxToken {
    pkt: PacketBox,
    range: Range<usize>,
}

impl smoltcp::phy::RxToken for RxToken {
    fn consume<R, F>(mut self, timestamp: SmolInstant, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        f(&mut self.pkt[self.range])
    }
}

pub struct TxToken<'a> {
    phy: &'a mut dyn Phy,
    pkt: PacketBox,
}

impl<'a> smoltcp::phy::TxToken for TxToken<'a> {
    fn consume<R, F>(mut self, timestamp: SmolInstant, len: usize, f: F) -> Result<R>
    where
        F: FnOnce(&mut [u8]) -> Result<R>,
    {
        let buf = &mut self.pkt[..len];
        let r = f(buf)?;
        self.phy.transmit(self.pkt, 0..len);
        Ok(r)
    }
}
