#![cfg_attr(not(feature = "std"), no_std)]
#![feature(const_fn)]
#![feature(const_in_array_repeat_expressions)]

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

mod config;
mod device;
mod packet_pool;
mod pool; // TODO extract to embassy, or to own crate
mod stack;
mod tcp_socket;

pub use device::Phy;
pub use packet_pool::{Packet, PacketBox};
pub use stack::{init, Stack};
pub use tcp_socket::TcpSocket;

pub use config::{Config, Configurator, DhcpConfigurator, StaticConfigurator, UpConfig};

// smoltcp reexports
pub use smoltcp::phy::{DeviceCapabilities, Medium};
pub use smoltcp::wire::{IpAddress, IpCidr, Ipv4Address, Ipv4Cidr};
pub type Interface = smoltcp::iface::Interface<'static, 'static, 'static, device::Device>;
pub type SocketSet = smoltcp::socket::SocketSet<'static, 'static, 'static>;
