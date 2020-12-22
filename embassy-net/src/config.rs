use core::cell::RefCell;
use core::task::Poll;
use embassy::executor::task;
use embassy::time::Instant;
use embassy::util::Forever;
use heapless::consts::*;
use heapless::Vec;
use smoltcp::dhcp::{Dhcpv4Client, Dhcpv4Config};
use smoltcp::iface::{InterfaceBuilder, Neighbor, NeighborCache, Route, Routes};
use smoltcp::phy::Medium;
use smoltcp::socket::SocketSetItem;
use smoltcp::socket::{RawPacketMetadata, RawSocketBuffer};
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address, Ipv4Cidr};

use super::{Interface, SocketSet};
use crate::fmt::{assert, panic, *};

#[derive(Debug, Clone)]
pub enum Config {
    Down,
    Up(UpConfig),
}

#[derive(Debug, Clone)]
pub struct UpConfig {
    pub address: Ipv4Cidr,
    pub gateway: Ipv4Address,
    pub dns_servers: Vec<Ipv4Address, U3>,
}

pub trait Configurator {
    fn poll(&mut self, iface: &mut Interface, sockets: &mut SocketSet) -> Option<Config>;
}

pub struct StaticConfigurator {
    config: UpConfig,
}

impl StaticConfigurator {
    pub fn new(config: UpConfig) -> Self {
        Self { config }
    }
}

impl Configurator for StaticConfigurator {
    fn poll(&mut self, iface: &mut Interface, sockets: &mut SocketSet) -> Option<Config> {
        Some(Config::Up(self.config.clone()))
    }
}
pub struct DhcpResources {
    rx_buffer: [u8; 900],
    tx_buffer: [u8; 600],
    rx_meta: [RawPacketMetadata; 1],
    tx_meta: [RawPacketMetadata; 1],
}

pub struct DhcpConfigurator {
    client: Option<Dhcpv4Client>,
}

impl DhcpConfigurator {
    pub fn new() -> Self {
        Self { client: None }
    }
}

static DHCP_RESOURCES: Forever<DhcpResources> = Forever::new();

impl Configurator for DhcpConfigurator {
    fn poll(&mut self, iface: &mut Interface, sockets: &mut SocketSet) -> Option<Config> {
        if self.client.is_none() {
            let res = DHCP_RESOURCES.put(DhcpResources {
                rx_buffer: [0; 900],
                tx_buffer: [0; 600],
                rx_meta: [RawPacketMetadata::EMPTY; 1],
                tx_meta: [RawPacketMetadata::EMPTY; 1],
            });
            let rx_buffer = RawSocketBuffer::new(&mut res.rx_meta[..], &mut res.rx_buffer[..]);
            let tx_buffer = RawSocketBuffer::new(&mut res.tx_meta[..], &mut res.tx_buffer[..]);
            let dhcp = Dhcpv4Client::new(sockets, rx_buffer, tx_buffer, Instant::now());
            info!("created dhcp");
            self.client = Some(dhcp)
        }

        let client = self.client.as_mut().unwrap();

        let link_up = iface.device_mut().phy.is_link_up();
        if !link_up {
            client.reset(Instant::now());
            return Some(Config::Down);
        }

        let config = client
            .poll(iface, sockets, Instant::now())
            .unwrap_or(None)?;

        if config.address.is_none() {
            return Some(Config::Down);
        }

        let mut dns_servers = Vec::new();
        for s in &config.dns_servers {
            if let Some(addr) = s {
                dns_servers.push(addr.clone()).unwrap();
            }
        }

        return Some(Config::Up(UpConfig {
            address: config.address.unwrap(),
            gateway: config.router.unwrap_or(Ipv4Address::UNSPECIFIED),
            dns_servers,
        }));
    }
}
