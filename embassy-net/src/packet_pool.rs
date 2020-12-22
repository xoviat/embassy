use as_slice::{AsMutSlice, AsSlice};
use core::ops::{Deref, DerefMut};
use heapless::consts::*;

use super::pool::{BitPool, Box, StaticPool};
use crate::fmt::{assert, panic, *};

pub struct PacketPool;
impl StaticPool for PacketPool {
    type Item = Packet;
    type Pool = BitPool<Packet, PacketPoolSize>;
    fn get() -> &'static Self::Pool {
        static POOL: BitPool<Packet, PacketPoolSize> = BitPool::new();
        &POOL
    }
}

pub const MTU: usize = 1514;
pub type PacketPoolSize = U2;
pub type PacketBox = Box<PacketPool>;
pub struct Packet(pub [u8; MTU]);

impl Packet {
    pub const fn new() -> Self {
        Self([0; MTU])
    }
}
impl AsSlice for Packet {
    type Element = u8;

    fn as_slice(&self) -> &[Self::Element] {
        &self.deref()[..]
    }
}

impl AsMutSlice for Packet {
    fn as_mut_slice(&mut self) -> &mut [Self::Element] {
        &mut self.deref_mut()[..]
    }
}

impl Deref for Packet {
    type Target = [u8; MTU];

    fn deref(&self) -> &[u8; MTU] {
        &self.0
    }
}

impl DerefMut for Packet {
    fn deref_mut(&mut self) -> &mut [u8; MTU] {
        &mut self.0
    }
}
