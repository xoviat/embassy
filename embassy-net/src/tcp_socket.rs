use core::cell::UnsafeCell;
use core::mem;
use core::pin::Pin;
use core::task::{Context, Poll};
use embassy::io;
use embassy::io::{AsyncBufRead, AsyncWrite};
use generic_array::{ArrayLength, GenericArray};
use smoltcp::socket::SocketHandle;
use smoltcp::socket::TcpSocket as SyncTcpSocket;
use smoltcp::socket::{TcpSocketBuffer, TcpState};
use smoltcp::time::{Duration, Instant};
use smoltcp::wire::IpEndpoint;
use smoltcp::{Error, Result};

use super::Stack;
use crate::fmt::*;

pub struct TcpSocket<TxLen, RxLen>
where
    TxLen: ArrayLength<u8>,
    RxLen: ArrayLength<u8>,
{
    stack: &'static Stack,
    handle: Option<SocketHandle>,
    tx_buffer: UnsafeCell<GenericArray<u8, TxLen>>,
    rx_buffer: UnsafeCell<GenericArray<u8, RxLen>>,
}

impl<TxLen, RxLen> TcpSocket<TxLen, RxLen>
where
    TxLen: ArrayLength<u8>,
    RxLen: ArrayLength<u8>,
{
    pub fn new(stack: &'static Stack) -> Self {
        Self {
            stack,
            rx_buffer: UnsafeCell::new(GenericArray::default()),
            tx_buffer: UnsafeCell::new(GenericArray::default()),

            // We cannot create the socket here, because self isn't pinned yet, so it's not safe
            // to take pointers to rx_buffer, tx_buffer.
            handle: None,
        }
    }

    pub async fn connect<T>(mut self: Pin<&mut Self>, remote_endpoint: T) -> Result<()>
    where
        T: Into<IpEndpoint>,
    {
        let local_port = self.stack.inner.borrow_mut().get_local_port();

        info!("LOCAL PORT: {:u16}", local_port);

        self.as_mut()
            .with_mut(|s| s.connect(remote_endpoint, local_port))?;

        futures::future::poll_fn(|cx| {
            self.as_mut().with_mut(|s| match s.state() {
                TcpState::Closed | TcpState::TimeWait => Poll::Ready(Err(Error::Unaddressable)),
                TcpState::Listen => Poll::Ready(Err(Error::Illegal)),
                TcpState::SynSent | TcpState::SynReceived => {
                    s.register_tx_waker(cx.waker());
                    Poll::Pending
                }
                _ => Poll::Ready(Ok(())),
            })
        })
        .await
    }

    pub fn set_timeout(self: Pin<&mut Self>, duration: Option<Duration>) {
        self.with_mut(|s| s.set_timeout(duration))
    }

    pub fn set_keep_alive(self: Pin<&mut Self>, interval: Option<Duration>) {
        self.with_mut(|s| s.set_keep_alive(interval))
    }

    pub fn set_hop_limit(self: Pin<&mut Self>, hop_limit: Option<u8>) {
        self.with_mut(|s| s.set_hop_limit(hop_limit))
    }

    pub fn local_endpoint(&self) -> IpEndpoint {
        self.with(|s| s.local_endpoint())
            .unwrap_or(IpEndpoint::UNSPECIFIED)
    }

    pub fn remote_endpoint(&self) -> IpEndpoint {
        self.with(|s| s.remote_endpoint())
            .unwrap_or(IpEndpoint::UNSPECIFIED)
    }

    pub fn state(&self) -> TcpState {
        self.with(|s| s.state()).unwrap_or(TcpState::Closed)
    }

    pub fn close(self: Pin<&mut Self>) {
        self.with_mut(|s| s.close())
    }

    pub fn abort(self: Pin<&mut Self>) {
        self.with_mut(|s| s.abort())
    }

    pub fn may_send(&self) -> bool {
        self.with(|s| s.may_send()).unwrap_or(false)
    }

    pub fn may_recv(&self) -> bool {
        self.with(|s| s.may_recv()).unwrap_or(false)
    }

    fn with<R>(&self, f: impl FnOnce(&SyncTcpSocket) -> R) -> Option<R> {
        self.handle.map(|handle| {
            let mut s = self.stack.inner.borrow_mut();
            let socket = s.sockets.get::<SyncTcpSocket>(handle);
            f(&*socket)
        })
    }

    fn with_mut<R>(mut self: Pin<&mut Self>, f: impl FnOnce(&mut SyncTcpSocket) -> R) -> R {
        let this = unsafe { self.get_unchecked_mut() };

        let mut s = this.stack.inner.borrow_mut();
        if this.handle.is_none() {
            let (rx_buffer, tx_buffer) = unsafe {
                let rx_buffer: &mut [u8] = (*this.rx_buffer.get()).as_mut_slice();
                let tx_buffer: &mut [u8] = (*this.tx_buffer.get()).as_mut_slice();
                let rx_buffer: &'static mut [u8] = mem::transmute(rx_buffer);
                let tx_buffer: &'static mut [u8] = mem::transmute(tx_buffer);
                (rx_buffer, tx_buffer)
            };
            this.handle = Some(s.sockets.add(SyncTcpSocket::new(
                TcpSocketBuffer::new(rx_buffer),
                TcpSocketBuffer::new(tx_buffer),
            )));
        }
        let mut socket = s.sockets.get::<SyncTcpSocket>(this.handle.unwrap());
        f(&mut *socket)
    }
}

fn to_ioerr(e: Error) -> io::Error {
    warn!("smoltcp err: {:?}", e);
    // todo
    io::Error::Other
}

impl<TxLen, RxLen> Drop for TcpSocket<TxLen, RxLen>
where
    TxLen: ArrayLength<u8>,
    RxLen: ArrayLength<u8>,
{
    fn drop(&mut self) {
        if let Some(handle) = self.handle {
            let mut s = self.stack.inner.borrow_mut();
            s.sockets.remove(handle);
        }
    }
}

impl<TxLen, RxLen> AsyncBufRead for TcpSocket<TxLen, RxLen>
where
    TxLen: ArrayLength<u8>,
    RxLen: ArrayLength<u8>,
{
    fn poll_fill_buf<'z>(
        mut self: Pin<&'z mut Self>,
        cx: &mut Context<'_>,
    ) -> Poll<io::Result<&'z [u8]>> {
        self.with_mut(|socket| match socket.peek(1 << 30) {
            // No data ready
            Ok(buf) if buf.len() == 0 => {
                socket.register_recv_waker(cx.waker());
                Poll::Pending
            }
            // Data ready!
            Ok(buf) => {
                // Safety:
                // - User can't touch the inner TcpSocket directly at all.
                // - The socket itself won't touch these bytes until consume() is called, which
                //   requires the user to release this borrow.
                let buf: &'z [u8] = unsafe { core::mem::transmute(&*buf) };
                Poll::Ready(Ok(buf))
            }
            // EOF
            Err(Error::Finished) => Poll::Ready(Ok(&[][..])),
            // Error
            Err(e) => Poll::Ready(Err(to_ioerr(e))),
        })
    }

    fn consume(mut self: Pin<&mut Self>, amt: usize) {
        self.with_mut(|s| s.recv(|_| (amt, ()))).unwrap()
    }
}

impl<TxLen, RxLen> AsyncWrite for TcpSocket<TxLen, RxLen>
where
    TxLen: ArrayLength<u8>,
    RxLen: ArrayLength<u8>,
{
    fn poll_write(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
        buf: &[u8],
    ) -> Poll<io::Result<usize>> {
        self.with_mut(|s| match s.send_slice(buf) {
            // Not ready to send (no space in the tx buffer)
            Ok(0) => {
                s.register_send_waker(cx.waker());
                Poll::Pending
            }
            // Some data sent
            Ok(n) => Poll::Ready(Ok(n)),
            // Error
            Err(e) => Poll::Ready(Err(to_ioerr(e))),
        })
    }
}
