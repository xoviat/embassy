use as_slice::{AsMutSlice, AsSlice};
use core::cmp;
use core::fmt;
use core::hash::{Hash, Hasher};
use core::mem::MaybeUninit;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::{AtomicU32, Ordering};
use generic_array::typenum::bit::B1;
use generic_array::typenum::consts::*;
use generic_array::typenum::type_operators::IsLessOrEqual;
use generic_array::{ArrayLength, GenericArray};

use crate::fmt::{assert, panic, *};

pub trait Pool<T> {
    fn alloc(&self) -> Option<*mut T>;
    unsafe fn free(&self, p: *mut T);
}

pub struct BitPool<T, N>
where
    N: IsLessOrEqual<U32, Output = B1>,
    N: ArrayLength<T>,
{
    used: AtomicU32,
    data: MaybeUninit<GenericArray<T, N>>,
}

impl<T, N> BitPool<T, N>
where
    N: IsLessOrEqual<U32, Output = B1>,
    N: ArrayLength<T>,
{
    pub const fn new() -> Self {
        Self {
            used: AtomicU32::new(0),
            data: MaybeUninit::uninit(),
        }
    }
}

impl<T, N> Pool<T> for BitPool<T, N>
where
    N: IsLessOrEqual<U32, Output = B1>,
    N: ArrayLength<T>,
{
    fn alloc(&self) -> Option<*mut T> {
        let r = self
            .used
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |x| {
                let n = x.trailing_ones();
                if n as usize >= N::USIZE {
                    None
                } else {
                    Some(x | (1 << n))
                }
            });

        match r {
            Ok(old) => {
                let n = old.trailing_ones() as usize;
                let origin = self.data.as_ptr() as *mut T;
                Some(unsafe { origin.add(n) })
            }
            Err(_) => None,
        }
    }

    /// safety: p must be a pointer obtained from self.alloc that hasn't been freed yet.
    unsafe fn free(&self, p: *mut T) {
        let origin = self.data.as_ptr() as *mut T;
        let n = p.offset_from(origin);
        assert!(n >= 0);
        assert!((n as usize) < N::USIZE);
        self.used.fetch_and(!(1 << (n as u32)), Ordering::AcqRel);
    }
}

pub trait StaticPool: 'static {
    type Item: 'static;
    type Pool: Pool<Self::Item>;
    fn get() -> &'static Self::Pool;
}

pub struct Box<P: StaticPool> {
    ptr: *mut P::Item,
}

impl<P: StaticPool> Box<P> {
    pub fn new(item: P::Item) -> Option<Self> {
        let p = match P::get().alloc() {
            Some(p) => p,
            None => {
                warn!("alloc failed!");
                return None;
            }
        };
        //trace!("allocated {:u32}", p as u32);
        unsafe { p.write(item) };
        Some(Self { ptr: p })
    }
}

impl<P: StaticPool> Drop for Box<P> {
    fn drop(&mut self) {
        unsafe {
            //trace!("dropping {:u32}", self.ptr as u32);
            self.ptr.drop_in_place();
            P::get().free(self.ptr);
        };
    }
}

unsafe impl<P: StaticPool> Send for Box<P> where P::Item: Send {}

unsafe impl<P: StaticPool> Sync for Box<P> where P::Item: Sync {}

unsafe impl<P: StaticPool> stable_deref_trait::StableDeref for Box<P> {}

impl<P: StaticPool> AsSlice for Box<P>
where
    P::Item: AsSlice,
{
    type Element = <P::Item as AsSlice>::Element;

    fn as_slice(&self) -> &[Self::Element] {
        self.deref().as_slice()
    }
}

impl<P: StaticPool> AsMutSlice for Box<P>
where
    P::Item: AsMutSlice,
{
    fn as_mut_slice(&mut self) -> &mut [Self::Element] {
        self.deref_mut().as_mut_slice()
    }
}

impl<P: StaticPool> Deref for Box<P> {
    type Target = P::Item;

    fn deref(&self) -> &P::Item {
        unsafe { &*self.ptr }
    }
}

impl<P: StaticPool> DerefMut for Box<P> {
    fn deref_mut(&mut self) -> &mut P::Item {
        unsafe { &mut *self.ptr }
    }
}

impl<P: StaticPool> fmt::Debug for Box<P>
where
    P::Item: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        <P::Item as fmt::Debug>::fmt(self, f)
    }
}

impl<P: StaticPool> fmt::Display for Box<P>
where
    P::Item: fmt::Display,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        <P::Item as fmt::Display>::fmt(self, f)
    }
}

impl<P: StaticPool> PartialEq for Box<P>
where
    P::Item: PartialEq,
{
    fn eq(&self, rhs: &Box<P>) -> bool {
        <P::Item as PartialEq>::eq(self, rhs)
    }
}

impl<P: StaticPool> Eq for Box<P> where P::Item: Eq {}

impl<P: StaticPool> PartialOrd for Box<P>
where
    P::Item: PartialOrd,
{
    fn partial_cmp(&self, rhs: &Box<P>) -> Option<cmp::Ordering> {
        <P::Item as PartialOrd>::partial_cmp(self, rhs)
    }
}

impl<P: StaticPool> Ord for Box<P>
where
    P::Item: Ord,
{
    fn cmp(&self, rhs: &Box<P>) -> cmp::Ordering {
        <P::Item as Ord>::cmp(self, rhs)
    }
}

impl<P: StaticPool> Hash for Box<P>
where
    P::Item: Hash,
{
    fn hash<H>(&self, state: &mut H)
    where
        H: Hasher,
    {
        <P::Item as Hash>::hash(self, state)
    }
}
