// SPDX-License-Identifier: GPL-2.0
#![allow(missing_docs)]

//! Devicetree and Open Firmware abstractions.
//!
//! C header: [`include/linux/of_*.h`](../../../../include/linux/of_*.h)

use core::marker::PhantomData;

use crate::{
    bindings, driver,
    prelude::*,
    str::{BStr, CStr},
};

/// An open firmware device id.
#[derive(Clone, Copy)]
pub enum DeviceId {
    /// An open firmware device id where only a compatible string is specified.
    Compatible(&'static BStr),
}

/// Defines a const open firmware device id table that also carries per-entry data/context/info.
///
/// The name of the const is `OF_DEVICE_ID_TABLE`, which is what buses are expected to name their
/// open firmware tables.
///
/// # Examples
///
/// ```
/// # use kernel::define_of_id_table;
/// use kernel::of;
///
/// define_of_id_table! {u32, [
///     (of::DeviceId::Compatible(b"test-device1,test-device2"), Some(0xff)),
///     (of::DeviceId::Compatible(b"test-device3"), None),
/// ]};
/// ```
#[macro_export]
macro_rules! define_of_id_table {
    ($data_type:ty, $($t:tt)*) => {
        $crate::define_id_table!(OF_DEVICE_ID_TABLE, $crate::of::DeviceId, $data_type, $($t)*);
    };
}

// SAFETY: `ZERO` is all zeroed-out and `to_rawid` stores `offset` in `of_device_id::data`.
unsafe impl const driver::RawDeviceId for DeviceId {
    type RawType = bindings::of_device_id;
    const ZERO: Self::RawType = bindings::of_device_id {
        name: [0; 32],
        type_: [0; 32],
        compatible: [0; 128],
        data: core::ptr::null(),
    };

    fn to_rawid(&self, offset: isize) -> Self::RawType {
        let DeviceId::Compatible(compatible) = self;
        let mut id = Self::ZERO;
        let mut i = 0;
        while i < compatible.len() {
            // If `compatible` does not fit in `id.compatible`, an "index out of bounds" build time
            // error will be triggered.
            id.compatible[i] = compatible[i] as _;
            i += 1;
        }
        id.compatible[i] = b'\0' as _;
        id.data = offset as _;
        id
    }
}

pub struct Node {
    raw_node: *mut bindings::device_node,
}

pub type PHandle = bindings::phandle;

impl Node {
    pub(crate) unsafe fn from_raw(raw_node: *mut bindings::device_node) -> Option<Node> {
        if raw_node.is_null() {
            None
        } else {
            Some(Node { raw_node })
        }
    }

    pub(crate) unsafe fn get_from_raw(raw_node: *mut bindings::device_node) -> Option<Node> {
        unsafe { Node::from_raw(bindings::of_node_get(raw_node)) }
    }

    fn node(&self) -> &bindings::device_node {
        unsafe { &*self.raw_node }
    }

    pub fn name(&self) -> &CStr {
        unsafe { CStr::from_char_ptr(self.node().name) }
    }

    pub fn phandle(&self) -> PHandle {
        self.node().phandle
    }

    pub fn full_name(&self) -> &CStr {
        unsafe { CStr::from_char_ptr(self.node().full_name) }
    }

    pub fn is_root(&self) -> bool {
        unsafe { bindings::of_node_is_root(self.raw_node) }
    }

    pub fn parent(&self) -> Option<Node> {
        unsafe { Node::from_raw(bindings::of_get_parent(self.raw_node)) }
    }

    // TODO: use type alias for return type once type_alias_impl_trait is stable
    pub fn children(
        &self,
    ) -> NodeIterator<'_, impl Fn(*mut bindings::device_node) -> *mut bindings::device_node + '_>
    {
        NodeIterator::new(|prev| unsafe { bindings::of_get_next_child(self.raw_node, prev) })
    }

    pub fn get_child_by_name(&self, name: &CStr) -> Option<Node> {
        unsafe {
            Node::from_raw(bindings::of_get_child_by_name(
                self.raw_node,
                name.as_char_ptr(),
            ))
        }
    }

    pub fn is_compatible(&self, compatible: &CStr) -> Option<u32> {
        let ret =
            unsafe { bindings::of_device_is_compatible(self.raw_node, compatible.as_char_ptr()) };

        match ret {
            0 => None,
            a => Some(a as u32),
        }
    }

    pub fn parse_phandle(&self, name: &CStr, index: usize) -> Option<Node> {
        unsafe {
            Node::from_raw(bindings::of_parse_phandle(
                self.raw_node,
                name.as_char_ptr(),
                index.try_into().ok()?,
            ))
        }
    }

    pub fn find_property(&self, propname: &CStr) -> Option<Property<'_>> {
        unsafe {
            Property::from_raw(bindings::of_find_property(
                self.raw_node,
                propname.as_char_ptr(),
                core::ptr::null_mut(),
            ))
        }
    }

    pub fn get_property<'a, T: TryFrom<Property<'a>>>(&'a self, propname: &CStr) -> Result<T>
    where
        crate::error::Error: From<<T as TryFrom<Property<'a>>>::Error>,
    {
        Ok(self.find_property(propname).ok_or(ENOENT)?.try_into()?)
    }

    pub fn get_opt_property<'a, T: TryFrom<Property<'a>>>(
        &'a self,
        propname: &CStr,
    ) -> Result<Option<T>>
    where
        crate::error::Error: From<<T as TryFrom<Property<'a>>>::Error>,
    {
        self.find_property(propname)
            .map_or(Ok(None), |p| Ok(Some(p.try_into()?)))
    }
}

#[derive(Copy, Clone)]
pub struct Property<'a> {
    raw: *mut bindings::property,
    _p: PhantomData<&'a Node>,
}

impl<'a> Property<'a> {
    unsafe fn from_raw(raw: *mut bindings::property) -> Option<Property<'a>> {
        if raw.is_null() {
            None
        } else {
            Some(Property {
                raw,
                _p: PhantomData,
            })
        }
    }

    pub fn name(&self) -> &CStr {
        unsafe { CStr::from_char_ptr((*self.raw).name) }
    }

    pub fn value(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts((*self.raw).value as *const u8, self.len()) }
    }

    pub fn len(&self) -> usize {
        unsafe { (*self.raw).length.try_into().unwrap() }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

pub trait PropertyUnit: Sized {
    const UNIT_SIZE: usize;

    fn from_bytes(data: &[u8]) -> Result<Self>;
}

// This doesn't work...
// impl<'a, T: PropertyUnit> TryFrom<Property<'a>> for T {
//     type Error = Error;
//
//     fn try_from(p: Property<'_>) -> core::result::Result<T, Self::Error> {
//         if p.value().len() != T::UNIT_SIZE {
//             Err(EINVAL)
//         } else {
//             Ok(T::from_bytes(p.value())?)
//         }
//     }
// }

impl<'a, T: PropertyUnit> TryFrom<Property<'a>> for Vec<T> {
    type Error = Error;

    fn try_from(p: Property<'_>) -> core::result::Result<Vec<T>, Self::Error> {
        if p.len() % T::UNIT_SIZE != 0 {
            return Err(EINVAL);
        }

        let mut v = Vec::new();
        let val = p.value();
        for off in (0..p.len()).step_by(T::UNIT_SIZE) {
            v.try_push(T::from_bytes(&val[off..off + T::UNIT_SIZE])?)?;
        }
        Ok(v)
    }
}

macro_rules! prop_int_type (
    ($type:ty) => {
        impl<'a> TryFrom<Property<'a>> for $type {
            type Error = Error;

            fn try_from(p: Property<'_>) -> core::result::Result<$type, Self::Error> {
                Ok(<$type>::from_be_bytes(p.value().try_into().or(Err(EINVAL))?))
            }
        }

        impl PropertyUnit for $type {
            const UNIT_SIZE: usize = <$type>::BITS as usize / 8;

            fn from_bytes(data: &[u8]) -> Result<Self> {
                Ok(<$type>::from_be_bytes(data.try_into().or(Err(EINVAL))?))
            }
        }
    }
);

prop_int_type!(u8);
prop_int_type!(u16);
prop_int_type!(u32);
prop_int_type!(u64);
prop_int_type!(i8);
prop_int_type!(i16);
prop_int_type!(i32);
prop_int_type!(i64);

pub struct NodeIterator<'a, T>
where
    T: Fn(*mut bindings::device_node) -> *mut bindings::device_node,
{
    cur: *mut bindings::device_node,
    done: bool,
    fn_next: T,
    _p: PhantomData<&'a T>,
}

impl<'a, T> NodeIterator<'a, T>
where
    T: Fn(*mut bindings::device_node) -> *mut bindings::device_node,
{
    fn new(next: T) -> NodeIterator<'a, T> {
        NodeIterator {
            cur: core::ptr::null_mut(),
            done: false,
            fn_next: next,
            _p: PhantomData,
        }
    }
}

impl<'a, T> Iterator for NodeIterator<'a, T>
where
    T: Fn(*mut bindings::device_node) -> *mut bindings::device_node,
{
    type Item = Node;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            None
        } else {
            self.cur = (self.fn_next)(self.cur);
            self.done = self.cur.is_null();
            unsafe { Node::from_raw(self.cur) }
        }
    }
}

pub fn root() -> Option<Node> {
    unsafe { Node::get_from_raw(bindings::of_root) }
}

pub fn chosen() -> Option<Node> {
    unsafe { Node::get_from_raw(bindings::of_chosen) }
}

pub fn aliases() -> Option<Node> {
    unsafe { Node::get_from_raw(bindings::of_aliases) }
}

pub fn stdout() -> Option<Node> {
    unsafe { Node::get_from_raw(bindings::of_stdout) }
}

pub fn find_node_by_phandle(handle: PHandle) -> Option<Node> {
    unsafe { Node::from_raw(bindings::of_find_node_by_phandle(handle)) }
}

impl Clone for Node {
    fn clone(&self) -> Node {
        unsafe { Node::get_from_raw(self.raw_node).unwrap() }
    }
}

impl Drop for Node {
    fn drop(&mut self) {
        unsafe { bindings::of_node_put(self.raw_node) };
    }
}
