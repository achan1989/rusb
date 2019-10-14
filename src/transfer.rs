use std::{
    alloc::{alloc_zeroed, dealloc, Layout},
    convert::TryFrom,
    ptr::{self, NonNull},
    slice::self,
    time::Duration,
};

use libc::{c_int, c_uint};
use libusb1_sys::{constants::*, *};

use crate::{
    device_handle::DeviceHandle,
    error::Error,
    UsbContext,
};

const CLEAR_TRANSFER_FLAGS: u8 = 0;


#[derive(Debug, Copy, Clone)]
enum TransferKind {
    Unfilled,
    Bulk,
    Control,
    Interrupt,
    // Isochronous,
}


#[derive(Debug, Copy, Clone)]
pub struct BufferLength(c_int);

impl BufferLength {
    fn as_c_int(&self) -> c_int {
        self.0
    }

    fn as_usize(&self) -> usize {
        self.0 as usize
    }
}

impl TryFrom<usize> for BufferLength {
    type Error = Error;

    fn try_from(n: usize) -> Result<BufferLength, Error> {
        match c_int::try_from(n) {
             Ok(len) => Ok(BufferLength(len)),
             Err(_) => Err(Error::InvalidParam)
         }
    }
}


#[derive(Debug, Copy, Clone)]
struct BufferInfo {
    pointer: Option<*mut u8>,
    length: usize,
}


#[derive(Debug, Copy, Clone)]
pub struct Timeout(c_uint);

impl Timeout {
    fn value(&self) -> c_uint {
        self.0
    }

    fn none() -> Self {
        Self(0)
    }
}

impl TryFrom<Duration> for Timeout {
    type Error = Error;

    fn try_from(d: Duration) -> Result<Timeout, Error> {
        match c_uint::try_from(d.as_millis()) {
            Ok(t) => Ok(Timeout(t)),
            Err(_) => Err(Error::InvalidParam),
        }
    }
}


#[derive(Debug, Copy, Clone)]
pub struct ControlSetup {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub data_length: u16,
}

impl ControlSetup {
    pub const fn packet_length() -> usize {
        8
    }
}


#[derive(Debug)]
// pub struct Transfer<T: UsbContext> {
pub struct Transfer {
    // _context: T,
    handle: NonNull<libusb_transfer>,
    max_iso_packets: u16,
    kind: TransferKind,
}

// impl<T: UsbContext> Transfer<T> {
impl Transfer {
    pub fn new(max_iso_packets: u16) -> Self {
        let mut handle = unsafe {
            let ptr = libusb_alloc_transfer(max_iso_packets.into());
            NonNull::new(ptr).expect("alloc failed")
        };

        unsafe { handle.as_mut().flags = CLEAR_TRANSFER_FLAGS; }

        Self {handle, max_iso_packets, kind: TransferKind::Unfilled}

        // TODO:
        // transfer->user_data = user_data;
        // transfer->callback = callback;
    }

    fn as_raw(&self) -> *mut libusb_transfer {
        self.handle.as_ptr()
    }

    pub fn get_buffer_mut<'a>(&'a mut self) -> Result<&'a mut[u8], Error> {
        match self.kind {
            TransferKind::Unfilled => return Err(Error::BadState),

            TransferKind::Control => {
                let transfer = self.as_raw();
                unimplemented!();
                // let pointer = unsafe { (*transfer).}
            },

            TransferKind::Bulk |
            TransferKind::Interrupt => {
                unimplemented!();
            },
        }

        let info = self.current_buffer_info();

        unimplemented!();
    }

    // TODO: set_out_data()

    pub fn fill_bulk<T: UsbContext>(
        &mut self,
        length: BufferLength,
        device_handle: &DeviceHandle<T>,
        endpoint: u8,
        timeout: Timeout
    ) -> Result<(), Error>
    {
        let transfer = self.as_raw();
        unsafe {
            self.setup_buffer(length)?;
            (*transfer).dev_handle = device_handle.as_raw();
            (*transfer).endpoint = endpoint;
            (*transfer).transfer_type = LIBUSB_TRANSFER_TYPE_BULK;
            (*transfer).timeout = timeout.value();
        }
        self.kind = TransferKind::Bulk;
        Ok(())
    }

    pub fn fill_control<T: UsbContext>(&mut self,
        setup: &ControlSetup,
        device_handle: &DeviceHandle<T>,
        endpoint: u8,
        timeout: Timeout
    ) -> Result<(), Error>
    {
        let buffer_length = BufferLength::try_from(
            ControlSetup::packet_length() + (setup.data_length as usize))?;
        let transfer = self.as_raw();
        unsafe {
            self.setup_buffer(buffer_length)?;
            (*transfer).dev_handle = device_handle.as_raw();
            (*transfer).endpoint = endpoint;
            (*transfer).transfer_type = LIBUSB_TRANSFER_TYPE_CONTROL;
            (*transfer).timeout = timeout.value();
            self.fill_control_setup(&setup);
        }
        self.kind = TransferKind::Control;
        Ok(())
    }

    unsafe fn fill_control_setup(&mut self, setup: &ControlSetup) {
        let transfer = self.as_raw();
        let header = slice::from_raw_parts_mut(
            (*transfer).buffer, ControlSetup::packet_length());

        header[0] = setup.request_type;
        header[1] = setup.request;
        header[2..=3].copy_from_slice(&setup.value.to_le_bytes()[..]);
        header[4..=5].copy_from_slice(&setup.index.to_le_bytes()[..]);
        header[6..=7].copy_from_slice(&setup.data_length.to_le_bytes()[..]);
    }

    pub fn fill_interrupt<T: UsbContext>(
        &mut self,
        length: BufferLength,
        device_handle: &DeviceHandle<T>,
        endpoint: u8,
        timeout: Timeout
    ) -> Result<(), Error>
    {
        let transfer = self.as_raw();
        unsafe {
            self.setup_buffer(length)?;
            (*transfer).dev_handle = device_handle.as_raw();
            (*transfer).endpoint = endpoint;
            (*transfer).transfer_type = LIBUSB_TRANSFER_TYPE_INTERRUPT;
            (*transfer).timeout = timeout.value();
        }
        self.kind = TransferKind::Interrupt;
        Ok(())
    }

    unsafe fn setup_buffer(
        &mut self, length: BufferLength
    ) -> Result<(), Error>
    {
        // We can just re-use the existing buffer if it's big enough.
        if self.must_allocate_for_buffer(length) {
            // Allocate the new buffer before freeing the existing one.
            // That way we are always in a valid state if allocation fails.
            // Zero the buffer in case the user sends to the device without
            // providing data -- prevents read of uninitialized memory.
            let new_buffer = alloc_zeroed(
                get_transfer_buffer_layout(length.as_usize()));
            if new_buffer.is_null() {
                return Err(Error::NoMem);
            }
            self.replace_buffer(new_buffer, length);
        }
        Ok(())
    }

    unsafe fn replace_buffer(&mut self, new_buffer: *mut u8, length: BufferLength)
    {
        let transfer = self.as_raw();
        let old_buffer = self.current_buffer_info();

        (*transfer).buffer = new_buffer;
        (*transfer).length = length.as_c_int();

        if let Some(old_pointer) = old_buffer.pointer {
            dealloc(old_pointer, get_transfer_buffer_layout(old_buffer.length));
        }
    }

    fn must_allocate_for_buffer(&self, length:BufferLength) -> bool {
        let current_buffer_capacity = self.current_buffer_info().length;
        current_buffer_capacity < length.as_usize()
    }

    fn current_buffer_info(&self) -> BufferInfo {
        match self.kind {
            TransferKind::Unfilled => BufferInfo{pointer: None, length: 0},

            TransferKind::Bulk |
            TransferKind::Control |
            TransferKind::Interrupt => {
                let transfer = self.as_raw();
                let pointer = unsafe { Some((*transfer).buffer) };
                let length = {
                    let c_length = unsafe { (*transfer).length };
                    if c_length < 0 {
                        0
                    } else {
                        c_length as usize
                    }
                };
                BufferInfo{pointer, length}
            }
        }
    }
}

impl Drop for Transfer {
    fn drop(&mut self) {
        unsafe { libusb_free_transfer(self.handle.as_ptr()); }
    }
}


fn get_transfer_buffer_layout(size: usize) -> Layout {
    Layout::from_size_align(size, 2).expect("alloc failed")
}
