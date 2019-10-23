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
    fields::{Direction, Recipient, RequestType, TransferType},
    UsbContext,
};

const CLEAR_TRANSFER_FLAGS: u8 = 0;
const FILL_WITH_ZERO: u8 = 0;
const CONTROL_SETUP_SIZE: usize = 8;


#[derive(Debug, Copy, Clone)]
pub struct ReadLength(c_int);

impl ReadLength {
    pub fn as_c_int(&self) -> c_int {
        self.0
    }

    pub fn as_usize(&self) -> usize {
        self.0 as usize
    }
}

impl TryFrom<usize> for ReadLength {
    type Error = Error;

    fn try_from(n: usize) -> Result<ReadLength, Error> {
        match c_int::try_from(n) {
             Ok(len) => Ok(ReadLength(len)),
             Err(_) => Err(Error::TooBig)
         }
    }
}


#[derive(Debug)]
pub struct WriteData<'a>(&'a [u8]);

impl<'a> TryFrom<&'a [u8]> for WriteData<'a> {
    type Error = Error;

    fn try_from(data: &[u8]) -> Result<WriteData<'_>, Error> {
        match c_int::try_from(data.len()) {
            Ok(_) => Ok(WriteData(data)),
            Err(_) => Err(Error::TooBig),
        }
    }
}

impl<'a> WriteData<'a> {
    pub fn length(&self) -> usize {
        self.0.len()
    }

    pub fn length_as_c_int(&self) -> c_int {
        self.0.len() as c_int
    }

    fn copy_data(&self, buffer: &mut [u8]) {
        if self.length() > 0 {
            buffer.copy_from_slice(self.0);
        }
    }
}


#[derive(Debug, Copy, Clone)]
pub struct Timeout(c_uint);

impl Timeout {
    pub fn as_c_uint(&self) -> c_uint {
        self.0
    }

    pub fn none() -> Self {
        Self(0)
    }

    pub fn from_millis(ms: u32) -> Self {
        Self(ms)
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


#[derive(Debug)]
pub struct ReadEndpoint(u8);

impl TryFrom<u8> for ReadEndpoint {
    type Error = Error;

    fn try_from(endpoint: u8) -> Result<ReadEndpoint, Error> {
        match endpoint & LIBUSB_ENDPOINT_DIR_MASK {
            LIBUSB_ENDPOINT_IN => Ok(ReadEndpoint(endpoint)),
            _ => Err(Error::InvalidParam)
        }
    }
}

impl ReadEndpoint {
    pub fn as_u8(&self) -> u8 {
        self.0
    }
}

#[derive(Debug)]
pub struct WriteEndpoint(u8);

impl TryFrom<u8> for WriteEndpoint {
    type Error = Error;

    fn try_from(endpoint: u8) -> Result<WriteEndpoint, Error> {
        match endpoint & LIBUSB_ENDPOINT_DIR_MASK {
            LIBUSB_ENDPOINT_OUT => Ok(WriteEndpoint(endpoint)),
            _ => Err(Error::InvalidParam)
        }
    }
}

impl WriteEndpoint {
    pub fn as_u8(&self) -> u8 {
        self.0
    }
}


#[derive(Debug)]
pub struct ControlReadSetup {
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    data_length: u16,
}

impl ControlReadSetup {
    pub fn new(
        recipient: Recipient,
        request_type: RequestType,
        request: u8,
        value: u16,
        index: u16,
        data_length: u16,
    ) -> Result<ControlReadSetup, Error>
    {
        match is_valid_control_data_length(data_length as usize) {
            true => {
                let request_type_byte = crate::fields::request_type(
                    Direction::In, request_type, recipient);
                Ok(ControlReadSetup{
                    request_type: request_type_byte, request, value, index, data_length})
            },
            false => Err(Error::TooBig),
        }
    }

    pub fn data_length(&self) -> u16 {
        self.data_length
    }

    pub fn total_length(&self) -> usize {
        self.data_length as usize + CONTROL_SETUP_SIZE
    }

    pub fn total_length_as_c_int(&self) -> c_int {
        self.total_length() as c_int
    }

    fn write_setup_packet(&self, buffer: &mut [u8]) {
        buffer[0] = self.request_type;
        buffer[1] = self.request;
        buffer[2..=3].copy_from_slice(&self.value.to_le_bytes()[..]);
        buffer[4..=5].copy_from_slice(&self.index.to_le_bytes()[..]);
        buffer[6..=7].copy_from_slice(&self.data_length.to_le_bytes()[..]);
    }
}

#[derive(Debug)]
pub struct ControlWriteSetup<'a> {
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    data: &'a [u8],
}

impl<'a> ControlWriteSetup<'a> {
    pub fn new(
        recipient: Recipient,
        request_type: RequestType,
        request: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> Result<ControlWriteSetup, Error>
    {
        let data_length = data.len();
        match is_valid_control_data_length(data_length) {
            true => {
                let request_type_byte = crate::fields::request_type(
                    Direction::Out, request_type, recipient);
                Ok(ControlWriteSetup{
                    request_type: request_type_byte, request, value, index, data})
            },
            false => Err(Error::TooBig),
        }
    }

    pub fn data_length(&self) -> u16 {
        self.data.len() as u16
    }

    pub fn total_length(&self) -> usize {
        self.data.len() + CONTROL_SETUP_SIZE
    }

    pub fn total_length_as_c_int(&self) -> c_int {
        self.total_length() as c_int
    }

    fn write_setup_packet(&self, buffer: &mut [u8]) {
        buffer[0] = self.request_type;
        buffer[1] = self.request;
        buffer[2..=3].copy_from_slice(&self.value.to_le_bytes()[..]);
        buffer[4..=5].copy_from_slice(&self.index.to_le_bytes()[..]);
        buffer[6..=7].copy_from_slice(&self.data_length().to_le_bytes()[..]);
    }

    fn copy_body_data(&self, buffer: &mut [u8]) {
        if self.data_length() > 0 {
            buffer.copy_from_slice(self.data);
        }
    }
}


#[derive(Debug)]
pub enum Transfer {
    Unfilled(UnfilledTransfer),
    ReadBulk(ReadTransfer),
    WriteBulk(WriteTransfer),
    ReadControl(ReadTransfer),
    WriteControl(WriteTransfer),
    ReadInterrupt(ReadTransfer),
    WriteInterrupt(WriteTransfer),
    // ReadIsochronous(TransferInternal),
    // WriteIsochronous(TransferInternal),
}

impl Transfer {
    pub fn new(max_iso_packets: u16) -> Self {
        Self::Unfilled(UnfilledTransfer::new(max_iso_packets))
    }
}

#[derive(Debug)]
pub struct UnfilledTransfer {
    inner: Internals,
}

impl UnfilledTransfer {
    fn new(max_iso_packets: u16) -> Self {
        Self{inner: Internals::new_unfilled(max_iso_packets)}
    }

    pub fn fill_bulk_read<T: UsbContext>(
        mut self,
        device_handle: &DeviceHandle<T>,
        endpoint: ReadEndpoint,
        length: ReadLength,
        timeout: Timeout
    ) -> Transfer {
        unsafe {
            self.inner.set_transfer_type(TransferType::Bulk);
            self.inner.set_device_handle(device_handle);
            self.inner.set_endpoint(endpoint.as_u8());
            self.inner.setup_read(length);
            self.inner.set_timeout(timeout);
            Transfer::ReadBulk(ReadTransfer{inner: self.inner})
        }
    }

    pub fn fill_bulk_write<T: UsbContext>(
        mut self,
        device_handle: &DeviceHandle<T>,
        endpoint: WriteEndpoint,
        data: &WriteData,
        timeout: Timeout
    ) -> Transfer {
        unsafe {
            self.inner.set_transfer_type(TransferType::Bulk);
            self.inner.set_device_handle(device_handle);
            self.inner.set_endpoint(endpoint.as_u8());
            self.inner.setup_write(data);
            self.inner.set_timeout(timeout);
            Transfer::WriteBulk(WriteTransfer{inner: self.inner})
        }
    }

    pub fn fill_interrupt_read<T: UsbContext>(
        mut self,
        device_handle: &DeviceHandle<T>,
        endpoint: ReadEndpoint,
        length: ReadLength,
        timeout: Timeout
    ) -> Transfer {
        unsafe {
            self.inner.set_transfer_type(TransferType::Interrupt);
            self.inner.set_device_handle(device_handle);
            self.inner.set_endpoint(endpoint.as_u8());
            self.inner.setup_read(length);
            self.inner.set_timeout(timeout);
            Transfer::ReadInterrupt(ReadTransfer{inner: self.inner})
        }
    }

    pub fn fill_interrupt_write<T: UsbContext>(
        mut self,
        device_handle: &DeviceHandle<T>,
        endpoint: WriteEndpoint,
        data: &WriteData,
        timeout: Timeout
    ) -> Transfer {
        unsafe {
            self.inner.set_transfer_type(TransferType::Interrupt);
            self.inner.set_device_handle(device_handle);
            self.inner.set_endpoint(endpoint.as_u8());
            self.inner.setup_write(data);
            self.inner.set_timeout(timeout);
            Transfer::WriteInterrupt(WriteTransfer{inner: self.inner})
        }
    }

    pub fn fill_control_read<T: UsbContext>(
        mut self,
        device_handle: &DeviceHandle<T>,
        setup: &ControlReadSetup,
        timeout: Timeout
    ) -> Transfer {
        unsafe {
            self.inner.set_transfer_type(TransferType::Control);
            self.inner.set_device_handle(device_handle);
            self.inner.set_endpoint(0);
            self.inner.setup_control_read(setup);
            self.inner.set_timeout(timeout);
            Transfer::ReadControl(ReadTransfer{inner: self.inner})
        }
    }

    pub fn fill_control_write<T: UsbContext>(
        mut self,
        device_handle: &DeviceHandle<T>,
        setup: &ControlWriteSetup,
        timeout: Timeout
    ) -> Transfer {
        unsafe {
            self.inner.set_transfer_type(TransferType::Control);
            self.inner.set_device_handle(device_handle);
            self.inner.set_endpoint(0);
            self.inner.setup_control_write(setup);
            self.inner.set_timeout(timeout);
            Transfer::WriteControl(WriteTransfer{inner: self.inner})
        }
    }
}

#[derive(Debug)]
pub struct ReadTransfer {
    inner: Internals,
}

impl ReadTransfer {
    pub fn clear(self) -> Transfer {
        Transfer::Unfilled(UnfilledTransfer{inner: self.inner})
    }

    pub fn clear_and_shrink(mut self) -> Transfer {
        unsafe { self.inner.shrink(); }
        Transfer::Unfilled(UnfilledTransfer{inner: self.inner})
    }
}

#[derive(Debug)]
pub struct WriteTransfer {
    inner: Internals,
}

impl WriteTransfer {
    pub fn clear(self) -> Transfer {
        Transfer::Unfilled(UnfilledTransfer{inner: self.inner})
    }

    pub fn clear_and_shrink(mut self) -> Transfer {
        unsafe { self.inner.shrink(); }
        Transfer::Unfilled(UnfilledTransfer{inner: self.inner})
    }
}

#[derive(Debug)]
struct Internals {
    handle: NonNull<libusb_transfer>,
    max_iso_packets: u16,
    buffer: Vec<u8>,
}

impl Internals {
    fn new_unfilled(max_iso_packets: u16) -> Self {
        let mut handle = unsafe {
            let ptr = libusb_alloc_transfer(max_iso_packets.into());
            NonNull::new(ptr).expect("alloc failed")
            // TODO: should this return a Result instead of panicking on OOM?
            // Probably not, since a transfer is small. If that fails we're
            // in real trouble.
        };

        unsafe { handle.as_mut().flags = CLEAR_TRANSFER_FLAGS; }

        // TODO:
        // transfer->user_data = user_data;
        // transfer->callback = callback;

        Self {handle, max_iso_packets, buffer: Vec::new()}
    }

    unsafe fn set_device_handle<T: UsbContext>(&mut self, dh: &DeviceHandle<T>) {
        let transfer = self.handle.as_mut();
        transfer.dev_handle = dh.as_raw();
    }

    unsafe fn set_endpoint(&mut self, endpoint: u8) {
        let transfer = self.handle.as_mut();
        transfer.endpoint = endpoint;
    }

    unsafe fn setup_read(&mut self, length: ReadLength) {
        self.buffer.resize(length.as_usize(), FILL_WITH_ZERO);

        let transfer = self.handle.as_mut();
        transfer.length = length.as_c_int();
        transfer.buffer = self.buffer.as_mut_ptr();
    }

    unsafe fn setup_write(&mut self, data: &WriteData) {
        self.buffer.resize(data.length(), FILL_WITH_ZERO);
        data.copy_data(&mut self.buffer[0..data.length()]);

        let transfer = self.handle.as_mut();
        transfer.length = data.length_as_c_int();
        transfer.buffer = self.buffer.as_mut_ptr();
    }

    unsafe fn setup_control_read(&mut self, setup: &ControlReadSetup) {
        self.buffer.resize(setup.total_length(), FILL_WITH_ZERO);
        setup.write_setup_packet(&mut self.buffer[0..CONTROL_SETUP_SIZE]);

        let transfer = self.handle.as_mut();
        transfer.length = setup.total_length_as_c_int();
        transfer.buffer = self.buffer.as_mut_ptr();
    }

    unsafe fn setup_control_write(&mut self, setup: &ControlWriteSetup) {
        self.buffer.resize(setup.total_length(), FILL_WITH_ZERO);
        setup.write_setup_packet(&mut self.buffer[0..CONTROL_SETUP_SIZE]);
        setup.copy_body_data(&mut self.buffer[CONTROL_SETUP_SIZE..]);

        let transfer = self.handle.as_mut();
        transfer.length = setup.total_length_as_c_int();
        transfer.buffer = self.buffer.as_mut_ptr();
    }

    unsafe fn set_timeout(&mut self, t: Timeout) {
        let transfer = self.handle.as_mut();
        transfer.timeout = t.as_c_uint();
    }

    unsafe fn set_transfer_type(&mut self, tt: TransferType) {
        let transfer = self.handle.as_mut();
        transfer.transfer_type = tt.as_raw();
    }

    unsafe fn shrink(&mut self) {
        self.buffer.truncate(0);
        self.buffer.shrink_to_fit();

        let transfer = self.handle.as_mut();
        transfer.length = 0;
        transfer.buffer = ptr::null_mut();
    }
}

impl Drop for Internals {
    fn drop(&mut self) {
        unsafe { libusb_free_transfer(self.handle.as_ptr()); }
    }
}


fn is_valid_control_data_length(data_length: usize) -> bool {
    if data_length > (u16::max_value() as usize) {
        return false;
    }

    let total_length = data_length + CONTROL_SETUP_SIZE;
    match u16::try_from(total_length) {
        Ok(_) => true,
        Err(_) => false,
    }
}
