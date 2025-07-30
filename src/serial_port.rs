use crate::buffer::{Buffer, DefaultBufferStore};
use crate::cdc_acm::*;
use core::borrow::BorrowMut;
use core::slice;
use usb_device::class_prelude::*;
use usb_device::descriptor::lang_id::LangID;
use usb_device::Result;

/// USB (CDC-ACM) serial port with built-in buffering to implement stream-like behavior.
///
/// The RS and WS type arguments specify the storage for the read/write buffers, respectively. By
/// default an internal 128 byte buffer is used for both directions.
pub struct SerialPort<'a, B, RS = DefaultBufferStore, WS = DefaultBufferStore>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    inner: CdcAcmClass<'a, B>,
    pub(crate) read_buf: Buffer<RS>,
    pub(crate) write_buf: Buffer<WS>,
    write_state: WriteState,
}

/// If this many full size packets have been sent in a row, a short packet will be sent so that the
/// host sees the data in a timely manner.
const SHORT_PACKET_INTERVAL: usize = 10;

/// Keeps track of the type of the last written packet.
enum WriteState {
    /// No packets in-flight
    Idle,

    /// Short packet currently in-flight
    Short,

    /// Full packet current in-flight. A full packet must be followed by a short packet for the host
    /// OS to see the transaction. The data is the number of subsequent full packets sent so far. A
    /// short packet is forced every SHORT_PACKET_INTERVAL packets so that the OS sees data in a
    /// timely manner.
    Full(usize),
}

impl<'a, B> SerialPort<'a, B>
where
    B: UsbBus,
{
    /// Creates a new USB serial port with the provided UsbBus and 128 byte read/write buffers.
    pub fn new<'alloc: 'a>(
        alloc: &'alloc UsbBusAllocator<B>,
    ) -> SerialPort<'a, B, DefaultBufferStore, DefaultBufferStore> {
        Self::new_with_interface_names(alloc, None, None)
    }
    /// Same as SerialPort::new, but allows specifying the names of the interfaces
    pub fn new_with_interface_names<'alloc: 'a>(
        alloc: &'alloc UsbBusAllocator<B>,
        comm_if_name: Option<&'static str>,
        data_if_name: Option<&'static str>,
    ) -> SerialPort<'a, B, DefaultBufferStore, DefaultBufferStore> {
        SerialPort::new_with_store_and_interface_names(
            alloc,
            DefaultBufferStore::default(),
            DefaultBufferStore::default(),
            comm_if_name,
            data_if_name,
        )
    }
}

impl<'a, B, RS, WS> SerialPort<'a, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    /// Creates a new USB serial port with the provided UsbBus and buffer backing stores.
    pub fn new_with_store<'alloc: 'a>(
        alloc: &'alloc UsbBusAllocator<B>,
        read_store: RS,
        write_store: WS,
    ) -> SerialPort<'a, B, RS, WS> {
        Self::new_with_store_and_interface_names(alloc, read_store, write_store, None, None)
    }

    /// Creates a new USB serial port with the provided UsbBus and buffer backing stores.
    pub fn new_with_store_and_interface_names<'alloc: 'a>(
        alloc: &'alloc UsbBusAllocator<B>,
        read_store: RS,
        write_store: WS,
        comm_if_name: Option<&'static str>,
        data_if_name: Option<&'static str>,
    ) -> SerialPort<'a, B, RS, WS> {
        SerialPort {
            inner: CdcAcmClass::new_with_interface_names(alloc, 64, comm_if_name, data_if_name),
            read_buf: Buffer::new(read_store),
            write_buf: Buffer::new(write_store),
            write_state: WriteState::Idle,
        }
    }

    /// Gets the current line coding.
    pub fn line_coding(&self) -> &LineCoding {
        self.inner.line_coding()
    }

    /// Gets the DTR (data terminal ready) state
    pub fn dtr(&self) -> bool {
        self.inner.dtr()
    }

    /// Gets the RTS (request to send) state
    pub fn rts(&self) -> bool {
        self.inner.rts()
    }

    /// Writes bytes from `data` into the port and returns the number of bytes written.
    ///
    /// # Errors
    ///
    /// * [`WouldBlock`](usb_device::UsbError::WouldBlock) - No bytes could be written because the
    ///   buffers are full.
    ///
    /// Other errors from `usb-device` may also be propagated.
    pub fn write(&mut self, data: &[u8]) -> Result<usize> {
        let count = self.write_buf.write(data);

        match self.flush() {
            Ok(_) | Err(UsbError::WouldBlock) => {}
            Err(err) => {
                return Err(err);
            }
        };

        if count == 0 {
            Err(UsbError::WouldBlock)
        } else {
            Ok(count)
        }
    }

    /// Poll the endpoint and try to put them into the serial buffer.
    pub(crate) fn poll(&mut self) -> Result<()> {
        let Self {
            inner, read_buf, ..
        } = self;

        read_buf.write_all(inner.max_packet_size() as usize, |buf_data| {
            match inner.read_packet(buf_data) {
                Ok(c) => Ok(c),
                Err(UsbError::WouldBlock) => Ok(0),
                Err(err) => Err(err),
            }
        })?;

        Ok(())
    }

    /// Reads bytes from the port into `data` and returns the number of bytes read.
    ///
    /// # Errors
    ///
    /// * [`WouldBlock`](usb_device::UsbError::WouldBlock) - No bytes available for reading.
    ///
    /// Other errors from `usb-device` may also be propagated.
    pub fn read(&mut self, data: &mut [u8]) -> Result<usize> {
        // Try to read a packet from the endpoint and write it into the buffer if it fits. Propagate
        // errors except `WouldBlock`.
        self.poll()?;

        if self.read_buf.available_read() == 0 {
            // No data available for reading.
            return Err(UsbError::WouldBlock);
        }

        self.read_buf.read(data.len(), |buf_data| {
            data[..buf_data.len()].copy_from_slice(buf_data);

            Ok(buf_data.len())
        })
    }

    /// Sends as much as possible of the current write buffer. Returns `Ok` if all data that has
    /// been written has been completely written to hardware buffers `Err(WouldBlock)` if there is
    /// still data remaining, and other errors if there's an error sending data to the host. Note
    /// that even if this method returns `Ok`, data may still be in hardware buffers on either side.
    pub fn flush(&mut self) -> Result<()> {
        let buf = &mut self.write_buf;
        let inner = &mut self.inner;
        let write_state = &mut self.write_state;

        let full_count = match *write_state {
            WriteState::Full(c) => c,
            _ => 0,
        };

        if buf.available_read() > 0 {
            // There's data in the write_buf, so try to write that first.

            let max_write_size = if full_count >= SHORT_PACKET_INTERVAL {
                inner.max_packet_size() - 1
            } else {
                inner.max_packet_size()
            } as usize;

            buf.read(max_write_size, |buf_data| {
                // This may return WouldBlock which will be propagated.
                inner.write_packet(buf_data)?;

                *write_state = if buf_data.len() == inner.max_packet_size() as usize {
                    WriteState::Full(full_count + 1)
                } else {
                    WriteState::Short
                };

                Ok(buf_data.len())
            })?;

            Err(UsbError::WouldBlock)
        } else if full_count != 0 {
            // Write a ZLP to complete the transaction if there's nothing else to write and the last
            // packet was a full one. This may return WouldBlock which will be propagated.
            inner.write_packet(&[])?;

            *write_state = WriteState::Short;

            Err(UsbError::WouldBlock)
        } else {
            // No data left in writer_buf.

            *write_state = WriteState::Idle;

            Ok(())
        }
    }
}

impl<B, RS, WS> UsbClass<B> for SerialPort<'_, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        self.inner.get_configuration_descriptors(writer)
    }

    fn get_string(&self, index: StringIndex, lang_id: LangID) -> Option<&str> {
        self.inner.get_string(index, lang_id)
    }

    fn reset(&mut self) {
        self.inner.reset();
        self.read_buf.clear();
        self.write_buf.clear();
        self.write_state = WriteState::Idle;
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr == self.inner.write_ep().address() {
            self.flush().ok();
        }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        self.inner.control_in(xfer);
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        self.inner.control_out(xfer);
    }
}

impl<B, RS, WS> embedded_hal::serial::Write<u8> for SerialPort<'_, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    type Error = UsbError;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        match <SerialPort<'_, B, RS, WS>>::write(self, slice::from_ref(&word)) {
            Ok(0) | Err(UsbError::WouldBlock) => Err(nb::Error::WouldBlock),
            Ok(_) => Ok(()),
            Err(err) => Err(nb::Error::Other(err)),
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        match <SerialPort<'_, B, RS, WS>>::flush(self) {
            Err(UsbError::WouldBlock) => Err(nb::Error::WouldBlock),
            Ok(_) => Ok(()),
            Err(err) => Err(nb::Error::Other(err)),
        }
    }
}

impl<B, RS, WS> embedded_hal::serial::Read<u8> for SerialPort<'_, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    type Error = UsbError;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut buf: u8 = 0;

        match <SerialPort<'_, B, RS, WS>>::read(self, slice::from_mut(&mut buf)) {
            Ok(0) | Err(UsbError::WouldBlock) => Err(nb::Error::WouldBlock),
            Ok(_) => Ok(buf),
            Err(err) => Err(nb::Error::Other(err)),
        }
    }
}

/// Reader handle for split serial port access
pub struct SerialReader<'a, B, RS = DefaultBufferStore, WS = DefaultBufferStore>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    pub(crate) serial_port: *mut SerialPort<'a, B, RS, WS>,
    _phantom: core::marker::PhantomData<&'a ()>,
}

/// Writer handle for split serial port access
pub struct SerialWriter<'a, B, RS = DefaultBufferStore, WS = DefaultBufferStore>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    pub(crate) serial_port: *mut SerialPort<'a, B, RS, WS>,
    _phantom: core::marker::PhantomData<&'a ()>,
}

// Safety: SerialReader can be Send if the underlying SerialPort is Send
// because it only accesses the read buffer and read-related operations
unsafe impl<'a, B, RS, WS> Send for SerialReader<'a, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
    SerialPort<'a, B, RS, WS>: Send,
{
}

// Safety: SerialWriter can be Send if the underlying SerialPort is Send
// because it only accesses the write buffer and write-related operations
unsafe impl<'a, B, RS, WS> Send for SerialWriter<'a, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
    SerialPort<'a, B, RS, WS>: Send,
{
}

impl<'a, B, RS, WS> SerialPort<'a, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    /// Split the serial port into separate reader and writer handles.
    ///
    /// This allows concurrent reading and writing operations. The split is safe because
    /// the reader only accesses the read buffer and read-related USB operations,
    /// while the writer only accesses the write buffer and write-related USB operations.
    ///
    /// Note: This consumes the SerialPort. To get it back, both handles must be
    /// passed to `unsplit()`.
    pub fn split(mut self) -> (SerialReader<'a, B, RS, WS>, SerialWriter<'a, B, RS, WS>) {
        let serial_ptr = &mut self as *mut SerialPort<'a, B, RS, WS>;

        // Leak the SerialPort so it doesn't get dropped
        core::mem::forget(self);

        let reader = SerialReader {
            serial_port: serial_ptr,
            _phantom: core::marker::PhantomData,
        };

        let writer = SerialWriter {
            serial_port: serial_ptr,
            _phantom: core::marker::PhantomData,
        };

        (reader, writer)
    }

    /// Combine reader and writer handles back into a SerialPort
    pub fn unsplit(
        reader: SerialReader<'a, B, RS, WS>,
        writer: SerialWriter<'a, B, RS, WS>,
    ) -> Self {
        // Ensure both handles point to the same SerialPort
        assert_eq!(reader.serial_port, writer.serial_port);

        // Reconstruct the SerialPort from the pointer
        let serial_port = unsafe { core::ptr::read(reader.serial_port) };

        // Forget the handles so they don't try to drop the SerialPort
        let _ = core::mem::ManuallyDrop::new(reader);
        let _ = core::mem::ManuallyDrop::new(writer);

        serial_port
    }
}

impl<'a, B, RS, WS> SerialReader<'a, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    /// Reads bytes from the port into `data` and returns the number of bytes read.
    ///
    /// # Errors
    ///
    /// * [`WouldBlock`](usb_device::UsbError::WouldBlock) - No bytes available for reading.
    ///
    /// Other errors from `usb-device` may also be propagated.
    pub fn read(&mut self, data: &mut [u8]) -> Result<usize> {
        unsafe { (*self.serial_port).read(data) }
    }
}

impl<'a, B, RS, WS> SerialWriter<'a, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    /// Writes bytes from `data` into the port and returns the number of bytes written.
    ///
    /// # Errors
    ///
    /// * [`WouldBlock`](usb_device::UsbError::WouldBlock) - No bytes could be written because the
    ///   buffers are full.
    ///
    /// Other errors from `usb-device` may also be propagated.
    pub fn write(&mut self, data: &[u8]) -> Result<usize> {
        unsafe { (*self.serial_port).write(data) }
    }

    /// Flush the write buffer
    pub fn flush(&mut self) -> Result<()> {
        unsafe { (*self.serial_port).flush() }
    }
}

// Implement embedded_hal traits for the split types
impl<B, RS, WS> embedded_hal::serial::Read<u8> for SerialReader<'_, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    type Error = UsbError;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut buf: u8 = 0;

        match <SerialReader<'_, B, RS, WS>>::read(self, slice::from_mut(&mut buf)) {
            Ok(0) | Err(UsbError::WouldBlock) => Err(nb::Error::WouldBlock),
            Ok(_) => Ok(buf),
            Err(err) => Err(nb::Error::Other(err)),
        }
    }
}

impl<B, RS, WS> embedded_hal::serial::Write<u8> for SerialWriter<'_, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    type Error = UsbError;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        match <SerialWriter<'_, B, RS, WS>>::write(self, slice::from_ref(&word)) {
            Ok(0) | Err(UsbError::WouldBlock) => Err(nb::Error::WouldBlock),
            Ok(_) => Ok(()),
            Err(err) => Err(nb::Error::Other(err)),
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        match <SerialWriter<'_, B, RS, WS>>::flush(self) {
            Err(UsbError::WouldBlock) => Err(nb::Error::WouldBlock),
            Ok(_) => Ok(()),
            Err(err) => Err(nb::Error::Other(err)),
        }
    }
}

// UsbClass implementations for split types
impl<B, RS, WS> UsbClass<B> for SerialReader<'_, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        unsafe { (*self.serial_port).get_configuration_descriptors(writer) }
    }

    fn get_string(&self, index: StringIndex, lang_id: LangID) -> Option<&str> {
        unsafe { (*self.serial_port).get_string(index, lang_id) }
    }

    fn reset(&mut self) {
        unsafe { (*self.serial_port).reset() }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        unsafe { (*self.serial_port).endpoint_in_complete(addr) }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        unsafe { (*self.serial_port).control_in(xfer) }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        unsafe { (*self.serial_port).control_out(xfer) }
    }
}

impl<B, RS, WS> UsbClass<B> for SerialWriter<'_, B, RS, WS>
where
    B: UsbBus,
    RS: BorrowMut<[u8]>,
    WS: BorrowMut<[u8]>,
{
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        unsafe { (*self.serial_port).get_configuration_descriptors(writer) }
    }

    fn get_string(&self, index: StringIndex, lang_id: LangID) -> Option<&str> {
        unsafe { (*self.serial_port).get_string(index, lang_id) }
    }

    fn reset(&mut self) {
        unsafe { (*self.serial_port).reset() }
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        unsafe { (*self.serial_port).endpoint_in_complete(addr) }
    }

    fn control_in(&mut self, xfer: ControlIn<B>) {
        unsafe { (*self.serial_port).control_in(xfer) }
    }

    fn control_out(&mut self, xfer: ControlOut<B>) {
        unsafe { (*self.serial_port).control_out(xfer) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // This is a simple compile-time test to ensure the split functionality compiles
    // We can't test the actual USB functionality without a mock USB bus
    #[test]
    fn test_split_compiles() {
        // This test just ensures that the split/unsplit API compiles correctly
        // We can't actually test functionality without a real or mock USB bus

        // The test ensures the types are correctly defined and the methods exist
        #[allow(dead_code)]
        fn test_split_api<B: UsbBus>(serial: SerialPort<'_, B>) {
            let (reader, writer) = serial.split();
            let _serial_back = SerialPort::unsplit(reader, writer);
        }

        // Just ensure the function compiles - it won't actually run without a USB bus
        fn _compile_only() {
            // This function never runs, it just tests compilation
            unreachable!()
        }
    }

    #[test]
    fn test_split_types_implement_traits() {
        // Test that split types implement the expected traits
        // This is a compile-time test to ensure trait implementations exist

        #[allow(dead_code)]
        fn check_reader_traits<B: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>>(
            _reader: &SerialReader<'_, B, RS, WS>,
        ) {
            // Ensure SerialReader implements expected traits
            fn _check_usb_class<T: UsbClass<B>, B: UsbBus>(_: &T) {}
            fn _check_embedded_io_error<T: embedded_io::ErrorType>(_: &T) {}
            fn _check_embedded_io_read<T: embedded_io::Read>(_: &T) {}
            fn _check_embedded_io_read_ready<T: embedded_io::ReadReady>(_: &T) {}
            fn _check_embedded_hal_read<T: embedded_hal::serial::Read<u8>>(_: &T) {}

            // These checks would fail to compile if traits aren't implemented
            _check_usb_class(_reader);
            _check_embedded_io_error(_reader);
            _check_embedded_io_read(_reader);
            _check_embedded_io_read_ready(_reader);
            _check_embedded_hal_read(_reader);
        }

        #[allow(dead_code)]
        fn check_writer_traits<B: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>>(
            _writer: &SerialWriter<'_, B, RS, WS>,
        ) {
            // Ensure SerialWriter implements expected traits
            fn _check_usb_class<T: UsbClass<B>, B: UsbBus>(_: &T) {}
            fn _check_embedded_io_error<T: embedded_io::ErrorType>(_: &T) {}
            fn _check_embedded_io_write<T: embedded_io::Write>(_: &T) {}
            fn _check_embedded_io_write_ready<T: embedded_io::WriteReady>(_: &T) {}
            fn _check_embedded_hal_write<T: embedded_hal::serial::Write<u8>>(_: &T) {}

            // These checks would fail to compile if traits aren't implemented
            _check_usb_class(_writer);
            _check_embedded_io_error(_writer);
            _check_embedded_io_write(_writer);
            _check_embedded_io_write_ready(_writer);
            _check_embedded_hal_write(_writer);
        }

        // Compile-time test - function bodies are never executed
        fn _never_runs() -> ! {
            unreachable!()
        }
    }

    #[cfg(feature = "async")]
    #[test]
    fn test_async_traits_compile() {
        // Test that async traits are implemented when the async feature is enabled
        // This is a compile-time test to ensure async trait implementations exist

        #[allow(dead_code)]
        fn check_serial_port_async<B: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>>(
            _port: &SerialPort<'_, B, RS, WS>,
        ) {
            fn _check_async_read<T: embedded_io_async::Read>(_: &T) {}
            fn _check_async_write<T: embedded_io_async::Write>(_: &T) {}

            _check_async_read(_port);
            _check_async_write(_port);
        }

        #[allow(dead_code)]
        fn check_reader_async<B: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>>(
            _reader: &SerialReader<'_, B, RS, WS>,
        ) {
            fn _check_async_read<T: embedded_io_async::Read>(_: &T) {}
            _check_async_read(_reader);
        }

        #[allow(dead_code)]
        fn check_writer_async<B: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>>(
            _writer: &SerialWriter<'_, B, RS, WS>,
        ) {
            fn _check_async_write<T: embedded_io_async::Write>(_: &T) {}
            _check_async_write(_writer);
        }

        // Compile-time test - function bodies are never executed
        fn _never_runs() -> ! {
            unreachable!()
        }
    }

    #[test]
    fn test_split_types_are_send() {
        // Test that split types implement Send when the underlying SerialPort is Send
        // This is a compile-time test to ensure Send implementations exist

        #[allow(dead_code)]
        fn check_send<T: Send>(_: &T) {}

        #[allow(dead_code)]
        fn test_send_when_serial_port_is_send<B: UsbBus + Send, RS: BorrowMut<[u8]> + Send, WS: BorrowMut<[u8]> + Send>(
            _reader: &SerialReader<'static, B, RS, WS>,
            _writer: &SerialWriter<'static, B, RS, WS>,
        ) where
            SerialPort<'static, B, RS, WS>: Send,
        {
            // These will fail to compile if Send is not implemented
            check_send(_reader);
            check_send(_writer);
        }

        // Compile-time test - function bodies are never executed
        fn _never_runs() -> ! {
            unreachable!()
        }
    }
}
