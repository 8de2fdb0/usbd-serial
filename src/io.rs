use super::SerialPort;
use core::borrow::BorrowMut;
use usb_device::bus::UsbBus;

#[derive(Debug)]
pub struct Error(usb_device::UsbError);

impl From<usb_device::UsbError> for Error {
    fn from(e: usb_device::UsbError) -> Self {
        Self(e)
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self.0 {
            usb_device::UsbError::Unsupported => embedded_io::ErrorKind::Unsupported,
            usb_device::UsbError::BufferOverflow
            | usb_device::UsbError::EndpointOverflow
            | usb_device::UsbError::EndpointMemoryOverflow => embedded_io::ErrorKind::OutOfMemory,
            _ => embedded_io::ErrorKind::Other,
        }
    }
}

impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io::ErrorType
    for SerialPort<'_, Bus, RS, WS>
{
    type Error = Error;
}

impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io::Read
    for SerialPort<'_, Bus, RS, WS>
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            match self.read(buf) {
                // We are required by `embedded-io` to continue reading until at least one byte is
                // read.
                Ok(0) => {}
                Err(usb_device::UsbError::WouldBlock) => {}
                other => return Ok(other?),
            }
        }
    }
}

impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io::ReadReady
    for SerialPort<'_, Bus, RS, WS>
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.poll()?;
        Ok(self.read_buf.available_read() != 0)
    }
}

impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io::Write
    for SerialPort<'_, Bus, RS, WS>
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        loop {
            match self.write(buf) {
                // We are required by `embedded-io` to continue writing until at least one byte is
                // written.
                Ok(0) => {}
                Err(usb_device::UsbError::WouldBlock) => {}
                other => return Ok(other?),
            }
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush().map_err(From::from)
    }
}

impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io::WriteReady
    for SerialPort<'_, Bus, RS, WS>
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.write_buf.available_write() != 0)
    }
}

// embedded-io implementations for SerialReader
impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io::ErrorType
    for crate::SerialReader<'_, Bus, RS, WS>
{
    type Error = Error;
}

impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io::Read
    for crate::SerialReader<'_, Bus, RS, WS>
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            match self.read(buf) {
                // We are required by `embedded-io` to continue reading until at least one byte is
                // read.
                Ok(0) => {}
                Err(usb_device::UsbError::WouldBlock) => {}
                other => return Ok(other?),
            }
        }
    }
}

impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io::ReadReady
    for crate::SerialReader<'_, Bus, RS, WS>
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        unsafe {
            (*self.serial_port).poll()?;
            Ok((*self.serial_port).read_buf.available_read() != 0)
        }
    }
}

// embedded-io implementations for SerialWriter
impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io::ErrorType
    for crate::SerialWriter<'_, Bus, RS, WS>
{
    type Error = Error;
}

impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io::Write
    for crate::SerialWriter<'_, Bus, RS, WS>
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        loop {
            match self.write(buf) {
                // We are required by `embedded-io` to continue writing until at least one byte is
                // written.
                Ok(0) => {}
                Err(usb_device::UsbError::WouldBlock) => {}
                other => return Ok(other?),
            }
        }
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush().map_err(From::from)
    }
}

impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io::WriteReady
    for crate::SerialWriter<'_, Bus, RS, WS>
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        unsafe { Ok((*self.serial_port).write_buf.available_write() != 0) }
    }
}

// embedded-io-async implementations for SerialPort
#[cfg(feature = "async")]
impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io_async::Read
    for SerialPort<'_, Bus, RS, WS>
{
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // For USB serial, we don't have real async support since it's interrupt driven
        // So we just do a blocking read
        self.read(buf).map_err(From::from)
    }
}

#[cfg(feature = "async")]
impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io_async::Write
    for SerialPort<'_, Bus, RS, WS>
{
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write(buf).map_err(From::from)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush().map_err(From::from)
    }
}

// embedded-io-async implementations for SerialReader
#[cfg(feature = "async")]
impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io_async::Read
    for crate::SerialReader<'_, Bus, RS, WS>
{
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read(buf).map_err(From::from)
    }
}

// embedded-io-async implementations for SerialWriter
#[cfg(feature = "async")]
impl<Bus: UsbBus, RS: BorrowMut<[u8]>, WS: BorrowMut<[u8]>> embedded_io_async::Write
    for crate::SerialWriter<'_, Bus, RS, WS>
{
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write(buf).map_err(From::from)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush().map_err(From::from)
    }
}
