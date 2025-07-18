#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{delay::Delay, main, usb_serial_jtag::UsbSerialJtag};
use log::info;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);

    esp_println::logger::init_logger_from_env();

    let delay = Delay::new();
    loop {
        usb_serial.write(b"Hello world!\n").unwrap();
        delay.delay_millis(500);
        info!("Log world!");
        delay.delay_millis(500);
    }
}
