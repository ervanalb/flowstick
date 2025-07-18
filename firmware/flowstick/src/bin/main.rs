#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{time, spi, delay, main, usb_serial_jtag::UsbSerialJtag};
use log::info;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    UsbSerialJtag::new(peripherals.USB_DEVICE);
    esp_println::logger::init_logger_from_env();

    let mut spi = spi::master::Spi::new(
        peripherals.SPI2,
        spi::master::Config::default()
            .with_frequency(time::Rate::from_khz(100))
            .with_mode(spi::Mode::_0),
    ).unwrap()
    .with_sck(peripherals.GPIO11).with_mosi(peripherals.GPIO12);

    let delay = delay::Delay::new();
    let mut h = 0;
    info!("Color fade!");
    loop {
        delay.delay_millis(10);
        let [r, g, b] = hsv2rgb([h, 255, 255]);
        spi.write(&[0x00, 0x00, 0x00, 0x00, 0xE1, b, g, r]).unwrap();
        h = h.wrapping_add(1);
    }
}

pub fn hsv2rgb(hsv: [u8; 3]) -> [u8; 3] {
    let [h, s, v] = hsv;
    let v: u16 = v as u16;
    let s: u16 = s as u16;
    let f: u16 = (h as u16 * 2 % 85) * 3;

    let p: u16 = v * (255 - s) / 255;
    let q: u16 = v * (255 - (s * f) / 255) / 255;
    let t: u16 = v * (255 - (s * (255 - f)) / 255) / 255;
    match h {
        0..=42 => [
            v as u8,
            t as u8,
            p as u8,
        ],
        43..=84 => [
            q as u8,
            v as u8,
            p as u8,
        ],
        85..=127 => [
            p as u8,
            v as u8,
            t as u8,
        ],
        128..=169 => [
            p as u8,
            q as u8,
            v as u8,
        ],
        170..=212 => [
            t as u8,
            p as u8,
            v as u8,
        ],
        213..=254 => [
            v as u8,
            p as u8,
            q as u8,
        ],
        255 => [
            v as u8,
            t as u8,
            p as u8,
        ],
    }
}
