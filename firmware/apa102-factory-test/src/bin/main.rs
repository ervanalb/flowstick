#![no_std]
#![no_main]

use esp_hal::{delay::Delay, main, spi, time, usb_serial_jtag::UsbSerialJtag};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

const LED_COUNT: usize = 120;
const LED_DATA_LEADER_BYTES: usize = 4;
const LED_DATA_TRAILER_BYTES: usize = 4;

fn populate_frame_buffer<F: Fn(usize) -> (u8, u8, u8)>(buf: &mut [u8], f: F) {
    let buf: &mut [u8; LED_DATA_LEADER_BYTES + 4 * LED_COUNT + LED_DATA_TRAILER_BYTES] =
        buf.try_into().unwrap();

    for i in 0..LED_DATA_LEADER_BYTES {
        buf[i] = 0x00;
    }
    for i in 0..LED_COUNT {
        let (r, g, b) = f(i);
        buf[LED_DATA_LEADER_BYTES + 4 * i + 0] = 0xE1;
        buf[LED_DATA_LEADER_BYTES + 4 * i + 1] = b;
        buf[LED_DATA_LEADER_BYTES + 4 * i + 2] = g;
        buf[LED_DATA_LEADER_BYTES + 4 * i + 3] = r;
    }
    for i in 0..LED_DATA_TRAILER_BYTES {
        buf[LED_DATA_LEADER_BYTES + 4 * LED_COUNT + i] = 0xFF;
    }
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    UsbSerialJtag::new(peripherals.USB_DEVICE);

    let mut spi = spi::master::Spi::new(
        peripherals.SPI3,
        spi::master::Config::default()
            .with_frequency(time::Rate::from_khz(1000))
            .with_mode(spi::Mode::_0),
    )
    .unwrap()
    .with_mosi(peripherals.GPIO12)
    .with_sck(peripherals.GPIO13);

    let mut buf = [0_u8; LED_DATA_LEADER_BYTES + 4 * LED_COUNT + LED_DATA_TRAILER_BYTES];

    let delay = Delay::new();

    loop {
        populate_frame_buffer(&mut buf, |_| (20, 0, 0));
        spi.write(&buf).unwrap();
        delay.delay_millis(500);
        populate_frame_buffer(&mut buf, |_| (0, 20, 0));
        spi.write(&buf).unwrap();
        delay.delay_millis(500);
        populate_frame_buffer(&mut buf, |_| (0, 0, 20));
        spi.write(&buf).unwrap();
        delay.delay_millis(500);
        populate_frame_buffer(&mut buf, |_| (20, 10, 10));
        spi.write(&buf).unwrap();
        delay.delay_millis(500);
    }
}
