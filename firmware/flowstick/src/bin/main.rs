#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{time, spi, main, usb_serial_jtag::UsbSerialJtag, timer::timg, rng, delay, gpio};
//use log::info;
use esp_println::println;

use bleps::{
    Ble,
    HciConnector,
    ad_structure::{
        AdStructure,
        BR_EDR_NOT_SUPPORTED,
        LE_GENERAL_DISCOVERABLE,
        create_advertising_data,
    },
    gatt,
    attribute_server::AttributeServer,
};
use esp_wifi::ble::controller::BleConnector;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::heap_allocator!(size: 72 * 1024);
    UsbSerialJtag::new(peripherals.USB_DEVICE);
    esp_println::logger::init_logger_from_env();

    let mut pwrhld = gpio::Output::new(peripherals.GPIO11, gpio::Level::Low, gpio::OutputConfig::default());
    let mut button = gpio::Input::new(peripherals.GPIO10, gpio::InputConfig::default().with_pull(gpio::Pull::Down));

    let mut spi = spi::master::Spi::new(
        peripherals.SPI2,
        spi::master::Config::default()
            .with_frequency(time::Rate::from_khz(100))
            .with_mode(spi::Mode::_0),
    ).unwrap()
    .with_sck(peripherals.GPIO37).with_mosi(peripherals.GPIO21);

/*

    // Bluetooth
    let timg0 = timg::TimerGroup::new(peripherals.TIMG0);
    let esp_wifi_ctrl = esp_wifi::init(timg0.timer0, rng::Rng::new(peripherals.RNG)).unwrap();
    let mut bluetooth = peripherals.BT;

    let now = || time::Instant::now().duration_since_epoch().as_millis();
    loop {
        let connector = BleConnector::new(&esp_wifi_ctrl, bluetooth.reborrow());
        let hci = HciConnector::new(connector, now);
        let mut ble = Ble::new(&hci);

        println!("{:?}", ble.init());
        println!("{:?}", ble.cmd_set_le_advertising_parameters());
        println!(
            "{:?}",
            ble.cmd_set_le_advertising_data(
                create_advertising_data(&[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                    AdStructure::CompleteLocalName("Flowstick"),
                ])
                .unwrap()
            )
        );
        println!("{:?}", ble.cmd_set_le_advertise_enable(true));

        println!("started advertising");

        let mut rf = |_offset: usize, data: &mut [u8]| {
            data[..20].copy_from_slice(&b"Hello Bare-Metal BLE"[..]);
            17
        };
        let mut wf = |offset: usize, data: &[u8]| {
            println!("RECEIVED: {} {:?}", offset, data);
        };

        let mut wf2 = |offset: usize, data: &[u8]| {
            println!("RECEIVED: {} {:?}", offset, data);
        };

        let mut rf3 = |_offset: usize, data: &mut [u8]| {
            data[..5].copy_from_slice(&b"Hola!"[..]);
            5
        };
        let mut wf3 = |offset: usize, data: &[u8]| {
            println!("RECEIVED: Offset {}, data {:?}", offset, data);
        };

        gatt!([service {
            uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
            characteristics: [
                characteristic {
                    uuid: "937312e0-2354-11eb-9f10-fbc30a62cf38",
                    read: rf,
                    write: wf,
                },
                characteristic {
                    uuid: "957312e0-2354-11eb-9f10-fbc30a62cf38",
                    write: wf2,
                },
                characteristic {
                    name: "my_characteristic",
                    uuid: "987312e0-2354-11eb-9f10-fbc30a62cf38",
                    notify: true,
                    read: rf3,
                    write: wf3,
                },
            ],
        },]);

        let mut rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(
            &mut ble,
            &mut gatt_attributes,
            &mut rng,
        );

        loop {}
    }
*/

    let delay = delay::Delay::new();
    let mut h = 0;
    loop {
        delay.delay_millis(10);
        spi.write(&[0x00, 0x00, 0x00, 0x00]).unwrap();
        let mut h2 = h;
        for i in 0..40 {
            let [r, g, b] = hsv2rgb([h2, 255, 255]);
            spi.write(&[0xE1, b, g, r]).unwrap();
            h2 = h2.wrapping_add(3);
        }
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
