#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    dma, dma_circular_buffers, gpio, i2s, main, rng, spi, time, timer,
    usb_serial_jtag::UsbSerialJtag, Blocking,
};
//use log::info;
use esp_println::println;

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    attribute_server::AttributeServer,
    gatt, Ble, HciConnector,
};
use esp_wifi::ble::controller::BleConnector;

esp_bootloader_esp_idf::esp_app_desc!();

pub const LED_COUNT: usize = 40;
pub const LED_DATA_LEADER_BYTES: usize = 4;
pub const LED_DATA_TRAILER_BYTES: usize = 4;
pub const FRAME_SIZE_BYTES: usize = LED_DATA_LEADER_BYTES + 4 * LED_COUNT + LED_DATA_TRAILER_BYTES;

pub const IMAGE_DATA: &[u8] = include_bytes!("../test.data");
pub const IMAGE_DATA_FRAMES: usize = IMAGE_DATA.len() / (3 * 40);

pub struct Hardware<'d> {
    pwrhld: &'d mut gpio::Output<'static>,
    button: &'d gpio::Input<'static>,
    vbus_sense: &'d gpio::Input<'static>,
    chg_sense: &'d gpio::Input<'static>,
    i2s_transfer: dma::DmaTransferTxCircular<'d, i2s::master::I2sTx<'static, Blocking>>,
    spi: spi::master::Spi<'static, Blocking>,
}

pub fn with_hardware<T, F: FnOnce(Hardware) -> T>(f: F) -> T {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::heap_allocator!(size: 72 * 1024);
    UsbSerialJtag::new(peripherals.USB_DEVICE);
    esp_println::logger::init_logger_from_env();

    let mut pwrhld = gpio::Output::new(
        peripherals.GPIO11,
        gpio::Level::Low,
        gpio::OutputConfig::default(),
    );

    // BATT_MEASURE_EN
    // This line should be held low always
    gpio::Output::new(
        peripherals.GPIO18,
        gpio::Level::Low,
        gpio::OutputConfig::default(),
    );

    let button = gpio::Input::new(
        peripherals.GPIO10,
        gpio::InputConfig::default().with_pull(gpio::Pull::Down),
    );
    let vbus_sense = gpio::Input::new(peripherals.GPIO38, gpio::InputConfig::default());
    let chg_sense = gpio::Input::new(
        peripherals.GPIO12,
        gpio::InputConfig::default().with_pull(gpio::Pull::Up),
    );

    // I2S for writing out to the LEDs
    let (_, _, mut i2s_tx_buffer, i2s_tx_descriptors) =
        dma_circular_buffers!(FRAME_SIZE_BYTES * 64); // Chunk size must be dma::CHUNK_SIZE

    let i2s = i2s::master::I2s::new(
        peripherals.I2S0,
        i2s::master::Standard::Philips,
        i2s::master::DataFormat::Data8Channel8,
        time::Rate::from_hz(624999), // 5 MHz bit clock. There is a bug preventing use of
        // 625000, but this gives the same clock register values
        peripherals.DMA_CH0,
    );

    let mut i2s_tx = i2s
        .i2s_tx
        .with_bclk(peripherals.GPIO37)
        .with_dout(peripherals.GPIO21)
        .build(i2s_tx_descriptors); // There is currently no way to build a descriptor chain with chunk
                                    // size other than dma::CHUNK_SIZE

    let i2s_transfer = i2s_tx.write_dma_circular(&mut i2s_tx_buffer).unwrap();

    // SPI for the accelerometer
    let mut spi = spi::master::Spi::new(
        peripherals.SPI2,
        spi::master::Config::default()
            .with_frequency(time::Rate::from_khz(100))
            .with_mode(spi::Mode::_0),
    )
    .unwrap()
    .with_miso(peripherals.GPIO33)
    .with_mosi(peripherals.GPIO34)
    .with_sck(peripherals.GPIO35)
    .with_cs(peripherals.GPIO36);

    /*
    // Bluetooth
    let timg0 = timer::timg::TimerGroup::new(peripherals.TIMG0);
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
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        loop {}
    }
    */

    f(Hardware {
        pwrhld: &mut pwrhld,
        button: &button,
        vbus_sense: &vbus_sense,
        chg_sense: &chg_sense,
        i2s_transfer,
        spi,
    })
}

impl Hardware<'_> {
    pub fn hold_power_on(&mut self) {
        self.pwrhld.set_high();
    }

    pub fn release_power(&mut self) {
        self.pwrhld.set_low();
    }

    pub fn button_pressed(&self) -> bool {
        self.button.is_high()
    }

    pub fn fill_led_output_buf<F: FnMut(&mut [u8])>(&mut self, mut f: F) {
        self.i2s_transfer
            .push_with(|buf| {
                let frames = buf.len() / FRAME_SIZE_BYTES;
                for i in 0..frames {
                    f(&mut buf[i * FRAME_SIZE_BYTES..(i + 1) * FRAME_SIZE_BYTES]);
                }
                frames * FRAME_SIZE_BYTES
            })
            .unwrap();
    }

    pub fn usb_power(&self) -> bool {
        self.vbus_sense.is_high()
    }

    pub fn charging(&self) -> bool {
        self.chg_sense.is_low()
    }
}

#[derive(Clone, Debug)]
pub enum Anim {
    Off,
    RedBar,
    GreenBar,
    Plasma { offset: u16 },
    Image { frame: usize },
}

impl Anim {
    pub fn off(&mut self) {
        *self = Anim::Off;
    }
    pub fn red_bar(&mut self) {
        *self = Anim::RedBar;
    }
    pub fn green_bar(&mut self) {
        *self = Anim::GreenBar;
    }
    pub fn plasma(&mut self) {
        *self = Anim::Plasma { offset: 0 };
    }
    pub fn image(&mut self) {
        *self = Anim::Image { frame: 0 };
    }
    pub fn populate_led_frame(&mut self, buf: &mut [u8]) {
        for i in 0..LED_DATA_LEADER_BYTES {
            buf[i] = 0x00;
        }
        match self {
            Anim::Off => {
                for i in 0..LED_COUNT {
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 0] = 0xE1;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 1] = 0x00;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 2] = 0x00;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 3] = 0x00;
                }
            }
            Anim::RedBar => {
                for i in 0..LED_COUNT {
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 0] = 0xE1;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 1] = 0x00;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 2] = 0x00;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 3] = 0xFF;
                }
            }
            Anim::GreenBar => {
                for i in 0..LED_COUNT {
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 0] = 0xE1;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 1] = 0xFF;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 2] = 0x00;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 3] = 0x00;
                }
            }
            Anim::Plasma { offset } => {
                for i in 0..LED_COUNT {
                    let [r, g, b] =
                        hsv2rgb([((*offset >> 6) as u8).wrapping_add(i as u8), 255, 255]);
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 0] = 0xE1;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 1] = g;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 2] = b;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 3] = r;
                }
                *offset = offset.wrapping_add(1);
            }
            Anim::Image { frame } => {
                let image_data_frame =
                    &IMAGE_DATA[*frame * 3 * LED_COUNT..(*frame + 1) * 3 * LED_COUNT];
                for i in 0..LED_COUNT {
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 0] = 0xE1;
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 1] = image_data_frame[3 * i + 2];
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 2] = image_data_frame[3 * i + 1];
                    buf[LED_DATA_LEADER_BYTES + 4 * i + 3] = image_data_frame[3 * i + 0];
                }
                *frame = (*frame + 1) % IMAGE_DATA_FRAMES;
            }
        }
        for i in 0..LED_DATA_TRAILER_BYTES {
            buf[LED_DATA_LEADER_BYTES + 4 * LED_COUNT + i] = 0xFF;
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum PowerState {
    Off,
    On,
    UsbPower,
}

impl PowerState {
    pub fn transition_to_on(&mut self, hardware: &mut Hardware, anim: &mut Anim) {
        anim.image();

        hardware.hold_power_on();
        *self = PowerState::On;
    }

    pub fn transition_to_off(&mut self, hardware: &mut Hardware, anim: &mut Anim) {
        anim.off();

        hardware.release_power();
        *self = PowerState::Off;
    }

    pub fn transition_to_usb_power(&mut self, hardware: &mut Hardware) {
        hardware.release_power();
        *self = PowerState::UsbPower;
    }
}

#[main]
fn main() -> ! {
    with_hardware(|mut hardware| {
        let mut power_state = PowerState::Off;
        let mut anim = Anim::Off;
        let mut last_button_pressed = false;

        loop {
            let button_pressed = hardware.button_pressed();
            let usb_power = hardware.usb_power();
            let charging = hardware.charging();

            println!("Usb power: {}", usb_power);
            println!("Charging: {}", charging);

            // Handle state transitions
            match power_state {
                PowerState::Off => {
                    if button_pressed && !last_button_pressed {
                        power_state.transition_to_on(&mut hardware, &mut anim);
                    } else if usb_power {
                        power_state.transition_to_usb_power(&mut hardware);
                    }
                }
                PowerState::On => {
                    if button_pressed && !last_button_pressed {
                        if usb_power {
                            power_state.transition_to_usb_power(&mut hardware);
                        } else {
                            power_state.transition_to_off(&mut hardware, &mut anim);
                        };
                    }
                }
                PowerState::UsbPower => {
                    if button_pressed && !last_button_pressed {
                        power_state.transition_to_on(&mut hardware, &mut anim);
                    } else if !usb_power {
                        power_state.transition_to_off(&mut hardware, &mut anim);
                    }
                }
            }
            last_button_pressed = button_pressed;

            // Handle current state
            match power_state {
                PowerState::Off => {}
                PowerState::On => {}
                PowerState::UsbPower => {
                    if charging {
                        anim.red_bar();
                    } else {
                        anim.green_bar();
                    }
                }
            }

            // Update output buffer with animation data
            hardware.fill_led_output_buf(|buf| anim.populate_led_frame(buf));
        }
    })
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
        0..=42 => [v as u8, t as u8, p as u8],
        43..=84 => [q as u8, v as u8, p as u8],
        85..=127 => [p as u8, v as u8, t as u8],
        128..=169 => [p as u8, q as u8, v as u8],
        170..=212 => [t as u8, p as u8, v as u8],
        213..=254 => [v as u8, p as u8, q as u8],
        255 => [v as u8, t as u8, p as u8],
    }
}
