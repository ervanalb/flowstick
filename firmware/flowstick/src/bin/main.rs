#![no_std]
#![no_main]

use core::cell::RefCell;
use embassy_futures::{
    select::{select, Either},
    yield_now,
};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    analog, delay, dma, dma_circular_descriptors, gpio, i2s, peripherals, rtc_cntl, spi, time,
    timer, usb_serial_jtag::UsbSerialJtag, Blocking,
};
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use heapless::Deque;
use log::{error, info};
use trouble_host::prelude::*;

esp_bootloader_esp_idf::esp_app_desc!();

pub const LED_COUNT: usize = 40;
pub const LED_DATA_LEADER_BYTES: usize = 4;
pub const LED_DATA_TRAILER_BYTES: usize = 4;
pub const FRAME_SIZE_BYTES: usize = LED_DATA_LEADER_BYTES + 4 * LED_COUNT + LED_DATA_TRAILER_BYTES;
pub const DMA_BUFFER_SIZE_FRAMES: usize = 120;

pub const IMAGE_DATA: &[u8] = include_bytes!("../test.data");
pub const IMAGE_DATA_FRAMES: usize = IMAGE_DATA.len() / (3 * 40);

struct LedDma<'d> {
    i2s_transfer: dma::DmaTransferTxCircular<'d, i2s::master::I2sTx<'d, Blocking>>,
}

struct LedDriverHighPower<'d> {
    i2s_tx: i2s::master::I2sTx<'d, Blocking>,
}

impl<'a> LedDriverHighPower<'a> {
    fn begin_dma(&'a mut self) -> LedDma<'a> {
        let buf = i2s_tx_buffer();
        let mut i2s_transfer = self.i2s_tx.write_dma_circular(buf).unwrap();

        // Initial fill
        i2s_transfer
            .push_with(|buf| {
                buf.fill(0x00);
                buf.len()
            })
            .unwrap();

        LedDma { i2s_transfer }
    }
}

impl LedDma<'_> {
    pub fn feed<F: FnMut(&mut [u8])>(&mut self, mut f: F) {
        let result = self.i2s_transfer.push_with(|buf| {
            let frames = buf.len() / FRAME_SIZE_BYTES;
            for i in 0..frames {
                f(&mut buf[i * FRAME_SIZE_BYTES..(i + 1) * FRAME_SIZE_BYTES]);
            }
            frames * FRAME_SIZE_BYTES
        });
        match result {
            Ok(_) => {}
            Err(e) => {
                error!("LED DMA error: {:?}", e)
            }
        }
    }
}

struct LedDriverLowPower<'d> {
    spi: spi::master::Spi<'d, Blocking>,
}

impl LedDriverLowPower<'_> {
    pub fn write_bytes(&mut self, frame: &[u8]) {
        self.spi.write(frame).unwrap();
    }
}

pub struct LedHardware {
    dat: peripherals::GPIO21<'static>,
    clk: peripherals::GPIO37<'static>,
    spi: peripherals::SPI3<'static>,
    i2s: peripherals::I2S0<'static>,
    dma: peripherals::DMA_CH0<'static>,
}

impl LedHardware {
    fn build_high_power(&mut self) -> LedDriverHighPower {
        let i2s = i2s::master::I2s::new(
            self.i2s.reborrow(),
            i2s::master::Standard::Philips,
            i2s::master::DataFormat::Data8Channel8,
            time::Rate::from_hz(624999), // 5 MHz bit clock. There is a bug preventing use of
            // 625000, but this gives the same clock register values
            self.dma.reborrow(),
        );

        let (_, i2s_tx_descriptors) = dma_circular_descriptors!(FRAME_SIZE_BYTES * DMA_BUFFER_SIZE_FRAMES);
        let i2s_tx = i2s
            .i2s_tx
            .with_bclk(self.clk.reborrow())
            .with_dout(self.dat.reborrow())
            .build(i2s_tx_descriptors); // There is currently no way to build a descriptor chain with chunk
                                        // size other than dma::CHUNK_SIZE

        LedDriverHighPower { i2s_tx }
    }

    fn build_low_power(&mut self) -> LedDriverLowPower {
        let spi = spi::master::Spi::new(
            self.spi.reborrow(),
            spi::master::Config::default()
                .with_frequency(time::Rate::from_khz(1000))
                .with_mode(spi::Mode::_0),
        )
        .unwrap()
        .with_mosi(self.dat.reborrow())
        .with_sck(self.clk.reborrow());

        LedDriverLowPower { spi }
    }
}

struct ControlDriver {
    pwrhld: gpio::Output<'static>,
    button: gpio::Input<'static>,
    vbus_sense: gpio::Input<'static>,
    chg_sense: gpio::Input<'static>,
    imu_spi: spi::master::Spi<'static, Blocking>,
    adc: analog::adc::Adc<'static, peripherals::ADC1<'static>, Blocking>,
    vref: analog::adc::AdcPin<peripherals::GPIO8<'static>, peripherals::ADC1<'static>>,
    batt_measure: analog::adc::AdcPin<peripherals::GPIO9<'static>, peripherals::ADC1<'static>>,
    rtc: rtc_cntl::Rtc<'static>,
    last_button_held: bool,
}

struct BleHardware {
    bt: peripherals::BT<'static>,
    radio: esp_radio::Controller<'static>,
}

impl BleHardware {
    async fn run(&mut self) {
        let connector = BleConnector::new(&self.radio, self.bt.reborrow());
        let controller: ExternalController<_, 20> = ExternalController::new(connector);

        // XXX Temporary: fixed random MAC address
        let address: Address = Address::random([0xff, 0x8f, 0x1b, 0x05, 0xe4, 0xff]);
        info!("Our BT address = {}", address);

        const CONNECTIONS_MAX: usize = 1;
        const L2CAP_CHANNELS_MAX: usize = 1;

        let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
            HostResources::new();
        let stack = trouble_host::new(controller, &mut resources).set_random_address(address);

        let Host {
            central,
            mut runner,
            ..
        } = stack.build();

        // SCAN
        let printer = Printer {};
        let mut scanner = Scanner::new(central);
        let result = select(runner.run_with_handler(&printer), async {
            let mut config = ScanConfig::default();
            config.active = true;
            config.phys = PhySet::M1;
            config.interval = Duration::from_millis(150);
            config.window = Duration::from_millis(50);
            let mut _session = scanner.scan(&config).await.unwrap();
            // Scan forever
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        })
        .await;
        panic!("Runner or infinite loop failed: {:?}", result);
    }
}

struct Printer {}

impl EventHandler for Printer {
    fn on_adv_reports(&self, mut it: LeAdvReportsIter<'_>) {
        while let Some(Ok(report)) = it.next() {
            info!("heard: {:?}", report);
        }
    }
}

fn init() -> (ControlDriver, LedHardware, BleHardware) {
    esp_alloc::heap_allocator!(size: 72 * 1024);
    let peripherals = esp_hal::init(esp_hal::Config::default());
    UsbSerialJtag::new(peripherals.USB_DEVICE);
    esp_println::logger::init_logger_from_env();

    let timg0 = timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_radio_preempt_baremetal::init(timg0.timer0);

    // Embassy
    let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    // Peripherals
    let pwrhld = gpio::Output::new(
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

    let mut button = gpio::Input::new(
        peripherals.GPIO10,
        gpio::InputConfig::default().with_pull(gpio::Pull::Down),
    );
    button
        .wakeup_enable(true, gpio::WakeEvent::HighLevel)
        .unwrap();

    let vbus_sense = gpio::Input::new(peripherals.GPIO38, gpio::InputConfig::default());
    let chg_sense = gpio::Input::new(
        peripherals.GPIO12,
        gpio::InputConfig::default().with_pull(gpio::Pull::Up),
    );

    // SPI for the accelerometer
    let imu_spi = spi::master::Spi::new(
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

    // ADC for battery
    let mut adc_config = analog::adc::AdcConfig::new();
    let vref = adc_config.enable_pin(peripherals.GPIO8, analog::adc::Attenuation::_11dB);
    let batt_measure = adc_config.enable_pin(peripherals.GPIO9, analog::adc::Attenuation::_11dB);
    let adc = analog::adc::Adc::new(peripherals.ADC1, adc_config);

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

    let rtc = rtc_cntl::Rtc::new(peripherals.LPWR);
    let radio = esp_radio::init().unwrap();

    (
        ControlDriver {
            pwrhld,
            button,
            vbus_sense,
            chg_sense,
            imu_spi,
            adc,
            vref,
            batt_measure,
            rtc,
            last_button_held: false,
        },
        LedHardware {
            dat: peripherals.GPIO21,
            clk: peripherals.GPIO37,
            spi: peripherals.SPI3,
            i2s: peripherals.I2S0,
            dma: peripherals.DMA_CH0,
        },
        BleHardware {
            bt: peripherals.BT,
            radio,
        },
    )
}

impl ControlDriver {
    pub fn hold_power_on(&mut self) {
        self.pwrhld.set_high();
    }

    pub fn release_power(&mut self) {
        self.pwrhld.set_low();
    }

    pub fn button_was_pressed(&mut self) -> bool {
        let button_held = self.button.is_high();
        let result = button_held && !self.last_button_held;
        self.last_button_held = button_held;
        result
    }

    /*
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
    */

    pub fn usb_power(&self) -> bool {
        self.vbus_sense.is_high()
    }

    pub fn charging(&self) -> bool {
        self.chg_sense.is_low()
    }

    pub fn read_batt_voltage(&mut self) -> u16 {
        let vref = self.adc.read_blocking(&mut self.vref);
        let batt_measure = self.adc.read_blocking(&mut self.batt_measure);
        println!("vref={:?}, batt_measure={:?}", vref, batt_measure);
        0
    }

    pub fn sleep(&mut self, duration: core::time::Duration) {
        let timer = rtc_cntl::sleep::TimerWakeupSource::new(duration);
        let gpio = rtc_cntl::sleep::GpioWakeupSource::new();

        const DISABLE_SLEEP: bool = const {
            match option_env!("FLOWSTICK_DISABLE_SLEEP") {
                Some(x) => match x.as_bytes() {
                    b"true" | b"1" => true,
                    b"false" | b"0" => false,
                    _ => panic!("FLOWSTICK_DISABLE_SLEEP must be a boolean value"),
                },
                None => false,
            }
        };

        if DISABLE_SLEEP {
            let target = self.rtc.time_since_boot()
                + time::Duration::from_millis(duration.as_millis() as u64);
            while self.rtc.time_since_boot() < target && !self.button.is_high() {}
        } else {
            self.rtc.sleep_light(&[&timer, &gpio]);
        }
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
                        hsv2rgb([((*offset >> 6) as u8).wrapping_add(i as u8), 255, 25]);
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

const I2S_TX_BUFFER_SIZE_BYTES: usize = FRAME_SIZE_BYTES * DMA_BUFFER_SIZE_FRAMES;
fn i2s_tx_buffer() -> &'static mut [u8; I2S_TX_BUFFER_SIZE_BYTES] {
    static mut I2S_TX_BUFFER: [u32; (I2S_TX_BUFFER_SIZE_BYTES + 3) / 4] =
        [0_u32; (I2S_TX_BUFFER_SIZE_BYTES + 3) / 4];
    #[allow(static_mut_refs)]
    unsafe {
        &mut *(I2S_TX_BUFFER.as_mut_ptr() as *mut [u8; I2S_TX_BUFFER_SIZE_BYTES])
    }
}

fn power_off_loop(control_driver: &mut ControlDriver, led_hardware: &mut LedHardware) {
    let mut led_driver = led_hardware.build_low_power();

    let mut led_bytes = [0_u8; FRAME_SIZE_BYTES];
    populate_frame_buffer(&mut led_bytes, |_| (0x00, 0x00, 0x00));
    led_driver.write_bytes(&led_bytes); // Turn off LEDs

    let mut frame = 0;

    loop {
        if control_driver.button_was_pressed() {
            return; // Power On
        }

        let usb_power = control_driver.usb_power();
        let charging = control_driver.charging();

        // Charging animation
        if usb_power {
            if charging {
                // Red
                populate_frame_buffer(&mut led_bytes, |i| {
                    if i == 0 && frame == 0 {
                        (0xFF, 0x00, 0x00)
                    } else {
                        (0x00, 0x00, 0x00)
                    }
                });
                frame = (frame + 1) % 2;
            } else {
                // Green
                populate_frame_buffer(&mut led_bytes, |i| {
                    if i == 0 {
                        (0x00, 0xFF, 0x00)
                    } else {
                        (0x00, 0x00, 0x00)
                    }
                });
            }
        } else {
            // Off
            populate_frame_buffer(&mut led_bytes, |_| (0x00, 0x00, 0x00));
        }
        led_driver.write_bytes(&led_bytes);

        control_driver.sleep(core::time::Duration::from_millis(1000));
    }
}

async fn power_on_loop(
    control_driver: &mut ControlDriver,
    led_hardware: &mut LedHardware,
    ble_hardware: &mut BleHardware,
) {
    control_driver.hold_power_on();
    info!("Power on!");

    let mut led_driver = led_hardware.build_high_power();

    let result = select(ble_hardware.run(), async {
        let mut led_dma = led_driver.begin_dma();

        let mut anim = Anim::Plasma { offset: 0 };

        loop {
            if control_driver.button_was_pressed() {
                control_driver.release_power();
                info!("Power off, goodbye!");
                return; // Power Off
            }

            // Animation
            led_dma.feed(|buf| anim.populate_led_frame(buf));
            yield_now().await;
        }
    })
    .await;

    match result {
        Either::First(_) => {
            panic!("BLE error");
        }
        Either::Second(_) => {}
    }
}

#[esp_hal_embassy::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    let (mut control_driver, mut led_hardware, mut ble_hardware) = init();

    if control_driver.button_was_pressed() {
        // Power-on happened due to button press
        power_on_loop(&mut control_driver, &mut led_hardware, &mut ble_hardware).await;
    // Go straight to power on loop
    } else {
        // Power-on happened due to USB plug in
        // 2 second period to allow debugger to be attached
        delay::Delay::new().delay(time::Duration::from_millis(2000));
    }

    loop {
        power_off_loop(&mut control_driver, &mut led_hardware);
        power_on_loop(&mut control_driver, &mut led_hardware, &mut ble_hardware).await;
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
        0..=42 => [v as u8, t as u8, p as u8],
        43..=84 => [q as u8, v as u8, p as u8],
        85..=127 => [p as u8, v as u8, t as u8],
        128..=169 => [p as u8, q as u8, v as u8],
        170..=212 => [t as u8, p as u8, v as u8],
        213..=254 => [v as u8, p as u8, q as u8],
        255 => [v as u8, t as u8, p as u8],
    }
}

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
