#![no_std]
#![no_main]

use core::cell::RefCell;
use core::hash::{BuildHasher, Hasher};
use embassy_futures::{
    select::{select, Either},
    yield_now,
};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    analog, delay, dma, dma_circular_descriptors, gpio, i2s, peripherals, rtc_cntl, spi, system,
    time, timer, usb_serial_jtag, Blocking, Config,
};
use esp_println::println;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

esp_bootloader_esp_idf::esp_app_desc!();

pub const LED_COUNT: usize = 40;
pub const LED_DATA_LEADER_BYTES: usize = 4;
pub const LED_DATA_TRAILER_BYTES: usize = 4;
pub const FRAME_SIZE_BYTES: usize = LED_DATA_LEADER_BYTES + 4 * LED_COUNT + LED_DATA_TRAILER_BYTES;
pub const DMA_BUFFER_SIZE_FRAMES: usize = 120;

include!(concat!(env!("OUT_DIR"), "/patterns.rs"));
const PATTERN_COUNT: u8 = PATTERNS.len() as u8;

pub const IMAGE_DATA: &[u8] = include_bytes!("../test.data");

const COMPANY_ID: u16 = 0xFFFF;
const MY_ID: u16 = 0x218C;

const FLOWSTICK_DISABLE_SLEEP: bool = const {
    match option_env!("FLOWSTICK_DISABLE_SLEEP") {
        Some(x) => match x.as_bytes() {
            b"true" | b"1" => true,
            b"false" | b"0" => false,
            _ => panic!("FLOWSTICK_DISABLE_SLEEP must be a boolean value"),
        },
        None => false,
    }
};

const FLOWSTICK_POWER_ON: bool = const {
    match option_env!("FLOWSTICK_POWER_ON") {
        Some(x) => match x.as_bytes() {
            b"true" | b"1" => true,
            b"false" | b"0" => false,
            _ => panic!("FLOWSTICK_POWER_ON must be a boolean value"),
        },
        None => false,
    }
};

#[derive(Clone, Debug)]
pub struct GroupState {
    group_id: u16,
    group_private_id: [u8; 16],
    revision: u16,
    pattern: u8,
    hmac: [u8; 16],
}

impl PartialEq for GroupState {
    fn eq(&self, other: &GroupState) -> bool {
        self.revision == other.revision && self.pattern == other.pattern
    }
}
impl Eq for GroupState {}

impl GroupState {
    fn hmac(group_id: u16, group_private_id: [u8; 16], revision: u16, pattern: u8) -> [u8; 16] {
        let mut encoded = [0_u8; 32];

        let length = bincode::encode_into_slice(
            (group_id, group_private_id, revision, pattern),
            &mut encoded,
            bincode::config::standard(),
        )
        .unwrap();

        let mut hasher = rs_shake128::Shake128State::<16>::default().build_hasher();
        hasher.write(&encoded[..length]);
        let mut result = [0_u8; 16];
        result[0..8].copy_from_slice(&hasher.finish().to_le_bytes());
        result[8..16].copy_from_slice(&hasher.finish().to_le_bytes());
        result
    }

    pub fn new(group_id: u16, group_private_id: [u8; 16], revision: u16, pattern: u8) -> Self {
        Self {
            group_id,
            group_private_id,
            revision,
            pattern,
            hmac: Self::hmac(group_id, group_private_id, revision, pattern),
        }
    }

    pub fn deserialize_merge(&mut self, data: &[u8]) {
        // Validation checks
        let ((group_id, revision, pattern, hmac), length) =
            match bincode::decode_from_slice(data, bincode::config::standard()) {
                Ok(fields) => fields,
                Err(_) => {
                    return;
                }
            };

        if length != data.len() {
            return;
        }
        if group_id != self.group_id {
            return;
        }
        if (revision, pattern) <= (self.revision, self.pattern) {
            return;
        }

        let computed_hmac = Self::hmac(group_id, self.group_private_id, revision, pattern);

        if computed_hmac != hmac {
            return;
        }

        // Assign new values
        self.revision = revision;
        self.pattern = pattern;
        self.hmac = hmac;
    }

    pub fn update(&mut self, pattern: u8) {
        self.revision = self.revision.saturating_add(1);
        self.pattern = pattern;
        self.hmac = Self::hmac(
            self.group_id,
            self.group_private_id,
            self.revision,
            self.pattern,
        );
    }

    pub fn serialize(&self, data: &mut [u8]) -> usize {
        let length = bincode::encode_into_slice(
            (self.group_id, self.revision, self.pattern, self.hmac),
            data,
            bincode::config::standard(),
        )
        .unwrap();
        length
    }
}

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
                println!("LED DMA error: {:?}", e)
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
            time::Rate::from_hz(250000), // 2 MHz bit clock
            //time::Rate::from_hz(624999), // 5 MHz bit clock. There is a bug preventing use of
            // 625000, but this gives the same clock register values
            self.dma.reborrow(),
        );

        let (_, i2s_tx_descriptors) =
            dma_circular_descriptors!(FRAME_SIZE_BYTES * DMA_BUFFER_SIZE_FRAMES);
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
    button_press_timestamp: time::Duration,
    button_longpress_active: bool,
}

struct BleHardware {
    bt: peripherals::BT<'static>,
    radio: esp_radio::Controller<'static>,
}

impl BleHardware {
    async fn run(&mut self, group_state: &RefCell<GroupState>) {
        let connector = BleConnector::new(&self.radio, self.bt.reborrow());
        let controller: ExternalController<_, 20> = ExternalController::new(connector);

        // XXX Temporary: fixed random MAC address
        let address: Address = Address::random([0xff, 0x8f, 0x1b, 0x05, 0xe4, 0xff]);
        println!("Our BT address = {}", address);

        const CONNECTIONS_MAX: usize = 1;
        const L2CAP_CHANNELS_MAX: usize = 1;

        let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
            HostResources::new();
        let stack = trouble_host::new(controller, &mut resources).set_random_address(address);

        let Host {
            central,
            mut peripheral,
            mut runner,
            ..
        } = stack.build();

        let mut adv_data = [0; 64];
        let mut last_group_state = group_state.borrow().clone();
        // Flags
        adv_data[0..3].copy_from_slice(&[
            0x02,
            0x01,
            LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED,
        ]);
        // Manufacturer-specific data
        adv_data[3..5].copy_from_slice(&[0x00, 0xFF]);
        adv_data[5..7].copy_from_slice(&COMPANY_ID.to_le_bytes());
        adv_data[7..9].copy_from_slice(&MY_ID.to_le_bytes());
        let len = last_group_state.serialize(&mut adv_data[9..]);
        adv_data[3] = (len + 5) as u8;
        // Overall advertisement length
        let len = len + 9;

        let mut params = AdvertisementParameters::default();
        params.interval_min = Duration::from_millis(60);
        params.interval_max = Duration::from_millis(120);

        let adv_set = &[AdvertisementSet {
            data: Advertisement::ExtConnectableNonscannableUndirected {
                adv_data: &adv_data[..len],
            },
            params,
        }];

        let handles = &mut AdvertisementSet::handles(adv_set);

        let adv_listener = AdvListener {
            group_state: &group_state,
        };
        let mut scanner = Scanner::new(central);
        let result = select(runner.run_with_handler(&adv_listener), async move {
            // Set up advertising
            let _advertiser = peripheral.advertise_ext(adv_set, handles).await.unwrap();

            // Set up scan
            let mut config = ScanConfig::default();
            config.active = true;
            config.phys = PhySet::M1;
            config.interval = Duration::from_millis(150);
            config.window = Duration::from_millis(50);
            let mut _session = scanner.scan_ext(&config).await.unwrap();
            // Scan forever
            loop {
                // See if we need to update the advertising data
                if let Some(adv_set) = {
                    let group_state_ref = group_state.borrow();
                    (*group_state_ref != last_group_state).then(|| {
                        last_group_state = group_state_ref.clone();
                        drop(group_state_ref); // Don't allow borrow to persist into an .await
                        let len = last_group_state.serialize(&mut adv_data[9..]);
                        adv_data[3] = (len + 5) as u8;
                        // Overall advertisement length
                        let len = len + 9;

                        println!("Updating advertising data to: {:?}", &adv_data[..len]);

                        [AdvertisementSet {
                            data: Advertisement::ExtConnectableNonscannableUndirected {
                                adv_data: &adv_data[..len],
                            },
                            params,
                        }]
                    })
                } {
                    peripheral
                        .update_adv_data_ext(&adv_set, handles)
                        .await
                        .unwrap();
                }
                Timer::after(Duration::from_millis(10)).await;
            }
        })
        .await;
        panic!("Runner or infinite loop failed: {:?}", result);
    }
}

struct AdvListener<'a> {
    group_state: &'a RefCell<GroupState>,
}

impl<'a> EventHandler for AdvListener<'a> {
    fn on_ext_adv_reports(&self, mut it: LeExtAdvReportsIter<'_>) {
        while let Some(Ok(report)) = it.next() {
            for ad_structure in AdStructure::decode(report.data).flatten() {
                match ad_structure {
                    AdStructure::ManufacturerSpecificData {
                        company_identifier,
                        payload,
                    } => {
                        if company_identifier != COMPANY_ID || payload[0..2] != MY_ID.to_le_bytes()
                        {
                            continue;
                        }
                        let payload = &payload[2..];
                        self.group_state.borrow_mut().deserialize_merge(payload);
                    }
                    _ => {}
                }
            }
        }
    }
}

fn init() -> (ControlDriver, LedHardware, BleHardware) {
    esp_alloc::heap_allocator!(size: 72 * 1024);
    let peripherals = esp_hal::init(Config::default());
    usb_serial_jtag::UsbSerialJtag::new(peripherals.USB_DEVICE);

    let mut rtc = rtc_cntl::Rtc::new(peripherals.LPWR);

    // Watchdog
    rtc.rwdt.set_timeout(
        rtc_cntl::RwdtStage::Stage0,
        time::Duration::from_millis(5_000),
    );
    rtc.rwdt.enable();

    // Embassy
    let systimer = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    // Scheduler
    let timg0 = timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_radio_preempt_baremetal::init(timg0.timer0);
    let radio = esp_radio::init().unwrap();

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

    let button_press_timestamp = rtc.time_since_boot();

    // Persist longpresses over software resets.
    // Otherwise, holding the button down while the device is "on"
    // will cause it to switch off and then switch back on.
    let button_held_through_sw_reset = match system::reset_reason() {
        Some(rtc_cntl::SocResetReason::CoreSw) => button.is_high(),
        _ => false,
    };

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
            last_button_held: button_held_through_sw_reset,
            button_press_timestamp,
            button_longpress_active: button_held_through_sw_reset,
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

pub enum ButtonEvent {
    None,
    ShortPress,
    LongPress,
}

impl ControlDriver {
    pub fn hold_power_on(&mut self) {
        self.pwrhld.set_high();
    }

    pub fn release_power(&mut self) {
        self.pwrhld.set_low();
    }

    pub fn button_poll(&mut self) -> (bool, ButtonEvent) {
        let now = self.rtc.time_since_boot();

        let button_held = self.button.is_high();
        let button_pressed = button_held && !self.last_button_held;
        let button_released = !button_held && self.last_button_held;

        let mut long_press = false;
        if button_pressed {
            self.button_press_timestamp = now;
        } else if !self.button_longpress_active
            && button_held
            && now > self.button_press_timestamp + time::Duration::from_millis(1000)
        {
            long_press = true;
            self.button_longpress_active = true;
        }

        let button_event = if long_press {
            ButtonEvent::LongPress
        } else if button_released && !self.button_longpress_active {
            ButtonEvent::ShortPress
        } else {
            ButtonEvent::None
        };

        if !button_held {
            self.button_longpress_active = false;
        }
        self.last_button_held = button_held;

        (button_held, button_event)
    }

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

    pub fn sleep(&mut self, duration: time::Duration) {
        let timer = rtc_cntl::sleep::TimerWakeupSource::new(core::time::Duration::from_millis(
            duration.as_millis(),
        ));
        //let gpio = rtc_cntl::sleep::GpioWakeupSource::new();

        //self.rtc.sleep_light(&[&timer, &gpio]);
        self.rtc.sleep_deep(&[&timer]);
    }

    pub fn feed_wdt(&mut self) {
        self.rtc.rwdt.feed();
    }
}

pub struct PatternData {
    pub texture: &'static [u8],
}

pub struct PatternState {
    pub frame: usize,
}

impl PatternData {
    pub fn initial_state(&self) -> PatternState {
        PatternState { frame: 0 }
    }

    pub fn populate_led_frame(&self, state: &mut PatternState, buf: &mut [u8]) {
        for i in 0..LED_DATA_LEADER_BYTES {
            buf[i] = 0x00;
        }
        let image_data_frame =
            &self.texture[state.frame * 3 * LED_COUNT..(state.frame + 1) * 3 * LED_COUNT];
        for i in 0..LED_COUNT {
            buf[LED_DATA_LEADER_BYTES + 4 * i + 0] = 0xE1;
            buf[LED_DATA_LEADER_BYTES + 4 * i + 1] = image_data_frame[3 * i + 2];
            buf[LED_DATA_LEADER_BYTES + 4 * i + 2] = image_data_frame[3 * i + 1];
            buf[LED_DATA_LEADER_BYTES + 4 * i + 3] = image_data_frame[3 * i + 0];
        }
        state.frame += 1;
        if (state.frame + 1) * 3 * LED_COUNT > self.texture.len() {
            state.frame = 0;
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

    let rr: usize = system::reset_reason().unwrap() as usize;
    populate_frame_buffer(&mut led_bytes, |i| {
        if i < rr {
            (0x0F, 0x0F, 0x00)
        } else {
            (0x00, 0x00, 0x00)
        }
    });
    led_driver.write_bytes(&led_bytes); // Turn off LEDs // XXX
    delay::Delay::new().delay(time::Duration::from_millis(50));

    match system::reset_reason() {
        Some(rtc_cntl::SocResetReason::SysRtcWdt) => {
            // See if we reset due to WDT expiration. If so, show a red bar to indicate error
            // and pause for 2 seconds to allow debugger to connect

            populate_frame_buffer(&mut led_bytes, |_| (0xFF, 0x00, 0x00));
            led_driver.write_bytes(&led_bytes);
            for _ in 0..20 {
                delay::Delay::new().delay(time::Duration::from_millis(100));
                control_driver.feed_wdt();
            }
        }
        _ => {}
    };

    const FRAME_PERIOD: time::Duration = time::Duration::from_millis(2_000);
    let mut frame = 0;
    let now = control_driver.rtc.time_since_boot();
    let mut next_frame_timestamp = now + FRAME_PERIOD;

    // Keep awake for a brief period after charger is plugged in
    // to allow for debugger to attach
    let keep_awake_until = now + time::Duration::from_millis(0); // XXX

    loop {
        let (button_held, button_event) = control_driver.button_poll();
        match button_event {
            ButtonEvent::None => {}
            ButtonEvent::ShortPress => {}
            ButtonEvent::LongPress => {
                return; // Power On
            }
        }

        if button_held {
            control_driver.hold_power_on();
        } else {
            control_driver.release_power();
        }

        let usb_power = control_driver.usb_power();
        let charging = control_driver.charging();

        // Charging animation
        if usb_power {
            if charging {
                // Red
                populate_frame_buffer(&mut led_bytes, |i| {
                    if i == 0 && frame == 0 {
                        (0x0F, 0x00, 0x00)
                    } else {
                        (0x00, 0x00, 0x00)
                    }
                });
            } else {
                // Green
                populate_frame_buffer(&mut led_bytes, |i| {
                    if i == 0 {
                        (0x00, 0x0F, 0x00)
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

        let now = control_driver.rtc.time_since_boot();
        if now >= next_frame_timestamp {
            frame = (frame + 1) % 2;
            next_frame_timestamp = now + FRAME_PERIOD;
        }

        if next_frame_timestamp > now {
            let sleep_duration = next_frame_timestamp - now;
            if !FLOWSTICK_DISABLE_SLEEP
                && !button_held
                && now > keep_awake_until
                && sleep_duration > time::Duration::from_millis(1)
            {
                control_driver.sleep(sleep_duration);
            } else {
                // Busy-wait
                while control_driver.rtc.time_since_boot() < next_frame_timestamp
                    && !control_driver.button.is_high()
                {}
            }
        }

        control_driver.feed_wdt();
    }
}

async fn power_on_loop(
    control_driver: &mut ControlDriver,
    led_hardware: &mut LedHardware,
    ble_hardware: &mut BleHardware,
) {
    control_driver.hold_power_on();
    println!("Power on!");

    let mut led_driver = led_hardware.build_high_power();

    let group_state = RefCell::new(GroupState::new(
        0x0001,
        [
            0xAB, 0xCD, 0x12, 0x34, 0xAB, 0xCD, 0x12, 0x34, 0xAB, 0xCD, 0x12, 0x34, 0xAB, 0xCD,
            0x12, 0x34,
        ],
        0,
        0,
    ));

    let result = select(ble_hardware.run(&group_state), async {
        // Give 100ms or so for BT to initialize, which is very CPU-intensive,
        // to avoid late DMA feeding
        Timer::after(Duration::from_millis(100)).await;

        let mut led_dma = led_driver.begin_dma();

        let mut pattern_ix = 0_u8;
        let mut pattern = &PATTERNS[pattern_ix as usize];
        let mut pattern_state = pattern.initial_state();

        loop {
            let mut new_pattern = false;
            // State sync
            {
                let group_state = group_state.borrow();
                if group_state.pattern != pattern_ix && group_state.pattern < PATTERN_COUNT {
                    pattern_ix = group_state.pattern;
                    new_pattern = true;
                    println!("Sync pattern to {:?}", pattern_ix);
                }
            }

            // User input
            let (_, button_event) = control_driver.button_poll();
            match button_event {
                ButtonEvent::None => {}
                ButtonEvent::ShortPress => {
                    pattern_ix = (pattern_ix + 1) % PATTERN_COUNT;
                    {
                        let mut group_state = group_state.borrow_mut();
                        group_state.update(pattern_ix);
                    }
                    new_pattern = true;
                    println!("Advance pattern to {:?}", pattern_ix);
                }
                ButtonEvent::LongPress => {
                    control_driver.release_power();
                    println!("Power off, goodbye!");
                    return; // Power Off
                }
            }

            if new_pattern {
                pattern = &PATTERNS[pattern_ix as usize];
                pattern_state = pattern.initial_state();
            }

            // Animation
            led_dma.feed(|buf| pattern.populate_led_frame(&mut pattern_state, buf));
            control_driver.feed_wdt();
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

    if !FLOWSTICK_POWER_ON {
        power_off_loop(&mut control_driver, &mut led_hardware);
    }

    power_on_loop(&mut control_driver, &mut led_hardware, &mut ble_hardware).await;

    if !FLOWSTICK_DISABLE_SLEEP {
        // Normal behavior: turn off all LEDs and perform a system reset
        let mut led_driver = led_hardware.build_low_power();
        let mut led_bytes = [0_u8; FRAME_SIZE_BYTES];
        populate_frame_buffer(&mut led_bytes, |_| (0x00, 0x00, 0x00));
        led_driver.write_bytes(&led_bytes); // Turn off LEDs
        system::software_reset();
    } else {
        // Debug behavior: don't reset, just loop between power on and power off
        loop {
            power_off_loop(&mut control_driver, &mut led_hardware);
            power_on_loop(&mut control_driver, &mut led_hardware, &mut ble_hardware).await;
        }
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
