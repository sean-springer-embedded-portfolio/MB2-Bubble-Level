#![no_main]
#![no_std]

use embedded_hal::digital::InputPin;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use core::sync::atomic::{
    AtomicU8,
    Ordering::{Acquire, Release},
};

use cortex_m_rt::entry;
use microbit::{
    board::Board,
    display::blocking::Display,
    hal::{
        Timer, gpiote,
        pac::{Interrupt, NVIC, TIMER1, interrupt},
        twim,
    },
    pac::twim0::frequency::FREQUENCY_A,
};

use critical_section_lock_mut::LockMut;
use lsm303agr::{AccelMode, AccelOutputDataRate, Lsm303agr};

const DISPLAY_REFRESH_RATE_MS: u32 = 200;
const LED_SIZE: usize = 5;
type LEDState = [[u8; LED_SIZE]; LED_SIZE];

// 100ms at 1MHz count rate.
const DEBOUNCE_TIME: u32 = 100 * 1_000_000 / 1000;

static RESOLUTION: AtomicU8 = AtomicU8::new(BubbleResolution::Coarse as u8);
static GPIOTE_PERIPHERAL: LockMut<gpiote::Gpiote> = LockMut::new();
static DEBOUNCE_TIMER: LockMut<Timer<TIMER1>> = LockMut::new();

enum BubbleResolution {
    Coarse = 0,
    Fine = 1,
}

impl TryFrom<u8> for BubbleResolution {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(BubbleResolution::Coarse),
            1 => Ok(BubbleResolution::Fine),
            _ => Err(()),
        }
    }
}

impl Into<u8> for BubbleResolution {
    fn into(self) -> u8 {
        match self {
            BubbleResolution::Coarse => BubbleResolution::Coarse as u8,
            BubbleResolution::Fine => BubbleResolution::Fine as u8,
        }
    }
}

struct LEDs {
    state: LEDState,
    mode: BubbleResolution,
}

impl LEDs {
    const COARSE_DIVS: f32 = 250.0;
    const FINE_DIVS: f32 = 25.0;

    fn new() -> Self {
        LEDs {
            state: [[0u8; 5]; 5],
            mode: BubbleResolution::Coarse,
        }
    }

    fn clear(&mut self) {
        self.state = [[0u8; 5]; 5];
    }

    fn set_mode(&mut self, mode: BubbleResolution) {
        self.mode = mode;
    }

    fn round(number: f32) -> u32 {
        let mut integer: u32 = number as u32; //round down
        let remainder: f32 = number - (integer as f32); //get decimal remainder
        if remainder >= 0.5 {
            integer += 1;
        }
        integer
    }

    fn clamp(value: f32) -> usize {
        if value <= 0.0 {
            0
        } else if value >= (LED_SIZE - 1usize) as f32 {
            LED_SIZE - 1
        } else {
            LEDs::round(value) as usize
        }
    }

    fn update(&mut self, x: i32, y: i32, z: i32) -> LEDState {
        if z > 0 {
            self.clear();
        } else {
            let divs = match self.mode {
                BubbleResolution::Coarse => LEDs::COARSE_DIVS,
                BubbleResolution::Fine => LEDs::FINE_DIVS,
            };

            self.clear();

            let x_pix: f32 = (-x as f32) / divs + 2.0; //needs to be flipped for axis
            let y_pix: f32 = (y as f32) / divs + 2.0;
            let x_index = LEDs::clamp(y_pix);
            let y_index = LEDs::clamp(x_pix);

            self.state[x_index][y_index] = 1;
        }

        self.state
    }
}

#[interrupt]
fn GPIOTE() {
    let mut debounced = false;
    DEBOUNCE_TIMER.with_lock(|debounce_timer| {
        if debounce_timer.read() == 0 {
            debounced = true;
            debounce_timer.start(DEBOUNCE_TIME);
        }
    });

    GPIOTE_PERIPHERAL.with_lock(|gpiote| {
        if gpiote.channel0().is_event_triggered() {
            //A button press
            gpiote.channel0().reset_events();
            if debounced {
                RESOLUTION.store(BubbleResolution::Coarse as u8, Release);
            }
        } else if gpiote.channel1().is_event_triggered() {
            //B button press
            gpiote.channel1().reset_events();

            if debounced {
                RESOLUTION.store(BubbleResolution::Fine as u8, Release);
            }
        }
    });
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let board = Board::take().unwrap();
    let mut display_timer = Timer::new(board.TIMER0);
    let mut display = Display::new(board.display_pins);
    let a_btn = board.buttons.button_a.into_floating_input();
    let b_btn = board.buttons.button_b.into_floating_input();

    //setup GPIOTE for both button press interrupts
    let gpiote = gpiote::Gpiote::new(board.GPIOTE);
    let channel0 = gpiote.channel0();
    let channel1 = gpiote.channel1();
    channel0
        .input_pin(&a_btn.degrade())
        .hi_to_lo()
        .enable_interrupt();
    channel0.reset_events();
    channel1
        .input_pin(&b_btn.degrade())
        .hi_to_lo()
        .enable_interrupt();
    channel1.reset_events();

    GPIOTE_PERIPHERAL.init(gpiote);

    //setup debounce timer
    let mut debounce_timer = Timer::new(board.TIMER1);
    debounce_timer.disable_interrupt();
    debounce_timer.reset_event();
    DEBOUNCE_TIMER.init(debounce_timer);

    let i2c = { twim::Twim::new(board.TWIM0, board.i2c_internal.into(), FREQUENCY_A::K100) };
    let mut sensor = Lsm303agr::new_with_i2c(i2c);
    sensor.init().unwrap();
    sensor
        .set_accel_mode_and_odr(
            &mut display_timer,
            AccelMode::HighResolution,
            AccelOutputDataRate::Hz50,
        )
        .unwrap();

    let mut leds = LEDs::new();

    // Set up the NVIC to handle interrupts.
    unsafe { NVIC::unmask(Interrupt::GPIOTE) };
    NVIC::unpend(Interrupt::GPIOTE);

    loop {
        leds.set_mode(BubbleResolution::try_from(RESOLUTION.load(Acquire)).unwrap());

        if sensor.accel_status().unwrap().xyz_new_data() {
            let (x, y, z) = sensor.acceleration().unwrap().xyz_mg();
            //rprintln!("Acceleration: x {} y {} z {}", x, y, z);

            display.show(
                &mut display_timer,
                leds.update(x, y, z),
                DISPLAY_REFRESH_RATE_MS,
            );
        }
    }
}
