//! Main.rs
//! Copyright © 2026 Sean Springer
//! [This program is licensed under the "MIT License"]
//! Please see the file LICENSE in the source distribution of this software for license terms.
//!
//! Uses the Accelerometer within the LSM303AGR eCompass module to transform the 5x5 LED Array
//! Into a Bubble Level with selectable resolution settings using the A and B buttons.
//!
//! The level will begin in Coarse Mode whereby each LED representes a 250 mG step. Selecting the B
//! button will switch the level in Fine Mode whereby each LED represents a 25 mG step. Selecting A
//! will return to Coarse Mode. The LED Refresh rate is fixed at 200ms.
//!
//! Note, GPIOTE Interrupt handlers are used to manage the button press actions with debounce logic

#![no_main]
#![no_std]

use panic_rtt_target as _;
use rtt_target::rtt_init_print;

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

/// constant refresh rate
const DISPLAY_REFRESH_RATE_MS: u32 = 200;
/// MB2 LED grid is 5x5
const LED_SIZE: usize = 5;
/// conveience type def
type LEDState = [[u8; LED_SIZE]; LED_SIZE];

/// 100ms at 1MHz count rate.
const DEBOUNCE_TIME: u32 = 100 * 1_000_000 / 1000;

/// Global Mutable objects: Used inside interrupt handler
static RESOLUTION: AtomicU8 = AtomicU8::new(BubbleResolution::Coarse as u8);
static GPIOTE_PERIPHERAL: LockMut<gpiote::Gpiote> = LockMut::new();
static DEBOUNCE_TIMER: LockMut<Timer<TIMER1>> = LockMut::new();

/// BubbleResolution Enum
///
/// Used to define the bubble level resolution state
enum BubbleResolution {
    Coarse = 0,
    Fine = 1,
}

/// TryFrom<u8> implementation for BubbleResolution enum
///
/// defines the try_form trait for converting a u8 into either
/// BubbleResolution::Coarse or BubbleResolution::Fine else returns
/// a unit Error. This implementatoin appears to be the accepted way
/// to convert a integral type into a C-style Enum.
impl TryFrom<u8> for BubbleResolution {
    // return Error type is unit bc seems obvious enough
    type Error = ();

    // Returns unit error unless the u8 value is 0 | 1
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(BubbleResolution::Coarse),
            1 => Ok(BubbleResolution::Fine),
            _ => Err(()),
        }
    }
}

/// LEDs Struct
///
/// Manages the LED state by calculating the proper LED to be lit based upon accelerometer data.
/// Also retains the current BubbleResolution mode in mode variable.
struct LEDs {
    state: LEDState,
    mode: BubbleResolution,
}

/// Impl LEDs
///
/// implements functions for generating a new instance, clearing state,
/// setting the resolution mode, a custom-built round function, and updating the LED state
/// using the accelerometer x,y,z mG readings
impl LEDs {
    /// mG per LED in mode == BubbleResolution::Coarse
    const COARSE_DIVS: f32 = 250.0;
    /// mG per LED in mode == BubbleResolution::Fine
    const FINE_DIVS: f32 = 25.0;

    /// Generates a new LEDs struct instance with state set to all zeros (all LEDs off)
    /// and resolution set to BubbleResolution::Coarse
    fn new() -> Self {
        LEDs {
            state: [[0u8; 5]; 5],
            mode: BubbleResolution::Coarse,
        }
    }

    /// reset the current state to be all zeros (all LEDs off)
    fn clear(&mut self) {
        self.state = [[0u8; 5]; 5];
    }

    /// update the current BubbleResolution mode. Changing the resolution mode will change
    /// the mG per LED divisions.
    ///
    /// BubbleResolution::Coarse -> LEDs::COARSE_DIVS
    /// BubbleResolution::Fine -> LEDs::Fine_DIVS
    fn set_mode(&mut self, mode: BubbleResolution) {
        self.mode = mode;
    }

    /// custom #![no_std] rounding static method. Given the f32 input number, rounding is performed
    /// by casting the f32 to an i32 and then calcuting the decimal precision that was lost from
    /// that downcast. Rounds up if the decimal component is >= 0.5
    fn round(number: f32) -> i32 {
        let mut integer: i32 = number as i32; //round down
        let remainder: f32 = number - (integer as f32); //get decimal remainder
        if remainder >= 0.5 {
            integer += 1;
        }
        integer
    }

    /// Clamp extreme mG magnitudes to the edges of the board. If not outside the range of the level,
    /// invokes the LEDs::round static method to round the division to the nearest pixel
    fn clamp(value: f32) -> usize {
        if value <= 0.0 {
            0 //clamp off to the left or off the top
        } else if value >= (LED_SIZE - 1usize) as f32 {
            LED_SIZE - 1 //clamp off to the right or bottom
        } else {
            // cast to usize is safe bc negative values handled above
            LEDs::round(value) as usize
        }
    }

    /// update and return the LED lit state. If the board is upside-down (z > 0),
    /// returns a cleared state (all LEDs are off). Otherwise, will return a state with
    /// one and only one LED in the On (lit) state.
    ///
    /// Depending upon the resolution mode, transforms the mG accelerometer inputs into a
    /// pixel value which will be set to the On (lit) state.
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

/// GPIOTE Interrupt handler (nrf52833 Peripheral Vector Table Entry #6)
///
/// Handles interrupts originating from either the A or B btn press with anti-bouncing logic.
/// MB2 TIMER1 is used to implement a 100ms cooldown on interrupt handling in order to protect against
/// button bounce. Then, the sending button is determined by checking which GPIOTE channel triggered the event
/// (Channel 0 is attached to the A btn and Channel 1 is attached to the B button). The RESOLUTION atomic is
/// updated using the intergral representation of the appropriate BubblerResolution variant.
#[interrupt]
fn GPIOTE() {
    // check for bouncing using a 100ms timer based coolddown:
    let mut debounced = false;
    DEBOUNCE_TIMER.with_lock(|debounce_timer| {
        if debounce_timer.read() == 0 {
            debounced = true;
            debounce_timer.start(DEBOUNCE_TIME);
        }
    });

    // grab a mutable reference to the Gpiote instance, determine which button sent the signal,
    // reset the interrupt, and update the RESOULTION atomic if debounced timer as timed out
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

/// Entry point
///
/// Set up the peripherals to be used and initialize the GPIO Events to trigger on either button press
/// and pass into global Mutex handlers. TIMER 0 is dedicated to the display and TIMER 1 is used to protect
/// aginst button bounce.
///
/// Becuase the LSM303AGR is attached via I2C to the MCU, the TWIM0 peripheral is used to communicate with the
/// accelerometer within the LSM303AGR.
#[entry]
fn main() -> ! {
    rtt_init_print!();

    let board = Board::take().unwrap();

    // TIMER0 will be dedicated to the LED display
    let mut display_timer = Timer::new(board.TIMER0);
    let mut display = Display::new(board.display_pins);

    // ensure buttons are in Floating mode
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

    // initialize the I2C TWIN0 communication with the accelerometer registers w/in the LSM303AGR
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

    // new LED state structure
    let mut leds = LEDs::new();

    // Set up the NVIC to handle interrupts.
    unsafe { NVIC::unmask(Interrupt::GPIOTE) }; // allow NVIC to handle GPIOTE signals
    NVIC::unpend(Interrupt::GPIOTE); //clear any currently pending GPIOTE state

    loop {
        // read state from atomic RESOLUTION, casting to a BubbleResolution enum variant
        leds.set_mode(BubbleResolution::try_from(RESOLUTION.load(Acquire)).unwrap());

        // in accelerometer data is new, grab the mG normalized X,Y,Z data and pass it
        // into the LED::update method before rendering
        if sensor.accel_status().unwrap().xyz_new_data() {
            let (x, y, z) = sensor.acceleration().unwrap().xyz_mg();

            // send state to LEDs
            display.show(
                &mut display_timer,
                leds.update(x, y, z),
                DISPLAY_REFRESH_RATE_MS,
            );
        }
    }
}
