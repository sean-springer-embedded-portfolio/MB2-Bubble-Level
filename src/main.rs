#![no_main]
#![no_std]

use embedded_hal::digital::InputPin;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use cortex_m_rt::entry;
use microbit::{
    board::Board,
    display::blocking::Display,
    hal::{Timer, twim},
    pac::twim0::frequency::FREQUENCY_A,
};

use lsm303agr::{AccelMode, AccelOutputDataRate, Lsm303agr};

const DISPLAY_REFRESH_RATE_MS: u32 = 200;
const LED_SIZE: usize = 5;
type LEDState = [[u8; LED_SIZE]; LED_SIZE];

enum BubbleResolution {
    Coarse,
    Fine,
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

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let board = Board::take().unwrap();
    let mut display_timer = Timer::new(board.TIMER0);
    let mut display = Display::new(board.display_pins);
    let mut a_btn = board.buttons.button_a;
    let mut b_btn = board.buttons.button_b;

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

    loop {
        if a_btn.is_low().unwrap() {
            leds.set_mode(BubbleResolution::Coarse);
        } else if b_btn.is_low().unwrap() {
            leds.set_mode(BubbleResolution::Fine);
        }

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
