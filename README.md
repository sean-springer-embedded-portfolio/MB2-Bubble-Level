# MB2-Bubble-Level

Copyright (c) 2026 Sean Springer

Transform the MB2 5x5 LED array into a digital **bubble level** using the `LSM303AGR` IMU

## User Interface (UI)

The **bubble level** has two resolution modes: Coarse and Fine. The **bubble level** will begin in Coarse mode where each LED represents
250mG worth of acceleration (see _How it Works_). Pressing the B btn will switch the level into Fine resolution mode where each LED represents
25mG work of acceleration (10x sensitivity improvement). The A btn can then be used to return to Coarse mode.

Additionally, if the board is upside-down (the LEDs are facing the ground) then the **bubble level** will temporarily disable until it has
been righted.

Lastly, the bubble refreshrate has been fixed at 200ms (5 frames per second) which provides an adequate update rate while minimizing jitter.

## Mechanics

Coarse mode divides each LED into 250mG bins while Fine mode is effectively 10x more sensitive (25mG per LED). The `TWIM` peripheral
on the MB2 handles the I2C communication protocol used to query the state of the accelerometer within the `LSM3030AGR`.

The GPIO tasks and events (`GPIOTE`) peripheral is used to connect the GPIO pin state of the A and B buttons to the Nested Vector Interrupt Controller `NVIC`.
Upon either button press, the `NVIC` will interupt the processor and call a custom built interrupt handler function. This handler function protects against
potential button bounce by using the MB2 `TIMER1` peripheral to apply a cooldown period of 100ms between interrupts.

## How it Works

I2C two-wire interface (`TWIM`) with EasyDMA is used to communicate with the `LSM3030AGR` accelerometer which is capable
of producing mG ([milli-Gauss](https://en.wikipedia.org/wiki/Gauss_(unit))) where 1000mG = 9.8m/sec^2 measures the acceleration
felt by the board in due to the acceleration of Earth's magnetic field. Becuase the accelerometer records this with respect to the
3-dimensional coordinate system, it can be used to mimic a bubble level.

## Build and Run

Assuming you have an attached MB2 with necessary permissions (see [Rust MB2 Discovery Book](https://docs.rust-embedded.org/discovery-mb2/))  
then this program can be `flashed` onto the MB2 nRF52820 using

```bash
cargo embed --release
```

## Sources

1. [Rust MB2 Discovery Book](https://docs.rust-embedded.org/discovery-mb2/)
2. [Rustdoc](https://doc.rust-lang.org/rustdoc/what-is-rustdoc.html)
3. Claude Sonnet 4.5 (free version)
4. nRF52833 Product Specification v1.6
5. MicroBit v2.21 Schematic
6. [Microbit Hal Docs](https://docs.rs/microbit/latest/microbit/hal/index.html)

## License

This program is licensed under the "MIT License". Please  
see the file `LICENSE` in the source distribution of this  
software for license terms.
