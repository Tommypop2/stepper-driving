//! # Pico Blinky Example
//!
//! Blinks the LED on a Pico board.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![cfg_attr(not(test), no_std)]
#![no_main]

use cortex_m::asm::wfi;
use cortex_m::delay::Delay;
// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::digital::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use rp_pico::hal::gpio::FunctionSioOutput;
use rp_pico::hal::gpio::Pin;
use rp_pico::hal::gpio::PinId;
use rp_pico::hal::gpio::PullDown;
// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;
use stepgen;

enum Direction {
    Clockwise,
    AntiClockwise,
}
fn step<T: PinId>(delay: &mut Delay, step_pin: &mut Pin<T, FunctionSioOutput, PullDown>) {
    // Only the rising edge is active, so delay is mostly irrelevant here (apart from being above the minimum, which is about 20ns)
    step_pin.set_high().unwrap();
    delay.delay_us(1);
    step_pin.set_low().unwrap();
}
fn steps<T: PinId, U: PinId>(
    delay: &mut Delay,
    step_pin: &mut Pin<T, FunctionSioOutput, PullDown>,
    dir_pin: &mut Pin<U, FunctionSioOutput, PullDown>,
    steps: u32,
    step_delay: u32,
    dir: Direction,
) {
    match dir {
        Direction::Clockwise => dir_pin.set_high().unwrap(),
        Direction::AntiClockwise => dir_pin.set_low().unwrap(),
    }
    for _ in 0..steps {
        step(delay, step_pin);
        delay.delay_us(step_delay);
    }
}
static STEPS_PER_REV: u32 = 200 * 8;
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the LED to be an output
    let mut yellow_pin = pins.gpio9.into_push_pull_output();
    let mut red_pin = pins.gpio10.into_push_pull_output();
    let mut dir_pin = pins.gpio18.into_push_pull_output();
    let mut en_pin = pins.gpio17.into_push_pull_output();
    let mut step_pin = pins.gpio16.into_push_pull_output();
    // Blink the LED at 1 Hz
    en_pin.set_high().unwrap();
    dir_pin.set_low().unwrap();
    en_pin.set_low().unwrap();
    delay.delay_ms(5000);
    let mut stepper = stepgen::Stepgen::new(1_000_000);
    _ = stepper.set_acceleration(2000 << 8);
    _ = stepper.set_target_speed(10000 << 8);
    _ = stepper.set_target_step(STEPS_PER_REV * 40);
    while let Some(time) = stepper.next() {
        // for _ in 0..200 {
        delay.delay_us((time + 128) >> 8);
        step(&mut delay, &mut step_pin);
    }
    delay.delay_us(100);
    // Disable driver
    en_pin.set_high().unwrap();
    loop {
        // continue;
        wfi();

        // delay.delay_ms(1000);
        // yellow_pin.set_high().unwrap();
        // steps(
        //     &mut delay,
        //     &mut step_pin,
        //     &mut dir_pin,
        //     1000,
        //     100,
        //     Direction::Clockwise,
        // );
        // yellow_pin.set_low().unwrap();
        // delay.delay_ms(1000);
        // red_pin.set_high().unwrap();
        // steps(
        //     &mut delay,
        //     &mut step_pin,
        //     &mut dir_pin,
        //     1000,
        //     100,
        //     Direction::AntiClockwise,
        // );
        // red_pin.set_low().unwrap();

        // dir_pin.set_low().unwrap();
        // delay.delay_ms(1000);
        // yellow_pin.set_high().unwrap();
        // for _ in 0..steps {
        //     step_pin.set_high().unwrap();
        //     delay.delay_us(300);
        //     step_pin.set_low().unwrap();
        //     delay.delay_us(20);
        // }
        // yellow_pin.set_low().unwrap();
        // dir_pin.set_high().unwrap();
        // delay.delay_ms(1000);
        // red_pin.set_high().unwrap();
        // for _ in 0..steps {
        //     step_pin.set_high().unwrap();
        //     delay.delay_us(300);
        //     step_pin.set_low().unwrap();
        //     delay.delay_us(20);
        // }
        // red_pin.set_low().unwrap();

        // dir_pin.set_high().unwrap();
        // delay.delay_ms(1000);
        // for _ in 0..steps {
        //     step_pin.set_high().unwrap();
        //     delay.delay_us(100);
        //     step_pin.set_low().unwrap();
        //     delay.delay_us(20);
        // }
    }
}

// End of file
