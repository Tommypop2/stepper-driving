#![no_std]
#![no_main]

use defmt::info;
// The macro for our start-up function
use defmt_rtt as _;
use rp_pico::entry;
// GPIO traits
use embedded_hal::digital::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pac::interrupt;
use rp_pico::hal::pac;
// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
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
    unsafe {
        pac::NVIC::unpend(pac::Interrupt::PIO0_IRQ_0);
        pac::NVIC::unmask(pac::Interrupt::PIO0_IRQ_0);
        pac::NVIC::unpend(pac::Interrupt::PIO0_IRQ_1);
        pac::NVIC::unmask(pac::Interrupt::PIO0_IRQ_1);
        pac::NVIC::unpend(pac::Interrupt::PIO1_IRQ_0);
        pac::NVIC::unmask(pac::Interrupt::PIO1_IRQ_0);
        pac::NVIC::unpend(pac::Interrupt::PIO1_IRQ_1);
        pac::NVIC::unmask(pac::Interrupt::PIO1_IRQ_1);
    }
    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();
    let program_with_defines = pio_proc::pio_asm!(
        "irq 0 rel",
        "pull block"
        options(max_program_size = 32) // Optional, defaults to 32
    );
    let program = program_with_defines.program;
    let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    let installed1 = pio.install(&program).unwrap();
    let (mut sm0, _, mut tx0) = hal::pio::PIOBuilder::from_installed_program(installed)
        // .set_pins(pio_pin_id, 1)
        // Tick every microsecond
        .clock_divisor_fixed_point(125, 0)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .build(sm0);
    let (mut sm1, _, mut tx1) = hal::pio::PIOBuilder::from_installed_program(installed1)
        // .set_pins(pio_pin_id, 1)
        // Tick every microsecond
        .clock_divisor_fixed_point(125, 0)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .build(sm1);
    pio.irq0().enable_sm_interrupt(0);
    pio.irq1().enable_sm_interrupt(1);
    // Blink the LED at 1 Hz
    delay.delay_ms(100);
    sm0.start();
    sm1.start();
    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}
// End of file
#[pac::interrupt]
unsafe fn PIO0_IRQ_0() {
    info!("PIO0_IRQ_0");
    let pio = unsafe { &*pac::PIO0::ptr() };

    // Clear interrupt flag
    pio.irq().write_with_zero(|w| w.irq().bits(1 << 1));
}
#[pac::interrupt]
unsafe fn PIO0_IRQ_1() {
    info!("PIO0_IRQ_1");
    let pio = unsafe { &*pac::PIO0::ptr() };

    // Clear interrupt flag
    pio.irq().write_with_zero(|w| w.irq().bits(1 << 1));
}
