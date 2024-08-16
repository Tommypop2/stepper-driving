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
use panic_probe as _;

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
static mut PIO0_IRQ_1_COUNT: u32 = 0;
static mut PIO1_IRQ_1_COUNT: u32 = 0;
#[entry]
fn main() -> ! {
    info!("Starting");
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    info!("1");
    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    info!("2");
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
    info!("3");
    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);
    info!("4");
    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    info!("4.5");
    unsafe {
        // pac::NVIC::unpend(pac::Interrupt::PIO0_IRQ_0);
        info!("4.6");
        // Code hangs at this point
        // pac::NVIC::unmask(pac::Interrupt::PIO0_IRQ_0);
        info!("4.7");
        pac::NVIC::unpend(pac::Interrupt::PIO0_IRQ_1);
        pac::NVIC::unmask(pac::Interrupt::PIO0_IRQ_1);
        info!("4.8");
        pac::NVIC::unpend(pac::Interrupt::PIO1_IRQ_1);
        pac::NVIC::unmask(pac::Interrupt::PIO1_IRQ_1);


        // pac::NVIC::unpend(pac::Interrupt::PIO1_IRQ_0);
        // pac::NVIC::unmask(pac::Interrupt::PIO1_IRQ_0);
        // pac::NVIC::unpend(pac::Interrupt::PIO1_IRQ_1);
        // pac::NVIC::unmask(pac::Interrupt::PIO1_IRQ_1);
    }
    info!("5");
    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();
    let program_with_defines = pio_proc::pio_asm!(
        "irq 0 rel",
        // "pull block"
        options(max_program_size = 32) // Optional, defaults to 32
    );
    info!("6");
    let program = program_with_defines.program;
    let (mut pio0, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let (mut pio1, sm02, sm12, _, _) = pac.PIO1.split(&mut pac.RESETS);
    info!("7");
    let installed = pio0.install(&program).unwrap();
    let installed1 = pio1.install(&program).unwrap();
    info!("7");
    let (mut sm1, _, mut _tx0) = hal::pio::PIOBuilder::from_installed_program(installed)
        // .set_pins(pio_pin_id, 1)
        // Tick every microsecond
        .clock_divisor_fixed_point(125, 0)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .build(sm1);
    info!("8");
    let (mut sm12, _, mut _tx1) = hal::pio::PIOBuilder::from_installed_program(installed1)
        // .set_pins(pio_pin_id, 1)
        // Tick every microsecond
        .clock_divisor_fixed_point(125, 0)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .build(sm12);
    info!("9");
    // pio0.irq0().enable_sm_interrupt(0);
    pio0.irq1().enable_sm_interrupt(1);
    pio1.irq1().enable_sm_interrupt(1);
    // Blink the LED at 1 Hz
    info!("Starting State Machines");
    delay.delay_ms(100);
    let sm1 = sm1.start();
    info!("9.5");
    let sm12 = sm12.start();
    delay.delay_ms(100);
    sm1.stop();
    sm12.stop();
    info!("10");
    info!("PIO0_IRQ_1_COUNT: {}", unsafe { PIO0_IRQ_1_COUNT });
    info!("PIO1_IRQ_1_COUNT: {}", unsafe { PIO1_IRQ_1_COUNT });
    info!("11");
    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
#[pac::interrupt]
unsafe fn PIO0_IRQ_1() {
    let pio = unsafe { &*pac::PIO0::ptr() };

    // Clear interrupt flag
    pio.irq().write_with_zero(|w| w.irq().bits(1 << 1));
    // info!("PIO0_IRQ_0");
    PIO0_IRQ_1_COUNT += 1;
}
#[pac::interrupt]
unsafe fn PIO1_IRQ_1() {
    let pio = unsafe { &*pac::PIO1::ptr() };

    // Clear interrupt flag
    pio.irq().write_with_zero(|w| w.irq().bits(1 << 1));
    // info!("PIO0_IRQ_1");
    PIO1_IRQ_1_COUNT += 1;
}
