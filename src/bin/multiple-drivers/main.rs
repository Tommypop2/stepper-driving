#![no_std]
#![no_main]

use cortex_m::asm::wfi;
// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::digital::OutputPin;

use defmt::info;
use defmt_rtt as _;

// use panic_probe as _;
use panic_probe as _;
use rp_pico::hal::gpio::{DynPinId, FunctionPio0, FunctionSioOutput, Pin, PinId, PullDown};
// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pac::interrupt;
use rp_pico::hal::pac;
// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

static STEPS_PER_REV: u32 = 200 * 8;
struct MotorDriver<T: PinId, U: PinId, V: PinId> {
    step_pin: Pin<T, FunctionPio0, PullDown>,
    dir_pin: Pin<U, FunctionSioOutput, PullDown>,
    en_pin: Pin<V, FunctionSioOutput, PullDown>,
    step_generator: stepgen::Stepgen,
}

static mut DRIVER_1: Option<MotorDriver<DynPinId, DynPinId, DynPinId>> = None;
static mut DRIVER_2: Option<MotorDriver<DynPinId, DynPinId, DynPinId>> = None;
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
    let mut dir_pin = pins.gpio26.into_push_pull_output();
    let mut en_pin = pins.gpio28.into_push_pull_output();
    let step_pin: Pin<_, FunctionPio0, _> = pins.gpio27.into_function();
    let program_with_defines = pio_proc::pio_file!(
        "src/bin/with_pio/step.pio",
        // select_program("step"), // Optional if only one program in the file
        options(max_program_size = 32) // Optional, defaults to 32
    );
    unsafe {
        DRIVER_2 = Some(MotorDriver {
            step_pin: step_pin.into_dyn_pin(),
            dir_pin: dir_pin.into_dyn_pin(),
            en_pin: en_pin.into_dyn_pin(),
            step_generator: stepgen::Stepgen::new(1_000_000),
        });
        DRIVER_1 = Some(MotorDriver {
            step_pin: pins.gpio21.into_function().into_dyn_pin(),
            dir_pin: pins.gpio20.into_push_pull_output().into_dyn_pin(),
            en_pin: pins.gpio22.into_push_pull_output().into_dyn_pin(),
            step_generator: stepgen::Stepgen::new(1_000_000),
        })
    };
    let driver_1 = unsafe { DRIVER_1.as_mut().unwrap() };
    let driver_2 = unsafe { DRIVER_2.as_mut().unwrap() };
    let pio_pin_id = driver_1.step_pin.id().num;
    let pio_pin_id2 = driver_2.step_pin.id().num;
    let program = program_with_defines.program;

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let (mut pio1, sm02, _, _, _) = pac.PIO1.split(&mut pac.RESETS);
    let installed1 = pio0.install(&program).unwrap();
    let installed2 = pio1.install(&program).unwrap();
    let (mut sm0, rx, mut tx) = hal::pio::PIOBuilder::from_installed_program(installed1)
        .set_pins(pio_pin_id, 1)
        // Tick every microsecond
        .clock_divisor_fixed_point(125, 0)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .build(sm0);
    let (mut sm02, rx1, mut tx1) = hal::pio::PIOBuilder::from_installed_program(installed2)
        .set_pins(pio_pin_id2, 1)
        // Tick every microsecond
        .clock_divisor_fixed_point(125, 0)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .build(sm02);
    _ = driver_1.step_generator.set_acceleration(1000 << 8);
    _ = driver_1.step_generator.set_target_speed(8000 << 8);
    _ = driver_1.step_generator.set_target_step(STEPS_PER_REV * 65);

    _ = driver_2.step_generator.set_acceleration(1000 << 8);
    _ = driver_2.step_generator.set_target_speed(8000 << 8);
    _ = driver_2.step_generator.set_target_step(STEPS_PER_REV * 40);
    tx.write((driver_1.step_generator.next().unwrap() + 128) >> 8);
    tx1.write((driver_2.step_generator.next().unwrap() + 128) >> 8);
    sm0.set_pindirs([(pio_pin_id, hal::pio::PinDir::Output)]);
    sm02.set_pindirs([(pio_pin_id2, hal::pio::PinDir::Output)]);
    driver_1.en_pin.set_high().unwrap();
    driver_1.dir_pin.set_low().unwrap();
    driver_1.en_pin.set_low().unwrap();

    driver_2.en_pin.set_high().unwrap();
    delay.delay_ms(1000);
    driver_2.en_pin.set_low().unwrap();
    pio0.irq1().enable_sm_interrupt(1);
    pio1.irq1().enable_sm_interrupt(1);
    info!("Starting state machine in 3s");
    delay.delay_ms(5000);
    let sm = sm0.start();
    let sm1 = sm02.start();

    delay.delay_ms(100);
    info!("State machines started");
    // let _sm = sm.stop();

    // driver_1.en_pin.set_high().unwrap();
    info!("Busy looping!");
    loop {
        delay.delay_ms(1000);
    }
}

// End of file
static mut DELAY: u32 = 200;
static mut STEPS: u32 = 0;

// Interrupts
// Using interrupts here as they *should* be fast enough to keep up with the PIO re-requesting data.
// If not, then pre-generating 5 at a time and buffering could work.

// #[pac::interrupt]
// unsafe fn PIO0_IRQ_0() {
//     info!("IRQ 0 executing");
//     let pio = unsafe { &*pac::PIO0::ptr() };
//     pio.irq().write_with_zero(|w| w.irq().bits(1 << 1));
// }
#[pac::interrupt]
unsafe fn PIO0_IRQ_1() {
    // info!("IRQ 1 Executing");
    let pio = unsafe { &*pac::PIO0::ptr() };

    // Clear interrupt flag
    pio.irq().write_with_zero(|w| w.irq().bits(1 << 1));
    // Write next value to the TX FIFO
    if let Some(delay) = DRIVER_1.as_mut().unwrap().step_generator.next() {
        pio.txf(0).write(|w| w.bits((delay + 128) >> 8));
    }
}
#[pac::interrupt]
unsafe fn PIO1_IRQ_1() {
    let pio = unsafe { &*pac::PIO1::ptr() };
    // Clear interrupt flag
    pio.irq().write_with_zero(|w| w.irq().bits(1 << 1));
    // Write next value to the TX FIFO
    match DRIVER_2.as_mut() {
        Some(driver) => {
            if let Some(delay) = driver.step_generator.next() {
                pio.txf(0).write(|w| w.bits((delay + 128) >> 8));
            }
        }
        None => {
            info!("Driver 2 not initialised")
        }
    }
}
