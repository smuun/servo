//! Uses timer0 and operator0 of the MCPWM0 peripheral to output a 50% duty
//! signal at 20 kHz.
//!
//! The signal will be output to the pin assigned to `pin`. (GPIO0)

//% CHIPS: esp32 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use embedded_svc::timer;
use esp_backtrace as _;
use esp_hal::{
    clock::{self, ClockControl}, delay::Delay, gpio::IO, mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, PeripheralClockConfig, MCPWM}, peripherals::Peripherals, prelude::*
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let delay = Delay::new(&clocks);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let pin = io.pins.gpio0;

    let clock_cfg = PeripheralClockConfig::with_prescaler(&clocks, u8::MAX);

    let mut mcpwm = MCPWM::new(peripherals.MCPWM0, clock_cfg);

    mcpwm.operator0.set_timer(&mcpwm.timer0);
    let mut pwm_pin = mcpwm
        .operator0
        .with_pin_a(pin, PwmPinConfig::UP_ACTIVE_HIGH);

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(200, PwmWorkingMode::Increase, 50.Hz()).unwrap();

    println!("timer frequency {fq}", fq=timer_clock_cfg.frequency());
    mcpwm.timer0.start(timer_clock_cfg);


    let min = 05;
    let zero = 15;
    let max = 25;
    loop {
        println!("min");
        pwm_pin.set_timestamp(min);
        delay.delay_millis(2000);
        println!("zero");
        pwm_pin.set_timestamp(zero);
        delay.delay_millis(2000);
        println!("max");
        pwm_pin.set_timestamp(max);
        delay.delay_millis(2000);
    }
}