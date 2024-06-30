#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::IO,
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, PeripheralClockConfig, MCPWM},
    peripherals::Peripherals,
    prelude::*,
};
use esp_println::println;

fn get_pulse(angle: i32) -> u16 {
    if angle > 180 || angle < 0 {
        panic!("angle OOB");
    }

    let min = 50;
    let max = 250;

    let scaled_angle = angle as i32 * 1000;
    let percentage = (scaled_angle + 90) / 180;
    let res = min + ((max - min) * percentage) / 1000;
    println!("{}", res);
    res as u16
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let delay = Delay::new(&clocks);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let pin0 = io.pins.gpio0;
    let pin1 = io.pins.gpio7;
    let pin2 = io.pins.gpio5;

    let clock_cfg = PeripheralClockConfig::with_prescaler(&clocks, u8::MAX);

    let mut mcpwm = MCPWM::new(peripherals.MCPWM0, clock_cfg);

    mcpwm.operator0.set_timer(&mcpwm.timer0);
    let mut beta = mcpwm
        .operator0
        .with_pin_a(pin0, PwmPinConfig::UP_ACTIVE_HIGH);

    mcpwm.operator1.set_timer(&mcpwm.timer1);
    let mut alpha = mcpwm
        .operator1
        .with_pin_a(pin1, PwmPinConfig::UP_ACTIVE_HIGH);

    mcpwm.operator2.set_timer(&mcpwm.timer2);
    let mut phi = mcpwm
        .operator2
        .with_pin_a(pin2, PwmPinConfig::UP_ACTIVE_HIGH);

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(2000, PwmWorkingMode::Increase, 50.Hz())
        .unwrap();

    println!("timer frequency {fq}", fq = timer_clock_cfg.frequency());
    mcpwm.timer0.start(timer_clock_cfg);
    mcpwm.timer1.start(timer_clock_cfg);
    mcpwm.timer2.start(timer_clock_cfg);

    loop {
        println!("min");
        phi.set_timestamp(get_pulse(0));
        delay.delay_millis(2000);
        println!("zero");
        phi.set_timestamp(get_pulse(90));
        delay.delay_millis(2000);
        println!("max");
        phi.set_timestamp(get_pulse(180));
        delay.delay_millis(2000);
    }
}
