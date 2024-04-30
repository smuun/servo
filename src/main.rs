#![no_std]
#![no_main]

use core::f32;

use esp_backtrace as _;
use esp_hal::{
    analog::adc::{AdcCalLine, AdcConfig, Attenuation, ADC},
    clock::ClockControl,
    delay::Delay,
    gpio::IO,
    peripherals::{Peripherals, ADC1},
    prelude::*,
};
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let delay = Delay::new(&clocks);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    type AdcCal = AdcCalLine<ADC1>;
    let pot_pin = io.pins.gpio1.into_analog();
    let mut adc1_config = AdcConfig::<ADC1>::new();
    let mut adc1_pin =
        adc1_config.enable_pin_with_cal::<_, AdcCal>(pot_pin, Attenuation::Attenuation11dB);
    let mut adc1 = ADC::<ADC1>::new(peripherals.ADC1, adc1_config);

    let mut highest = 3.41;
    let mut lowest = 0.0;

    println!("calibration -- turn knob through its range and click to accept");

    let cal_pin = io.pins.gpio45.into_pull_down_input();

    let mut cal = false;

    let mut get_voltage = || nb::block!(adc1.read_oneshot(&mut adc1_pin)).unwrap() as f32 / 1000.00;
    while !cal {
        let voltage = get_voltage();
        cal = cal_pin.is_low();
        if voltage > highest {
            highest = voltage;
        }
        if voltage < lowest {
            lowest = voltage;
        }
        delay.delay_millis(20 as u32);
    }
    println!("cal high {highest} low {lowest}");

    let mut servo_pin = io.pins.gpio46.into_push_pull_output();
    // nanoseconds
    let period: u32 = 20_000_000;

    let mut running_average: f32 = 0.0;

    let mut get_on_off = || {
        let tmp_angle = ((get_voltage() - lowest) / (highest - lowest)).clamp(0.0, 1.0);
        let window = 5.0;
        running_average = (running_average * window + tmp_angle) / (window + 1.0);
        let angle = running_average;

        let zero = 0.025;
        let one_eighty = 0.125;
        let duty_cycle = angle * (one_eighty - zero) + zero;
        let on = (duty_cycle * period as f32) as u32;
        // println!("dc {duty_cycle} on {on} / period {period}");
        // delay.delay_millis(500);

        let off: u32 = period - on;
        // println!("ratio {ratio} \t dc {duty_cycle}", ratio = (on as f32 / period as f32));
        (on, off)
    };

    loop {
        let (on, off) = get_on_off();
        servo_pin.set_low();
        delay.delay_nanos(off);
        servo_pin.set_high();
        delay.delay_nanos(on);
    }
}
