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

type AdcCal = AdcCalLine<ADC1>;

fn setup() -> (
    IO,
    esp_hal::gpio::GpioPin<esp_hal::gpio::Analog, 1>,
    esp_hal::analog::adc::AdcPin<
        esp_hal::gpio::GpioPin<esp_hal::gpio::Analog, 1>,
        ADC1,
        AdcCalLine<ADC1>,
    >,
    ADC<'static, ADC1>,
    Delay,
) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut adc1_config = AdcConfig::<ADC1>::new();
    let mut adc1 = ADC::<ADC1>::new(peripherals.ADC1, adc1_config);
    let pot_pin = io.pins.gpio1.into_analog();
    let mut adc1_pin =
        adc1_config.enable_pin_with_cal::<_, AdcCal>(pot_pin, Attenuation::Attenuation11dB);
    (io, pot_pin, adc1_pin, adc1, delay)
}

fn calibrate(
    delay: &Delay,
    cal_pin: esp_hal::gpio::GpioPin<esp_hal::gpio::Input<esp_hal::gpio::PullDown>, 45>,
    get_voltage: &mut impl FnMut() -> f32,
) -> (f32, f32) {
    let mut highest = 3.41;
    let mut lowest = 0.0;

    println!("calibration -- turn knob through its range and click to accept");

    while cal_pin.is_low() {
        let voltage = get_voltage();
        if voltage > highest {
            highest = voltage;
        }
        if voltage < lowest {
            lowest = voltage;
        }
        delay.delay_millis(20 as u32);
    }
    println!("cal high {highest} low {lowest}");

    (highest, lowest)
}

fn get_on_off(
    get_voltage: &mut impl FnMut() -> f32,
    lowest: f32,
    highest: f32,
    running_average: f32,
    period: u32,
) -> (u32, u32, f32) {
    let tmp_angle = ((get_voltage() - lowest) / (highest - lowest)).clamp(0.0, 1.0);
    let window = 5.0;
    let r_avg = (running_average * window + tmp_angle) / (window + 1.0);
    let angle = running_average;

    let zero = 0.025;
    let one_eighty = 0.125;
    let duty_cycle = angle * (one_eighty - zero) + zero;
    let on = (duty_cycle * period as f32) as u32;

    let off: u32 = period - on;
    (on, off, r_avg)
}

#[entry]
fn main() -> ! {
    let (io, pot_pin, adc1_pin, adc1, delay) = setup();

    let mut get_voltage = || nb::block!(adc1.read_oneshot(&mut adc1_pin)).unwrap() as f32 / 1000.00;

    let cal_pin = io.pins.gpio45.into_pull_down_input();
    let (highest, lowest) = calibrate(&delay, cal_pin, &mut get_voltage);

    let mut servo_pin = io.pins.gpio46.into_push_pull_output();
    // nanoseconds
    let period: u32 = 20_000_000;

    let mut running_average: f32 = 0.0;

    loop {
        let (on, off, r_avg) =
            get_on_off(&mut get_voltage, lowest, highest, running_average, period);
        running_average = r_avg;
        servo_pin.set_low();
        delay.delay_nanos(off);
        servo_pin.set_high();
        delay.delay_nanos(on);
    }
}
