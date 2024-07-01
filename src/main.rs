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

type Angle = i32;
type Distance = i32;

struct ArmCoordinates {
    phi: Angle,
    alpha: Angle,
    beta: Angle,
}

const ARM_ZERO: ArmCoordinates = ArmCoordinates {
    phi: 0,
    alpha: 10,
    beta: 0,
};

// unit distance is hundredths of arm length (70mm)
// so the total possible range is at a radius 200 from the base
struct CartesianCoordinates {
    x: Distance,
    y: Distance,
    z: Distance,
}

// integer square root
fn sqrt(x: i32) -> i32 {
    let mut result = 0;
    while (result + 1) * (result + 1) <= x {
        result += 1;
    }
    result
}

// power series for arcsin and arccos
fn arcsin(x: i32) -> i32 {
    let precision = 100;
    // 57 is rad to degree conversion
    // integer division -- TODO see if there is a better way
    (57 * ((x + i32::pow(x, 3)) * precision / 6 + (3 * i32::pow(x, 5) * precision) / 40))
        / precision
}

fn sin(x: i32) -> i32 {
    (x - i32::pow(x, 3) / i32::pow(57, 2) / 6 + i32::pow(x, 5) / 120 / i32::pow(57, 4)) / 57
}

fn euclidian_distance(x: &CartesianCoordinates) -> i32 {
    sqrt(x.x * x.x + x.y * x.y + x.z * x.z)
}

fn solve(target: CartesianCoordinates) -> ArmCoordinates {
    let r = euclidian_distance(&target);

    // calculate phi -> phi
    let phi = arcsin(target.y / r);

    // calculate r -> beta + the neutral position of alpha
    // FIXME don't think this is correct
    let beta = r / 100;

    println!("r: {} phi: {}  beta: {}", r, phi, beta);

    // calculate theta

    // alpha is alpha_neutral + theta

    // subtract the calibrated zero
    ARM_ZERO
}

fn get_pulse(angle: i32) -> u16 {
    if !(0..180).contains(&angle) {
        panic!("angle OOB");
    }

    let min = 50;
    let max = 250;

    let scaled_angle = angle * 1000;
    let percentage = (scaled_angle + 90) / 180;
    let res = min + ((max - min) * percentage) / 1000;
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

    let mut travel = |coordinates: ArmCoordinates| {
        phi.set_timestamp(get_pulse(coordinates.phi));
        alpha.set_timestamp(get_pulse(coordinates.alpha));
        beta.set_timestamp(get_pulse(coordinates.beta));
    };

    println!("ZERO");
    travel(ARM_ZERO);

    println!("testing math");
    solve(CartesianCoordinates {
        x: 100,
        y: 100,
        z: 100,
    });
    loop {}
}
