#![no_std]
#![no_main]

use core::ops::Add;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::Io,
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, McPwm, PeripheralClockConfig},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use esp_println::{dbg, println};

type Angle = i32;
type Distance = i32;

#[derive(Debug, Copy, Clone, PartialEq)]
struct ArmCoordinates {
    phi: Angle,
    alpha: Angle,
    beta: Angle,
}

impl Add for ArmCoordinates {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            phi: self.phi + other.phi,
            alpha: self.alpha + other.alpha,
            beta: self.beta + other.beta,
        }
    }
}

const ARM_ZERO: ArmCoordinates = ArmCoordinates {
    phi: 0,
    alpha: 10,
    beta: 0,
};

fn calibrated(raw: ArmCoordinates) -> ArmCoordinates {
    raw + ARM_ZERO
}

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
    let bounded_angle = if 0 > angle {
        0
    } else if 180 < angle {
        180
    } else {
        angle
    };

    let min = 50;
    let max = 250;

    let scaled_angle = bounded_angle * 1000;
    let percentage = (scaled_angle + 90) / 180;
    let res = min + ((max - min) * percentage) / 1000;
    res as u16
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let delay = Delay::new(&clocks);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let pin0 = io.pins.gpio6;
    let pin1 = io.pins.gpio7;
    let pin2 = io.pins.gpio5;

    let clock_cfg = PeripheralClockConfig::with_prescaler(&clocks, u8::MAX);

    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);

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

    let mut travel = |coordinates: &ArmCoordinates| {
        phi.set_timestamp(get_pulse(coordinates.phi));
        alpha.set_timestamp(get_pulse(coordinates.alpha));
        beta.set_timestamp(get_pulse(coordinates.beta));
    };

    let sleep = || {
        delay.delay_millis(1_000);
    };

    let zero = ArmCoordinates {
        phi: 0,
        alpha: 0,
        beta: 0,
    };
    let ninety = ArmCoordinates {
        phi: 90,
        alpha: 90,
        beta: 90,
    };
    let max = ArmCoordinates {
        phi: 179,
        alpha: 179,
        beta: 179,
    };

    loop {
        dbg!(calibrated(zero));
        travel(&calibrated(zero));
        sleep();
        dbg!(calibrated(ninety));
        travel(&calibrated(ninety));
        sleep();
        dbg!(calibrated(max));
        travel(&calibrated(max));
        sleep();
    }
}
