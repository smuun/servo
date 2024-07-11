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
use micromath::F32Ext;

type Angle = f32;
type Distance = f32;

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
    phi: 0.0,
    alpha: 0.0,
    beta: 0.0,
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

fn euclidian_distance(x: &CartesianCoordinates) -> Distance {
    f32::sqrt(x.x * x.x + x.y * x.y + x.z * x.z)
}

fn solve(target: CartesianCoordinates) -> ArmCoordinates {
    let r = euclidian_distance(&target);

    // calculate phi -> phi
    let phi = f32::asin(target.y / r);

    // calculate r -> beta + the neutral position of alpha
    // FIXME don't think this is correct
    let beta = r;
    let alpha_neutral = 0.0;

    println!("r: {} phi: {}  beta: {}", r, phi, beta);

    // calculate theta
    let theta = 0.0;

    // alpha is alpha_neutral + theta
    let alpha = alpha_neutral + theta;

    // subtract the calibrated zero
    ARM_ZERO + ArmCoordinates { phi, alpha, beta }
}

fn get_pulse(angle: Angle) -> u16 {
    let bounded_angle = if 0.0 > angle {
        0.0
    } else if 180.0 < angle {
        180.0
    } else {
        angle
    };

    const ZERO_PULSE: i32 = 50;
    const MAX_PULSE: i32 = 250;

    let percentage = bounded_angle / 180.0;
    let pulse = f32::ceil((MAX_PULSE - ZERO_PULSE) as f32 * percentage) as i32;
    (ZERO_PULSE + pulse) as u16
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

    let clock_cfg = PeripheralClockConfig::with_prescaler(&clocks, u8::MAX / 35);
    // let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 49.Hz()).unwrap();

    dbg!(clock_cfg.frequency());

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
        phi: 0.0,
        alpha: 0.0,
        beta: 0.0,
    };
    let ninety = ArmCoordinates {
        phi: 90.0,
        alpha: 90.0,
        beta: 90.0,
    };
    let max = ArmCoordinates {
        phi: 179.0,
        alpha: 179.0,
        beta: 179.0,
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
