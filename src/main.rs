#![no_std]
#![no_main]

use core::cell::RefCell;
use core::ops::Add;
use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{GpioPin, Io},
    mcpwm::{
        operator::{PwmPin, PwmPinConfig},
        timer::PwmWorkingMode,
        McPwm, PeripheralClockConfig,
    },
    peripherals::{Peripherals, MCPWM0},
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
    alpha: 5.0,
    beta: 0.0,
};

fn calibrated(raw: ArmCoordinates) -> ArmCoordinates {
    raw + ARM_ZERO
}

#[derive(Debug, Copy, Clone, PartialEq)]
struct CartesianCoordinates {
    x: Distance,
    y: Distance,
    z: Distance,
}

fn euclidian_distance(x: &CartesianCoordinates) -> Distance {
    f32::sqrt(x.x * x.x + x.y * x.y + x.z * x.z)
}

fn solve(target: &CartesianCoordinates) -> ArmCoordinates {
    let epsilon = 1e-6; // Small value to avoid issues at the origin
    let r_raw = euclidian_distance(target);

    // define the unit lengths of the arms
    const L1: f32 = 0.5;
    const L2: f32 = 0.5;

    let r_max = L1 + L2 - epsilon;

    // if we're trying to go too far,
    let r = if r_raw < r_max {
        r_raw
    } else {
        println! {"{r_raw} too far, setting r = {r_max}"};
        r_max
    };

    // law of cosines for beta
    let beta = if r < epsilon {
        epsilon
    } else {
        f32::to_degrees(f32::acos((L1 * L1 + L2 * L2 - r * r) / (2.0 * L1 * L2)))
    };

    // law of cosines for alpha_base
    let alpha_base = if r < epsilon {
        epsilon
    } else {
        f32::to_degrees(f32::acos((L1 * L1 + r * r - L2 * L2) / (2.0 * r * L1)))
    };

    // simple trig for elevation and rotation angles
    let alpha_elev = if r < epsilon {
        epsilon
    } else {
        f32::to_degrees(f32::asin(target.z / r))
    };
    let phi_calculated = f32::to_degrees(f32::atan2(target.y, target.x));
    let phi = if f32::is_nan(phi_calculated) {
        epsilon
    } else {
        phi_calculated
    };

    // combine base and elevation angles for alpha
    let alpha = alpha_base + alpha_elev;

    ArmCoordinates { phi, alpha, beta }
}

enum Direction {
    Go,
    Return,
}

fn get_pulse(angle: Angle) -> u16 {
    let bounded_angle = if 0.0 > angle {
        println!("clamping {angle} to 0");
        0.0
    } else if 180.0 < angle {
        println!("clamping {angle} to 180");
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

struct Device<'a> {
    system: esp_hal::system::SystemControl<'a>,
}

static DEVICE: Mutex<RefCell<Option<Device>>> = Mutex::new(RefCell::new(None));

struct Joints<'a> {
    phi: PwmPin<'a, GpioPin<5>, MCPWM0, 2, true>,
    alpha: PwmPin<'a, GpioPin<6>, MCPWM0, 1, true>,
    beta: PwmPin<'a, GpioPin<7>, MCPWM0, 0, true>,
}

static JOINTS: Mutex<RefCell<Option<Joints>>> = Mutex::new(RefCell::new(None));
static DELAY: Mutex<RefCell<Option<Delay>>> = Mutex::new(RefCell::new(None));

fn init_system() {
    critical_section::with(|cs| {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);

        DEVICE.borrow_ref_mut(cs).replace(Device { system });

        let mut device_binding = DEVICE.borrow_ref_mut(cs);
        let device = device_binding.as_mut().unwrap();

        let clocks = ClockControl::boot_defaults(&mut device.system.clock_control).freeze();

        let delay = Delay::new(&clocks);
        DELAY.borrow_ref_mut(cs).replace(delay);

        let clock_cfg = PeripheralClockConfig::with_prescaler(&clocks, u8::MAX / 35);
        dbg!(clock_cfg.frequency());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let pin0 = io.pins.gpio7;
        let pin1 = io.pins.gpio6;
        let pin2 = io.pins.gpio5;

        let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);

        mcpwm.operator0.set_timer(&mcpwm.timer0);
        let beta = mcpwm
            .operator0
            .with_pin_a(pin0, PwmPinConfig::UP_ACTIVE_HIGH);

        mcpwm.operator1.set_timer(&mcpwm.timer1);
        let alpha = mcpwm
            .operator1
            .with_pin_a(pin1, PwmPinConfig::UP_ACTIVE_HIGH);

        mcpwm.operator2.set_timer(&mcpwm.timer2);
        let phi = mcpwm
            .operator2
            .with_pin_a(pin2, PwmPinConfig::UP_ACTIVE_HIGH);

        let timer_clock_cfg = clock_cfg
            .timer_clock_with_frequency(2000, PwmWorkingMode::Increase, 50.Hz())
            .unwrap();

        println!("timer frequency {fq}", fq = timer_clock_cfg.frequency());
        mcpwm.timer0.start(timer_clock_cfg);
        mcpwm.timer1.start(timer_clock_cfg);
        mcpwm.timer2.start(timer_clock_cfg);

        JOINTS
            .borrow_ref_mut(cs)
            .replace(Joints { phi, alpha, beta });
    });
}

fn set_timestamps(coordinates: &ArmCoordinates, direction: Direction) {
    let delay = critical_section::with(|cs| DELAY.borrow_ref(cs).unwrap());
    critical_section::with(|cs| {
        let mut joints_binding = JOINTS.borrow_ref_mut(cs);
        let joints = joints_binding.as_mut().unwrap();
        match direction {
            Direction::Go => {
                joints.beta.set_timestamp(get_pulse(coordinates.beta));
                delay.delay_millis(200);
                joints.alpha.set_timestamp(get_pulse(coordinates.alpha));
                joints.phi.set_timestamp(get_pulse(coordinates.phi));
            }

            Direction::Return => {
                joints.alpha.set_timestamp(get_pulse(coordinates.alpha));
                delay.delay_millis(200);
                joints.beta.set_timestamp(get_pulse(coordinates.beta));
                joints.phi.set_timestamp(get_pulse(coordinates.phi));
            }
        };
    });
}

fn travel(coordinates: &ArmCoordinates) {
    let safe_travel = calibrated(ArmCoordinates {
        phi: 0.0,
        alpha: 90.0,
        beta: 0.0,
    });
    let delay = critical_section::with(|cs| DELAY.borrow_ref(cs).unwrap());
    set_timestamps(&safe_travel, Direction::Return);
    delay.delay_millis(500);
    set_timestamps(coordinates, Direction::Go);
}

fn sweep_rom() {
    let delay = critical_section::with(|cs| DELAY.borrow_ref(cs).unwrap());
    let test_points = [
        CartesianCoordinates {
            x: 0.,
            y: 0.,
            z: 0.,
        },
        CartesianCoordinates {
            x: 0.99,
            y: 0.,
            z: 0.,
        },
        CartesianCoordinates {
            x: 100.,
            y: 0.,
            z: 0.,
        },
        CartesianCoordinates {
            x: 0.,
            y: 0.99,
            z: 0.,
        },
        CartesianCoordinates {
            x: 0.,
            y: 0.,
            z: 0.99,
        },
        CartesianCoordinates {
            x: 0.576,
            y: 0.576,
            z: 0.576,
        },
    ];
    for target_cartesian in test_points {
        delay.delay_millis(1_000);
        dbg!(target_cartesian);
        let target_arm = solve(&target_cartesian);
        dbg!(target_arm);
        travel(&calibrated(target_arm));
    }
}

#[entry]
fn main() -> ! {
    init_system();

    sweep_rom();
    panic!();
}
