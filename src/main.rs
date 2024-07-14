#![no_std]
#![no_main]

use core::cell::RefCell;
use core::ops::{Add, Sub};
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

// coordinate math

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

impl Sub for ArmCoordinates {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            phi: self.phi - other.phi,
            alpha: self.alpha - other.alpha,
            beta: self.beta - other.beta,
        }
    }
}

const ARM_ZERO: ArmCoordinates = ArmCoordinates {
    phi: 0.,
    alpha: 5.,
    beta: 0.,
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

// hardware code

struct Device<'a> {
    system: esp_hal::system::SystemControl<'a>,
}

struct Joints<'a> {
    phi: PwmPin<'a, GpioPin<5>, MCPWM0, 2, true>,
    alpha: PwmPin<'a, GpioPin<6>, MCPWM0, 1, true>,
    beta: PwmPin<'a, GpioPin<7>, MCPWM0, 0, true>,
}

static DEVICE: Mutex<RefCell<Option<Device>>> = Mutex::new(RefCell::new(None));
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

// actual robot driver

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

    const ZERO_PULSE: i32 = 100;
    const MAX_PULSE: i32 = 250;

    let percentage = bounded_angle / 180.0;
    let pulse = f32::ceil((MAX_PULSE - ZERO_PULSE) as f32 * percentage) as i32;
    ZERO_PULSE as u16 + pulse as u16
}

// MUST BE CALLED WITH CALIBRATED COORDINATES
fn set_timestamps(coordinates: &ArmCoordinates) {
    critical_section::with(|cs| {
        let mut joints_binding = JOINTS.borrow_ref_mut(cs);
        let joints = joints_binding.as_mut().unwrap();
        joints.alpha.set_timestamp(get_pulse(coordinates.alpha));
        joints.beta.set_timestamp(get_pulse(coordinates.beta));
        joints.phi.set_timestamp(get_pulse(coordinates.phi));
    });
    set_current(*coordinates);
}

static CURRENT_COORDINATES: Mutex<RefCell<Option<ArmCoordinates>>> = Mutex::new(RefCell::new(None));

const DEGREES_PER_US: u32 = 250_000_000;
const FRAME_US: u32 = 100;

fn get_current() -> ArmCoordinates {
    critical_section::with(|cs| CURRENT_COORDINATES.borrow_ref(cs).unwrap())
}

fn set_current(coordinates: ArmCoordinates) {
    critical_section::with(|cs| CURRENT_COORDINATES.borrow_ref_mut(cs).replace(coordinates));
}

fn init_arm() {
    set_timestamps(&calibrated(ArmCoordinates {
        phi: 0.,
        alpha: 0.,
        beta: 0.,
    }));
}

fn step_until(end: ArmCoordinates) {
    let start = get_current();
    let delay = critical_section::with(|cs| DELAY.borrow_ref(cs).unwrap());

    let resolution = (DEGREES_PER_US / FRAME_US) as f32;
    let difference = end - start;

    if f32::abs(difference.phi) <= resolution
        && f32::abs(difference.alpha) <= resolution
        && f32::abs(difference.beta) <= resolution
    {
        set_timestamps(&end);
        delay.delay_micros(FRAME_US);
        return;
    };

    let mut modified = start.clone();

    if f32::abs(difference.phi) > resolution {
        if difference.phi > 0. {
            modified.phi += resolution;
        }
        if difference.phi < 0. {
            modified.phi -= resolution;
        }
    };

    if f32::abs(difference.alpha) > resolution {
        if difference.alpha > 0. {
            modified.alpha += resolution;
        }
        if difference.alpha < 0. {
            modified.alpha -= resolution;
        }
    };

    if f32::abs(difference.beta) > resolution {
        if difference.beta > 0. {
            modified.beta += resolution;
        }
        if difference.beta < 0. {
            modified.beta -= resolution;
        }
    };

    set_timestamps(&modified);
    delay.delay_micros(FRAME_US);

    step_until(end)
}

// MUST BE CALLED WITH CALIBRATED COORDINATES
fn travel_direct(coordinates: &ArmCoordinates, direction: Direction) {
    let start = get_current();

    match direction {
        Direction::Go => {
            // if we're Go ing, first set beta, then set alpha
            let intermediate = ArmCoordinates {
                phi: start.phi,
                alpha: start.alpha,
                beta: coordinates.beta,
            };
            step_until(intermediate);
            step_until(*coordinates);
        }
        Direction::Return => {
            // if we're Return ing, first set alpha, then set beta
            let intermediate = ArmCoordinates {
                phi: start.phi,
                alpha: coordinates.alpha,
                beta: start.beta,
            };
            step_until(intermediate);
            step_until(*coordinates);
        }
    };
}

fn travel(coordinates: &ArmCoordinates) {
    let safe_travel = &calibrated(ArmCoordinates {
        phi: 0.0,
        alpha: 90.0,
        beta: 0.0,
    });
    travel_direct(safe_travel, Direction::Return);
    travel_direct(&calibrated(*coordinates), Direction::Go);
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
        dbg!(target_cartesian);
        let target_arm = solve(&target_cartesian);
        dbg!(target_arm);
        travel(&target_arm);
        dbg!(get_current());
        let tmp = calibrated(target_arm);
        dbg!(get_pulse(tmp.phi));
        dbg!(get_pulse(tmp.alpha));
        dbg!(get_pulse(tmp.beta));
        panic!();
        delay.delay_millis(3_000);
    }
}

// main
#[entry]
fn main() -> ! {
    init_system();
    init_arm();
    sweep_rom();

    panic!();
}
