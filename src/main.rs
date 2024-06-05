#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::{ClockControl, Clocks}, delay::Delay, gpio::IO, mcpwm::{
        operator::{PwmPin, PwmPinConfig}, timer::PwmWorkingMode, PeripheralClockConfig, PwmPeripheral, MCPWM
    }, peripheral, peripherals::{Peripherals, MCPWM0, SYSTEM}, prelude::*, system::SystemParts
};
use esp_println::println;

fn get_pulse(angle: i32) -> u16 {
    if angle > 180 || angle < 0 {
        panic!("angle OOB");
    }

    let min = 050;
    let max = 250;

    let scaled_angle = angle as i32 * 1000;
    let percentage = (scaled_angle + 90) / 180;
    let res = min + ((max - min) * percentage) / 1000;
    res as u16
}

struct ESPSystem {
    delay: Delay,
    io: IO,
    clock_cfg: PeripheralClockConfig<'static>,
    mcpwm: MCPWM<'static, esp_hal::peripherals::MCPWM0, > 
}

fn init_system<'a>() -> ESPSystem {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let clock_cfg = PeripheralClockConfig::with_prescaler(&clocks, u8::MAX);
    let delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let mcpwm = MCPWM::new(peripherals.MCPWM0, clock_cfg);

    ESPSystem {
        delay,
        io,
        clock_cfg,
        mcpwm,
    }
}

#[entry]
fn main() -> ! {
    let mut system: ESPSystem = init_system();


    let pin = system.io.pins.gpio0;

    system.mcpwm.operator0.set_timer(&system.mcpwm.timer0);
    let mut pwm_pin = system.mcpwm
        .operator0
        .with_pin_a(pin, PwmPinConfig::UP_ACTIVE_HIGH);

    // actually this results in something like 52 hz -- does not matter for servos, but the angles will be slightly off.
    let timer_clock_cfg = system.clock_cfg
        .timer_clock_with_frequency(2000, PwmWorkingMode::Increase, 50.Hz())
        .unwrap();

    println!("timer frequency {fq}", fq = timer_clock_cfg.frequency());
    system.mcpwm.timer0.start(timer_clock_cfg);

    let mut i = 0;
    let mut countdown = false;
    loop {
        if i > 180 {
            countdown = !countdown;
            i = 0;

        }
        let angle = if countdown {180 - i} else {i};
        pwm_pin.set_timestamp(get_pulse(angle));
        i += 1;
        system.delay.delay_millis(5);
    }
}
