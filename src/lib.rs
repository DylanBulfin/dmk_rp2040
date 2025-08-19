#![no_std]

use core::default;

use dmk::{
    behavior::NoArgBehavior, controller::PinSet, layer::Layer, physical_layout::MAX_KEYS,
    state::State,
};
use embedded_hal::digital::{InputPin, OutputPin};
#[cfg(feature = "active_low")]
use rp2040_hal::gpio::PullUp;
#[cfg(feature = "active_high")]
use rp2040_hal::gpio::{FunctionSio, SioInput, SioOutput};
use rp2040_hal::{
    Timer,
    gpio::{
        DynPinId, FunctionNull, Pin, PinId, Pins, PullDown,
        bank0::{
            Gpio0, Gpio1, Gpio2, Gpio3, Gpio4, Gpio5, Gpio6, Gpio7, Gpio8, Gpio9, Gpio10, Gpio11,
            Gpio12, Gpio13, Gpio14, Gpio15, Gpio16, Gpio17, Gpio18, Gpio19, Gpio20, Gpio21, Gpio22,
            Gpio23, Gpio24, Gpio25, Gpio26, Gpio27, Gpio28, Gpio29,
        },
    },
};

pub struct ControllerState {
    timer: Timer,
}

pub struct PinsWrapper {
    active_high: bool,
    pins: MaybePins,
    input_pins: [Option<Pin<DynPinId, FunctionNull, PullDown>>; 30],
    #[cfg(feature = "active_high")]
    output_pins: [Option<Pin<DynPinId, FunctionSio<SioOutput>, PullDown>>; 30],
    #[cfg(feature = "active_low")]
    output_pins: [Option<Pin<DynPinId, FunctionSio<SioOutput>, PullDown>>; 30],
}

pub struct MaybePins {
    pub gpio0: Option<Pin<Gpio0, FunctionNull, PullDown>>,
    pub gpio1: Option<Pin<Gpio1, FunctionNull, PullDown>>,
    pub gpio2: Option<Pin<Gpio2, FunctionNull, PullDown>>,
    pub gpio3: Option<Pin<Gpio3, FunctionNull, PullDown>>,
    pub gpio4: Option<Pin<Gpio4, FunctionNull, PullDown>>,
    pub gpio5: Option<Pin<Gpio5, FunctionNull, PullDown>>,
    pub gpio6: Option<Pin<Gpio6, FunctionNull, PullDown>>,
    pub gpio7: Option<Pin<Gpio7, FunctionNull, PullDown>>,
    pub gpio8: Option<Pin<Gpio8, FunctionNull, PullDown>>,
    pub gpio9: Option<Pin<Gpio9, FunctionNull, PullDown>>,
    pub gpio10: Option<Pin<Gpio10, FunctionNull, PullDown>>,
    pub gpio11: Option<Pin<Gpio11, FunctionNull, PullDown>>,
    pub gpio12: Option<Pin<Gpio12, FunctionNull, PullDown>>,
    pub gpio13: Option<Pin<Gpio13, FunctionNull, PullDown>>,
    pub gpio14: Option<Pin<Gpio14, FunctionNull, PullDown>>,
    pub gpio15: Option<Pin<Gpio15, FunctionNull, PullDown>>,
    pub gpio16: Option<Pin<Gpio16, FunctionNull, PullDown>>,
    pub gpio17: Option<Pin<Gpio17, FunctionNull, PullDown>>,
    pub gpio18: Option<Pin<Gpio18, FunctionNull, PullDown>>,
    pub gpio19: Option<Pin<Gpio19, FunctionNull, PullDown>>,
    pub gpio20: Option<Pin<Gpio20, FunctionNull, PullDown>>,
    pub gpio21: Option<Pin<Gpio21, FunctionNull, PullDown>>,
    pub gpio22: Option<Pin<Gpio22, FunctionNull, PullDown>>,
    pub gpio23: Option<Pin<Gpio23, FunctionNull, PullDown>>,
    pub gpio24: Option<Pin<Gpio24, FunctionNull, PullDown>>,
    pub gpio25: Option<Pin<Gpio25, FunctionNull, PullDown>>,
    pub gpio26: Option<Pin<Gpio26, FunctionNull, PullDown>>,
    pub gpio27: Option<Pin<Gpio27, FunctionNull, PullDown>>,
    pub gpio28: Option<Pin<Gpio28, FunctionNull, PullDown>>,
    pub gpio29: Option<Pin<Gpio29, FunctionNull, PullDown>>,
}

impl From<Pins> for MaybePins {
    fn from(value: Pins) -> Self {
        Self {
            gpio0: Some(value.gpio0),
            gpio1: Some(value.gpio1),
            gpio2: Some(value.gpio2),
            gpio3: Some(value.gpio3),
            gpio4: Some(value.gpio4),
            gpio5: Some(value.gpio5),
            gpio6: Some(value.gpio6),
            gpio7: Some(value.gpio7),
            gpio8: Some(value.gpio8),
            gpio9: Some(value.gpio9),
            gpio10: Some(value.gpio10),
            gpio11: Some(value.gpio11),
            gpio12: Some(value.gpio12),
            gpio13: Some(value.gpio13),
            gpio14: Some(value.gpio14),
            gpio15: Some(value.gpio15),
            gpio16: Some(value.gpio16),
            gpio17: Some(value.gpio17),
            gpio18: Some(value.gpio18),
            gpio19: Some(value.gpio19),
            gpio20: Some(value.gpio20),
            gpio21: Some(value.gpio21),
            gpio22: Some(value.gpio22),
            gpio23: Some(value.gpio23),
            gpio24: Some(value.gpio24),
            gpio25: Some(value.gpio25),
            gpio26: Some(value.gpio26),
            gpio27: Some(value.gpio27),
            gpio28: Some(value.gpio28),
            gpio29: Some(value.gpio29),
        }
    }
}

macro_rules! match_pin {
    (input: $pin:ident, $pins:expr) => {
        match $pin {
            0 => $pins.gpio0.take().expect("Missing pin").into_dyn_pin(),
            1 => $pins.gpio1.take().expect("Missing pin").into_dyn_pin(),
            2 => $pins.gpio2.take().expect("Missing pin").into_dyn_pin(),
            3 => $pins.gpio3.take().expect("Missing pin").into_dyn_pin(),
            4 => $pins.gpio4.take().expect("Missing pin").into_dyn_pin(),
            5 => $pins.gpio5.take().expect("Missing pin").into_dyn_pin(),
            6 => $pins.gpio6.take().expect("Missing pin").into_dyn_pin(),
            7 => $pins.gpio7.take().expect("Missing pin").into_dyn_pin(),
            8 => $pins.gpio8.take().expect("Missing pin").into_dyn_pin(),
            9 => $pins.gpio9.take().expect("Missing pin").into_dyn_pin(),
            10 => $pins.gpio10.take().expect("Missing pin").into_dyn_pin(),
            11 => $pins.gpio11.take().expect("Missing pin").into_dyn_pin(),
            12 => $pins.gpio12.take().expect("Missing pin").into_dyn_pin(),
            13 => $pins.gpio13.take().expect("Missing pin").into_dyn_pin(),
            14 => $pins.gpio14.take().expect("Missing pin").into_dyn_pin(),
            15 => $pins.gpio15.take().expect("Missing pin").into_dyn_pin(),
            16 => $pins.gpio16.take().expect("Missing pin").into_dyn_pin(),
            17 => $pins.gpio17.take().expect("Missing pin").into_dyn_pin(),
            18 => $pins.gpio18.take().expect("Missing pin").into_dyn_pin(),
            19 => $pins.gpio19.take().expect("Missing pin").into_dyn_pin(),
            20 => $pins.gpio20.take().expect("Missing pin").into_dyn_pin(),
            21 => $pins.gpio21.take().expect("Missing pin").into_dyn_pin(),
            22 => $pins.gpio22.take().expect("Missing pin").into_dyn_pin(),
            23 => $pins.gpio23.take().expect("Missing pin").into_dyn_pin(),
            24 => $pins.gpio24.take().expect("Missing pin").into_dyn_pin(),
            25 => $pins.gpio25.take().expect("Missing pin").into_dyn_pin(),
            26 => $pins.gpio26.take().expect("Missing pin").into_dyn_pin(),
            27 => $pins.gpio27.take().expect("Missing pin").into_dyn_pin(),
            28 => $pins.gpio28.take().expect("Missing pin").into_dyn_pin(),
            29 => $pins.gpio29.take().expect("Missing pin").into_dyn_pin(),
            _ => panic!("Nonexistent pin"),
        }
    };
    (output: $pin:ident, $pins:expr) => {
        match $pin {
            0 => $pins
                .gpio0
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            1 => $pins
                .gpio1
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            2 => $pins
                .gpio2
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            3 => $pins
                .gpio3
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            4 => $pins
                .gpio4
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            5 => $pins
                .gpio5
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            6 => $pins
                .gpio6
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            7 => $pins
                .gpio7
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            8 => $pins
                .gpio8
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            9 => $pins
                .gpio9
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            10 => $pins
                .gpio10
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            11 => $pins
                .gpio11
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            12 => $pins
                .gpio12
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            13 => $pins
                .gpio13
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            14 => $pins
                .gpio14
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            15 => $pins
                .gpio15
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            16 => $pins
                .gpio16
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            17 => $pins
                .gpio17
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            18 => $pins
                .gpio18
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            19 => $pins
                .gpio19
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            20 => $pins
                .gpio20
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            21 => $pins
                .gpio21
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            22 => $pins
                .gpio22
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            23 => $pins
                .gpio23
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            24 => $pins
                .gpio24
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            25 => $pins
                .gpio25
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            26 => $pins
                .gpio26
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            27 => $pins
                .gpio27
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            28 => $pins
                .gpio28
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            29 => $pins
                .gpio29
                .take()
                .expect("Missing pin")
                .into_push_pull_output()
                .into_dyn_pin(),
            _ => panic!("Nonexistent pin"),
        }
    };
}
impl PinsWrapper {
    pub fn init(pins: Pins, active_high: bool) -> Self {
        let pins = MaybePins::from(pins);

        let input_pins = [const { None }; 30];
        let output_pins = [const { None }; 30];

        Self {
            pins,
            active_high,
            input_pins,
            output_pins,
        }
    }
}

impl PinSet for PinsWrapper {
    fn len(&self) -> usize {
        30
    }

    fn set_pin_input(&mut self, pin: usize) {
        let mut p = match_pin!(input: pin, self.pins);
        p.set_input_enable(true);
        self.input_pins[pin] = Some(p);
    }

    fn set_pin_output(&mut self, pin: usize) {
        self.output_pins[pin] = Some(match_pin!(output: pin, self.pins));
    }

    fn get_pin_state(&self, pin: usize) -> bool {
        if self.active_high {
            self.input_pins[pin]
                .as_ref()
                .expect("Missing pin")
                .as_input()
                .is_high()
                .expect("Error reading input pin")
        } else {
            self.input_pins[pin]
                .as_ref()
                .expect("Missing pin")
                .as_input()
                .is_low()
                .expect("Error reading input pin")
        }
    }

    fn set_pin_state(&mut self, pin: usize, state: bool) {
        if self.active_high == state {
            self.output_pins[pin]
                .as_mut()
                .expect("Missing pin")
                .set_high()
                .expect("Error writing output pin");
        } else {
            self.output_pins[pin]
                .as_mut()
                .expect("Missing pin")
                .set_low()
                .expect("Error writing output pin");
        }
    }
}

pub struct TimerWrapper<'t> {
    timer: &'t Timer,
}

impl<'t> dmk::timer::Timer for TimerWrapper<'t> {
    fn microseconds(&self) -> u64 {
        self.timer.get_counter().ticks()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
}
