#![no_std]
#![deny(nonstandard_style)]
#![warn(future_incompatible, unreachable_pub, rust_2018_idioms)]
use core::ops::*;

pub struct PID<F: Ord + Copy> {
    kp: F,
    ki: F,
    kd: F,
    last_error: F,
    error_sum: F,
    error_sum_min: Option<F>,
    error_sum_max: Option<F>,
}

impl<F> PID<F>
where
    F: Default + Add<Output = F> + Sub<Output = F> + Mul<Output = F> + Ord + Copy,
{
    pub fn new(kp: impl Into<F>, ki: impl Into<F>, kd: impl Into<F>) -> Self {
        Self {
            kp: kp.into(),
            ki: ki.into(),
            kd: kd.into(),
            last_error: F::default(),
            error_sum: F::default(),
            error_sum_min: None,
            error_sum_max: None,
        }
    }

    pub fn set_error_sum_min(&mut self, min: Option<F>) {
        self.error_sum_min = min;
    }

    pub fn set_error_sum_max(&mut self, max: Option<F>) {
        self.error_sum_max = max;
    }

    pub fn next(&mut self, sp: impl Into<F>, fb: impl Into<F>) -> F {
        let error = sp.into() - fb.into();
        let error_delta = error - self.last_error;
        self.last_error = error;

        let error_sum = self.error_sum + error;
        let error_sum = match &self.error_sum_min {
            Some(min) if min > &error_sum => *min,
            _ => error_sum,
        };
        let error_sum = match &self.error_sum_max {
            Some(max) if max < &error_sum => *max,
            _ => error_sum,
        };
        self.error_sum = error_sum;

        let p = error * self.kp;
        let i = error_sum * self.ki;
        let d = error_delta * self.kd;

        p + i + d
    }
}
