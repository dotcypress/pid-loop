//! A tiny `no_std` PID controller library.
//!
//! This crate implements the classic PID loop over abstracted data type.
//!
//! # Introduction
//!
//! A proportional–integral–derivative controller is a control loop mechanism
//! employing feedback that is widely used in industrial control systems
//! and a variety of other applications requiring continuously modulated control.
//!
//! # Examples
//!
//! ```no_run
//! use pid_loop::PID;
//!
//! let target = 42.0;
//! let mut controller = PID::new(2.0, 0.7, 0.3);
//! loop {
//!     let correction = controller.next(target, measure());
//!     apply_correction(correction);
//!     sleep();
//! }
//!
//! fn sleep() { todo!() }
//! fn measure() -> f32 { todo!() }
//! fn apply_correction(_: f32) { todo!() }
//! ```

#![no_std]
#![deny(nonstandard_style, future_incompatible, rust_2018_idioms)]
use core::ops::*;

/// PID controller
///
/// # Examples
///
/// ```no_run
/// use pid_loop::PID;
///
/// let target = 42.0;
/// let mut controller = PID::<f64>::new(2.0, 0.7, 0.3);
/// let correction = controller.next(target, measure());
///
/// fn measure() -> f64 { todo!() }
/// ```
pub struct PID<F> {
    /// Proportional gain.
    pub kp: F,
    /// Integral gain.
    pub ki: F,
    /// Derivative gain.
    pub kd: F,
    last_error: F,
    error_sum: F,
    min_error_sum: Option<F>,
    max_error_sum: Option<F>,
}

impl<F> PID<F>
where
    F: Default + Add<Output = F> + Sub<Output = F> + Mul<Output = F> + PartialOrd + Copy,
{
    /// Create a new instance of `PID`.
    ///
    /// # Examples
    ///
    /// ```
    /// #![allow(unused_assignments)]
    /// use pid_loop::PID;
    ///
    /// let mut controller = PID::<f32>::new(0.7, 0.034, 0.084);
    /// ```
    pub fn new(kp: impl Into<F>, ki: impl Into<F>, kd: impl Into<F>) -> Self {
        Self {
            kp: kp.into(),
            ki: ki.into(),
            kd: kd.into(),
            last_error: F::default(),
            error_sum: F::default(),
            min_error_sum: None,
            max_error_sum: None,
        }
    }

    /// Reset controller internal state.
    ///
    /// # Examples
    ///
    /// ```
    /// #![allow(unused_assignments)]
    /// use pid_loop::PID;
    ///
    /// let target = 30.0;
    /// let mut controller = PID::<f32>::new(0.7, 0.034, 0.084);
    /// controller.next(target, 42.0);
    /// controller.reset();
    ///
    /// ```
    pub fn reset(&mut self) {
        self.last_error = F::default();
        self.error_sum = F::default();
    }

    /// Set minimum limit for error sum.
    ///
    /// # Examples
    ///
    /// ```
    /// #![allow(unused_assignments)]
    /// use pid_loop::PID;
    ///
    /// let target = 30;
    /// let mut controller = PID::<i128>::new(7, 34, 4);
    /// controller.set_min_error_sum(Some(-300));
    /// ```
    pub fn set_min_error_sum(&mut self, min: Option<F>) {
        self.min_error_sum = min;
    }

    /// Set maximum limit for error sum.
    ///
    /// # Examples
    ///
    /// ```
    /// #![allow(unused_assignments)]
    /// use pid_loop::PID;
    ///
    /// let target = 30.0;
    /// let mut controller = PID::<f32>::new(0.7, 0.034, 0.084);
    /// controller.set_max_error_sum(Some(300.0));
    /// ```
    pub fn set_max_error_sum(&mut self, max: Option<F>) {
        self.max_error_sum = max;
    }

    /// Push next measurement into the controller and return correction.
    ///
    /// # Examples
    ///
    /// ```
    /// #![allow(unused_assignments)]
    /// use pid_loop::PID;
    ///
    /// let target = 30.0;
    /// let mut controller = PID::<f64>::new(0.7, 0.034, 0.084);
    /// let correction = controller.next(target, 42.0);
    /// ```
    pub fn next(&mut self, sp: impl Into<F>, fb: impl Into<F>) -> F {
        let error = sp.into() - fb.into();
        let error_delta = error - self.last_error;
        self.last_error = error;

        let error_sum = self.error_sum + error;
        let error_sum = match &self.max_error_sum {
            Some(max) if &error_sum > max => *max,
            _ => error_sum,
        };
        let error_sum = match &self.min_error_sum {
            Some(min) if &error_sum < min => *min,
            _ => error_sum,
        };
        self.error_sum = error_sum;

        let p = error * self.kp;
        let i = error_sum * self.ki;
        let d = error_delta * self.kd;

        p + i + d
    }
}
