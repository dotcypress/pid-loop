//! A tiny `no_std` discrete PID controller library.
//!
//! This crate implements the classic discrete PID loop over abstract number type.
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

/// Discrete PID controller
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
    kp: F,
    ki: F,
    kd: F,
    errors: [F; 3],
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
        let mut pid = Self {
            kp: F::default(),
            ki: F::default(),
            kd: F::default(),
            errors: [F::default(); 3],
        };
        pid.set_config(kp, ki, kd);
        pid
    }

    /// Set loop coefficients.
    ///
    /// # Examples
    ///
    /// ```
    /// #![allow(unused_assignments)]
    /// use pid_loop::PID;
    ///
    /// let target = 40.0;
    /// let mut controller = PID::<f32>::new(1.7, 0.34, 0.4);
    /// controller.set_config(0.7, 0.34, 0.4);
    /// ```
    pub fn set_config(&mut self, kp: impl Into<F>, ki: impl Into<F>, kd: impl Into<F>) {
        let kp = kp.into();
        let ki = ki.into();
        let kd = kd.into();

        self.kp = kp + ki + kd;
        self.ki = F::default() - kp - (kd * kd);
        self.kd = kd;
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
        self.errors = [F::default(); 3]
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
        self.errors[2] = self.errors[1];
        self.errors[1] = self.errors[0];
        self.errors[0] = sp.into() - fb.into();
        self.kp * self.errors[0] + self.ki * self.errors[1] + self.kd * self.errors[2]
    }
}
