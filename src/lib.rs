//! A tiny `no_std` PID controller library.
//!
//! This crate implements the classic windowed PID loop over abstract data type.
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
//! let mut controller = PID::<f32, 1>::new(2.0, 0.7, 0.3, 0.0, 0.0);
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
/// let mut controller = PID::<f64, 1>::new(2.0, 0.7, 0.3, 0.0, 0.0);
/// let correction = controller.next(target, measure());
///
/// fn measure() -> f64 { todo!() }
/// ```
pub struct PID<F, const W: usize> {
    /// Proportional gain.
    pub kp: F,
    /// Integral gain.
    pub ki: F,
    /// Derivative gain.
    pub kd: F,
    ///Feed forward gain
    pub kf: F,
    ///Velocity
    pub kv: F,
    last_sp: F,
    last_error_idx: usize,
    errors: [F; W],
}

impl<F, const W: usize> PID<F, W>
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
    /// let mut controller = PID::<f32, 1>::new(0.7, 0.034, 0.084, 0.1, 0.0);
    /// ```
    pub fn new(
        kp: impl Into<F>,
        ki: impl Into<F>,
        kd: impl Into<F>,
        kf: impl Into<F>,
        kv: impl Into<F>,
    ) -> Self {
        assert!(W > 0);
        Self {
            kp: kp.into(),
            ki: ki.into(),
            kd: kd.into(),
            kf: kf.into(),
            kv: kv.into(),
            last_sp: F::default(),
            errors: [F::default(); W],
            last_error_idx: 0,
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
    /// let mut controller = PID::<f32, 1>::new(0.7, 0.034, 0.084, 0.1, 0.0);
    /// controller.next(target, 42.0);
    /// controller.reset();
    ///
    /// ```
    pub fn reset(&mut self) {
        self.last_sp = F::default();
        self.last_error_idx = 0;
        self.errors = [F::default(); W];
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
    /// let mut controller = PID::<f64, 1>::new(0.7, 0.034, 0.084, 0.1, 0.1);
    /// let correction = controller.next(target, 42.0);
    /// ```
    pub fn next(&mut self, sp: impl Into<F>, fb: impl Into<F>) -> F {
        let sp = sp.into();
        let fb = fb.into();
        let error = sp - fb;

        let error_delta = error - self.errors[self.last_error_idx];
        self.last_error_idx += 1;
        if self.last_error_idx >= W {
            self.last_error_idx = 0
        }
        self.errors[self.last_error_idx] = error;
        let err_history = self.errors.iter().fold(F::default(), |acc, i| acc + *i);

        let sp_delta = sp - self.last_sp;
        self.last_sp = sp;

        let p = self.kp * error;
        let i = self.ki * err_history;
        let d = self.kd * error_delta;
        let f = self.kf * sp_delta;
        let v = self.kv * fb;

        p + i + d + f + v
    }
}
