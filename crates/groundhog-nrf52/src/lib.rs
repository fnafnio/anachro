//! Contains a nrf52-compatible implementation of the `RollingTimer` trait.
//!
//! The `GlobalRollingTimer` is especially helpful if you are running RTIC on
//! a nrf52 board. The built-in cycle counter (`CYCCNT`) which is commonly used
//! as a monotonic counter will not work when the debugger is not attached, which
//! in turn will make scheduling operations not work as expected.
//!
//! # Usage
//!
//! To use the the `GlobalRollingTimer` with RTIC, it first needs to be selected
//! as the monotonic timer (here on top of the nrf52840 hal):
//!
//! ```
//! #[rtic::app(device = nrf52840_hal::pac, peripherals = true, monotonic = groundhog_nrf52::GlobalRollingTimer)]
//! ```
//!
//! During the init phase it needs to be initialized with a concrete timer implementation:
//!
//! ```
//! #[init]
//! fn init(ctx: init::Context) -> init::LateResources {
//!     // using TIMER0 here
//!     GlobalRollingTimer::init(ctx.device.TIMER0);
//!     // ...
//! }
//! ```
//!
//! Then, you can specify the schedule interval in microseconds as part of your task:
//!
//! ```
//! #[task]
//! fn my_task(ctx: my_task::Context) {
//!     ctx.schedule
//!         .my_task(ctx.scheduled + 1_000_000)
//!         .unwrap();
//! }
//! ```
//! In this case the task will be scheduled again one second later.
//!
#![no_std]

use groundhog::RollingTimer;
use nrf52840_hal::{
    pac::timer0::RegisterBlock as TimerRegBlock0,
    timer::Instance,
};
use nrf52840_hal::pac::TIMER0;
use rtic::{Fraction, Monotonic};

use core::{debug_assert, fmt::write, ptr::null_mut, sync::atomic::{AtomicPtr, AtomicU32, Ordering}};

static TIMER_PTR: AtomicPtr<TimerRegBlock0> = AtomicPtr::new(core::ptr::null_mut());
// static STATE_PTR: AtomicPtr<GlobalRollingTimer64> = AtomicPtr::new(core::ptr::null_mut());

static UPPER_COUNT: AtomicU32 = AtomicU32::new(0);

pub struct GlobalRollingTimer;

impl GlobalRollingTimer {
    pub const fn new() -> Self {
        Self
    }

    pub fn init<T: Instance>(timer: T) {
        timer.set_periodic();
        timer.timer_start(0xFFFF_FFFFu32);
        let t0 = timer.as_timer0();

        let old_ptr = TIMER_PTR.swap(t0 as *const _ as *mut _, Ordering::SeqCst);

        debug_assert!(old_ptr == core::ptr::null_mut());
    }
}

impl Monotonic for GlobalRollingTimer {
    type Instant = i32;

    fn ratio() -> Fraction {
        Fraction {
            numerator: 64,
            denominator: 1,
        }
    }

    fn now() -> Self::Instant {
        Self::new().get_ticks() as i32
    }

    fn zero() -> Self::Instant {
        0
    }

    unsafe fn reset() {
        if let Some(t0) = TIMER_PTR.load(Ordering::SeqCst).as_ref() {
            t0.tasks_clear.write(|w| w.bits(1));
        }
    }
}

impl RollingTimer for GlobalRollingTimer {
    type Tick = u32;
    const TICKS_PER_SECOND: u32 = 1_000_000;

    fn get_ticks(&self) -> u32 {
        if let Some(t0) = unsafe { TIMER_PTR.load(Ordering::SeqCst).as_ref() } {
            t0.tasks_capture[1].write(|w| unsafe { w.bits(1) });
            t0.cc[1].read().bits()
        } else {
            0
        }
    }
}

pub struct GlobalRollingTimer64 {
    // upper: u64,
}

impl GlobalRollingTimer64 {
    // cannot be made const, as some state is needed
    // pub fn new() -> Self {
    //     let mut grt = Self { upper: 0 };

    //     // let old_ptr = STATE_PTR.swap(&mut grt, Ordering::SeqCst);
    //     // debug_assert!(old_ptr == core::ptr::null_mut());

    //     grt
    // }
    pub const fn get_ref() -> Self{
        Self{}
    }

    pub fn init<T: Instance>(timer: T) {
        // let grt = Self{ upper: 0};
        // let old_ptr = STATE_PTR.swap(&grt as *const _ as *mut _, Ordering::SeqCst);
        // debug_assert!(old_ptr == core::ptr::null_mut());
        UPPER_COUNT.store(0, Ordering::SeqCst);
        
        timer.set_periodic();
        let t0 = timer.as_timer0();
        t0.bitmode.write(|w| w.bitmode()._32bit());
        t0.shorts.write(|w| w.compare0_clear().enabled());
        t0.intenset.write(|w| w.compare0().set_bit());

        t0.tasks_clear.write(|w| unsafe { w.bits(1) });
        t0.cc[0].write(|w| unsafe { w.bits(0xFFFF_FFFF) });

        t0.tasks_start.write(|w| w.tasks_start().set_bit());
        let old_ptr = TIMER_PTR.swap(t0 as *const _ as *mut _, Ordering::SeqCst);

        debug_assert!(old_ptr == core::ptr::null_mut());
    }

    pub fn interrupt() {
        // let mut grt = STATE_PTR.load(Ordering::SeqCst);
        let t0 = TIMER_PTR.load(Ordering::SeqCst);

        unsafe { (*t0).events_compare[0].reset() };
        // debug_assert!(grt != core::ptr::null_mut());
        // increment the upper part of the counter value
        // unsafe { (*grt).upper += 0x000_0001_0000_0000 };
        UPPER_COUNT.fetch_add(1, Ordering::AcqRel);
    }
}

impl RollingTimer for GlobalRollingTimer64 {
    type Tick = u64;
    const TICKS_PER_SECOND: u64 = 1_000_000;

    fn get_ticks(&self) -> u64 {
        if let Some(t0) = unsafe { TIMER_PTR.load(Ordering::SeqCst).as_ref() } {
            t0.tasks_capture[1].write(|w| unsafe { w.bits(1) });
            let res = t0.cc[1].read().bits() as u64 | (UPPER_COUNT.load(Ordering::Acquire) as u64) << 32;
            defmt::info!("get_ticks: {:?}", res);
            res
        } else {
            0
        }
    }
}

impl Monotonic for GlobalRollingTimer64 {
    type Instant = i64;
    fn ratio() -> Fraction {
        Fraction {
            numerator: 64,
            denominator: 1,
        }
    }
    fn now() -> Self::Instant {
        // let grt = STATE_PTR.load(Ordering::SeqCst);
        // if grt == core::ptr::null_mut() {
        //     0
        // } else {
        //     unsafe { (*grt).get_ticks() as i64 }
        // }
        Self::get_ref().get_ticks() as i64

    }
    fn zero() -> Self::Instant {
        0
    }
    unsafe fn reset() {}
}
