#![no_std]
#![no_main]

#[macro_use]
extern crate lazy_static;

#[allow(unused_imports)]
use panic_itm;

use core::cell::RefCell;
use cortex_m;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use stm32f3::stm32f303;

lazy_static! {
    static ref MUTEX_GPIOA: Mutex<RefCell<Option<stm32f303::GPIOA>>> = Mutex::new(RefCell::new(None));
    static ref MUTEX_GPIOE: Mutex<RefCell<Option<stm32f303::GPIOE>>> = Mutex::new(RefCell::new(None));
    static ref MUTEX_EXTI:  Mutex<RefCell<Option<stm32f303::EXTI>>>  = Mutex::new(RefCell::new(None));
}


#[entry]
fn main() -> ! {
    // 1. get peripherals
    let cortexm_peripherals = cortex_m::Peripherals::take().unwrap();
    let stm32f3_peripherals = stm32f303::Peripherals::take().unwrap();

    // 2. enable GPIOA and SYSCFG clocks
    let rcc = &stm32f3_peripherals.RCC;
    rcc.ahbenr.modify(|_, w| {
        w.iopaen().set_bit()
         .iopeen().set_bit()
    });
    rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());

    // 3. Configure PA0 pin as input, pull-down
    let gpioa = &stm32f3_peripherals.GPIOA;
    gpioa.moder.modify(|_, w| w.moder0().input());
    gpioa.pupdr.modify(|_, w| w.pupdr0().pull_down());

    // configure PE8, PE9 as output
    let gpioe = &stm32f3_peripherals.GPIOE;
    gpioe.moder.modify(|_, w| {
        w.moder8().output()     // LED: LED4 User on pin PE8  (blue)
         .moder9().output()     // LED: LED3 User on pin PE9  (red)
    });

    // 4. connect EXTI0 line to PA0 pin
    let syscfg = &stm32f3_peripherals.SYSCFG_COMP_OPAMP;
    syscfg.syscfg_exticr1.modify(|_, w| unsafe { w.exti0().bits(0b000) }); // w.exti0().pa0()

    // 5. Configure EXTI0 line (external interrupts) mode=interrupt and trigger=rising-edge
    let exti = &stm32f3_peripherals.EXTI;
    exti.imr1.modify(|_, w| w.mr0().set_bit());   // unmask interrupt
    exti.rtsr1.modify(|_, w| w.tr0().set_bit());  // trigger=rising-edge

    // 6. Move shared peripherals into mutexes
    //    After this we can only access them via their respective mutex
    cortex_m::interrupt::free(|cs| {
        MUTEX_GPIOA.borrow(cs).replace(Some(stm32f3_peripherals.GPIOA));
        MUTEX_GPIOE.borrow(cs).replace(Some(stm32f3_peripherals.GPIOE));
        MUTEX_EXTI.borrow(cs).replace(Some(stm32f3_peripherals.EXTI))
    });

    // 7. Enable EXTI0 Interrupt
    let mut nvic = cortexm_peripherals.NVIC;
    nvic.enable(stm32f303::Interrupt::EXTI0);

    loop {
        // read the state of the user button
        let user_button = cortex_m::interrupt::free(|cs| {
            let refcell = MUTEX_GPIOA.borrow(cs).borrow();
            let gpioa = match refcell.as_ref() { None => return false, Some(v) => v };
            gpioa.idr.read().idr0().bit_is_set()
        });

        // set LED3 according to the state of the user button
        cortex_m::interrupt::free(|cs| {
            let refcell = MUTEX_GPIOE.borrow(cs).borrow();
            let gpioe = match refcell.as_ref() { None => return, Some(v) => v };
            if user_button {
                gpioe.odr.modify(|_, w| w.odr9().set_bit());
            } else {
                gpioe.odr.modify(|_, w| w.odr9().clear_bit());
            }
        });
    }
}


// 8. Handle interrupt
use stm32f303::interrupt;

#[interrupt]
fn EXTI0() {
    // clear the EXTI line 0 pending bit
    cortex_m::interrupt::free(|cs| {
        let refcell = MUTEX_EXTI.borrow(cs).borrow();
        let exti = match refcell.as_ref() { None => return, Some(v) => v };
        exti.pr1.modify(|_, w| w.pr0().set_bit());
    });

    // toggle LED4
    cortex_m::interrupt::free(|cs| {
        let refcell = MUTEX_GPIOE.borrow(cs).borrow();
        let gpioe = match refcell.as_ref() { None => return, Some(v) => v };
        gpioe.odr.modify(|r, w| {
            let led4 = r.odr8().bit();
            if led4 {
                w.odr8().clear_bit()
            } else {
                w.odr8().set_bit()
            }
        });
    });
}
