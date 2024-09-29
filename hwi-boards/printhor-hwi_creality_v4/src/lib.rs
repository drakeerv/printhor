#![no_std]

pub use defmt::{trace, debug, info, warn, error};
pub use defmt;

cfg_if::cfg_if! {
    if #[cfg(feature="creality_v422_rc")] {
        //#region Creality V422 RC (STM32f1rc MCU) setup and reexports
        mod board_v422_stm32f1rc;
        pub mod board {
            pub use crate::board_v422_stm32f1rc::device;
            pub use crate::board_v422_stm32f1rc::SysDevices;
            pub use crate::board_v422_stm32f1rc::IODevices;
            pub use crate::board_v422_stm32f1rc::Controllers;
            pub use crate::board_v422_stm32f1rc::MotionDevices;
            pub use crate::board_v422_stm32f1rc::PwmDevices;
            pub use crate::board_v422_stm32f1rc::init;
            pub use crate::board_v422_stm32f1rc::setup;
            pub use crate::board_v422_stm32f1rc::heap_current_size;
            pub use crate::board_v422_stm32f1rc::stack_reservation_current_size;
            pub use crate::board_v422_stm32f1rc::MACHINE_BOARD;
            pub use crate::board_v422_stm32f1rc::MACHINE_TYPE;
            pub use crate::board_v422_stm32f1rc::MACHINE_PROCESSOR;
            pub use crate::board_v422_stm32f1rc::PROCESSOR_SYS_CK_MHZ;
            pub use crate::board_v422_stm32f1rc::HEAP_SIZE_BYTES;
            pub use crate::board_v422_stm32f1rc::MAX_STATIC_MEMORY;
            pub use crate::board_v422_stm32f1rc::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
            pub use crate::board_v422_stm32f1rc::STEPPER_PLANNER_CLOCK_FREQUENCY;
            pub use crate::board_v422_stm32f1rc::io;
            pub use crate::board_v422_stm32f1rc::Spawner;
        }
        pub use board::*;
        #[cfg(feature = "with-sdcard")]
        pub use crate::board_v422_stm32f1rc::SDCARD_PARTITION;
        #[cfg(feature = "with-serial-usb")]
        const USBSERIAL_BUFFER_SIZE: usize = 32;
        #[cfg(feature = "with-serial-port-1")]
        const UART_PORT1_BUFFER_SIZE: usize = 512;
        #[cfg(feature = "with-serial-port-1")]
        const UART_PORT1_BAUD_RATE: u32 = 115200;
        #[cfg(feature = "with-serial-port-2")]
        const UART_PORT2_BUFFER_SIZE: usize = 1024;
        #[cfg(feature = "with-serial-port-2")]
        const UART_PORT2_BAUD_RATE: u32 = 115200;

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                /// The maximum number of movements that can be queued. Warning! each one takes too memory as of now
                pub const SEGMENT_QUEUE_SIZE: u8 = 100;
            }
        }

        pub use crate::board_v422_stm32f1rc::ADC_START_TIME_US;
        pub use crate::board_v422_stm32f1rc::ADC_VREF_DEFAULT_MV;
        cfg_if::cfg_if! {
            if #[cfg(feature="without-vref-int")] {
                pub use crate::board_v422_stm32f1rc::ADC_VREF_DEFAULT_SAMPLE;
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature="executor-interrupt")] {
                use embassy_stm32::interrupt;
                use embassy_stm32::interrupt::Priority;
                use embassy_stm32::interrupt::InterruptExt;
                use embassy_executor::{InterruptExecutor};

                pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
                #[interrupt]
                unsafe fn RTC_ALARM() {
                    EXECUTOR_HIGH.on_interrupt()
                }

                #[inline]
                pub fn launch_high_priotity<S: 'static + Send>(_core: printhor_hwa_common::NoDevice, token: embassy_executor::SpawnToken<S>) -> Result<(),()> {
                    interrupt::RTC_ALARM.set_priority(Priority::P2);
                    interrupt::USB_LP_CAN1_RX0.set_priority(Priority::P3);
                    interrupt::USART1.set_priority(Priority::P3);
                    interrupt::USART2.set_priority(Priority::P3);
                    interrupt::UART4.set_priority(Priority::P4);
                    let spawner = EXECUTOR_HIGH.start(interrupt::RTC_ALARM);
                    spawner.spawn(token).map_err(|_| ())
                }
            }
        }
        //#endregion
    }
    else if #[cfg(feature="creality_v422_re")] {
        //#region SKR Mini E3 V2 (STM32F1 MCU) setup and reexports
        mod board_v422_stm32f1re;
        pub mod board {
            pub use crate::board_v422_stm32f1re::device;
            pub use crate::board_v422_stm32f1re::SysDevices;
            pub use crate::board_v422_stm32f1re::IODevices;
            pub use crate::board_v422_stm32f1re::Controllers;
            pub use crate::board_v422_stm32f1re::MotionDevices;
            pub use crate::board_v422_stm32f1re::PwmDevices;
            pub use crate::board_v422_stm32f1re::init;
            pub use crate::board_v422_stm32f1re::setup;
            pub use crate::board_v422_stm32f1re::heap_current_size;
            pub use crate::board_v422_stm32f1re::stack_reservation_current_size;
            pub use crate::board_v422_stm32f1re::MACHINE_BOARD;
            pub use crate::board_v422_stm32f1re::MACHINE_TYPE;
            pub use crate::board_v422_stm32f1re::MACHINE_PROCESSOR;
            pub use crate::board_v422_stm32f1re::PROCESSOR_SYS_CK_MHZ;
            pub use crate::board_v422_stm32f1re::HEAP_SIZE_BYTES;
            pub use crate::board_v422_stm32f1re::MAX_STATIC_MEMORY;
            pub use crate::board_v422_stm32f1re::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
            pub use crate::board_v422_stm32f1re::STEPPER_PLANNER_CLOCK_FREQUENCY;
            pub use crate::board_v422_stm32f1re::io;
            pub use crate::board_v422_stm32f1re::Spawner;
        }
        pub use board::*;
        #[cfg(feature = "with-sdcard")]
        pub use crate::board_v422_stm32f1re::SDCARD_PARTITION;
        #[cfg(feature = "with-serial-usb")]
        const USBSERIAL_BUFFER_SIZE: usize = 32;
        #[cfg(feature = "with-serial-port-1")]
        const UART_PORT1_BUFFER_SIZE: usize = 512;
        #[cfg(feature = "with-serial-port-1")]
        const UART_PORT1_BAUD_RATE: u32 = 115200;
        #[cfg(feature = "with-serial-port-2")]
        const UART_PORT2_BUFFER_SIZE: usize = 1024;
        #[cfg(feature = "with-serial-port-2")]
        const UART_PORT2_BAUD_RATE: u32 = 115200;

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                /// The maximum number of movements that can be queued. Warning! each one takes too memory as of now
                pub const SEGMENT_QUEUE_SIZE: u8 = 100;
            }
        }

        pub use crate::board_v422_stm32f1re::ADC_START_TIME_US;
        pub use crate::board_v422_stm32f1re::ADC_VREF_DEFAULT_MV;
        cfg_if::cfg_if! {
            if #[cfg(feature="without-vref-int")] {
                pub use crate::board_v422_stm32f1re::ADC_VREF_DEFAULT_SAMPLE;
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature="executor-interrupt")] {
                use embassy_stm32::interrupt;
                use embassy_stm32::interrupt::Priority;
                use embassy_stm32::interrupt::InterruptExt;
                use embassy_executor::{InterruptExecutor};

                pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
                #[interrupt]
                unsafe fn RTC_ALARM() {
                    EXECUTOR_HIGH.on_interrupt()
                }

                #[inline]
                pub fn launch_high_priotity<S: 'static + Send>(_core: printhor_hwa_common::NoDevice, token: embassy_executor::SpawnToken<S>) -> Result<(),()> {
                    interrupt::RTC_ALARM.set_priority(Priority::P2);
                    interrupt::USB_LP_CAN1_RX0.set_priority(Priority::P3);
                    interrupt::USART1.set_priority(Priority::P3);
                    interrupt::USART2.set_priority(Priority::P3);
                    interrupt::UART4.set_priority(Priority::P4);
                    let spawner = EXECUTOR_HIGH.start(interrupt::RTC_ALARM);
                    spawner.spawn(token).map_err(|_| ())
                }
            }
        }
        //#endregion
    }
    else if #[cfg(feature="creality_v427_rc")] {
        //#region SKR Mini E3 V2 (STM32F1 MCU) setup and reexports
        mod board_v427_stm32f1rc;
        pub mod board {
            pub use crate::board_v427_stm32f1rc::device;
            pub use crate::board_v427_stm32f1rc::SysDevices;
            pub use crate::board_v427_stm32f1rc::IODevices;
            pub use crate::board_v427_stm32f1rc::Controllers;
            pub use crate::board_v427_stm32f1rc::MotionDevices;
            pub use crate::board_v427_stm32f1rc::PwmDevices;
            pub use crate::board_v427_stm32f1rc::init;
            pub use crate::board_v427_stm32f1rc::setup;
            pub use crate::board_v427_stm32f1rc::heap_current_size;
            pub use crate::board_v427_stm32f1rc::stack_reservation_current_size;
            pub use crate::board_v427_stm32f1rc::MACHINE_BOARD;
            pub use crate::board_v427_stm32f1rc::MACHINE_TYPE;
            pub use crate::board_v427_stm32f1rc::MACHINE_PROCESSOR;
            pub use crate::board_v427_stm32f1rc::PROCESSOR_SYS_CK_MHZ;
            pub use crate::board_v427_stm32f1rc::HEAP_SIZE_BYTES;
            pub use crate::board_v427_stm32f1rc::MAX_STATIC_MEMORY;
            pub use crate::board_v427_stm32f1rc::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
            pub use crate::board_v427_stm32f1rc::STEPPER_PLANNER_CLOCK_FREQUENCY;
            pub use crate::board_v427_stm32f1rc::io;
            pub use crate::board_v427_stm32f1rc::Spawner;
        }
        pub use board::*;
        #[cfg(feature = "with-sdcard")]
        pub use crate::board_v427_stm32f1rc::SDCARD_PARTITION;
        #[cfg(feature = "with-serial-usb")]
        const USBSERIAL_BUFFER_SIZE: usize = 32;
        #[cfg(feature = "with-serial-port-1")]
        const UART_PORT1_BUFFER_SIZE: usize = 512;
        #[cfg(feature = "with-serial-port-1")]
        const UART_PORT1_BAUD_RATE: u32 = 115200;
        #[cfg(feature = "with-serial-port-2")]
        const UART_PORT2_BUFFER_SIZE: usize = 1024;
        #[cfg(feature = "with-serial-port-2")]
        const UART_PORT2_BAUD_RATE: u32 = 115200;

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                /// The maximum number of movements that can be queued. Warning! each one takes too memory as of now
                pub const SEGMENT_QUEUE_SIZE: u8 = 100;
            }
        }

        pub use crate::board_v427_stm32f1rc::ADC_START_TIME_US;
        pub use crate::board_v427_stm32f1rc::ADC_VREF_DEFAULT_MV;
        cfg_if::cfg_if! {
            if #[cfg(feature="without-vref-int")] {
                pub use crate::board_v427_stm32f1rc::ADC_VREF_DEFAULT_SAMPLE;
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature="executor-interrupt")] {
                use embassy_stm32::interrupt;
                use embassy_stm32::interrupt::Priority;
                use embassy_stm32::interrupt::InterruptExt;
                use embassy_executor::{InterruptExecutor};

                pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
                #[interrupt]
                unsafe fn RTC_ALARM() {
                    EXECUTOR_HIGH.on_interrupt()
                }

                #[inline]
                pub fn launch_high_priotity<S: 'static + Send>(_core: printhor_hwa_common::NoDevice, token: embassy_executor::SpawnToken<S>) -> Result<(),()> {
                    interrupt::RTC_ALARM.set_priority(Priority::P2);
                    interrupt::USB_LP_CAN1_RX0.set_priority(Priority::P3);
                    interrupt::USART1.set_priority(Priority::P3);
                    interrupt::USART2.set_priority(Priority::P3);
                    interrupt::UART4.set_priority(Priority::P4);
                    let spawner = EXECUTOR_HIGH.start(interrupt::RTC_ALARM);
                    spawner.spawn(token).map_err(|_| ())
                }
            }
        }
        //#endregion
    }
    else if #[cfg(feature="creality_v427_re")] {
        //#region SKR Mini E3 V2 (STM32F1 MCU) setup and reexports
        mod board_v427_stm32f1re;
        pub mod board {
            pub use crate::board_v427_stm32f1re::device;
            pub use crate::board_v427_stm32f1re::SysDevices;
            pub use crate::board_v427_stm32f1re::IODevices;
            pub use crate::board_v427_stm32f1re::Controllers;
            pub use crate::board_v427_stm32f1re::MotionDevices;
            pub use crate::board_v427_stm32f1re::PwmDevices;
            pub use crate::board_v427_stm32f1re::init;
            pub use crate::board_v427_stm32f1re::setup;
            pub use crate::board_v427_stm32f1re::heap_current_size;
            pub use crate::board_v427_stm32f1re::stack_reservation_current_size;
            pub use crate::board_v427_stm32f1re::MACHINE_BOARD;
            pub use crate::board_v427_stm32f1re::MACHINE_TYPE;
            pub use crate::board_v427_stm32f1re::MACHINE_PROCESSOR;
            pub use crate::board_v427_stm32f1re::PROCESSOR_SYS_CK_MHZ;
            pub use crate::board_v427_stm32f1re::HEAP_SIZE_BYTES;
            pub use crate::board_v427_stm32f1re::MAX_STATIC_MEMORY;
            pub use crate::board_v427_stm32f1re::STEPPER_PLANNER_MICROSEGMENT_FREQUENCY;
            pub use crate::board_v427_stm32f1re::STEPPER_PLANNER_CLOCK_FREQUENCY;
            pub use crate::board_v427_stm32f1re::io;
            pub use crate::board_v427_stm32f1re::Spawner;
        }
        pub use board::*;
        #[cfg(feature = "with-sdcard")]
        pub use crate::board_v427_stm32f1re::SDCARD_PARTITION;
        #[cfg(feature = "with-serial-usb")]
        const USBSERIAL_BUFFER_SIZE: usize = 32;
        #[cfg(feature = "with-serial-port-1")]
        const UART_PORT1_BUFFER_SIZE: usize = 512;
        #[cfg(feature = "with-serial-port-1")]
        const UART_PORT1_BAUD_RATE: u32 = 115200;
        #[cfg(feature = "with-serial-port-2")]
        const UART_PORT2_BUFFER_SIZE: usize = 1024;
        #[cfg(feature = "with-serial-port-2")]
        const UART_PORT2_BAUD_RATE: u32 = 115200;

        cfg_if::cfg_if! {
            if #[cfg(feature = "with-motion")] {
                /// The maximum number of movements that can be queued. Warning! each one takes too memory as of now
                pub const SEGMENT_QUEUE_SIZE: u8 = 100;
            }
        }

        pub use crate::board_v427_stm32f1re::ADC_START_TIME_US;
        pub use crate::board_v427_stm32f1re::ADC_VREF_DEFAULT_MV;
        cfg_if::cfg_if! {
            if #[cfg(feature="without-vref-int")] {
                pub use crate::board_v427_stm32f1re::ADC_VREF_DEFAULT_SAMPLE;
            }
        }
        cfg_if::cfg_if! {
            if #[cfg(feature="executor-interrupt")] {
                use embassy_stm32::interrupt;
                use embassy_stm32::interrupt::Priority;
                use embassy_stm32::interrupt::InterruptExt;
                use embassy_executor::{InterruptExecutor};

                pub static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
                #[interrupt]
                unsafe fn RTC_ALARM() {
                    EXECUTOR_HIGH.on_interrupt()
                }

                #[inline]
                pub fn launch_high_priotity<S: 'static + Send>(_core: printhor_hwa_common::NoDevice, token: embassy_executor::SpawnToken<S>) -> Result<(),()> {
                    interrupt::RTC_ALARM.set_priority(Priority::P2);
                    interrupt::USB_LP_CAN1_RX0.set_priority(Priority::P3);
                    interrupt::USART1.set_priority(Priority::P3);
                    interrupt::USART2.set_priority(Priority::P3);
                    interrupt::UART4.set_priority(Priority::P4);
                    let spawner = EXECUTOR_HIGH.start(interrupt::RTC_ALARM);
                    spawner.spawn(token).map_err(|_| ())
                }
            }
        }
        //#endregion
    }
    else {
        compile_error!("Board not set");
    }
}


#[inline]
pub fn init_logger() {
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-motion")] {
        pub fn setup_timer() {
            unsafe {
                let p = cortex_m::Peripherals::steal();
                let mut syst = p.SYST;
                syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
                let reload: u32 = (board::PROCESSOR_SYS_CK_MHZ / STEPPER_PLANNER_CLOCK_FREQUENCY).max(1) - 1;
                defmt::info!("SYST reload set to {}", reload);
                syst.set_reload(reload);
                syst.enable_counter();
                syst.enable_interrupt();
            }
        }

        extern "Rust" {fn do_tick();}

        use cortex_m_rt::exception;
        #[exception]
        fn SysTick() {
            unsafe {
                do_tick();
            }
        }
    }
}

#[inline]
pub fn sys_reset() {
    cortex_m::peripheral::SCB::sys_reset();
}

// Execute closure f in an interrupt-free context.
// Required to safety lock a resource that can be also requested by an ISR.
// Such ISR, hence, won't miss its interrupt and won't be too much delayed
pub fn interrupt_free<F, R>(f: F) -> R
    where F: FnOnce() -> R,
{
    cortex_m::interrupt::free(|_| {
        f()
    })
}
