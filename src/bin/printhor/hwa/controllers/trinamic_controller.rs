//! TODO: This feature is still very experimental/preliminar
use embassy_time::Instant;
use hwa::soft_uart::SerialError;
use crate::hwa;

#[allow(unused)]
pub enum TrinamicError {
    Timeout,
    WriteError,
    ReadError,
}

pub struct TrinamicController
{
    uart: hwa::device::TrinamicUart,
    motion_config: hwa::controllers::MotionConfigRef,

}
impl TrinamicController
{
    pub const fn new(uart: hwa::device::TrinamicUart, motion_config: hwa::controllers::MotionConfigRef) -> Self {
        Self {
            uart,
            motion_config
        }
    }

    pub async fn init(&mut self) -> Result<(),TrinamicError> {
        hwa::debug!("Trinamic_uart CMD");

        let mcfg = self.motion_config.lock().await;

        cfg_if::cfg_if! {
            if #[cfg(feature = "trinamic-uart-multi-channel")] {
                self.uart.set_axis_channel(Some(hwa::device::AxisChannel::TMCUartX));
            }
        }
        if self.init_stepper(0, get_microsteps(mcfg.usteps[0])).await.is_ok() {
            hwa::info!("Trinamic_uart X init OK");
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "trinamic-uart-multi-channel")] {
                self.uart.set_axis_channel(Some(hwa::device::AxisChannel::TMCUartY));
            }
        }
        if self.init_stepper(1, get_microsteps(mcfg.usteps[1])).await.is_ok() {
            hwa::info!("Trinamic_uart Y init OK");
        }
        cfg_if::cfg_if! {
            if #[cfg(feature = "trinamic-uart-multi-channel")] {
                self.uart.set_axis_channel(Some(hwa::device::AxisChannel::TMCUartZ));
            }
        }
        if self.init_stepper(2, get_microsteps(mcfg.usteps[2])).await.is_ok() {
            hwa::info!("Trinamic_uart Z init OK");
        }
        cfg_if::cfg_if! {
            if #[cfg(all(feature = "trinamic-uart-multi-channel", feature="with-hot-end"))] {
                self.uart.set_axis_channel(Some(hwa::device::AxisChannel::TMCUartE));
                if self.init_stepper(3, get_microsteps(mcfg.usteps[3])).await.is_ok() {
                    hwa::info!("Trinamic_uart E init OK");
                }
            }
        }

        Ok(())
    }

    async fn init_stepper(&mut self,  addr: u8, micro_steps_pow_of_2: u32) -> Result<(), TrinamicError> {
        hwa::debug!("Trinamic_uart applying gconf on {}", addr);

        let mut gconf = tmc2209::reg::GCONF::default();
        // Invert direction
        gconf.set_shaft(false);
        // Enable UART Comm
        gconf.set_pdn_disable(true);
        // Enable Spread Cycle: Higher Torque, more noise
        gconf.set_en_spread_cycle(true);
        // Enable multistepping
        gconf.set_mstep_reg_select(true);
        // Enable multistepping filtering
        gconf.set_multistep_filt(false);
        if self.write_register(addr, gconf).await.is_ok() {
            hwa::debug!("Trinamic_uart applying chopconf on {}", addr);
            let mut chopconf = tmc2209::reg::CHOPCONF::default();
            chopconf.set_intpol(false);
            // Nice to have
            //chopconf.set_dedge(true);
            chopconf.set_mres(micro_steps_pow_of_2);
            if self.write_register(addr, chopconf).await.is_ok() {
                hwa::debug!("Done {}", addr);
            }
        }
        else {
            hwa::error!("Error initializing stepper {}", addr);
        }

        Ok(())
    }

    #[inline]
    async fn write_register<T: tmc2209::WritableRegister>(&mut self, slave_addr: u8, reg: T) -> Result<(), TrinamicError>
    {
        self.raw_write(tmc2209::WriteRequest::new(slave_addr, reg).bytes()).await
    }

    #[inline]
    #[allow(unused)]
    async fn read_register<T: tmc2209::ReadableRegister + core::fmt::Debug>(&mut self, slave_addr: u8) -> Result<T, TrinamicError>
    {
        //hwa::info!("Flush");
        //let _ = self.uart.consume().await;
        hwa::debug!("Send req");
        self.raw_write(tmc2209::read_request::<T>(slave_addr).bytes()).await?;
        //embassy_time::Timer::after_millis(5).await;
        hwa::debug!("Now reading response...");
        let mut buff = [0u8; 32];
        let mut reader = tmc2209::Reader::default();
        let reception_timeout = Instant::now() + embassy_time::Duration::from_secs(5);
        loop {
            if Instant::now() > reception_timeout {
                hwa::debug!("TO Req");
                return Err(TrinamicError::Timeout);
            }
            match self.uart.read_until_idle(&mut buff).await {
                Ok(num_bytes_read) => {
                    if num_bytes_read > 0 {
                        hwa::debug!("Uart read {} bytes", num_bytes_read);
                        match reader.read_response(&buff[0..num_bytes_read]) {
                            (_n, Some(response)) => {
                                return match response.register::<T>() {
                                    Ok(r) => {
                                        let x = alloc::format!("{:?}", r);
                                        hwa::debug!("response[0]. l={} : {}", _n, x.as_str());
                                        Ok(r)
                                    }
                                    _ => {
                                        Err(TrinamicError::ReadError)
                                    }
                                }
                            }
                            (n, None) => {
                                let x = alloc::format!("{:?}", reader.awaiting());
                                hwa::warn!("Uncompleted. (readed {}: {:?}) awaiting {}", n, &buff[0..num_bytes_read], x.as_str());
                            }
                        }
                    }
                }
                Err(SerialError::Timeout) => {
                    // Lost one frame.
                    // Continue until [reception_timeout]
                }
                Err(err) => {
                    hwa::error!("Read error reading trinamic: {:?}", err);
                    return Err(TrinamicError::ReadError)
                }
            }
        }
    }

    pub async fn raw_write(&mut self, bytes: &[u8]) -> Result<(), TrinamicError> {
        hwa::debug!("Trinamic_uart sent {:?}", bytes);
        self.uart.write(bytes).await.map_err(|_| TrinamicError::WriteError)?;
        let _ = self.uart.blocking_flush().map_err(|_| TrinamicError::WriteError)?;
        Ok(())
    }
}

fn get_microsteps(msteps: u16) -> u32 {
    match msteps {
        1 => 8,
        2 => 7,
        4 => 6,
        8 => 5,
        16 => 4,
        32 => 3,
        64 => 2,
        128 => 1,
        256 => 0,
        _ => panic!("Unspected microstepping value"),
    }
}
