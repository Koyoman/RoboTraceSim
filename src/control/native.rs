//! Safe in-process firmware boundary; dynamic loading is intentionally not performed.
use super::{ControllerInput, TimedCommand};
pub const ABI_VERSION: u32 = 1;
pub const MAX_OPTICAL_CHANNELS: usize = 256;
/// Implementations receive only delivered measurements. They must not block on wall time.
/// reset is called exactly once on installation; stop on host destruction.
pub trait Firmware: Send {
    fn reset(&mut self) -> Result<(), String>;
    fn step(&mut self, input: &ControllerInput) -> Result<TimedCommand, String>;
    fn stop(&mut self) {}
}
pub struct NativeAdapter {
    firmware: Box<dyn Firmware>,
}
impl NativeAdapter {
    pub fn new(mut firmware: Box<dyn Firmware>) -> Result<Self, String> {
        if let Err(e) = firmware.reset() {
            firmware.stop();
            return Err(e);
        }
        Ok(Self { firmware })
    }
    pub fn step(&mut self, input: &ControllerInput) -> Result<TimedCommand, String> {
        if input.frame.optical.len() > MAX_OPTICAL_CHANNELS {
            return Err("firmware channel capacity exceeded".into());
        }
        let c = self.firmware.step(input)?;
        if c.t_us != input.frame.t_us
            || c.pwm.iter().any(|x| !x.is_finite() || x.abs() > 1.)
            || !c.downforce_pwm.is_finite()
            || !(0.0..=1.0).contains(&c.downforce_pwm)
        {
            return Err("invalid firmware command/timestamp".into());
        }
        Ok(c)
    }
}
impl Drop for NativeAdapter {
    fn drop(&mut self) {
        self.firmware.stop();
    }
}
/// Future C ABI header. Buffers are host-owned and valid only during step;
/// capacity/count are explicit, errors are numeric and no Rust object crosses the ABI.
#[repr(C)]
pub struct AbiHeader {
    pub version: u32,
    pub size_bytes: u32,
    pub t_us: u64,
    pub channel_count: u32,
    pub channel_capacity: u32,
}
#[repr(C)]
pub struct AbiOptical {
    pub acquired_us: u64,
    pub available_us: u64,
    pub age_us: u64,
    pub adc: u32,
    pub flags: u32,
}
#[repr(C)]
pub struct AbiCommand {
    pub header: AbiHeader,
    pub pwm_left: f64,
    pub pwm_right: f64,
    pub downforce_pwm: f64,
    pub mode_left: u32,
    pub mode_right: u32,
    pub error_code: i32,
}

#[repr(C)]
pub struct AbiFrame {
    pub header: AbiHeader,
    pub optical: *const AbiOptical,
    pub encoder_acquired_us: u64,
    pub encoder_available_us: u64,
    pub left_ticks: i64,
    pub right_ticks: i64,
    pub left_velocity_rad_s: f64,
    pub right_velocity_rad_s: f64,
    pub imu_acquired_us: u64,
    pub imu_available_us: u64,
    pub yaw_rate_rad_s: f64,
    pub accel_x_m_s2: f64,
    pub accel_y_m_s2: f64,
    pub applied_pwm_left: f64,
    pub applied_pwm_right: f64,
    pub flags: u32,
}
