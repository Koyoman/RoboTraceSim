use crate::config::TimeConfig;

/// All events refer to the state at the current physical tick. Zero-latency
/// acquisition order is line -> encoder -> gyro -> controller -> observation.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Events {
    pub sensor: bool,
    pub encoder: bool,
    pub imu: bool,
    pub controller: bool,
    pub log: bool,
}

pub(crate) fn events_at(time: TimeConfig, t_us: u64) -> Events {
    Events {
        sensor: t_us % time.sensor_period_us == 0,
        encoder: t_us % time.encoder_period_us == 0,
        imu: t_us % time.imu_period_us == 0,
        controller: t_us % time.controller_period_us == 0,
        log: t_us % time.log_period_us == 0,
    }
}

pub(crate) fn validate_time(time: &TimeConfig) -> Result<(), String> {
    if time.physics_dt_us == 0 || time.render_period_us == 0 {
        return Err(
            "physics_dt_us and render_period_us must be positive integer microseconds".into(),
        );
    }
    for (name, period) in [
        ("controller_period_us", time.controller_period_us),
        ("sensor_period_us", time.sensor_period_us),
        ("encoder_period_us", time.encoder_period_us),
        ("imu_period_us", time.imu_period_us),
        ("log_period_us", time.log_period_us),
    ] {
        if period == 0 || period % time.physics_dt_us != 0 {
            return Err(format!("{name} ({period}) must be a positive multiple of physics_dt_us ({}); use k * {} us with integer k >= 1", time.physics_dt_us, time.physics_dt_us));
        }
    }
    Ok(())
}
