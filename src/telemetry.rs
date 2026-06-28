use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

#[derive(Debug, Clone)]
pub struct TelemetrySample {
    pub t_us: u64,
    pub x_m: f64,
    pub y_m: f64,
    pub yaw_rad: f64,
    pub vx_body_m_s: f64,
    pub vy_body_m_s: f64,
    pub yaw_rate_rad_s: f64,
    pub line_position_m: f64,
    pub line_error_m: f64,
    pub line_visible: bool,
    pub line_confidence: f64,
    pub pwm_left: f64,
    pub pwm_right: f64,
    pub pwm_downforce: f64,
    pub motor_current_left_a: f64,
    pub motor_current_right_a: f64,
    pub motor_torque_left_nm: f64,
    pub motor_torque_right_nm: f64,
    pub motor_voltage_left_v: f64,
    pub motor_voltage_right_v: f64,
    pub wheel_force_left_n: f64,
    pub wheel_force_right_n: f64,
    pub desired_wheel_force_left_n: f64,
    pub desired_wheel_force_right_n: f64,
    pub slip_left: f64,
    pub slip_right: f64,
    pub wheel_surface_speed_left_m_s: f64,
    pub wheel_surface_speed_right_m_s: f64,
    pub normal_left_n: f64,
    pub normal_right_n: f64,
    pub normal_front_left_n: f64,
    pub normal_front_right_n: f64,
    pub normal_rear_left_n: f64,
    pub normal_rear_right_n: f64,
    pub downforce_extra_n: f64,
    pub downforce_fan_n: f64,
    pub downforce_suction_n: f64,
    pub downforce_current_a: f64,
    pub battery_voltage_v: f64,
    pub battery_current_a: f64,
    pub encoder_left_ticks: i64,
    pub encoder_right_ticks: i64,
    pub encoder_left_velocity_rad_s: f64,
    pub encoder_right_velocity_rad_s: f64,
    pub gyro_yaw_rate_rad_s: f64,
    pub gyro_bias_rad_s: f64,
    pub sensor_adc: Vec<u32>,
}

pub struct CsvLogger {
    writer: BufWriter<File>,
    sensor_count: usize,
}

impl CsvLogger {
    pub fn create(path: &Path, sensor_count: usize) -> std::io::Result<Self> {
        let file = File::create(path)?;
        let mut logger = Self {
            writer: BufWriter::new(file),
            sensor_count,
        };
        logger.write_header()?;
        Ok(logger)
    }

    fn write_header(&mut self) -> std::io::Result<()> {
        write!(
            self.writer,
            "t_us,t_s,x_m,y_m,yaw_rad,vx_body_m_s,vy_body_m_s,yaw_rate_rad_s,line_position_m,line_error_m,line_visible,line_confidence,pwm_left,pwm_right,pwm_downforce,motor_current_left_a,motor_current_right_a,motor_torque_left_nm,motor_torque_right_nm,motor_voltage_left_v,motor_voltage_right_v,wheel_force_left_n,wheel_force_right_n,desired_wheel_force_left_n,desired_wheel_force_right_n,slip_left,slip_right,wheel_surface_speed_left_m_s,wheel_surface_speed_right_m_s,normal_left_n,normal_right_n,normal_front_left_n,normal_front_right_n,normal_rear_left_n,normal_rear_right_n,downforce_extra_n,downforce_fan_n,downforce_suction_n,downforce_current_a,battery_voltage_v,battery_current_a,encoder_left_ticks,encoder_right_ticks,encoder_left_velocity_rad_s,encoder_right_velocity_rad_s,gyro_yaw_rate_rad_s,gyro_bias_rad_s"
        )?;
        for i in 0..self.sensor_count {
            write!(self.writer, ",sensor_{:02}_adc", i)?;
        }
        writeln!(self.writer)?;
        Ok(())
    }

    pub fn write_sample(&mut self, s: &TelemetrySample) -> std::io::Result<()> {
        write!(
            self.writer,
            "{},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{},{:.9},{:.6},{:.6},{:.6},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{:.9},{},{},{:.9},{:.9},{:.9},{:.9}",
            s.t_us,
            s.t_us as f64 / 1_000_000.0,
            s.x_m,
            s.y_m,
            s.yaw_rad,
            s.vx_body_m_s,
            s.vy_body_m_s,
            s.yaw_rate_rad_s,
            s.line_position_m,
            s.line_error_m,
            s.line_visible as u8,
            s.line_confidence,
            s.pwm_left,
            s.pwm_right,
            s.pwm_downforce,
            s.motor_current_left_a,
            s.motor_current_right_a,
            s.motor_torque_left_nm,
            s.motor_torque_right_nm,
            s.motor_voltage_left_v,
            s.motor_voltage_right_v,
            s.wheel_force_left_n,
            s.wheel_force_right_n,
            s.desired_wheel_force_left_n,
            s.desired_wheel_force_right_n,
            s.slip_left,
            s.slip_right,
            s.wheel_surface_speed_left_m_s,
            s.wheel_surface_speed_right_m_s,
            s.normal_left_n,
            s.normal_right_n,
            s.normal_front_left_n,
            s.normal_front_right_n,
            s.normal_rear_left_n,
            s.normal_rear_right_n,
            s.downforce_extra_n,
            s.downforce_fan_n,
            s.downforce_suction_n,
            s.downforce_current_a,
            s.battery_voltage_v,
            s.battery_current_a,
            s.encoder_left_ticks,
            s.encoder_right_ticks,
            s.encoder_left_velocity_rad_s,
            s.encoder_right_velocity_rad_s,
            s.gyro_yaw_rate_rad_s,
            s.gyro_bias_rad_s,
        )?;
        for i in 0..self.sensor_count {
            let value = s.sensor_adc.get(i).copied().unwrap_or(0);
            write!(self.writer, ",{}", value)?;
        }
        writeln!(self.writer)?;
        Ok(())
    }

    pub fn flush(&mut self) -> std::io::Result<()> {
        self.writer.flush()
    }
}
