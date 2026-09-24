use crate::telemetry::TelemetrySample;
use std::fs::File;
use std::io::{self, BufWriter, Read, Write};
use std::path::Path;

const MAGIC: &[u8; 8] = b"RTSRPL03";
const VERSION: u16 = 3;
const FIXED_F64_COUNT: usize = 44;

pub struct LegacyReplayLogger {
    writer: BufWriter<File>,
    sensor_count: usize,
}

impl LegacyReplayLogger {
    pub fn create(path: &Path, sensor_count: usize) -> io::Result<Self> {
        let file = File::create(path)?;
        let mut logger = Self {
            writer: BufWriter::new(file),
            sensor_count,
        };
        logger.write_header()?;
        Ok(logger)
    }

    fn write_header(&mut self) -> io::Result<()> {
        self.writer.write_all(MAGIC)?;
        write_u16(&mut self.writer, VERSION)?;
        write_u16(&mut self.writer, self.sensor_count as u16)?;
        write_u32(&mut self.writer, FIXED_F64_COUNT as u32)?;
        Ok(())
    }

    pub fn write_sample(&mut self, s: &TelemetrySample) -> io::Result<()> {
        write_u64(&mut self.writer, s.t_us)?;
        write_f64(&mut self.writer, s.x_m)?;
        write_f64(&mut self.writer, s.y_m)?;
        write_f64(&mut self.writer, s.yaw_rad)?;
        write_f64(&mut self.writer, s.vx_body_m_s)?;
        write_f64(&mut self.writer, s.vy_body_m_s)?;
        write_f64(&mut self.writer, s.yaw_rate_rad_s)?;
        write_f64(&mut self.writer, s.line_position_m)?;
        write_f64(&mut self.writer, s.line_error_m)?;
        write_f64(&mut self.writer, s.line_confidence)?;
        write_f64(&mut self.writer, s.pwm_left)?;
        write_f64(&mut self.writer, s.pwm_right)?;
        write_f64(&mut self.writer, s.pwm_downforce)?;
        write_f64(&mut self.writer, s.motor_current_left_a)?;
        write_f64(&mut self.writer, s.motor_current_right_a)?;
        write_f64(&mut self.writer, s.motor_torque_left_nm)?;
        write_f64(&mut self.writer, s.motor_torque_right_nm)?;
        write_f64(&mut self.writer, s.wheel_force_left_n)?;
        write_f64(&mut self.writer, s.wheel_force_right_n)?;
        write_f64(&mut self.writer, s.desired_wheel_force_left_n)?;
        write_f64(&mut self.writer, s.desired_wheel_force_right_n)?;
        write_f64(&mut self.writer, s.slip_left)?;
        write_f64(&mut self.writer, s.slip_right)?;
        write_f64(&mut self.writer, s.normal_left_n)?;
        write_f64(&mut self.writer, s.normal_right_n)?;
        write_f64(&mut self.writer, s.normal_front_left_n)?;
        write_f64(&mut self.writer, s.normal_front_right_n)?;
        write_f64(&mut self.writer, s.normal_rear_left_n)?;
        write_f64(&mut self.writer, s.normal_rear_right_n)?;
        write_f64(&mut self.writer, s.downforce_extra_n)?;
        write_f64(&mut self.writer, s.downforce_fan_n)?;
        write_f64(&mut self.writer, s.downforce_suction_n)?;
        write_f64(&mut self.writer, s.downforce_current_a)?;
        write_f64(&mut self.writer, s.battery_voltage_v)?;
        write_f64(&mut self.writer, s.battery_current_a)?;
        write_f64(&mut self.writer, s.encoder_left_ticks as f64)?;
        write_f64(&mut self.writer, s.encoder_right_ticks as f64)?;
        write_f64(&mut self.writer, s.encoder_left_velocity_rad_s)?;
        write_f64(&mut self.writer, s.encoder_right_velocity_rad_s)?;
        write_f64(&mut self.writer, s.gyro_yaw_rate_rad_s)?;
        write_f64(&mut self.writer, s.gyro_bias_rad_s)?;
        write_f64(&mut self.writer, s.motor_voltage_left_v)?;
        write_f64(&mut self.writer, s.motor_voltage_right_v)?;
        write_f64(&mut self.writer, s.wheel_surface_speed_left_m_s)?;
        write_f64(&mut self.writer, s.wheel_surface_speed_right_m_s)?;
        self.writer.write_all(&[s.line_visible as u8])?;
        for i in 0..self.sensor_count {
            write_u32(&mut self.writer, s.sensor_adc.get(i).copied().unwrap_or(0))?;
        }
        Ok(())
    }

    pub fn flush(&mut self) -> io::Result<()> {
        self.writer.flush()
    }
}

struct ReplayRow {
    t_us: u64,
    f: [f64; FIXED_F64_COUNT],
    line_visible: bool,
    adc: Vec<u32>,
}

fn read_sample_values<R: Read>(
    reader: &mut R,
    sensor_count: usize,
) -> io::Result<Option<ReplayRow>> {
    let mut first = [0u8; 8];
    match reader.read_exact(&mut first) {
        Ok(()) => {}
        Err(err) if err.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(err) => return Err(err),
    }
    let t_us = u64::from_le_bytes(first);
    let mut f = [0.0f64; FIXED_F64_COUNT];
    for value in &mut f {
        *value = read_f64(reader)?;
    }
    let mut visible = [0u8; 1];
    reader.read_exact(&mut visible)?;
    let mut adc = Vec::with_capacity(sensor_count);
    for _ in 0..sensor_count {
        adc.push(read_u32(reader)?);
    }
    Ok(Some(ReplayRow {
        t_us,
        f,
        line_visible: visible[0] != 0,
        adc,
    }))
}

#[derive(Debug, Clone)]
pub struct ReplayData {
    pub sensor_count: usize,
    pub samples: Vec<TelemetrySample>,
}

fn row_to_telemetry(row: ReplayRow) -> TelemetrySample {
    TelemetrySample {
        t_us: row.t_us,
        x_m: row.f[0],
        y_m: row.f[1],
        yaw_rad: row.f[2],
        vx_body_m_s: row.f[3],
        vy_body_m_s: row.f[4],
        yaw_rate_rad_s: row.f[5],
        line_position_m: row.f[6],
        line_error_m: row.f[7],
        line_visible: row.line_visible,
        line_confidence: row.f[8],
        pwm_left: row.f[9],
        pwm_right: row.f[10],
        pwm_downforce: row.f[11],
        motor_current_left_a: row.f[12],
        motor_current_right_a: row.f[13],
        motor_torque_left_nm: row.f[14],
        motor_torque_right_nm: row.f[15],
        wheel_force_left_n: row.f[16],
        wheel_force_right_n: row.f[17],
        desired_wheel_force_left_n: row.f[18],
        desired_wheel_force_right_n: row.f[19],
        slip_left: row.f[20],
        slip_right: row.f[21],
        normal_left_n: row.f[22],
        normal_right_n: row.f[23],
        normal_front_left_n: row.f[24],
        normal_front_right_n: row.f[25],
        normal_rear_left_n: row.f[26],
        normal_rear_right_n: row.f[27],
        downforce_extra_n: row.f[28],
        downforce_fan_n: row.f[29],
        downforce_suction_n: row.f[30],
        downforce_current_a: row.f[31],
        battery_voltage_v: row.f[32],
        battery_current_a: row.f[33],
        encoder_left_ticks: row.f[34].round() as i64,
        encoder_right_ticks: row.f[35].round() as i64,
        encoder_left_velocity_rad_s: row.f[36],
        encoder_right_velocity_rad_s: row.f[37],
        gyro_yaw_rate_rad_s: row.f[38],
        gyro_bias_rad_s: row.f[39],
        motor_voltage_left_v: row.f[40],
        motor_voltage_right_v: row.f[41],
        wheel_surface_speed_left_m_s: row.f[42],
        wheel_surface_speed_right_m_s: row.f[43],
        sensor_adc: row.adc,
    }
}

fn write_u16<W: Write>(writer: &mut W, value: u16) -> io::Result<()> {
    writer.write_all(&value.to_le_bytes())
}
fn write_u32<W: Write>(writer: &mut W, value: u32) -> io::Result<()> {
    writer.write_all(&value.to_le_bytes())
}
fn write_u64<W: Write>(writer: &mut W, value: u64) -> io::Result<()> {
    writer.write_all(&value.to_le_bytes())
}
fn write_f64<W: Write>(writer: &mut W, value: f64) -> io::Result<()> {
    writer.write_all(&value.to_le_bytes())
}

fn read_u16<R: Read>(reader: &mut R) -> io::Result<u16> {
    let mut b = [0u8; 2];
    reader.read_exact(&mut b)?;
    Ok(u16::from_le_bytes(b))
}
fn read_u32<R: Read>(reader: &mut R) -> io::Result<u32> {
    let mut b = [0u8; 4];
    reader.read_exact(&mut b)?;
    Ok(u32::from_le_bytes(b))
}
fn read_f64<R: Read>(reader: &mut R) -> io::Result<f64> {
    let mut b = [0u8; 8];
    reader.read_exact(&mut b)?;
    Ok(f64::from_le_bytes(b))
}

fn write_record<W: Write>(
    writer: &mut W,
    s: &TelemetrySample,
    sensor_count: usize,
) -> io::Result<()> {
    write_u64(writer, s.t_us)?;
    write_f64(writer, s.x_m)?;
    write_f64(writer, s.y_m)?;
    write_f64(writer, s.yaw_rad)?;
    write_f64(writer, s.vx_body_m_s)?;
    write_f64(writer, s.vy_body_m_s)?;
    write_f64(writer, s.yaw_rate_rad_s)?;
    write_f64(writer, s.line_position_m)?;
    write_f64(writer, s.line_error_m)?;
    write_f64(writer, s.line_confidence)?;
    write_f64(writer, s.pwm_left)?;
    write_f64(writer, s.pwm_right)?;
    write_f64(writer, s.pwm_downforce)?;
    write_f64(writer, s.motor_current_left_a)?;
    write_f64(writer, s.motor_current_right_a)?;
    write_f64(writer, s.motor_torque_left_nm)?;
    write_f64(writer, s.motor_torque_right_nm)?;
    write_f64(writer, s.wheel_force_left_n)?;
    write_f64(writer, s.wheel_force_right_n)?;
    write_f64(writer, s.desired_wheel_force_left_n)?;
    write_f64(writer, s.desired_wheel_force_right_n)?;
    write_f64(writer, s.slip_left)?;
    write_f64(writer, s.slip_right)?;
    write_f64(writer, s.normal_left_n)?;
    write_f64(writer, s.normal_right_n)?;
    write_f64(writer, s.normal_front_left_n)?;
    write_f64(writer, s.normal_front_right_n)?;
    write_f64(writer, s.normal_rear_left_n)?;
    write_f64(writer, s.normal_rear_right_n)?;
    write_f64(writer, s.downforce_extra_n)?;
    write_f64(writer, s.downforce_fan_n)?;
    write_f64(writer, s.downforce_suction_n)?;
    write_f64(writer, s.downforce_current_a)?;
    write_f64(writer, s.battery_voltage_v)?;
    write_f64(writer, s.battery_current_a)?;
    write_f64(writer, s.encoder_left_ticks as f64)?;
    write_f64(writer, s.encoder_right_ticks as f64)?;
    write_f64(writer, s.encoder_left_velocity_rad_s)?;
    write_f64(writer, s.encoder_right_velocity_rad_s)?;
    write_f64(writer, s.gyro_yaw_rate_rad_s)?;
    write_f64(writer, s.gyro_bias_rad_s)?;
    write_f64(writer, s.motor_voltage_left_v)?;
    write_f64(writer, s.motor_voltage_right_v)?;
    write_f64(writer, s.wheel_surface_speed_left_m_s)?;
    write_f64(writer, s.wheel_surface_speed_right_m_s)?;
    writer.write_all(&[s.line_visible as u8])?;
    for i in 0..sensor_count {
        write_u32(writer, s.sensor_adc.get(i).copied().unwrap_or(0))?;
    }
    Ok(())
}

include!("io/replay_v4.rs");

/// Channel order and units of the v4 binary record.
pub const CHANNEL_SCHEMA_JSON: &str = r#"[{"name":"t_us","encoding":"u64-le","unit":"us"},{"name":"x_m","encoding":"f64-le","unit":"m"},{"name":"y_m","encoding":"f64-le","unit":"m"},{"name":"yaw_rad","encoding":"f64-le","unit":"rad"},{"name":"vx_body_m_s","encoding":"f64-le","unit":"m/s"},{"name":"vy_body_m_s","encoding":"f64-le","unit":"m/s"},{"name":"yaw_rate_rad_s","encoding":"f64-le","unit":"rad/s"},{"name":"line_position_m","encoding":"f64-le","unit":"m"},{"name":"line_error_m","encoding":"f64-le","unit":"m"},{"name":"line_confidence","encoding":"f64-le","unit":"1"},{"name":"pwm_left","encoding":"f64-le","unit":"1"},{"name":"pwm_right","encoding":"f64-le","unit":"1"},{"name":"pwm_downforce","encoding":"f64-le","unit":"1"},{"name":"motor_current_left_a","encoding":"f64-le","unit":"A"},{"name":"motor_current_right_a","encoding":"f64-le","unit":"A"},{"name":"motor_torque_left_nm","encoding":"f64-le","unit":"N m"},{"name":"motor_torque_right_nm","encoding":"f64-le","unit":"N m"},{"name":"wheel_force_left_n","encoding":"f64-le","unit":"N"},{"name":"wheel_force_right_n","encoding":"f64-le","unit":"N"},{"name":"desired_wheel_force_left_n","encoding":"f64-le","unit":"N"},{"name":"desired_wheel_force_right_n","encoding":"f64-le","unit":"N"},{"name":"slip_left","encoding":"f64-le","unit":"1"},{"name":"slip_right","encoding":"f64-le","unit":"1"},{"name":"normal_left_n","encoding":"f64-le","unit":"N"},{"name":"normal_right_n","encoding":"f64-le","unit":"N"},{"name":"normal_front_left_n","encoding":"f64-le","unit":"N"},{"name":"normal_front_right_n","encoding":"f64-le","unit":"N"},{"name":"normal_rear_left_n","encoding":"f64-le","unit":"N"},{"name":"normal_rear_right_n","encoding":"f64-le","unit":"N"},{"name":"downforce_extra_n","encoding":"f64-le","unit":"N"},{"name":"downforce_fan_n","encoding":"f64-le","unit":"N"},{"name":"downforce_suction_n","encoding":"f64-le","unit":"N"},{"name":"downforce_current_a","encoding":"f64-le","unit":"A"},{"name":"battery_voltage_v","encoding":"f64-le","unit":"V"},{"name":"battery_current_a","encoding":"f64-le","unit":"A"},{"name":"encoder_left_ticks_legacy_f64","encoding":"f64-le","unit":"tick"},{"name":"encoder_right_ticks_legacy_f64","encoding":"f64-le","unit":"tick"},{"name":"encoder_left_velocity_rad_s","encoding":"f64-le","unit":"rad/s"},{"name":"encoder_right_velocity_rad_s","encoding":"f64-le","unit":"rad/s"},{"name":"gyro_yaw_rate_rad_s","encoding":"f64-le","unit":"rad/s"},{"name":"gyro_bias_rad_s","encoding":"f64-le","unit":"rad/s"},{"name":"motor_voltage_left_v","encoding":"f64-le","unit":"V"},{"name":"motor_voltage_right_v","encoding":"f64-le","unit":"V"},{"name":"wheel_surface_speed_left_m_s","encoding":"f64-le","unit":"m/s"},{"name":"wheel_surface_speed_right_m_s","encoding":"f64-le","unit":"m/s"},{"name":"line_visible","encoding":"u8","unit":"boolean"},{"name":"sensor_adc","encoding":"u32-le[]","unit":"ADC code; frozen sensor order"},{"name":"encoder_left_ticks","encoding":"i64-le","unit":"tick"},{"name":"encoder_right_ticks","encoding":"i64-le","unit":"tick"}]"#;
