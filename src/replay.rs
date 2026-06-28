use crate::telemetry::TelemetrySample;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::Path;

const MAGIC: &[u8; 8] = b"RTSRPL03";
const VERSION: u16 = 3;
const FIXED_F64_COUNT: usize = 44;

pub struct BinaryReplayLogger {
    writer: BufWriter<File>,
    sensor_count: usize,
}

impl BinaryReplayLogger {
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

pub fn export_replay_to_csv(input: &Path, output: &Path) -> io::Result<usize> {
    let mut reader = BufReader::new(File::open(input)?);
    let mut magic = [0u8; 8];
    reader.read_exact(&mut magic)?;
    if &magic != MAGIC {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid RTS replay magic",
        ));
    }
    let version = read_u16(&mut reader)?;
    if version != VERSION {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("unsupported replay version {version}"),
        ));
    }
    let sensor_count = read_u16(&mut reader)? as usize;
    let fixed_count = read_u32(&mut reader)? as usize;
    if fixed_count != FIXED_F64_COUNT {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "unsupported replay fixed field count",
        ));
    }

    let mut writer = BufWriter::new(File::create(output)?);
    write_csv_header(&mut writer, sensor_count)?;
    let mut count = 0usize;
    loop {
        match read_sample_values(&mut reader, sensor_count) {
            Ok(Some(sample)) => {
                write_csv_row(&mut writer, &sample, sensor_count)?;
                count += 1;
            }
            Ok(None) => break,
            Err(err) => return Err(err),
        }
    }
    writer.flush()?;
    Ok(count)
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

fn write_csv_header<W: Write>(writer: &mut W, sensor_count: usize) -> io::Result<()> {
    write!(
        writer,
        "t_us,t_s,x_m,y_m,yaw_rad,vx_body_m_s,vy_body_m_s,yaw_rate_rad_s,line_position_m,line_error_m,line_visible,line_confidence,pwm_left,pwm_right,pwm_downforce,motor_current_left_a,motor_current_right_a,motor_torque_left_nm,motor_torque_right_nm,wheel_force_left_n,wheel_force_right_n,desired_wheel_force_left_n,desired_wheel_force_right_n,slip_left,slip_right,normal_left_n,normal_right_n,normal_front_left_n,normal_front_right_n,normal_rear_left_n,normal_rear_right_n,downforce_extra_n,downforce_fan_n,downforce_suction_n,downforce_current_a,battery_voltage_v,battery_current_a,encoder_left_ticks,encoder_right_ticks,encoder_left_velocity_rad_s,encoder_right_velocity_rad_s,gyro_yaw_rate_rad_s,gyro_bias_rad_s,motor_voltage_left_v,motor_voltage_right_v,wheel_surface_speed_left_m_s,wheel_surface_speed_right_m_s"
    )?;
    for i in 0..sensor_count {
        write!(writer, ",sensor_{:02}_adc", i)?;
    }
    writeln!(writer)
}

fn write_csv_row<W: Write>(writer: &mut W, row: &ReplayRow, sensor_count: usize) -> io::Result<()> {
    write!(writer, "{},{}", row.t_us, row.t_us as f64 / 1_000_000.0)?;
    for (idx, value) in row.f.iter().enumerate() {
        if idx == 8 {
            write!(writer, ",{}", row.line_visible as u8)?;
        }
        write!(writer, ",{:.9}", value)?;
    }
    for i in 0..sensor_count {
        write!(writer, ",{}", row.adc.get(i).copied().unwrap_or(0))?;
    }
    writeln!(writer)
}

#[derive(Debug, Clone)]
pub struct ReplayData {
    pub sensor_count: usize,
    pub samples: Vec<TelemetrySample>,
}

pub fn load_replay_samples(input: &Path, max_samples: usize) -> io::Result<ReplayData> {
    let mut reader = BufReader::new(File::open(input)?);
    let mut magic = [0u8; 8];
    reader.read_exact(&mut magic)?;
    if &magic != MAGIC {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid RTS replay magic",
        ));
    }
    let version = read_u16(&mut reader)?;
    if version != VERSION {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("unsupported replay version {version}"),
        ));
    }
    let sensor_count = read_u16(&mut reader)? as usize;
    let fixed_count = read_u32(&mut reader)? as usize;
    if fixed_count != FIXED_F64_COUNT {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "unsupported replay fixed field count",
        ));
    }

    let mut samples = Vec::new();
    let cap = max_samples.max(1);
    loop {
        match read_sample_values(&mut reader, sensor_count) {
            Ok(Some(row)) => {
                if samples.len() < cap {
                    samples.push(row_to_telemetry(row));
                } else {
                    break;
                }
            }
            Ok(None) => break,
            Err(err) => return Err(err),
        }
    }
    Ok(ReplayData {
        sensor_count,
        samples,
    })
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
