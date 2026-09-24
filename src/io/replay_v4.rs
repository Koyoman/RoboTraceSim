fn read_u64<R: std::io::Read>(reader: &mut R) -> std::io::Result<u64> {
    let mut b = [0; 8];
    reader.read_exact(&mut b)?;
    Ok(u64::from_le_bytes(b))
}
// Included by replay.rs; fixed v3 payload is extended with exact i64 encoder counters.
use std::io::{Seek, SeekFrom};
const MAGIC4: &[u8; 8] = b"RTSRPL04";
const END4: &[u8; 8] = b"ENDRPL04";
fn hash64(data: &[u8]) -> u64 {
    data.iter().fold(14695981039346656037, |h, b| {
        (h ^ *b as u64).wrapping_mul(1099511628211)
    })
}
fn bad(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}
fn record_size(sensors: usize) -> usize {
    8 + 44 * 8 + 1 + sensors * 4
}
pub struct BinaryReplayLogger {
    writer: BufWriter<File>,
    sensor_count: usize,
    buffer: Vec<u8>,
    first: u64,
    last: Option<u64>,
    count: u64,
    total: u64,
    index: Vec<[u64; 4]>,
    finished: bool,
}
impl BinaryReplayLogger {
    pub fn create(path: &Path, sensor_count: usize) -> io::Result<Self> {
        Self::create_with_metadata(path, sensor_count, "{}")
    }
    pub fn create_with_metadata(
        path: &Path,
        sensor_count: usize,
        metadata: &str,
    ) -> io::Result<Self> {
        if sensor_count > 256 || metadata.len() > 16 * 1024 * 1024 {
            return Err(bad("replay metadata/channel limit"));
        }
        crate::json::parse_json(metadata).map_err(|_| bad("metadata JSON"))?;
        let mut writer = BufWriter::new(File::create(path)?);
        writer.write_all(MAGIC4)?;
        write_u16(&mut writer, 4)?;
        write_u16(&mut writer, sensor_count as u16)?;
        write_u32(&mut writer, 44)?;
        write_u32(&mut writer, metadata.len() as u32)?;
        write_u64(&mut writer, hash64(metadata.as_bytes()))?;
        writer.write_all(metadata.as_bytes())?;
        Ok(Self {
            writer,
            sensor_count,
            buffer: Vec::with_capacity((record_size(sensor_count) + 16) * 64),
            first: 0,
            last: None,
            count: 0,
            total: 0,
            index: Vec::new(),
            finished: false,
        })
    }
    pub fn write_sample(&mut self, s: &TelemetrySample) -> io::Result<()> {
        if self.finished {
            return Err(bad("replay already finalized"));
        }
        if self.last.is_some_and(|t| s.t_us <= t) {
            return Err(bad("non-increasing replay time"));
        }
        if self.count == 0 {
            self.first = s.t_us;
        }
        write_record(&mut self.buffer, s, self.sensor_count)?;
        self.buffer
            .extend_from_slice(&s.encoder_left_ticks.to_le_bytes());
        self.buffer
            .extend_from_slice(&s.encoder_right_ticks.to_le_bytes());
        self.last = Some(s.t_us);
        self.count += 1;
        self.total += 1;
        if self.count == 64 {
            self.block()?;
        }
        Ok(())
    }
    fn block(&mut self) -> io::Result<()> {
        if self.count == 0 {
            return Ok(());
        }
        let offset = self.writer.stream_position()?;
        self.writer.write_all(b"BLK4")?;
        write_u32(&mut self.writer, self.count as u32)?;
        write_u64(&mut self.writer, hash64(&self.buffer))?;
        self.writer.write_all(&self.buffer)?;
        self.index
            .push([self.first, self.last.unwrap(), offset, self.count]);
        self.buffer.clear();
        self.count = 0;
        Ok(())
    }
    pub fn finish(&mut self, reason: &str) -> io::Result<()> {
        if reason.len() > 4096 {
            return Err(bad("termination metadata limit"));
        }
        if self.finished {
            return Ok(());
        }
        self.block()?;
        let offset = self.writer.stream_position()?;
        let mut footer = Vec::new();
        footer.extend_from_slice(b"IDX4");
        write_u64(&mut footer, self.index.len() as u64)?;
        for entry in &self.index {
            for value in entry {
                write_u64(&mut footer, *value)?;
            }
        }
        write_u64(&mut footer, self.total)?;
        write_u32(&mut footer, reason.len() as u32)?;
        footer.extend_from_slice(reason.as_bytes());
        self.writer.write_all(&footer)?;
        write_u64(&mut self.writer, offset)?;
        write_u64(&mut self.writer, hash64(&footer))?;
        self.writer.write_all(END4)?;
        self.writer.flush()?;
        self.finished = true;
        Ok(())
    }
    pub fn flush(&mut self) -> io::Result<()> {
        self.finish("complete")
    }
}
/// Only one block is cached. The time index stays on disk and is binary-searched.
pub struct IndexedReplay {
    file: File,
    pub sensor_count: usize,
    pub metadata: String,
    pub termination: String,
    pub samples: u64,
    pub duration_us: u64,
    pub legacy: bool,
    index_offset: u64,
    blocks: u64,
    stride: usize,
    cache: Vec<u8>,
    cached_block: Option<u64>,
    cache_count: usize,
    budget: usize,
}
impl IndexedReplay {
    pub fn open(path: &Path, budget: usize) -> io::Result<Self> {
        let mut file = File::open(path)?;
        let len = file.metadata()?.len();
        let mut magic = [0; 8];
        file.read_exact(&mut magic)?;
        let version = read_u16(&mut file)?;
        let sensors = read_u16(&mut file)? as usize;
        let fixed = read_u32(&mut file)?;
        if sensors > 256 || fixed != 44 {
            return Err(bad("unsupported replay channels/layout"));
        }
        let legacy = &magic == MAGIC && version == 3;
        let mut reader = Self {
            file,
            sensor_count: sensors,
            metadata: String::new(),
            termination: String::new(),
            samples: 0,
            duration_us: 0,
            legacy,
            index_offset: 0,
            blocks: 0,
            stride: record_size(sensors) + if legacy { 0 } else { 16 },
            cache: Vec::new(),
            cached_block: None,
            cache_count: 0,
            budget,
        };
        if legacy {
            if len < 16 || (len - 16) % reader.stride as u64 != 0 {
                return Err(bad("incomplete v3 record"));
            }
            reader.samples = (len - 16) / reader.stride as u64;
            reader.termination = "legacy-v3-end-unverifiable".into();
            if budget < reader.stride {
                return Err(bad("viewer budget smaller than a record"));
            }
        } else {
            if &magic != MAGIC4 || version != 4 {
                return Err(bad("unsupported replay version"));
            }
            let n = read_u32(&mut reader.file)? as usize;
            let checksum = read_u64(&mut reader.file)?;
            if n > 16 * 1024 * 1024 {
                return Err(bad("metadata limit"));
            }
            let mut bytes = vec![0; n];
            reader.file.read_exact(&mut bytes)?;
            if hash64(&bytes) != checksum {
                return Err(bad("metadata checksum mismatch"));
            }
            reader.metadata = String::from_utf8(bytes).map_err(|_| bad("metadata UTF-8"))?;
            crate::json::parse_json(&reader.metadata).map_err(|_| bad("metadata JSON"))?;
            let data_start = 28 + n as u64;
            if len < data_start + 24 {
                return Err(bad("incomplete replay: missing footer"));
            }
            reader.file.seek(SeekFrom::End(-24))?;
            let offset = read_u64(&mut reader.file)?;
            let expected = read_u64(&mut reader.file)?;
            let mut end = [0; 8];
            reader.file.read_exact(&mut end)?;
            if &end != END4 || offset < data_start || offset > len - 24 {
                return Err(bad("incomplete replay footer"));
            }
            reader.file.seek(SeekFrom::Start(offset))?;
            let mut remaining = len - 24 - offset;
            let mut h = 14695981039346656037u64;
            let mut chunk = [0u8; 8192];
            while remaining > 0 {
                let n = remaining.min(chunk.len() as u64) as usize;
                reader.file.read_exact(&mut chunk[..n])?;
                for b in &chunk[..n] {
                    h = (h ^ *b as u64).wrapping_mul(1099511628211);
                }
                remaining -= n as u64;
            }
            if h != expected {
                return Err(bad("replay index checksum mismatch"));
            }
            reader.file.seek(SeekFrom::Start(offset))?;
            let mut tag = [0; 4];
            reader.file.read_exact(&mut tag)?;
            if &tag != b"IDX4" {
                return Err(bad("invalid replay index"));
            }
            reader.blocks = read_u64(&mut reader.file)?;
            if reader.blocks > (len - offset - 24) / 32 {
                return Err(bad("invalid index size"));
            }
            reader.index_offset = offset + 12;
            reader
                .file
                .seek(SeekFrom::Start(reader.index_offset + reader.blocks * 32))?;
            reader.samples = read_u64(&mut reader.file)?;
            let n = read_u32(&mut reader.file)? as usize;
            if n > 4096 {
                return Err(bad("termination metadata limit"));
            }
            let mut reason = vec![0; n];
            reader.file.read_exact(&mut reason)?;
            reader.termination = String::from_utf8(reason).map_err(|_| bad("termination UTF-8"))?;
            if reader.file.stream_position()? != len - 24 {
                return Err(bad("invalid index extent"));
            }
            let mut total = 0;
            let mut next_offset = data_start;
            let mut previous = None;
            for index in 0..reader.blocks {
                crate::experiments::jobs::check_cancelled().map_err(io::Error::other)?;
                let e = reader.entry(index)?;
                if e[2] != next_offset
                    || e[3] == 0
                    || e[3] > 64
                    || (index + 1 < reader.blocks && e[3] != 64)
                    || e[0] > e[1]
                    || previous.is_some_and(|t| e[0] <= t)
                {
                    return Err(bad("invalid block index"));
                }
                next_offset = e[2] + 16 + e[3] * reader.stride as u64;
                previous = Some(e[1]);
                total += e[3];
            }
            if total != reader.samples || next_offset != offset {
                return Err(bad("replay sample/extent mismatch"));
            }
            if budget < reader.stride * 64 {
                return Err(bad("viewer cache budget smaller than one replay block"));
            }
        }
        if !reader.legacy {
            reader.cache = Vec::with_capacity(reader.stride * 64);
        }
        if reader.samples > 0 {
            reader.duration_us = reader.sample(reader.samples - 1)?.t_us;
        }
        Ok(reader)
    }
    fn entry(&mut self, index: u64) -> io::Result<[u64; 4]> {
        self.file
            .seek(SeekFrom::Start(self.index_offset + index * 32))?;
        Ok([
            read_u64(&mut self.file)?,
            read_u64(&mut self.file)?,
            read_u64(&mut self.file)?,
            read_u64(&mut self.file)?,
        ])
    }
    pub fn cached_bytes(&self) -> usize {
        self.cache.capacity()
    }
    pub fn sample(&mut self, index: u64) -> io::Result<TelemetrySample> {
        if index >= self.samples {
            return Err(bad("replay sample out of bounds"));
        }
        if self.legacy {
            self.file
                .seek(SeekFrom::Start(16 + index * self.stride as u64))?;
            return Ok(row_to_telemetry(
                read_sample_values(&mut self.file, self.sensor_count)?
                    .ok_or_else(|| bad("missing sample"))?,
            ));
        }
        let block = index / 64;
        if self.cached_block != Some(block) {
            let e = self.entry(block)?;
            self.file.seek(SeekFrom::Start(e[2]))?;
            let mut tag = [0; 4];
            self.file.read_exact(&mut tag)?;
            let count = read_u32(&mut self.file)? as usize;
            let checksum = read_u64(&mut self.file)?;
            if &tag != b"BLK4" || count as u64 != e[3] || count * self.stride > self.budget {
                return Err(bad("invalid replay block"));
            }
            self.cache.resize(count * self.stride, 0);
            self.file.read_exact(&mut self.cache)?;
            if hash64(&self.cache) != checksum {
                return Err(bad("replay block checksum mismatch"));
            }
            let mut previous = None;
            for (i, row) in self.cache.chunks_exact(self.stride).enumerate() {
                let t = u64::from_le_bytes(row[..8].try_into().unwrap());
                if previous.is_some_and(|v| t <= v)
                    || (i == 0 && t != e[0])
                    || (i + 1 == count && t != e[1])
                {
                    return Err(bad("invalid block timestamps"));
                }
                previous = Some(t);
            }
            self.cached_block = Some(block);
            self.cache_count = count;
        }
        let offset = (index % 64) as usize * self.stride;
        let mut cursor = io::Cursor::new(&self.cache[offset..offset + self.stride]);
        let mut sample = row_to_telemetry(
            read_sample_values(&mut cursor, self.sensor_count)?
                .ok_or_else(|| bad("missing sample"))?,
        );
        let mut bytes = [0; 8];
        cursor.read_exact(&mut bytes)?;
        sample.encoder_left_ticks = i64::from_le_bytes(bytes);
        cursor.read_exact(&mut bytes)?;
        sample.encoder_right_ticks = i64::from_le_bytes(bytes);
        Ok(sample)
    }
    pub fn at_time(&mut self, time: u64, interpolate: bool) -> io::Result<TelemetrySample> {
        if self.samples == 0 {
            return Err(bad("empty replay"));
        }
        let mut lo = 0;
        let mut hi = self.samples;
        while lo < hi {
            let mid = lo + (hi - lo) / 2;
            if self.sample(mid)?.t_us <= time {
                lo = mid + 1
            } else {
                hi = mid
            }
        }
        let index = lo.saturating_sub(1);
        let mut a = self.sample(index)?;
        if interpolate && index + 1 < self.samples && time >= a.t_us {
            let b = self.sample(index + 1)?;
            let alpha = (time - a.t_us) as f64 / (b.t_us - a.t_us) as f64;
            a.x_m += (b.x_m - a.x_m) * alpha;
            a.y_m += (b.y_m - a.y_m) * alpha;
            let dy = (b.yaw_rad - a.yaw_rad + std::f64::consts::PI)
                .rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            a.yaw_rad += dy * alpha;
        }
        Ok(a)
    }
}
pub fn load_replay_samples(input: &Path, max_samples: usize) -> io::Result<ReplayData> {
    let mut r = IndexedReplay::open(input, 2 * 1024 * 1024)?;
    if r.samples > max_samples as u64 {
        return Err(bad(
            "replay exceeds sample budget; use IndexedReplay instead of truncation",
        ));
    }
    let samples = (0..r.samples)
        .map(|i| r.sample(i))
        .collect::<io::Result<Vec<_>>>()?;
    Ok(ReplayData {
        sensor_count: r.sensor_count,
        samples,
    })
}
pub fn export_replay_to_csv(input: &Path, output: &Path) -> io::Result<usize> {
    if input == output
        || (output.exists() && std::fs::canonicalize(input)? == std::fs::canonicalize(output)?)
    {
        return Err(bad("input and output must differ"));
    }
    let mut r = IndexedReplay::open(input, 2 * 1024 * 1024)?;
    crate::experiments::jobs::write_output_atomic(output, |temporary| {
        let mut logger = crate::telemetry::CsvLogger::create(temporary, r.sensor_count)
            .map_err(|e| e.to_string())?;
        for i in 0..r.samples {
            crate::experiments::jobs::check_cancelled()?;
            logger
                .write_sample(&r.sample(i).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
        }
        logger.flush().map_err(|e| e.to_string())?;
        drop(logger);
        Ok(r.samples as usize)
    })
    .map_err(io::Error::other)
}
