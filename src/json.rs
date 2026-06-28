use std::collections::BTreeMap;
use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum JsonValue {
    Null,
    Bool(bool),
    Number(f64),
    String(String),
    Array(Vec<JsonValue>),
    Object(BTreeMap<String, JsonValue>),
}

impl JsonValue {
    pub fn get(&self, key: &str) -> Option<&JsonValue> {
        match self {
            JsonValue::Object(obj) => obj.get(key),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            JsonValue::String(s) => Some(s),
            _ => None,
        }
    }

    pub fn as_f64(&self) -> Option<f64> {
        match self {
            JsonValue::Number(n) => Some(*n),
            _ => None,
        }
    }

    pub fn as_u64(&self) -> Option<u64> {
        self.as_f64().and_then(|n| {
            if n >= 0.0 && n.fract().abs() < f64::EPSILON {
                Some(n as u64)
            } else {
                None
            }
        })
    }

    pub fn as_array(&self) -> Option<&[JsonValue]> {
        match self {
            JsonValue::Array(v) => Some(v),
            _ => None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct JsonError {
    pub pos: usize,
    pub message: String,
}

impl JsonError {
    fn new(pos: usize, message: impl Into<String>) -> Self {
        Self { pos, message: message.into() }
    }
}

impl fmt::Display for JsonError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "JSON parse error at byte {}: {}", self.pos, self.message)
    }
}

impl std::error::Error for JsonError {}

pub fn parse_json(input: &str) -> Result<JsonValue, JsonError> {
    let mut parser = Parser { bytes: input.as_bytes(), pos: 0 };
    let value = parser.parse_value()?;
    parser.skip_ws();
    if parser.pos != parser.bytes.len() {
        return Err(JsonError::new(parser.pos, "trailing characters after JSON value"));
    }
    Ok(value)
}

struct Parser<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Parser<'a> {
    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.pos).copied()
    }

    fn bump(&mut self) -> Option<u8> {
        let b = self.peek()?;
        self.pos += 1;
        Some(b)
    }

    fn skip_ws(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\n' | b'\r' | b'\t')) {
            self.pos += 1;
        }
    }

    fn parse_value(&mut self) -> Result<JsonValue, JsonError> {
        self.skip_ws();
        match self.peek() {
            Some(b'n') => self.parse_literal(b"null", JsonValue::Null),
            Some(b't') => self.parse_literal(b"true", JsonValue::Bool(true)),
            Some(b'f') => self.parse_literal(b"false", JsonValue::Bool(false)),
            Some(b'\"') => Ok(JsonValue::String(self.parse_string()?)),
            Some(b'[') => self.parse_array(),
            Some(b'{') => self.parse_object(),
            Some(b'-' | b'0'..=b'9') => self.parse_number(),
            Some(other) => Err(JsonError::new(self.pos, format!("unexpected byte {:?}", other as char))),
            None => Err(JsonError::new(self.pos, "unexpected end of input")),
        }
    }

    fn parse_literal(&mut self, literal: &[u8], value: JsonValue) -> Result<JsonValue, JsonError> {
        if self.bytes.len() >= self.pos + literal.len()
            && &self.bytes[self.pos..self.pos + literal.len()] == literal
        {
            self.pos += literal.len();
            Ok(value)
        } else {
            Err(JsonError::new(self.pos, "invalid literal"))
        }
    }

    fn parse_string(&mut self) -> Result<String, JsonError> {
        let quote = self.bump();
        debug_assert_eq!(quote, Some(b'\"'));
        let mut out = String::new();
        while let Some(b) = self.bump() {
            match b {
                b'\"' => return Ok(out),
                b'\\' => {
                    let esc = self.bump().ok_or_else(|| JsonError::new(self.pos, "unfinished escape"))?;
                    match esc {
                        b'\"' => out.push('\"'),
                        b'\\' => out.push('\\'),
                        b'/' => out.push('/'),
                        b'b' => out.push('\u{0008}'),
                        b'f' => out.push('\u{000C}'),
                        b'n' => out.push('\n'),
                        b'r' => out.push('\r'),
                        b't' => out.push('\t'),
                        b'u' => {
                            let code = self.parse_hex4()?;
                            let ch = char::from_u32(code as u32).ok_or_else(|| {
                                JsonError::new(self.pos, "invalid unicode scalar value")
                            })?;
                            out.push(ch);
                        }
                        _ => return Err(JsonError::new(self.pos, "invalid escape sequence")),
                    }
                }
                0x00..=0x1F => return Err(JsonError::new(self.pos, "control character in string")),
                _ => out.push(b as char),
            }
        }
        Err(JsonError::new(self.pos, "unterminated string"))
    }

    fn parse_hex4(&mut self) -> Result<u16, JsonError> {
        let mut value = 0u16;
        for _ in 0..4 {
            let b = self.bump().ok_or_else(|| JsonError::new(self.pos, "short unicode escape"))?;
            let digit = match b {
                b'0'..=b'9' => (b - b'0') as u16,
                b'a'..=b'f' => (b - b'a' + 10) as u16,
                b'A'..=b'F' => (b - b'A' + 10) as u16,
                _ => return Err(JsonError::new(self.pos, "invalid unicode escape digit")),
            };
            value = (value << 4) | digit;
        }
        Ok(value)
    }

    fn parse_array(&mut self) -> Result<JsonValue, JsonError> {
        self.bump();
        let mut values = Vec::new();
        loop {
            self.skip_ws();
            if matches!(self.peek(), Some(b']')) {
                self.bump();
                break;
            }
            values.push(self.parse_value()?);
            self.skip_ws();
            match self.peek() {
                Some(b',') => {
                    self.bump();
                }
                Some(b']') => {
                    self.bump();
                    break;
                }
                _ => return Err(JsonError::new(self.pos, "expected ',' or ']' in array")),
            }
        }
        Ok(JsonValue::Array(values))
    }

    fn parse_object(&mut self) -> Result<JsonValue, JsonError> {
        self.bump();
        let mut values = BTreeMap::new();
        loop {
            self.skip_ws();
            if matches!(self.peek(), Some(b'}')) {
                self.bump();
                break;
            }
            if !matches!(self.peek(), Some(b'\"')) {
                return Err(JsonError::new(self.pos, "expected object key string"));
            }
            let key = self.parse_string()?;
            self.skip_ws();
            if self.bump() != Some(b':') {
                return Err(JsonError::new(self.pos, "expected ':' after object key"));
            }
            let value = self.parse_value()?;
            values.insert(key, value);
            self.skip_ws();
            match self.peek() {
                Some(b',') => {
                    self.bump();
                }
                Some(b'}') => {
                    self.bump();
                    break;
                }
                _ => return Err(JsonError::new(self.pos, "expected ',' or '}' in object")),
            }
        }
        Ok(JsonValue::Object(values))
    }

    fn parse_number(&mut self) -> Result<JsonValue, JsonError> {
        let start = self.pos;
        if matches!(self.peek(), Some(b'-')) {
            self.bump();
        }
        match self.peek() {
            Some(b'0') => {
                self.bump();
            }
            Some(b'1'..=b'9') => {
                while matches!(self.peek(), Some(b'0'..=b'9')) {
                    self.bump();
                }
            }
            _ => return Err(JsonError::new(self.pos, "invalid number")),
        }
        if matches!(self.peek(), Some(b'.')) {
            self.bump();
            let frac_start = self.pos;
            while matches!(self.peek(), Some(b'0'..=b'9')) {
                self.bump();
            }
            if self.pos == frac_start {
                return Err(JsonError::new(self.pos, "expected digits after decimal point"));
            }
        }
        if matches!(self.peek(), Some(b'e' | b'E')) {
            self.bump();
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.bump();
            }
            let exp_start = self.pos;
            while matches!(self.peek(), Some(b'0'..=b'9')) {
                self.bump();
            }
            if self.pos == exp_start {
                return Err(JsonError::new(self.pos, "expected exponent digits"));
            }
        }
        let s = std::str::from_utf8(&self.bytes[start..self.pos])
            .map_err(|_| JsonError::new(start, "number is not UTF-8"))?;
        let number = s.parse::<f64>().map_err(|_| JsonError::new(start, "invalid f64 number"))?;
        Ok(JsonValue::Number(number))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_object() {
        let v = parse_json(r#"{"a": 1, "b": [true, null, "x"]}"#).unwrap();
        assert_eq!(v.get("a").and_then(JsonValue::as_u64), Some(1));
        assert_eq!(v.get("b").and_then(JsonValue::as_array).unwrap().len(), 3);
    }
}
