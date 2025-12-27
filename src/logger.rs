use bincode::{
    config::standard,
    // decode_from_slice, encode_into_slice,
    serde::{decode_from_std_read, encode_into_std_write},
};
use std::fs::File;
use std::path::PathBuf;
use std::{
    fs,
    io::{BufReader, BufWriter, Write},
};
use serde::{de::DeserializeOwned, Serialize};
pub struct ObjectWriter {
    writer: BufWriter<File>,
}

impl ObjectWriter {
    pub fn new(file_path: &str) -> Self {
        let file = File::create(file_path).expect("Could not open baseline file!");
        Self {
            writer: BufWriter::new(file),
        }
    }

    pub fn write_obj<T: Serialize>(&mut self, object: &T) -> Result<usize,bincode::error::EncodeError>{
        encode_into_std_write(&object, &mut self.writer, standard())
    }
}

pub struct ObjectReader {
    reader: BufReader<File>,
}
impl ObjectReader {
    pub fn new(file_path: &str) -> Self {
        let file = File::open(file_path).expect("Could not open baseline file!");
        Self {
            reader: BufReader::new(file),
        }
    }

    pub fn load_obj<T: DeserializeOwned>(&mut self) -> Result<T, bincode::error::DecodeError> {
        decode_from_std_read(&mut self.reader, standard())
    }
}
