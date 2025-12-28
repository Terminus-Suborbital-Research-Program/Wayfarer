// use aether::

use aether::math::Vector;

use bincode::{
    config::standard,
    serde::{decode_from_std_read, encode_into_std_write},
};
mod logger;
use logger::{
    ObjectWriter,
    ObjectReader
};
mod stars;
use stars::{
    CamConfig,
    Star,
    StarPair,
};
mod tests;
mod init;
use init::{init_data, init_k_vector};

mod image_tools;


fn main() {
    init_data();
    init_k_vector();
    // let mut stars_reader = ObjectReader::new("stars");
    // let mut stars = stars_reader.load_obj::<Vec<Star>>().expect("Failed to load star set");

    // let mut pairs_reader = ObjectReader::new("star_pairs");
    // let mut pairs = pairs_reader.load_obj::<Vec<StarPair>>().expect("Failed to load pairs vec");

    // let mut k_reader = ObjectReader::new("k_vector");
    // let k_vec: Vec<usize> = k_reader.load_obj::<Vec<usize>>().expect("Failed to load k vector table");
    
}

