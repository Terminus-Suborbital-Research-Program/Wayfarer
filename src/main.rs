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
use image_tools::Starfinder;
use image::{GrayImage, ImageReader, Luma, RgbImage};
use image::DynamicImage;

use crate::image_tools::CameraModel2;
fn main() {
    // init_data();
    // init_k_vector();
    // let mut stars_reader = ObjectReader::new("stars");
    // let mut stars = stars_reader.load_obj::<Vec<Star>>().expect("Failed to load star set");

    // let mut pairs_reader = ObjectReader::new("star_pairs");
    // let mut pairs = pairs_reader.load_obj::<Vec<StarPair>>().expect("Failed to load pairs vec");

    // let mut k_reader = ObjectReader::new("k_vector");
    // let k_vec: Vec<usize> = k_reader.load_obj::<Vec<usize>>().expect("Failed to load k vector table");

    let mut starfinder = Starfinder::default();
    let path = "/home/supergoodname77/Desktop/Elara/startracking/images/set1/bright-VISNIR-310ms_24d1g_50ict_0bl_0d80gam_set1_1.tiff";
    let gray_path = "/home/supergoodname77/Desktop/Elara/startracking/images/set1/output_gray.png";
    let centr_path = "/home/supergoodname77/Desktop/Elara/startracking/images/centr.png";


    let img = ImageReader::open(path).unwrap().decode().unwrap();
    let mut gray_image = GrayImage::from(img.clone());
    let mut centroids = starfinder.star_find(&mut gray_image);

    let cam = CameraModel2::new();
    cam.undistort_centroids(&mut centroids);

    for centroid in centroids {
        println!("Solution 2: X: {} Y: {}", centroid.unit_loc[0], centroid.unit_loc[1])
    }
}

