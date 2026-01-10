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
mod startrack;
use startrack::{
    solver::Startracker
};
mod tests;
mod init;
use init::{init_data, init_k_vector};

// mod image_tools;
mod perception;


use perception::{
    camera_model::CameraModel,
    centroiding::{
        Starfinder,
        Centroid,
    }
};
use image::{GrayImage, ImageReader, Luma, RgbImage};
use image::DynamicImage;



fn main() {
    init_data();
    init_k_vector();

    let path = "/home/supergoodname77/Desktop/Elara/startracking/images/set1/bright-VISNIR-310ms_24d1g_50ict_0bl_0d80gam_set1_1.tiff";

    let img = ImageReader::open(path).unwrap().decode().unwrap();
    let mut gray_image = GrayImage::from(img.clone());

    let mut starfinder = Starfinder::default();

    let mut centroids = starfinder.star_find(&mut gray_image);

    let camera_view_centroids: Vec<Centroid> = centroids.iter().cloned().collect();

    let camera_model = CameraModel::default();

    camera_model.undistort_centroids(&mut centroids);

    let startracker = Startracker::default();

    startracker.pyramid_solve(centroids);

}

