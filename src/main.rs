// #![allow(warnings)]
mod logger;
mod startrack;
mod init;
mod perception;
mod tests;

use std::io;
use std::fs::{self, DirEntry};
use std::path::Path;
use aether::math::{Matrix, Vector};
use image::{GrayImage, ImageReader, Luma, RgbImage};
use image::DynamicImage;
use std::time::Instant;

use bincode::{
    config::standard,
    serde::{decode_from_std_read, encode_into_std_write},
};
use logger::{
    ObjectWriter,
    ObjectReader
};
use startrack::{
    solver::Startracker,
    quest::quest,
};
use init::{init_data, init_k_vector};
use perception::{
    camera_model::CameraModel,
    centroiding::{
        Starfinder,
        Centroid,
    }
};


fn main() -> io::Result<()>{
    // init_data();
    // init_k_vector();
    // let camera_view_centroids: Vec<Centroid> = centroids.iter().cloned().collect();
    
    let path = "/home/supergoodname77/Desktop/Elara/startracking/images/set1/bright-VISNIR-310ms_24d1g_50ict_0bl_0d80gam_set1_1.tiff";
    let mut starfinder = Starfinder::default();
    let camera_model = CameraModel::default();
    let startracker = Startracker::default();

    full_solve(path, starfinder, camera_model, startracker);

    
    
    Ok(())
}


fn full_solve(path: &str, starfinder: Starfinder, camera_model: CameraModel, startracker: Startracker) {
    let img = ImageReader::open(path).unwrap().decode().unwrap();
    let mut gray_image = GrayImage::from(img);
    let mut centroids = starfinder.star_find(&mut gray_image);
    camera_model.undistort_centroids(&mut centroids);
    match startracker.pyramid_solve(centroids) {
    // match startracker.exhaustive_solve(centroids, 100) {
        Ok((reference_vectors, body_vectors)) => {
            let q = quest(&reference_vectors, &body_vectors);

            for (reference, body) in reference_vectors.iter().zip(body_vectors.iter()) {
                let star = q.rotate_vector(*reference);

                if let Some((cat_x, cat_y)) = camera_model.project_vector(star) {
                    if let Some((body_x, body_y)) = camera_model.project_vector(*body) {
                    println!("Check Coords");
                    println!("------------");
                    println!("Cat:  X: {:.5} Y: {:.5}", cat_x, cat_y);
                    println!("Body: X: {:.5} Y: {:.5}", body_x, body_y);
                    println!("------------");             
                    } else {
                        eprintln!("Measured star body vector failed projection (should be impossible?)");
                    }
                } else {
                    eprintln!("Solved star is behind camera (Bad Quaternion?)");
                }
                

            }
            // let star = q.rotate_vector(reference_vectors[0]);
            // let (cat_x, cat_y) = camera_model.project_vector(star).unwrap();
            // let (body_x, body_y) = camera_model.project_vector(body_vectors[0]).unwrap();
        
        }

        Err(e) => {
            eprintln!("{}",e);
        }
    }
}
/*
Q: Vector { data: [0.22732334822563513, 0.16095940395572375, -0.26835912918251825, 0.9221711031116218] }


Q: Vector { data: [0.22732334822563513, 0.16095940395572375, -0.26835912918251825, 0.9221711031116218] }


New weighting:
Q: Vector { data: [0.6749978441939833, -0.6161430734387161, -0.4050549700611159, 0.02600181947011219] }
--- VERIFICATION ---
Star Error: 0.66430 degrees
Star Error: 0.18064 degrees
Star Error: 0.53094 degrees
Star Error: 0.29163 degrees
Average Error: 0.41688 degrees

Old weighting:

warning: `usno_gnc_catalog` (lib) generated 1 warning (run `cargo fix --lib -p usno_gnc_catalog` to apply 1 suggestion)
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.03s
     Running `target/debug/wayfarer`
Q: Vector { data: [-0.22732334822563513, -0.16095940395572375, 0.26835912918251825, 0.9221711031116218] }
--- VERIFICATION ---
Star Error: 0.17467 degrees
Star Error: 0.10410 degrees
Star Error: 0.11383 degrees
Star Error: 0.04710 degrees
Average Error: 0.10992 degrees


*/
