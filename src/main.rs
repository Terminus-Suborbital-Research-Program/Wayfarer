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
    Startracker,
};
mod tests;
mod init;
use init::{init_data, init_k_vector};

mod image_tools;
use image_tools::Starfinder;
use image::{GrayImage, ImageReader, Luma, RgbImage};
use image::DynamicImage;

use crate::image_tools::CameraModel;
fn main() {
    // init_data();
    // init_k_vector();
    // let mut stars_reader = ObjectReader::new("stars");
    // let mut stars = stars_reader.load_obj::<Vec<Star>>().expect("Failed to load star set");

    // let mut pairs_reader = ObjectReader::new("star_pairs");
    // let mut pairs = pairs_reader.load_obj::<Vec<StarPair>>().expect("Failed to load pairs vec");

    let mut k_reader = ObjectReader::new("k_vector");
    let k_vec: Vec<usize> = k_reader.load_obj::<Vec<usize>>().expect("Failed to load k vector table");

    let path = "/home/supergoodname77/Desktop/Elara/startracking/images/set1/bright-VISNIR-310ms_24d1g_50ict_0bl_0d80gam_set1_1.tiff";
    let gray_path = "/home/supergoodname77/Desktop/Elara/startracking/images/set1/output_gray.png";
    let centr_path = "/home/supergoodname77/Desktop/Elara/startracking/images/centr.png";


    let img = ImageReader::open(path).unwrap().decode().unwrap();
    let mut gray_image = GrayImage::from(img.clone());

    let mut starfinder = Starfinder::default();

    let mut centroids = starfinder.star_find(&mut gray_image);

    starfinder.undistort_centroids(&mut centroids);

    let startracker = Startracker::default();
    // let cam = CameraModel::default();
    // cam.undistort_centroids(&mut centroids);

    let mut j = 0;
    let mut k = 0;

    // Core Lost in space identification loop
    let n = centroids.len();
    for dj in 0..n-1 {
        for dk in 0..n-dj-1 {
            for i in 0.. n-dj-dk {
                j = i + dj;
                k = j + dk;

                // We take the unit vectors relative to the center of the camer, of 3 centroids
                let b1 = &centroids[i].unit_loc;
                let b2 = &centroids[j].unit_loc;
                let b3 = &centroids[k].unit_loc;

                // We compute their "legs", cosine theta values that can be used to represent the 
                // xx
                let c_theta_1 = b1.dot(b2);
                let c_theta_2 = b1.dot(b3);
                let c_theta_3 = b2.dot(b3);

                match startracker.query_triangle_topology(&[c_theta_1, c_theta_2, c_theta_3]) {
                    Ok(ids) => {
                        for r in 0..n {
                            if r == i || r == j || r == k {continue;}

                            let b4 = &centroids[r].unit_loc;
                            let confirmation_leg_1 = b1.dot(b4);
                            let confirmation_leg_2 = b2.dot(b4);
                            let confirmation_leg_3 = b3.dot(b4);

                            match startracker.pyramid_confirmation(
                                &[confirmation_leg_1, confirmation_leg_2, confirmation_leg_3], &ids) {
                                Ok(star_r_index) => {
                                    let star_r = startracker.retrieve_unit_vector(star_r_index);
                                }
                                Err(startracker_err) => {eprintln!("{}",startracker_err)}
                            }
                            
                            

                        }
                    }
                    Err(startracker_err) => {eprintln!("{}",startracker_err)}
                }
            }
        }
    }

    // for centroid in centroids {
    //     println!("Solution 2: X: {} Y: {}", centroid.unit_loc[0], centroid.unit_loc[1])
    // }
}

