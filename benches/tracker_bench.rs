use divan::{Bencher, black_box};
// use crate::logger::{
//     ObjectWriter,
//     ObjectReader
// };

// use crate::startrack::stardat::{
//     // CamConfig,
//     Star,
//     StarPair,
// };


    use wayfarer::perception::{
        camera_model::CameraModel,
        centroiding::{
            Starfinder,
            Centroid,
        }
    };

    use wayfarer::startrack::{
        solver::Startracker,
        quest::quest,
    };
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



fn main() {
    divan::main();
}



#[divan::bench]
fn tracker_bench(bencher: Bencher) {
    let starfinder = Starfinder::default();
    let camera_model = CameraModel::default();
    let startracker = Startracker::default();
    
    let images: Vec<_> = fs::read_dir("../images/mixed")
        .unwrap()
        .filter_map(|e| e.ok())
        .map(|e| {
            let img = image::open(e.path()).unwrap();
            image::GrayImage::from(img)
        })
        .collect();

    bencher.bench_local(move || {
        for mut gray_image in images.clone() {
            let mut centroids = starfinder.star_find(&mut gray_image);
            camera_model.undistort_centroids(&mut centroids);
            let _ = black_box(startracker.pyramid_solve(centroids));
        }
    });
}

#[divan::bench]
fn tracker_bench_single_frame(bencher: Bencher) {
    // 1. Global Setup (Run once)
    let starfinder = Starfinder::default();
    let camera_model = CameraModel::default();
    let startracker = Startracker::default();
    
    // Load ALL images, but we will just pick the first one to benchmark
    // This represents a "typical" frame.
    let images: Vec<GrayImage> = fs::read_dir("../images/mixed")
        .unwrap()
        .filter_map(|e| e.ok())
        .map(|e| {
            let img = image::open(e.path()).unwrap();
            image::GrayImage::from(img)
        })
        .collect();

    let sample_image = images.first().expect("No images in folder!").clone();

    bencher
        // 2. Per-Iteration Setup (NOT TIMED)
        // We clone the image here so the cost of "copying" is excluded from the result.
        // We need a fresh image every time because star_find modifies it.
        .with_inputs(|| sample_image.clone()) 
        
        // 3. The Measurement (TIMED)
        .bench_values(|mut img| {
            // A. Find
            let mut centroids = starfinder.star_find(&mut img);
            
            // B. Undistort
            camera_model.undistort_centroids(&mut centroids);
            
            // C. Solve
            // Wrap in black_box so the compiler doesn't optimize it away
            black_box(startracker.pyramid_solve(centroids))
        });
}