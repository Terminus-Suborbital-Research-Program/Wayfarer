use divan::{Bencher, black_box};

use wayfarer::{
    perception::{
    camera_model::CameraModel,
    centroiding::Starfinder,
    },
    startrack::solver::Startracker,
};
use image::GrayImage;

use rand::RngExt;
use std::fs;


fn main() {
    divan::main();
}



#[divan::bench]
fn tracker_bench(bencher: Bencher) {
    let starfinder = Starfinder::default();
    let camera_model = CameraModel::default();
    let startracker = Startracker::default();

    // Load all images and select a random one, repeat bench against it
    let images: Vec<GrayImage> = fs::read_dir("../images/mixed")
        .unwrap()
        .filter_map(|e| e.ok())
        .map(|e| {
            let img = image::open(e.path()).unwrap();
            image::GrayImage::from(img)
        })
        .collect();

    let mut rng = rand::rng();
    let len = images.len();
    let idx = rng.random_range(0..len);

    let sample_image = images[idx].clone();

    bencher
        .with_inputs(|| sample_image.clone()) 
        .bench_values(|mut img| {
            let mut centroids = starfinder.star_find(&mut img);
            
            camera_model.undistort_centroids(&mut centroids);
            
            // Wrap in black_box so the compiler doesn't optimize it away
            black_box(startracker.pyramid_solve(centroids))
        });
}