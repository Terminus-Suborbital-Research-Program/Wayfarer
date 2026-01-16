


#[cfg(test)]
mod tests {
    // use aether::math::{Vector, Matrix};

    use crate::perception::{
        camera_model::CameraModel,
        centroiding::{
            Starfinder,
            Centroid,
        }
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
    use crate::logger::{
        ObjectWriter,
        ObjectReader
    };
    use crate::startrack::{
        solver::Startracker,
        quest::quest,
    };


    #[test]
    fn benchmark_full_solve() {
        let mut starfinder = Starfinder::default();
        let camera_model = CameraModel::default();
        let startracker = Startracker::default();

        for entry in fs::read_dir("/home/supergoodname77/Desktop/Elara/startracking/images/mixed").unwrap() {
            let entry = entry.unwrap();
            let path = entry.path();

            if path.is_file() {
                // 1. Measure IO/Decoding
                let t0 = Instant::now();
                let img = ImageReader::open(&path).unwrap().decode().unwrap();
                let mut gray_image = GrayImage::from(img);
                let t_load = t0.elapsed();

                // 2. Measure Star Finding
                // Note: This is usually the bottleneck due to pixel iterating
                let t1 = Instant::now();
                let mut centroids = starfinder.star_find(&mut gray_image);
                let t_find = t1.elapsed();

                // 3. Measure Undistortion
                let t2 = Instant::now();
                camera_model.undistort_centroids(&mut centroids);
                let t_undist = t2.elapsed();

                // 4. Measure Solver (Pyramid + QUEST)
                let t3 = Instant::now();
                let result = startracker.pyramid_solve(centroids); // or exhaustive_solve
                let t_solve = t3.elapsed();

                let t_total = t0.elapsed();
                // Print table row
                println!("| {:<30} | {:<10.3} | {:<10.3} | {:<10.3} | {:<10.3} | {:<10.3} |",
                    path.file_name().unwrap().to_string_lossy(),
                    t_load.as_secs_f64() * 1000.0,
                    t_find.as_secs_f64() * 1000.0,
                    t_undist.as_secs_f64() * 1000.0,
                    t_solve.as_secs_f64() * 1000.0,
                    t_total.as_secs_f64() * 1000.0
                );

            }

        }
        
    }
}