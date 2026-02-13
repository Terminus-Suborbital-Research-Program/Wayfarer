


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

    #[test]
    fn compare_coordinate_output() {
        let mut starfinder = Starfinder::default();
        let camera_model = CameraModel::default();
        let startracker = Startracker::default();
        let path = "../images/mixed/bright-VISNIR-310ms_24d1g_50ict_0bl_0d80gam_set1_1.tiff";

        let img = ImageReader::open(path).unwrap().decode().unwrap();
        let mut gray_image = GrayImage::from(img);
        let mut centroids = starfinder.star_find(&mut gray_image);
        camera_model.undistort_centroids(&mut centroids);
        match startracker.pyramid_solve(centroids) {
        // match startracker.exhaustive_solve(centroids, 100) {
            Ok((reference_vectors, body_vectors)) => {
                let q = quest(&reference_vectors, &body_vectors);
                println!("Q: {:?}", q);
                println!("");
                println!("Check Angle");
                let mut total_error = 0.0;
                for (r, b) in reference_vectors.iter().zip(body_vectors.iter()) {
                    
                    let b_predicted = q.rotate_vector(*r);
                    
                    
                    let dot = b.dot(&b_predicted).min(1.0).max(-1.0); // Clamp for safety
                    let angle_rad = dot.acos();
                    let angle_deg = angle_rad.to_degrees();
                    
                    println!("Star Error: {:.5} degrees", angle_deg);
                    total_error += angle_deg;
                }
                println!("Average Error: {:.5} degrees", total_error / reference_vectors.len() as f64);
                println!("");

                for (reference, body) in reference_vectors.iter().zip(body_vectors.iter()) {
                    let star = q.rotate_vector(*reference);

                    if let Some((cat_x, cat_y)) = camera_model.project_vector(star) {
                        if let Some((body_x, body_y)) = camera_model.project_vector(*body) {
                        println!("Check Coords");
                        println!("------------");
                        println!("Cat:  X: {:.5} Y: {:.5}", cat_x, cat_y);
                        println!("Body: X: {:.5} Y: {:.5}", body_x, body_y);
                        // println!("({:.8}, {:.8}),", cat_x, cat_y);
                        // println!("({:.8}, {:.8}),", body_x, body_y);
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

    #[test]
    fn compare_coordinate_output_exhaustive() {
        let mut starfinder = Starfinder::default();
        let camera_model = CameraModel::default();
        let startracker = Startracker::default();
        let path = "../images/mixed/bright-VISNIR-310ms_24d1g_50ict_0bl_0d80gam_set1_1.tiff";

        let img = ImageReader::open(path).unwrap().decode().unwrap();
        let mut gray_image = GrayImage::from(img);
        let mut centroids = starfinder.star_find(&mut gray_image);
        camera_model.undistort_centroids(&mut centroids);
        // startracker.exhaustive_solve(centroids, 10, camera_model);

        startracker.exhaustive_solve(centroids, 100, camera_model.clone()).unwrap();
       
    }
}