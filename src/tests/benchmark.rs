


#[cfg(test)]
mod tests {
    // use aether::math::{Vector, Matrix};

    use crate::{perception::{
        camera_model::CameraModel,
        centroiding::{
            Centroid, Starfinder
        }
    }, startrack::error::StartrackerError};

    use std::io;
    use std::fs::{self, DirEntry};
    use std::path::Path;
    use aether::math::{Matrix, Vector};
    use image::{GrayImage, ImageReader, Luma, RgbImage};
    use std::time::Instant;


    use crate::startrack::{
        solver::Startracker,
        quest::quest,
    };

    struct StarMatch {
        raw_x: f64,
        raw_y: f64,
        catalog_vector: Vector<f64, 3>,
    }

    pub struct StarMetrics {
        pub cat_x: f64,
        pub cat_y: f64,
        pub body_x: f64,
        pub body_y: f64,
        pub angular_error_deg: f64,
    }

    use std::fs::File;
    use std::io::Write;
    use aether::attitude::Quaternion;
    use aether::reference_frame::{ICRF, Body};

    use std::fs::OpenOptions;

    pub fn export_solve_diagnostics(
        filename: &str,
        q: &Quaternion<f64, ICRF<f64>, Body<f64>>,
        reference_vectors: &[Vector<f64, 3>],
        body_vectors: &[Vector<f64, 3>],
        camera_model: &CameraModel,
    ) -> std::io::Result<()> {
        let mut matches = Vec::new();

        for (cat_vec, body_vec) in reference_vectors.iter().zip(body_vectors.iter()) {
            let rotated_cat = q.rotate_vector(*cat_vec);
            
            let dot = body_vec.dot(&rotated_cat).clamp(-1.0, 1.0);
            let angular_error_deg = dot.acos().to_degrees();

            if let (Some((cat_x, cat_y)), Some((body_x, body_y))) = 
                (camera_model.project_vector(rotated_cat), camera_model.project_vector(*body_vec)) 
            {
                matches.push(StarMetrics {
                    cat_x, cat_y, body_x, body_y, angular_error_deg
                });
            }
        }

        // Sort ascending by angular error
        matches.sort_by(|a, b| a.angular_error_deg.partial_cmp(&b.angular_error_deg).unwrap());

        // Check if the file exists before we open it so we know whether to write the header
        let file_exists = Path::new(filename).exists();

        let mut file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(filename)?;

        // Only write the CSV header if this is a brand new file
        if !file_exists {
            writeln!(file, "cat_x,cat_y,body_x,body_y,error_deg")?;
        }

        for m in matches {
            writeln!(file, "{:.5},{:.5},{:.5},{:.5},{:.7}", m.cat_x, m.cat_y, m.body_x, m.body_y, m.angular_error_deg)?;
        }

        Ok(())
    }

  
    #[test]
    fn optimize_batch() {
        use std::fs;
        use ndarray::{Array1, Array2};
        use scirs2_optimize::least_squares::robust::{robust_least_squares, HuberLoss, RobustOptions};

        let mut starfinder = Starfinder::default();
        
         let camera_model = CameraModel::new(
            73.0,               // fov (reference)
            1746.05449867,      // fx: Optimized
            1747.87368210,      // fy: Optimized
            1296.00000000,      // cx: LOCKED
            972.00000000,       // cy: LOCKED
            -0.00247914,        // k1: Optimized Radial
            -0.00611177,        // k2: Optimized Radial
            0.0,                // k3: Baseline
            -0.00107069,        // p1: Optimized Tangential
            -0.00253018         // p2: Optimized Tangential
        );



        let startracker = Startracker::default();
        let dir_path = "/home/supergoodname77/Desktop/Elara/startracking/images/jericho_cleaned/";

        println!("Loading images and extracting raw centroids...");
        let entries = fs::read_dir(dir_path).expect("Failed to read image directory");

        let mut accepted_count = 0;
        let mut rejected_count = 0;
        let mut locked_matches: Vec<StarMatch> = Vec::new();

        for entry in entries {
            let path = entry.unwrap().path();
            if path.is_file() {
                let file_name = path.file_name().unwrap().to_string_lossy().to_lowercase();
                if file_name.ends_with(".png") || file_name.ends_with(".jpg") || file_name.ends_with(".tiff") {
                    
                    let img = ImageReader::open(&path).unwrap().decode().unwrap();
                    let mut gray_image = GrayImage::from(img);
                    
                    let mut raw_centroids = starfinder.star_find(&mut gray_image);
                    raw_centroids.retain(|c| c.is_valid);
                    if raw_centroids.is_empty() { continue; }

                    let mut test_centroids = raw_centroids.clone();
                    camera_model.undistort_centroids(&mut test_centroids);

                    match startracker.adaptive_pyramid_solve(test_centroids.clone()) {
                        Ok((reference_vectors, body_vectors)) => {
                            let q = quest(&reference_vectors, &body_vectors);
                            let mut total_angle_error = 0.0;
                            
                            for (r, b) in reference_vectors.iter().zip(body_vectors.iter()) {
                                let b_predicted = q.rotate_vector(*r);
                                let dot = b.dot(&b_predicted).min(1.0).max(-1.0);
                                total_angle_error += dot.acos().to_degrees();
                            }
                            
                            let avg_error = total_angle_error / reference_vectors.len() as f64;
                            
                            // Let slightly sloppy uncalibrated images into the solver
                            if avg_error < 0.5 {
                                accepted_count += 1;
                                for (i, reference) in reference_vectors.iter().enumerate() {
                                    let body_vec = body_vectors[i];
                                    let matched_idx = test_centroids.iter()
                                        .position(|c| (c.unit_loc[0] - body_vec[0]).abs() < 1e-6)
                                        .expect("Failed to link body vector to centroid");
                                        
                                    locked_matches.push(StarMatch {
                                        raw_x: raw_centroids[matched_idx].raw_x, 
                                        raw_y: raw_centroids[matched_idx].raw_y,
                                        catalog_vector: q.rotate_vector(*reference),
                                    });
                                }
                            } else {
                                rejected_count += 1;
                            }
                        },
                        Err(_) => { rejected_count += 1; }
                    }
                }
            }
        }

        println!("Loaded {} clean images (Rejected {} bad frames).", accepted_count, rejected_count);
        println!("Total locked stars for optimization: {}", locked_matches.len());
        println!("Starting Robust Least Squares Optimization (Locked Principal Point)...");

        if locked_matches.is_empty() {
            panic!("No valid star matches found. Cannot optimize.");
        }

        // Robust objective function, solves for everrything except cx cy (center of cam)
        let objective = |params: &[f64], _data: &[f64]| -> Array1<f64> {
            let mut test_camera = camera_model.clone();
            
            test_camera.fx = 1751.44 * params[0];
            test_camera.fy = 1751.15 * params[1];
            test_camera.k1 = params[2];
            test_camera.k2 = params[3];
            test_camera.p1 = params[4];
            test_camera.p2 = params[5];
            
            test_camera.cx = 1296.0;
            test_camera.cy = 972.0;

            let mut residuals = Vec::with_capacity(locked_matches.len() * 2 + 2);
            
            for m in &locked_matches {
                if let Some((proj_x, proj_y)) = test_camera.project_vector(m.catalog_vector) {
                    residuals.push(proj_x - m.raw_x);
                    residuals.push(proj_y - m.raw_y);
                } else {
                    residuals.push(10_000.0);
                    residuals.push(10_000.0);
                }
            }

            // Focal length soft bounds
            residuals.push(if params[0] < 0.8 { (0.8 - params[0]) * 100.0 } else if params[0] > 1.2 { (params[0] - 1.2) * 100.0 } else { 0.0 });
            residuals.push(if params[1] < 0.8 { (0.8 - params[1]) * 100.0 } else if params[1] > 1.2 { (params[1] - 1.2) * 100.0 } else { 0.0 });

            Array1::from_vec(residuals)
        };

        let initial_guess = Array1::from_vec(vec![1.0, 1.0, 0.0, 0.0, 0.0, 0.0]);
        let dummy_data = Array1::<f64>::zeros(0);
        let no_jacobian: Option<fn(&[f64], &[f64]) -> Array2<f64>> = None;
        
        let loss = HuberLoss::new(5.0);
        let options = RobustOptions::default();

        let result = robust_least_squares(
            objective, 
            &initial_guess, 
            loss, 
            no_jacobian, 
            &dummy_data, 
            Some(options)
        ).expect("Robust Least Squares failed to converge");

        println!("--- GLOBAL OPTIMIZATION COMPLETE ---");
        println!("Final Cost: {:.4}", result.fun); 
        println!("Optimized fx: {:.8}", 1751.44 * result.x[0]);
        println!("Optimized fy: {:.8}", 1751.15 * result.x[1]);
        println!("Optimized k1: {:.8}", result.x[2]);
        println!("Optimized k2: {:.8}", result.x[3]);
        println!("Optimized p1: {:.8}", result.x[4]);
        println!("Optimized p2: {:.8}", result.x[5]);
        println!("Optimized cx: {:.8}", 1296.0);
        println!("Optimized cy: {:.8}", 972.0);
    }

    #[test]
    fn benchmark_full_solve() {
        let mut starfinder = Starfinder::default();
        let camera_model = CameraModel::new(
            73.0,          // fov: Datasheet H-FOV
            1751.44,       // fx: Calculated from 73.0 deg H-FOV
            1751.15,       // fy: Calculated from 58.1 deg V-FOV
            1296.0,        // cx: Image width / 2
            972.0,         // cy: Image height / 2
            0.0,           // k1: Baseline
            0.0,           // k2: Baseline
            0.0,           // k3: Baseline
            0.0,           // p1: Baseline
            0.0            // p2: Baseline
        );
        let startracker = Startracker::default();

        for entry in fs::read_dir("/home/supergoodname77/Desktop/Elara/startracking/images/jericho_cleaned/").unwrap() {
            let entry = entry.unwrap();
            let path = entry.path();

            if path.is_file() {
                // Measure IO/Decoding
                let t0 = Instant::now();
                let img = ImageReader::open(&path).unwrap().decode().unwrap();
                let mut gray_image = GrayImage::from(img);
                let t_load = t0.elapsed();

                // Measure Star Finding
                let t1 = Instant::now();
                let mut centroids = starfinder.star_find(&mut gray_image);
                let t_find = t1.elapsed();

                // Measure Undistortion
                let t2 = Instant::now();
                camera_model.undistort_centroids(&mut centroids);
                let t_undist = t2.elapsed();

                // Measure Solver (Pyramid + QUEST)
                let t3 = Instant::now();
                let result = startracker.adaptive_pyramid_solve(centroids); // or exhaustive_solve
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
        let camera_model = CameraModel::new(
            73.0,          // fov: Datasheet H-FOV
            1751.44,       // fx: Calculated from 73.0 deg H-FOV
            1751.15,       // fy: Calculated from 58.1 deg V-FOV
            1296.0,        // cx: Image width / 2
            972.0,         // cy: Image height / 2
            -0.00001176,           // k1: Baseline
            0.00004157,           // k2: Baseline
            0.0,           // k3: Baseline
            0.00023262,           // p1: Baseline
            -0.00004511   // p2: Baselines
        );
        
        let startracker = Startracker::default();
        let path = "/home/supergoodname77/Desktop/Elara/startracking/images/jericho_cleaned/cor0.png";

        let img = ImageReader::open(path).unwrap().decode().unwrap();
        let mut gray_image = GrayImage::from(img);
        let mut centroids = starfinder.star_find(&mut gray_image);
        camera_model.undistort_centroids(&mut centroids);
        match startracker.adaptive_pyramid_solve(centroids) {
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
                
            }
        }
    }

    #[test]    
    fn compare_coordinate_output_batch() {
        use std::fs;
        use std::path::Path;

        let mut starfinder = Starfinder::default();
        
        let camera_model = CameraModel::new(
            73.0,               
            1745.42976959,      
            1748.07552314,      
            1296.0,             
            972.0,              
            -0.00143655,        
            -0.00742772,        
            0.0,                
            -0.00079702,        
            -0.00275967         
        );

        let startracker = Startracker::default();
        let dir_path = "/home/supergoodname77/Desktop/Elara/startracking/images/jericho_cleaned/";
        let output_dir = "/home/supergoodname77/Desktop/Elara/startracking/diagnostics/";

        fs::create_dir_all(output_dir).expect("Failed to create diagnostics directory");

        let mut global_total_sq_error = 0.0;
        let mut global_total_error = 0.0;
        let mut global_star_count = 0;
        let mut successful_solves = 0;
        let mut failed_solves = 0;

        let entries = fs::read_dir(dir_path).expect("Failed to read image directory");

        for entry in entries {
            let entry = entry.unwrap();
            let path = entry.path();

            if path.is_file() {
                let file_name = path.file_name().unwrap().to_string_lossy();
                let ext = file_name.to_lowercase();
                
                if !ext.ends_with(".png") && !ext.ends_with(".jpg") && !ext.ends_with(".tiff") {
                    continue; 
                }

                println!("==================================================");
                println!("Processing Image: {}", file_name);

                let img = match ImageReader::open(&path).and_then(|r| r.decode().map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))) {
                    Ok(i) => i,
                    Err(e) => { eprintln!("Failed to load/decode image: {}", e); continue; }
                };

                let mut gray_image = GrayImage::from(img);
                let mut centroids = starfinder.star_find(&mut gray_image);
                
                centroids.retain(|c| c.is_valid);
                camera_model.undistort_centroids(&mut centroids);

                let centroid_search_range = 80;
                let solver_centroids = if centroids.len() > centroid_search_range {
                    centroids[0..centroid_search_range].to_vec()
                } else {
                    centroids.clone()
                };
                
                match startracker.adaptive_pyramid_solve(solver_centroids) {
                    Ok((reference_vectors, body_vectors)) => {
                        successful_solves += 1;
                        let q = quest(&reference_vectors, &body_vectors);
                        
                        println!("Q: {:?}", q);
                        println!("\nCheck Angle");
                        
                        let mut image_total_sq_error = 0.0;
                        let mut image_total_error = 0.0;
                        let star_count = reference_vectors.len();

                        for (r, b) in reference_vectors.iter().zip(body_vectors.iter()) {
                            let b_predicted = q.rotate_vector(*r);
                            let dot = b.dot(&b_predicted).min(1.0).max(-1.0); 
                            let angle_deg = dot.acos().to_degrees();
                            
                            println!("Star Error: {:.5} degrees", angle_deg);
                            
                            image_total_error += angle_deg;
                            global_total_error += angle_deg;
                            image_total_sq_error += angle_deg * angle_deg;
                            global_total_sq_error += angle_deg * angle_deg;
                            global_star_count += 1;
                        }

                        println!("Image Average Error: {:.5} degrees\n", image_total_error / star_count as f64);
                        let image_rms = (image_total_sq_error / star_count as f64).sqrt();
                        println!("Image RMS Error: {:.5} degrees\n", image_rms);

                        let file_stem = path.file_stem().unwrap().to_string_lossy();
                        let out_filename = format!("{}/{}.csv", output_dir, file_stem);
                        
                        if let Err(e) = export_solve_diagnostics(&out_filename, &q, &reference_vectors, &body_vectors, &camera_model) {
                            eprintln!("Failed to write CSV for {}: {}", file_name, e);
                        } else {
                            println!("Exported match data to: {}.csv", file_stem);
                        }
                    }
                    Err(e) => {
                        failed_solves += 1;
                        eprintln!("Solver Failed: {:?}", e);
                    }
                }
            }
        }

        println!("==================================================");
        println!("--- GLOBAL BENCHMARK RESULTS ---");
        println!("Images Solved Successfully: {}", successful_solves);
        println!("Images Failed to Solve:     {}", failed_solves);
        
        if global_star_count > 0 {
            let global_average_error = global_total_error / global_star_count as f64;
            println!("Total Stars Evaluated:      {}", global_star_count);
            println!("Global Average Error:       {:.5} degrees", global_average_error);
            
            let global_rms = (global_total_sq_error / global_star_count as f64).sqrt();
            println!("Global RMS Error: {:.5} degrees", global_rms)
        } else {
            println!("No stars evaluated.");
        }
        println!("==================================================");
    }



    use std::process::Command;


    pub fn export_solve_diagnostics_raw(
        filename: &str,
        q: &Quaternion<f64, ICRF<f64>, Body<f64>>,
        reference_vectors: &[Vector<f64, 3>],
        body_vectors: &[Vector<f64, 3>],
        solver_centroids: &[Centroid], // <-- ADD THIS
        raw_centroids: &[Centroid],
        camera_model: &CameraModel,
    ) -> std::io::Result<()> {
        let mut matches = Vec::new();

        for (cat_vec, body_vec) in reference_vectors.iter().zip(body_vectors.iter()) {
            let rotated_cat = q.rotate_vector(*cat_vec);
            let dot = body_vec.dot(&rotated_cat).clamp(-1.0, 1.0);
            let angular_error_deg = dot.acos().to_degrees();

            // Match against the UNDISTORTED solver array to find the index
            if let Some(idx) = solver_centroids.iter().position(|c| {
                (c.unit_loc[0] - body_vec[0]).abs() < 1e-6 &&
                (c.unit_loc[1] - body_vec[1]).abs() < 1e-6
            }) {
                // If it matches, project the catalog vector...
                if let Some((cat_x, cat_y)) = camera_model.project_vector(rotated_cat) {
                    matches.push(StarMetrics {
                        cat_x, 
                        cat_y, 
                        // ...but extract the RAW pixels using that index
                        body_x: raw_centroids[idx].raw_x, 
                        body_y: raw_centroids[idx].raw_y, 
                        angular_error_deg
                    });
                }
            }
        }

        matches.sort_by(|a, b| a.angular_error_deg.partial_cmp(&b.angular_error_deg).unwrap());

        let mut file = File::create(filename)?;
        writeln!(file, "cat_x,cat_y,body_x,body_y,error_deg")?;
        for m in matches {
            writeln!(file, "{:.5},{:.5},{:.5},{:.5},{:.7}", m.cat_x, m.cat_y, m.body_x, m.body_y, m.angular_error_deg)?;
        }
        Ok(())
    }


    #[test]
    fn batch_test() {
        use std::fs;
        use std::path::Path;

        let mut starfinder = Starfinder::default();
        
        let camera_model = CameraModel::new(
            73.0, 1751.41490455, 1750.67566653, 1296.0, 972.0,
            0.00015049, -0.00074404, 0.0, -0.00021602, 0.00000284
        );

        let startracker = Startracker::default();
        let dir_path = "/home/supergoodname77/Desktop/Elara/startracking/images/jericho_cleaned/";
        let diag_dir = "/home/supergoodname77/Desktop/Elara/startracking/diagnostics/";
        let plot_dir = "/home/supergoodname77/Desktop/Elara/startracking/diagnostics/plots/";
        let python_script = "/home/supergoodname77/Desktop/Learning/ml/startracker_test/csv_plot.py";

        fs::create_dir_all(diag_dir).expect("Failed to create diagnostics dir");
        fs::create_dir_all(plot_dir).expect("Failed to create plots dir");

        let mut successful_solves = 0;

        let entries = fs::read_dir(dir_path).expect("Failed to read image directory");

        for entry in entries {
            let entry = entry.unwrap();
            let path = entry.path();

            if path.is_file() {
                let file_name = path.file_name().unwrap().to_string_lossy();
                let ext = file_name.to_lowercase();
                if !ext.ends_with(".png") && !ext.ends_with(".jpg") && !ext.ends_with(".tiff") {
                    continue; 
                }

                println!("==================================================");
                println!("Processing Image: {}", file_name);

                let img = match ImageReader::open(&path).and_then(|r| r.decode().map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))) {
                    Ok(i) => i,
                    Err(_) => continue,
                };

                let mut gray_image = GrayImage::from(img);
                let mut centroids = starfinder.star_find(&mut gray_image);
                
                let raw_centroids = centroids.clone();
                camera_model.undistort_centroids(&mut centroids);

                let solver_centroids = if centroids.len() > 80 {
                    centroids[0..80].to_vec()
                } else {
                    centroids.clone()
                };
                
                if let Ok((reference_vectors, body_vectors)) = startracker.adaptive_pyramid_solve(solver_centroids.clone()) {
                    successful_solves += 1;
                    let q = quest(&reference_vectors, &body_vectors);
                    
                    let file_stem = path.file_stem().unwrap().to_string_lossy();
                    let out_csv = format!("{}/{}.csv", diag_dir, file_stem);
                    
                    if let Err(e) = export_solve_diagnostics_raw(&out_csv, &q, &reference_vectors, &body_vectors, &solver_centroids, &raw_centroids, &camera_model) {
                        eprintln!("Failed to write CSV: {}", e);
                    } else {
                        println!("Calling Python to generate plot...");
                        
                        let venv_python = "/home/supergoodname77/Desktop/Learning/ml/.venv/bin/python";
                        
                        let status = Command::new(venv_python)
                            .arg(python_script)
                            .arg(&out_csv)
                            .arg("--image")
                            .arg(path.to_str().unwrap())
                            // .arg("--save")
                            // .arg(plot_dir)
                            .arg("-r")
                            .arg("20") // Circle radius
                            .status()
                            .expect("Failed to execute python script from venv.");
                            
                        if !status.success() {
                            eprintln!("Python script failed on image {}", file_name);
                        }
                    }
                }
            }
        }
        println!("Finished processing {} images. Plots saved to {}", successful_solves, plot_dir);
    }

    #[test]
    fn validate_centroids_interactive() {
        use std::fs;
        use std::io::Write;
        use std::process::Command;

        let starfinder = Starfinder::default();
        
        let target_image = "cor9.png"; // Test on the moon image!
        let image_path = format!("/home/supergoodname77/Desktop/Elara/startracking/images/jericho_cleaned/{}", target_image);
        let csv_path = "/home/supergoodname77/Desktop/Elara/startracking/diagnostics/raw_centroids.csv";
        let python_script = "/home/supergoodname77/Desktop/Learning/ml/startracker_test/plot_centroids.py";

        println!("Extracting raw morphological centroids from {}...", target_image);

        let img = ImageReader::open(&image_path).unwrap().decode().unwrap();
        let mut gray_image = GrayImage::from(img);
        
        // Find ALL blobs (valid and invalid)
        let centroids = starfinder.star_find(&mut gray_image);

        // Export to CSV
        let mut file = fs::File::create(csv_path).unwrap();
        writeln!(file, "x,y,brightness,pixel_count,aspect_ratio,is_valid").unwrap();
        for c in &centroids {
            writeln!(file, "{:.3},{:.3},{},{},{:.3},{}", 
                c.raw_x, c.raw_y, c.brightness, c.pixel_count, c.aspect_ratio, c.is_valid).unwrap();
        }

        println!("Found {} total blobs. Calling Python visualizer...", centroids.len());

        let venv_python = "/home/supergoodname77/Desktop/Learning/ml/.venv/bin/python";
        let status = Command::new(venv_python)
            .arg(python_script)
            .arg(csv_path)
            .arg(&image_path)
            .status()
            .expect("Failed to execute python visualizer.");
            
        assert!(status.success());
    }
}