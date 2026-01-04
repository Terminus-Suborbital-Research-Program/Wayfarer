use image::{GrayImage, ImageReader, Luma, RgbImage, Rgb};
use image::DynamicImage;
use aether::math::Vector;
#[cfg(feature = "opencv")]
use opencv::{
    core::{Mat, MatTraitConst, Point2d, Vector as CV_Vector, CV_64F},
    calib3d,
    prelude::*,
};
// Newtype for centroids vec
// pub struct Centroids(pub Vec<Centroid>);

// use std::ops::Deref;

// impl Deref for Centroids {
//     type Target = Vec<Centroid>;

//     fn deref(&self) -> &Self::Target {
//         &self.0
//     }
// }


pub struct Centroid {
    pub unit_loc: Vector<f64, 2>,
    pub brightness: u64,
}

impl Centroid {
    pub fn new(x: f64, y: f64, brightness: u64) -> Self {
        Self {
            // X and Y can be used for image centroiding to verify we identify stars
            // Z component 1 is so these coordinates can be converted into
            // a normalized unit vector ; key step for pyramid 
            unit_loc: Vector::new([x,y]),
            brightness,
        }
    }
}


/// Convert images to star detectable format, find centroids, and allow easy access to vector data used for identification calculations
pub struct Starfinder {
    // Threshold for what pixels we simply don't want to check to see if they are a centroid, because they look like background noise
    threshold: u8,
    // background_noise threshold
    bg_threshold: u8,
    // Camera model for undistorting pixels
    camera_model: CameraModel,

    // Impl cam driver / interace here in the future

}

impl Default for Starfinder {
    fn default() -> Self {
        Self { 
            threshold: 20u8,
            bg_threshold: 3u8,
            camera_model: CameraModel::default(),
        }
    }
}

impl Starfinder {
    pub fn new(threshold: u8, bg_threshold: u8, camera_model: CameraModel) -> Self {
        Self {
            threshold,
            bg_threshold,
            camera_model,
        }
    }

    pub fn star_find(&self, gray_image: &mut GrayImage) -> Vec<Centroid> {
        let mut centroids: Vec<Centroid> = Vec::new();
        let (width, height) = gray_image.dimensions();

        // Manually iterate over image so that we don't try to index outside of the array when checking for dead pixels / centroids
        for y in 1..(height - 1) {
            for x in 1..(width - 1) {
                let pixel = gray_image.get_pixel(x, y);
                let brightness = pixel[0];
                if brightness > self.threshold {
                    if self.is_dead_pixel(&gray_image, x, y, brightness) {
                        continue;
                    } else {
                        centroids.push(self.centroid(gray_image, x, y));
                    }
                }
            }
        }
        centroids.sort_by_key(|centroid| centroid.brightness);
        centroids

    }

    fn is_dead_pixel(&self, image: &GrayImage, x: u32, y: u32, brightness: u8,) -> bool {
        let bleed_threshold = brightness / 2;
        for x_shift in -1..=1 {
            for y_shift in -1..=1 {
                // Skip center pixel
                if x_shift == 0 && y_shift == 0 {
                    continue; 
                }
                // Get each pixel around the center one
                let pix =  image.get_pixel((x as i32 + x_shift) as u32, (y as i32 + y_shift) as u32)[0];

                // If even one surrounding pixel is bright enough, we can assume this is a star
                if pix > bleed_threshold {
                    return false
                }
            }
        }
        
        // No valid surrounding star bleed detected ; must not be a star
        return true;
    }

    // Image Testing shows that this creates many centroids around ultra bright objects such as the moon, therefore
    // may need to be refactored later if that creates a bottleneck, but the smart triangles scanning method
    // will also likely make scanning robust against this issue.
    //
    // For now stack based solution. - Will benchmark with Pi 5's speed, may need to be migrated to either recursive
    // or row by row, but for now this is sensible and less complicated to prototype
    fn centroid(&self, image: &mut GrayImage, x: u32, y: u32) -> Centroid {
        let (width, height) = image.dimensions();

        let mut pixel_stack = vec![(x,y)];
        // Variables used to calculate the x and y coordinate of the centroid based on relative
        // brightness of each pixel contributing to the star
        let mut weighted_x_sum: u64 = 0;
        let mut weighted_y_sum: u64 = 0;
        let mut sum_of_brightness: u64 = 0;

        while let Some((current_x, current_y)) = pixel_stack.pop() {
            if current_x == 0 
            || current_x >= width 
            || current_y == 0 
            || current_y >= height 
            { continue; }
            let current_brightness = image.get_pixel(current_x, current_y)[0];
            if current_brightness > self.threshold {
                weighted_x_sum += current_x as u64 * current_brightness as u64;
                weighted_y_sum += current_y as u64 * current_brightness as u64;
                sum_of_brightness += current_brightness as u64;

                // Blot out current pixel so that this blob is not rescanned later
                image.put_pixel(current_x, current_y, Luma([0]));

                // Examine other pixels to be added
                pixel_stack.push((current_x + 1, current_y));
                pixel_stack.push((current_x, current_y + 1));
                pixel_stack.push((current_x - 1, current_y));
                pixel_stack.push((current_x,current_y - 1));

            } else if current_brightness > self.bg_threshold {
                weighted_x_sum += current_x as u64 * current_brightness as u64;
                weighted_y_sum += current_y as u64 * current_brightness as u64;
                sum_of_brightness += current_brightness as u64;
                
                image.put_pixel(current_x, current_y, Luma([0]));
            }
        }

        let centroid_x: f64 = weighted_x_sum as f64 / sum_of_brightness as f64;
        let centroid_y: f64 = weighted_y_sum as f64 / sum_of_brightness as f64;

        Centroid::new(centroid_x, centroid_y, sum_of_brightness)
    }

    pub fn undistort_centroids(&self, centroids: &mut [Centroid]) {
        self.camera_model.undistort_centroids(centroids);
    }


    pub fn image_report(&self, gray_image: &mut GrayImage) {
        let mut centroids: Vec<Centroid> = Vec::new();
        let (width, height) = gray_image.dimensions();

    
        let mut vec: Vec<u8> = Vec::new();
        for y in 1..(height - 1) {
            for x in 1..(width - 1) {

                let pixel = gray_image.get_pixel(x, y);
                let brightness = pixel[0];
                if brightness > self.threshold {
                    if self.is_dead_pixel(&gray_image, x, y, brightness) {
                        continue;
                    } else {
                        centroids.push(self.centroid(gray_image, x, y));
                    }
                }
                // println!("Brightness: {}",brightness);
                vec.push(brightness);

            }
        }

        centroids.sort_by_key(|centroid| centroid.brightness);
        println!("Num centroids: {}", centroids.len());
        for centroid in centroids {
            println!("({},{}),",centroid.unit_loc[0], centroid.unit_loc[1])
        }

        vec.sort();
        let mut new: Vec<u8> = vec.into_iter().filter(|brightness| *brightness > 20u8).collect();
        
        new.sort();
        println!("LEN: {} ",new.len());

        println!("Lowest Brightness: {}, Highest: {}",new[0], new.last().unwrap());
        let sum = new.iter().map(|&b| b as u32).sum::<u32>();
        // let average = sum / new.len() as u32;
        let mid_index = (new.len() - 1) / 2;
        let average = sum / new.len() as u32;

        println!("Average: {}, Median {}",average, new[mid_index]);
    }

}

#[derive(Debug, Clone)]
pub struct CameraModel {
    // Intrinsics 
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    // Distortion (Brown-Conrady)
    pub k1: f64, pub k2: f64, pub p1: f64, pub p2: f64, pub k3: f64,

    // For changing out undistortion method to use opencv later on if Rust native doesn't work
    #[cfg(feature = "opencv")]
    camera_matrix: Mat,
    #[cfg(feature = "opencv")]
    dist_coeffs: Mat,
}
#[cfg(feature = "opencv")]
impl CameraModel {
    /// (Alvium 1800 U-501m + 12mm Lens)
    pub fn new() -> opencv::Result<Self> {
        // 1. Setup Camera Matrix (3x3)
        // [ fx,  0, cx ]
        // [  0, fy, cy ]
        // [  0,  0,  1 ]
        let k_data: Vec<f64> = vec![
            5424.41488, 0.0,        1295.5,
            0.0,        5424.35391, 971.5,
            0.0,        0.0,        1.0,
        ];

        let camera_matrix = Mat::new_rows_cols_with_data(3, 3, &k_data)?.try_clone().unwrap();

        // 2. Setup Distortion Coefficients (1x5)
        // [k1, k2, p1, p2, k3]
        let d_data: Vec<f64> = vec![
            0.51059,    // k1
            -19.48679,  // k2
            -0.00291,   // p1 (tangential)
            0.0000206,  // p2 (tangential)
            171.20994,  // k3
        ];
        // Note: openCV usually expects rows=1, cols=5 for dist coeffs
        //&d_data, 
        let dist_coeffs = Mat::new_rows_cols_with_data(1, 5, &d_data)?.try_clone().unwrap();

        Ok(Self {
            // Horizontal Scaling
            fx: 5424.41488,
            // Vertical Scaling
            fy: 5424.35391,
            // Horizontal Center
            cx: 1295.5,
            // Vertical Center
            cy: 971.5,
            // k 1..3 are coefficients that account for warping around the edges / corners (Radial Coefficients)
            k1: 0.51059,
            k2: -19.48679,
            k3: 171.20994,
            // p1 & 2 are coefficients that account for lens crookedness (tilt) relative to the sensor
            p1: -0.00291,
            p2: 0.0000206,
            camera_matrix,
            dist_coeffs,
        })
    }

    /// Takes raw centroids (pixels) and returns normalized Unit Vectors (Camera Frame)
    pub fn undistort_centroids(&self, centroids: &[Centroid]) -> opencv::Result<Vec<Vector<f64, 3>>> {
        if centroids.is_empty() {
            return Ok(Vec::new());
        }

        // A. Convert input to OpenCV Vector<Point2d>
        let mut src_points = CV_Vector::<Point2d>::new();
        for c in centroids {
            src_points.push(Point2d::new(c.unit_loc[0], c.unit_loc[1]));
        }

        // B. Prepare Output Buffer
        let mut dst_points = CV_Vector::<Point2d>::new();

        // C. The Magic Call: undistort_points
        // We pass no_array() for R (Rectification) and P (New Camera Matrix).
        // EFFECT: The output points will be in "Normalized Image Coordinates" 
        // (i.e., x' = x/z, y' = y/z) centered at (0,0) with focal length 1.
        calib3d::undistort_points(
            &src_points, 
            &mut dst_points, 
            &self.camera_matrix, 
            &self.dist_coeffs, 
            &opencv::core::no_array(), // R (Identity)
            &opencv::core::no_array(), // P (Identity -> Normalized coords)
        )?;

        // D. Convert Normalized Points (2D) -> Unit Vectors (3D)
        let mut unit_vectors = Vec::with_capacity(dst_points.len());
        
        for point in dst_points {
            // Construct vector [x', y', 1]
            let vec = Vector::new([point.x, point.y, 1.0]);
            
            // Normalize to sphere
            unit_vectors.push(vec.normalize());
        }

        Ok(unit_vectors)
    }
}

#[cfg(not(feature = "opencv"))]
impl Default for CameraModel {
    fn default() -> Self {
        Self {
            // Horizontal Scaling
            fx: 5424.41488,
            // Vertical Scaling
            fy: 5424.35391,
            // Horizontal Center
            cx: 1295.5,
            // Vertical Center
            cy: 971.5,
            // k 1..3 are coefficients that account for warping around the edges / corners (Radial Coefficients)
            k1: 0.51059,
            k2: -19.48679,
            k3: 171.20994,
            // p1 & 2 are coefficients that account for lens crookedness (tilt) relative to the sensor
            p1: -0.00291,
            p2: 0.0000206,
        }
    }
}
#[cfg(not(feature = "opencv"))]
impl CameraModel {

    pub fn new(
        fx: f64, 
        fy: f64, 
        cx: f64, 
        cy: f64, 
        k1: f64, 
        k2: f64, 
        k3: f64, 
        p1: f64, 
        p2: f64
    ) -> Self {
        Self { fx, fy, cx, cy, k1, k2, k3, p1, p2 }
    }

    /// The "Fundamental" Operation: Pixel (x, y) -> Unit Vector (x, y, z)
    /// This runs the iterative solver to remove distortion.
    pub fn undistort_centroid(&self, centroid: &mut Centroid) {
        let x = centroid.unit_loc[0];
        let y = centroid.unit_loc[1];
        // 1. Normalize pixels to "Camera Coordinates" (centered, scale-invariant)
        // These are effectively "Distorted Slopes"
        let y_distorted = (y - self.cy) / self.fy;
        let x_distorted = (x - self.cx) / self.fx;

        // 2. Iteratively Undistort (Fixed-Point Iteration)
        // We want to find the true (ideal) x and y such that applying distortion
        // yields x_distorted and y_distorted.
        let mut x_ideal = x_distorted;
        let mut y_ideal = y_distorted;

        // 5 iterations is usually sufficient for sub-pixel precision
        for _ in 0..5 {
            let r2 = x_ideal * x_ideal + y_ideal * y_ideal;
            let r4 = r2 * r2;
            let r6 = r2 * r4;

            // Radial component
            let k_radial = 1.0 + self.k1 * r2 + self.k2 * r4 + self.k3 * r6;

            // Tangential component
            let delta_x = 2.0 * self.p1 * x_ideal * y_ideal + self.p2 * (r2 + 2.0 * x_ideal * x_ideal);
            let delta_y = self.p1 * (r2 + 2.0 * y_ideal * y_ideal) + 2.0 * self.p2 * x_ideal * y_ideal;

            // The Inverse Step:
            // We know: x_distorted = x_ideal * k_radial + delta_x
            // Therefore: x_ideal = (x_distorted - delta_x) / k_radial
            x_ideal = (x_distorted - delta_x) / k_radial;
            y_ideal = (y_distorted - delta_y) / k_radial;
        }

        // 3. Convert to Unit Vector
        // We now have the ideal slope (X/Z, Y/Z). We assume Z=1 to make a vector.
        let mut raw_vector = Vector::new([x_ideal, y_ideal, 1.0]);
        
        raw_vector = raw_vector.normalize();

        centroid.unit_loc.data.copy_from_slice(&raw_vector.data[..2]);

        // centroid.unit_loc.data.copy_from_slice(&raw_vector.data[..2]);

    }

    pub fn undistort_centroids(&self, centroids: &mut [Centroid]){
        for centroid in centroids {
            self.undistort_centroid(centroid);
        }
    }
}