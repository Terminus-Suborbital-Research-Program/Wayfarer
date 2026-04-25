use image::{GrayImage, ImageReader, Luma, RgbImage, Rgb, ImageBuffer};
use image::DynamicImage;
use aether::math::{Matrix, Vector};
use std::cmp::Reverse;

#[derive(Debug, Copy, Clone)]
pub struct Centroid {
    pub raw_x: f64,
    pub raw_y: f64,
    pub unit_loc: Vector<f64, 3>,
    pub brightness: u64,
    // Diagnostics
    pub is_valid: bool,
    pub pixel_count: usize,
    pub aspect_ratio: f64, // 1.0 is perfectly round. > 2.0 is a streak.
}

impl Centroid {
    pub fn new(x: f64, y: f64, brightness: u64, is_valid: bool, pixel_count: usize, aspect_ratio: f64) -> Self {
        Self {
            raw_x: x,
            raw_y: y,
            unit_loc: Vector::new([x, y, 1.0]), 
            brightness,
            is_valid,
            pixel_count,
            aspect_ratio,
        }
    }
}



/// Convert images to star detectable format, find centroids, and allow easy access to vector data used for identification calculations
pub struct Starfinder {
    // Threshold for what pixels we simply don't want to check to see if they are a centroid, because they look like background noise
    threshold: u8,
    // background_noise threshold
    bg_threshold: u8,
    // Impl cam driver / interace here in the future

}

impl Default for Starfinder {
    fn default() -> Self {
        Self { 
            threshold: 20u8,
            bg_threshold: 0u8,
        }
    }
}

pub type GrayRef<'a> = ImageBuffer<Luma<u8>, &'a [u8]>;

use std::ops::{
    Deref, DerefMut
};
impl Starfinder {
    pub fn new(threshold: u8, bg_threshold: u8) -> Self {
        Self {
            threshold,
            bg_threshold,
        }
    }

   pub fn star_find<C>(&self, gray_image: &ImageBuffer<Luma<u8>, C>) -> Vec<Centroid> 
    where
        C: Deref<Target = [u8]> 
    {
        let mut raw_blobs: Vec<Centroid> = Vec::new();
        let (width, height) = gray_image.dimensions();
        let mut visited = vec![false; (width * height) as usize];

        for y in 2..(height - 2) {
            for x in 2..(width - 2) {
                let idx = (y * width + x) as usize;
                if visited[idx] { continue; }

                let center_val = gray_image.get_pixel(x, y)[0];
                
                if center_val > self.threshold {
                    let mut is_max = true;
                    
                    for dy in -2i32..=2 {
                        for dx in -2i32..=2 {
                            if dx == 0 && dy == 0 { continue; }
                            let nx = (x as i32 + dx) as u32;
                            let ny = (y as i32 + dy) as u32;
                            if gray_image.get_pixel(nx, ny)[0] > center_val {
                                is_max = false;
                                break;
                            }
                        }
                        if !is_max { break; }
                    }

                    if is_max {
                        // Pass x and y as the peak coordinate reference for the Gaussian Fit
                        if let Some(blob) = self.extract_blob(gray_image, &mut visited, x, y) {
                            raw_blobs.push(blob);
                        }
                    }
                }
            }
        }
        
        // Dyanmic exclusion zone - get rid of the moon or earth by killing the validity of
        // all centroids in a large radius around them. Done because just killing them
        // directly caused halo effects
        let massive_bodies: Vec<(f64, f64, f64)> = raw_blobs.iter()
            .filter(|b| b.pixel_count > 200) 
            .map(|b| {
                let base_radius = (b.pixel_count as f64 / std::f64::consts::PI).sqrt();
                let exclusion_radius = base_radius * 3.5; 
                (b.raw_x, b.raw_y, exclusion_radius)
            })
            .collect();

        let mut final_centroids: Vec<Centroid> = Vec::new();
        
        for mut blob in raw_blobs {
            let mut masked = false;
            
            for &(mx, my, excl_radius) in &massive_bodies {
                let dx = blob.raw_x - mx;
                let dy = blob.raw_y - my;
                let dist = (dx * dx + dy * dy).sqrt();
                
                if dist < excl_radius && blob.pixel_count <= 200 { 
                    masked = true;
                    break;
                }
            }

            if masked {
                blob.is_valid = false;
            }

            final_centroids.push(blob);
        }

        final_centroids.sort_by_key(|c| std::cmp::Reverse(c.brightness));
        final_centroids
    }

    fn extract_blob<C>(
        &self, 
        image: &ImageBuffer<Luma<u8>, C>, 
        visited: &mut [bool], 
        start_x: u32, 
        start_y: u32
    ) -> Option<Centroid> 
    where
        C: Deref<Target = [u8]> 
    {
        let (width, height) = image.dimensions();
        let peak_val = image.get_pixel(start_x, start_y)[0];
        
        // Calculate Local Background
        let mut ring_pixels = Vec::with_capacity(50);
        for dy in -6i32..=6 {
            for dx in -6i32..=6 {
                if dx.abs() == 6 || dy.abs() == 6 {
                    let px = start_x as i32 + dx;
                    let py = start_y as i32 + dy;
                    if px >= 0 && px < width as i32 && py >= 0 && py < height as i32 {
                        ring_pixels.push(image.get_pixel(px as u32, py as u32)[0]);
                    }
                }
            }
        }
        ring_pixels.sort_unstable();
        let local_bg = if ring_pixels.is_empty() { 0 } else { ring_pixels[ring_pixels.len() / 2] };
        
        if peak_val < local_bg.saturating_add(12) {
            visited[(start_y * width + start_x) as usize] = true;
            return None; 
        }

        let fill_threshold = self.threshold.max(local_bg.saturating_add(3));
        let mut pixel_stack = vec![(start_x, start_y)];
        let mut pixels = Vec::with_capacity(64);

        // Flood fill with visited mask
        while let Some((cx, cy)) = pixel_stack.pop() {
            if cx == 0 || cx >= width - 1 || cy == 0 || cy >= height - 1 { continue; }

            let idx = (cy * width + cx) as usize;
            if visited[idx] { continue; } 

            let val = image.get_pixel(cx, cy)[0];
            
            if val > fill_threshold {
                visited[idx] = true; 
                pixels.push((cx, cy, val));

                pixel_stack.push((cx + 1, cy));
                pixel_stack.push((cx - 1, cy));
                pixel_stack.push((cx, cy + 1));
                pixel_stack.push((cx, cy - 1));
            }
        }

        let pixel_count = pixels.len();
        if pixel_count == 0 { return None; }

        // Center of mass and covariance
        let mut sum_intensity = 0.0;
        let mut w_x = 0.0;
        let mut w_y = 0.0;

        // also need to format the pixels for the Gaussian Fitter: (dx, dy, net_intensity)
        let mut fit_pixels = Vec::with_capacity(pixel_count);

        for &(px, py, val) in &pixels {
            let intensity = val.saturating_sub(local_bg).max(1) as f64; 
            
            w_x += (px as f64) * intensity;
            w_y += (py as f64) * intensity;
            sum_intensity += intensity;

            // Offset relative to the initial peak
            let dx = (px as i32 - start_x as i32) as f64;
            let dy = (py as i32 - start_y as i32) as f64;
            fit_pixels.push((dx, dy, intensity));
        }

        // Center of Mass Fallback
        let mut final_x = w_x / sum_intensity;
        let mut final_y = w_y / sum_intensity;

        // SUB-PIXEL GAUSSIAN FIT
        // Overwrite the CoM coordinates if the Gaussian solver converges
        if let Some((dx_fit, dy_fit)) = self.fast_gaussian_fit(&fit_pixels) {
            final_x = start_x as f64 + dx_fit;
            final_y = start_y as f64 + dy_fit;
        }

        // Covariance
        let mut var_x = 0.0;
        let mut var_y = 0.0;
        let mut cov_xy = 0.0;

        for &(px, py, val) in &pixels {
            let intensity = val.saturating_sub(local_bg).max(1) as f64;
            // Use CoM for covariance shape testing, not the fitted peak
            let dx = (px as f64) - (w_x / sum_intensity);
            let dy = (py as f64) - (w_y / sum_intensity);
            var_x += dx * dx * intensity;
            var_y += dy * dy * intensity;
            cov_xy += dx * dy * intensity;
        }

        var_x /= sum_intensity;
        var_y /= sum_intensity;
        cov_xy /= sum_intensity;

        let trace = var_x + var_y;
        let det = var_x * var_y - cov_xy * cov_xy;
        let discriminant = (trace * trace - 4.0 * det).max(0.0).sqrt();
        let l1 = (trace + discriminant) / 2.0; 
        let l2 = (trace - discriminant).max(1e-6) / 2.0; 

        let aspect_ratio = (l1 / l2).sqrt();

        // THE GATES
        let valid_shape = if pixel_count > 6 { aspect_ratio < 2.5 } else { true };
        let is_valid = pixel_count >= 2 && pixel_count <= 150 && valid_shape;

        let total_brightness: u64 = pixels.iter().map(|&(_, _, v)| v as u64).sum();

        Some(Centroid::new(final_x, final_y, total_brightness, is_valid, pixel_count, aspect_ratio))
    }
   

    /// Executes Fast Gaussian Fitting on a local window of pixels.
    /// Returns the sub-pixel (dx, dy) offset relative to the peak_x, peak_y.
    fn fast_gaussian_fit(&self, pixels: &[(f64, f64, f64)]) -> Option<(f64, f64)> {
        if pixels.len() < 5 {
            return None; // Not enough data points to fit a 4-parameter model
        }

        let mut wtw = Matrix::<f64, 4, 4>::zeros();
        let mut wtz = Vector::<f64, 4>::default();

        // Build the normal equations: (W^T * W) * a = W^T * Z
        for &(dx, dy, intensity) in pixels {
            // Protect against log(0) or negative numbers after background subtraction
            if intensity <= 0.0 { continue; } 
            
            let z = intensity.ln();
            let h = dx * dx + dy * dy;
            let w_row = [h, dx, dy, 1.0];

            for i in 0..4 {
                wtz[i] += w_row[i] * z;
                for j in 0..4 {
                    wtw[(i, j)] += w_row[i] * w_row[j];
                }
            }
        }

        // Solve for coefficients 'a' using Gauss-Jordan inversion
        if let Some(wtw_inv) = wtw.inverse_gauss_jordan() {
            let a = wtw_inv * wtz;
            
            let a0 = a[0];
            let a1 = a[1];
            let a2 = a[2];

            // If a0 is positive, the parabola is facing upwards (not a star bell curve)
            if a0 >= 0.0 { return None; }

            let xc_offset = -a1 / (2.0 * a0);
            let yc_offset = -a2 / (2.0 * a0);

            // Sanity check: the sub-pixel offset shouldn't be wildly outside the window
            if xc_offset.abs() > 4.0 || yc_offset.abs() > 4.0 {
                return None;
            }

            return Some((xc_offset, yc_offset));
        }
        
        None // Matrix was singular (perfectly flat or degenerate data)
    }

    
}
