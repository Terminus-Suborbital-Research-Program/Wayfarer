use image::{GrayImage, ImageReader, Luma, RgbImage, Rgb};
use image::DynamicImage;
use aether::math::Vector;

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
    unit_loc: Vector<f64, 2>,
    brightness: u64,
}

impl Centroid {
    pub fn new(x: f64, y: f64, brightness: u64) -> Self {
        Self {
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
}

impl Default for Starfinder {
    fn default() -> Self {
        Self { 
            threshold: 20u8,
            bg_threshold: 3u8,
        }
    }
}

impl Starfinder {
    pub fn new(threshold: u8, bg_threshold: u8) -> Self {
        Self {
            threshold,
            bg_threshold,
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