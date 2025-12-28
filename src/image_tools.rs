use image::{GrayImage, ImageReader, Luma};
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
    unit_loc: Vector<u32, 2>,
    brightness: u8,
}

impl Centroid {
    pub fn new(x: u32, y: u32, brightness: u8) -> Self {
        Self {
            unit_loc: Vector::new([x,y]),
            brightness,
        }
    }
}

/// Convert images to star detectable format, find centroids, and allow easy access to vector data used for identification calculations
pub struct Starfinder {
    // Threshold for what pixels we simply don't want to check to see if they are a centroid, because they look like background noise
    threshold: u8
}

impl Default for Starfinder {
    fn default() -> Self {
        Self { 
            threshold: 20u8 
        }
    }
}

impl Starfinder {
    fn new(threshold: u8) -> Self {
        Self {
            threshold,
        }
    }

    fn pre_process(&self) {
        let path = "/home/supergoodname77/Desktop/Elara/startracking/images/set1/bright-VISNIR-310ms_24d1g_50ict_0bl_0d80gam_set1_1.tiff";
        let gray_path = "/home/supergoodname77/Desktop/Elara/startracking/images/set1/output_gray.png";

        let img = ImageReader::open(path).unwrap().decode().unwrap();
        let mut gray_image = GrayImage::from(img);
        // gray_image.save(gray_path).unwrap();
        let centroids: Vec<Centroid> = Vec::new();
        let (width, height) = gray_image.dimensions();

        

        // Manually iterate over image so that we don't try to index outside of the array when 
        // let mut vec: Vec<u8> = Vec::new();
        for y in 1..(height - 1) {
            for x in 1..(width - 1) {
                let pixel = gray_image.get_pixel(x, y);
                let brightness = pixel[0];
                if self.is_dead_pixel(&gray_image, x, y, brightness) {
                    // gray_image.put_pixel(x, y, Luma([0]));
                    continue;
                }
                // println!("Brightness: {}",brightness);
                // vec.push(brightness);

            }
        }
        // vec.sort();
        // let mut new: Vec<u8> = vec.into_iter().filter(|brightness| *brightness > 20u8).collect();
        
        // new.sort();
        // println!("LEN: {} ",new.len());

        // println!("Lowest Brightness: {}, Highest: {}",new[0], new.last().unwrap());
        // let sum = new.iter().map(|&b| b as u32).sum::<u32>();
        // // let average = sum / new.len() as u32;
        // let mid_index = (new.len() - 1) / 2;
        // let average = sum / new.len() as u32;

        // println!("Average: {}, Median {}",average, new[mid_index]);

        // new
        
        // let true_gray = GrayImage::from(gray_img);
        // for star in star_centers {
        //     let point = star.coord();
        //     println!("X: {} Y: {}", point.x, point.y);
        // }
    }

    fn is_dead_pixel(&self, image: &GrayImage, x: u32, y: u32, brightness: u8,) -> bool {
        if brightness > self.threshold {
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
        }
        // No valid surrounding star bleed detected ; must not be a star
        return true;
    }

    fn centroid(&self, image: &GrayImage, x: u32, y: u32, brightness: u8) {
        let (width, height) = image.dimensions();

    }

}


// fn remove_hot_pixels()