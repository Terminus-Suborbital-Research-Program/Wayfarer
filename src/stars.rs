use serde::{Deserialize, Serialize};
use aether::math::Vector;
use image::{GrayImage, ImageReader};
use image::DynamicImage;

#[derive(Serialize, Deserialize, Debug)]
pub struct Star {
    pub vector: Vector<f64,3>,
    // k_vectors: [f64; 3],
    pub mag: f32,
}

impl Star {
    pub fn new(vector: Vector<f64, 3>, mag: f32) -> Self {
        Self { vector, mag }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct StarPair {
    pub cos_theta: f64,
    pub id_1: usize,
    pub id_2: usize,
}

pub struct CamConfig {
    pub fov: f64,
}

impl  Default for  CamConfig {
    fn default() -> Self {
        Self {
            fov: 62.2
        }
    }
}



// impl TryFrom<&str> for Centroids {
//     fn try_from(value: &str) -> Result<Self, Self::Error> {
        
//     }
// }