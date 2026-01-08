use serde::{Deserialize, Serialize};
use aether::math::Vector;
use crate::startrack::error::StartrackerError;

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
pub struct Star {
    pub vector: Vector<f64,3>,
    pub mag: f32,
}
// k_vectors: [f64; 3],

impl Star {
    pub fn new(vector: Vector<f64, 3>, mag: f32) -> Self {
        Self { vector, mag }
    }
}
#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
pub struct StarPair {
    pub cos_theta: f64,
    pub id_1: usize,
    pub id_2: usize,
}
impl StarPair {
    /// Returns the ID of the star paired with `target`, or None if not found.
    pub fn partner_of(&self, target: usize) -> Option<usize> {
        if self.id_1 == target {
            Some(self.id_2)
        } else if self.id_2 == target {
            Some(self.id_1)
        } else {
            None
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct K_Vector {
    k_vector_table: Vec<usize>,
    cos_min: f64,
    cos_max: f64,
    m: f64,
    epsilon: f64,
}


impl K_Vector {
    pub fn new(k_vector_table: Vec<usize>, cos_min: f64,cos_max: f64, m: f64, epsilon: f64) -> Self {
        Self {
            k_vector_table,
            cos_min,
            cos_max,
            m,
            epsilon,
        }
    }

    // Based on 3 cos(theta) from a triangle of stars, compute the index into the table
    // Storing the ranges of star pairs we want to look at - pre_computed
    pub fn get_pairs_range(&self, leg: &f64) -> Result<(usize,usize), StartrackerError> {

        let start_cos = leg - self.epsilon;
        let end_cos = leg + self.epsilon;
        // Don't calculate index to access table if the index would be out of bounds
        if start_cos < self.cos_min || end_cos > (self.cos_max + 0.00001) { 
            // println!("Pairs range edge case: Cos min: {}, Cos max: {}, Leg: {}",self.cos_min, self.cos_max, leg);
            return Err(StartrackerError::NoPairsRange)
        }

        let k_index_start = ((start_cos - self.cos_min) * self.m).floor() as usize;
        let k_index_end = ((end_cos - self.cos_min) * self.m).floor() as usize;


        // Double verify we don't access index out of bounds
        if k_index_end >= self.k_vector_table.len() - 1 {
            return Err(StartrackerError::NoPairsRange)
        }
        
        let bound_1 = self.k_vector_table[k_index_start];
        let bound_2 = self.k_vector_table[k_index_end + 1];
        Ok((bound_1, bound_2))
    }
}