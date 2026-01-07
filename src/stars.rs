use std::collections::HashMap;
use std::path::Path;

use serde::{Deserialize, Serialize};
use aether::math::Vector;
use image::{GrayImage, ImageReader};
use image::DynamicImage;
use crate::logger::ObjectReader;
use thiserror::Error;
use heapless;
use indexmap::IndexMap;

#[derive(Error, Debug)]
pub enum StartrackerError {
    #[error("No unique solution found for triangle {0}")]
    TriangleQueryFailure(String),
    #[error("One or more of the legs of the triangle being observed does not have a range in the k_vector table, so does not exist in the catalog")]
    NoStarsInCatalog,
    #[error("No unique solution found for pyramid")]
    PyramidConfirmation,
    #[error("Calculated leg falls outside of fov range / valid bounds")]
    NoPairsRange
}

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
pub struct CamConfig {
    pub fov: f64,
}

impl  Default for  CamConfig {
    fn default() -> Self {
        Self {
            fov: 27.2
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


struct StartrackerConfig {
    k_vector_path: Path,

}

pub struct Startracker {
    k_vector: K_Vector,
    star_pairs: Vec<StarPair>,
    star_cat: Vec<Star>,
}

impl Default for Startracker {
    fn default() -> Self {
        let mut stars_reader = ObjectReader::new("stars.dat");
        let mut star_cat = stars_reader.load_obj::<Vec<Star>>().expect("Failed to load star set");

        let mut pairs_reader = ObjectReader::new("star_pairs.dat");
        let mut star_pairs = pairs_reader.load_obj::<Vec<StarPair>>().expect("Failed to load pairs vec");

        let mut k_reader = ObjectReader::new("k_vector.dat");
        let k_vector: K_Vector = k_reader.load_obj::<K_Vector>().expect("Failed to load k vector table");

        for (id, star) in  star_cat.iter().enumerate() {
            if !validate_star_vec(&star.vector, "LOAD", id) {
                panic!("Stopping initialization due to bad serialization load.");
            }
        }
        
        
        Self {
            k_vector,
            star_pairs,
            star_cat,
        }
    }
}


// Function to validate that star unit vectors are normalized - used for troubleshooting star
// vec serialization error. REMOVE when vector serialization correction is added to main aether repo
pub fn validate_star_vec(vec: &Vector<f64, 3>, context: &str, id: usize) -> bool {
    // Check for NaNs or Infinite values
    if !vec[0].is_finite() || !vec[1].is_finite() || !vec[2].is_finite() {
        eprintln!("Error: [{}] Star {}: Vector contains NaN or Inf: {:?}", context, id, vec);
        return false;
    }

    // Unit vectors must be <= 1.0, with minor leniency
    if vec[0].abs() > 1.001 || vec[1].abs() > 1.001 || vec[2].abs() > 1.001 {
        eprintln!("Error:  [{}] Star {}: Components exceed 1.0 (Not normalized?): {:?}", context, id, vec);
        return false;
    }

    // Magnitude should be about 1
    let mag_sq = vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2];
    if (mag_sq - 1.0).abs() > 0.01 {
        eprintln!("Error:  [{}] Star {}: Vector is not normalized. Length^2: {}", context, id, mag_sq);
        return false;
    }

    true
}

impl Startracker {

    // Retrieve the unit vec of a specific star from the starid unit vector
    // Maybe make this by reference later, but we'll see
    pub fn retrieve_unit_vector(&self, star_id: usize) -> Star {
        self.star_cat[star_id].clone()
    }
    // Iterative implementation of checking 
    // pub fn query_triangle(&self, legs: &[f64;3]) -> Result<[usize;3], StartrackerError> {
    //     let mut counts: IndexMap<usize, usize> = IndexMap::new();
    //     let mut triangle: heapless::Vec<usize, 3> = heapless::Vec::new();
    //     for leg in legs {
    //         let (bound_1, bound_2) = self.k_vector.get_pairs_range(leg);

    //         if bound_1 == bound_2 {
    //             return Err(StartrackerError::TriangleQueryFailure(String::from("")));
    //         }

    //         let pairs = &self.star_pairs[bound_1..bound_2];

    //         for pair in pairs {
    //             // Count the occurence of each star ID that could be associated with the legs
    //             // if one has 3 occurences (unique solution), it can be checked against a fourth reference star (pyramid)
    //             *counts.entry(pair.id_1).or_insert(0) += 1;
    //             *counts.entry(pair.id_2).or_insert(0) += 1;
    //         }
    //     }
    //     for (star_id, count) in counts.iter() {
    //         if *count == 2 {
    //             triangle.push(*star_id).map_err(|_| StartrackerError::TriangleQueryFailure)?;
    //         }
    //     }
    //     match triangle.into_array::<3>() {
    //         Ok(stars) => Ok(stars),
    //         Err(_) => Err(StartrackerError::TriangleQueryFailure),
    //     }
    // }

    /// 
    
    // This is the "Pyramid" part of the algorthm
    // We have a triangle. This means 3 star ids, and three legs (angular distances)
    // We want to find a fourth reference star that exists, that we can validate has an exisitng
    // pair from the catalog with each of the 3 triangle stars. If this is true
    // then statistically these are definitely real stars
    pub fn pyramid_confirmation(&self, legs: &[f64; 3], ids: &[usize; 3]) -> Result<usize, StartrackerError> {
        // Get first star, and a leg corresponding to it
        let candidate_star = ids[0];
        let (lower_bound, upper_bound) = self.k_vector.get_pairs_range(&legs[0])?;
        let initial_pairs = &self.star_pairs[lower_bound..upper_bound];

        // Generate pairs for the first leg, convert to star ids of stars in the pairs,
        // and filter to find all stars that are the other member of a pair with the 
        // candidte star
        let mut r_vec: Vec<usize> = initial_pairs.iter()
                                             .filter_map(|star_pair| {
                                                if star_pair.id_1 == candidate_star {
                                                    Some(star_pair.id_2)
                                                } else if star_pair.id_2 == candidate_star {
                                                    Some(star_pair.id_1)
                                                } else {
                                                    None
                                                }
                                             }).collect();


        // Generate a second set of pairs for the second leg
        let (lower_bound, upper_bound) = self.k_vector.get_pairs_range(&legs[1])?;
        let validation_pairs = &self.star_pairs[lower_bound..upper_bound];
        let validation_id = ids[1];
        
        // Of the stars from the first pairs, only retain the ones that match to a cataloged pair from
        // the second leg, and that share an id value with the second star from the triangle query
        // This confirms the same candidate star exists in a pair for the first and second corners of
        // the triangle
        r_vec
            .retain(|candidate_id| {
                    validation_pairs
                            .iter()
                            .any(|pair| {
                                        (pair.id_1 == *candidate_id && pair.id_2 == validation_id)
                                        || (pair.id_1 == validation_id && pair.id_2 == * candidate_id)
                    })
            });
            
        
        // check against third leg
        let (lower_bound, upper_bound) = self.k_vector.get_pairs_range(&legs[2])?;
        let final_validation_pairs = &self.star_pairs[lower_bound..upper_bound];
        let final_validation_id = ids[2];


        // The same process as last time, confirming that a pair exists for the between the final
        // corner of the triangle, and a candidate id
        // By this point, if the stars are real, there should only be one candidate id left retained
        r_vec   
             .retain(|candidate_id| {
                    final_validation_pairs
                            .iter()
                            .any(|pair| {
                                        (pair.id_1 == *candidate_id && pair.id_2 == final_validation_id)
                                        || (pair.id_1 == final_validation_id && pair.id_2 == * candidate_id)
                    })
            });

        if r_vec.len() == 1 {
            Ok(r_vec[0])
        } else {
            Err(StartrackerError::PyramidConfirmation)
        }  
    }

    // Returns a list of ALL candidate triangles [ID_C0, ID_C1, ID_C2]
    // Basically, trying to find star ids that are shared between star pairs,
    // and build valid triangles (ones where each vertex is a member of two edges, shared by neighbor stars)
    pub fn query_triangle_topology(&self, legs: &[f64; 3]) -> Result<Vec<[usize; 3]>, StartrackerError> {
        // Get ranges
        let (s0, e0) = self.k_vector.get_pairs_range(&legs[0])?;
        let (s1, e1) = self.k_vector.get_pairs_range(&legs[1])?;
        let (s2, e2) = self.k_vector.get_pairs_range(&legs[2])?;

        if s0 == e0 || s1 == e1 || s2 == e2 {
            return Err(StartrackerError::TriangleQueryFailure(String::from("Leg does not exist in catalog")));
        }

        // Get legs
        let pairs_leg0 = &self.star_pairs[s0..e0];
        let pairs_leg1 = &self.star_pairs[s1..e1];
        let pairs_leg2 = &self.star_pairs[s2..e2];

        let mut valid_candidates = Vec::new();

        // Iterate over the pairs of leg0
        for base_pair in pairs_leg0 {
            
            // Check for both orientations (A->B and B->A) 
            for (c0, c1) in [(base_pair.id_1, base_pair.id_2), (base_pair.id_2, base_pair.id_1)] {
                
                // Find c2 candidates connected to c0 in Leg 1
                for pair1 in pairs_leg1 {
                    let c2 = match pair1.partner_of(c0) {
                        Some(id) => id,
                        None => continue, // Not connected to c0
                    };

                    // Verify the triangle is closed: Check if c2 connects to c1 in Leg 2
                    if self.list_contains_pair(pairs_leg2, c1, c2) {
                        valid_candidates.push([c0, c1, c2]);
                    }
                }
            }
        }

        if valid_candidates.is_empty() {
            Err(StartrackerError::TriangleQueryFailure(String::from("No valid triangles found")))
        } else {
            Ok(valid_candidates)
        }
    }

    // Check to see if a specific pair exists in a list
    fn list_contains_pair(&self, pairs: &[StarPair], a: usize, b: usize) -> bool {
        pairs.iter().any(|p| (p.id_1 == a && p.id_2 == b) || (p.id_1 == b && p.id_2 == a))
    }
}