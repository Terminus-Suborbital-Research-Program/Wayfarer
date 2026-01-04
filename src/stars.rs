use std::collections::HashMap;
use std::path::Path;

use aether::terrestrial::wgs84::constants::c;
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
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
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
#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
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
}


impl K_Vector {
    pub fn new(k_vector_table: Vec<usize>, cos_min: f64,cos_max: f64, m: f64) -> Self {
        Self {
            k_vector_table,
            cos_min,
            cos_max,
            m,
        }
    }

    // Based on 3 cos(theta) from a triangle of stars, compute the index into the table
    // Storing the ranges of star pairs we want to look at - pre_computed
    pub fn get_pairs_range(&self, leg: &f64) -> (usize,usize) {
        // Don't calculate index to access table if the index would be out of bounds
        if *leg < self.cos_min || *leg > (self.cos_max + 0.00001) { 
            println!("Edge case 1: Cos min: {}, Cos max: {}, Leg: {}",self.cos_min, self.cos_max, leg);
            return (0, 0);
        }

        let k_index = ((leg - self.cos_min) * self.m).floor() as usize;

        // Double verify we don't access index out of bounds
        if k_index >= self.k_vector_table.len() - 1 {
            println!("Edge case 2");

            return (0, 0); 
        }
        
        let bound_1 = self.k_vector_table[k_index];
        let bound_2 = self.k_vector_table[k_index + 1];
        (bound_1, bound_2)
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
        
        Self {
            k_vector,
            star_pairs,
            star_cat,
        }
    }
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

    // Return starid of R
    pub fn pyramid_confirmation(&self, legs: &[f64; 3], ids: &[usize; 3]) -> Result<usize, StartrackerError> {
        
        let candidate_star = ids[0];
        let (lower_bound, upper_bound) = self.k_vector.get_pairs_range(&legs[0]);


        let initial_pairs = &self.star_pairs[lower_bound..upper_bound];

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
                                                // .filter(|star_pair| 
                                                //     star_pair.id_1 == candidate_star || 
                                                //     star_pair.id_2 == candidate_star)
                                                // .flat_map(|pair| [pair.id_1, pair.id_2])
                                                // .collect();


        // Check against second leg
        let (lower_bound, upper_bound) = self.k_vector.get_pairs_range(&legs[1]);
        let validation_pairs = &self.star_pairs[lower_bound..upper_bound];
        let validation_id = ids[1];

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
        let (lower_bound, upper_bound) = self.k_vector.get_pairs_range(&legs[2]);
        let final_validation_pairs = &self.star_pairs[lower_bound..upper_bound];
        let final_validation_id = ids[2];



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
    pub fn query_triangle_topology(&self, legs: &[f64; 3]) -> Result<Vec<[usize; 3]>, StartrackerError> {
        
        // 1. Get the lists of pairs for each leg
        // Leg 0: Connects C0 - C1
        let (s0, e0) = self.k_vector.get_pairs_range(&legs[0]);
        // Leg 1: Connects C0 - C2
        let (s1, e1) = self.k_vector.get_pairs_range(&legs[1]);
        // Leg 2: Connects C1 - C2
        let (s2, e2) = self.k_vector.get_pairs_range(&legs[2]);

        if s0 == e0 || s1 == e1 || s2 == e2 {
            return Err(StartrackerError::TriangleQueryFailure(String::from("No pairs length - leg does not exist in star pairs")));
        }

        let pairs_leg0 = &self.star_pairs[s0..e0]; // Potential edges for C0-C1
        let pairs_leg1 = &self.star_pairs[s1..e1]; // Potential edges for C0-C2
        let pairs_leg2 = &self.star_pairs[s2..e2]; // Potential edges for C1-C2

        let mut valid_candidates = Vec::new();

        // 2. Iterate through Leg 0 to find the "Base" of the triangle
        for pair_0 in pairs_leg0 {
            // We have two possible orientations for this pair:
            // Case A: C0 = pair_0.id_1, C1 = pair_0.id_2
            // Case B: C0 = pair_0.id_2, C1 = pair_0.id_1
            
            // --- CHECK CASE A ---
            let c0 = pair_0.id_1;
            let c1 = pair_0.id_2;
            
            // Find candidates for C2. 
            // C2 must be a partner of C0 in Leg 1 list.
            let c2_candidates = self.find_partners_in_list(pairs_leg1, c0);
            
            for c2 in c2_candidates {
                // Verify C2 is also connected to C1 via Leg 2
                if self.list_contains_pair(pairs_leg2, c1, c2) {
                    valid_candidates.push([c0, c1, c2]);
                }
            }

            // --- CHECK CASE B ---
            let c0 = pair_0.id_2; // Swap
            let c1 = pair_0.id_1; // Swap
            
            let c2_candidates = self.find_partners_in_list(pairs_leg1, c0);
            for c2 in c2_candidates {
                if self.list_contains_pair(pairs_leg2, c1, c2) {
                    valid_candidates.push([c0, c1, c2]);
                }
            }
        }

        if valid_candidates.is_empty() {
            Err(StartrackerError::TriangleQueryFailure(String::from("No valid triangle candidates sharing pair intersections")))
        } else {
            Ok(valid_candidates)
        }
    }

    // Helper: Find all stars paired with 'target' in a specific list of pairs
    fn find_partners_in_list(&self, pairs: &[StarPair], target: usize) -> Vec<usize> {
        let mut partners = Vec::new();
        for p in pairs {
            if p.id_1 == target { partners.push(p.id_2); }
            else if p.id_2 == target { partners.push(p.id_1); }
        }
        partners
    }

    // Helper: Check if a specific pair exists in a list
    fn list_contains_pair(&self, pairs: &[StarPair], a: usize, b: usize) -> bool {
        pairs.iter().any(|p| 
            (p.id_1 == a && p.id_2 == b) || (p.id_1 == b && p.id_2 == a)
        )
    }
}