use aether::math::Vector;
use crate::logger::ObjectReader;
use crate::perception::camera_model::{self, CameraModel};
use crate::startrack::stardat::{K_Vector, StarPair, Star};
use crate::startrack::error::StartrackerError;
use crate::perception::centroiding::Centroid;
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
    pub fn retrieve_unit_vector(&self, star_id: &usize) -> Star {
        self.star_cat[*star_id].clone()
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

impl Startracker {
    pub fn pyramid_solve(&self, centroids: Vec<Centroid> ) -> Result<(Vec<Vector<f64,3>>, Vec<Vector<f64,3>>), StartrackerError> {

        // let reference_vectors: Vector<f64,3>
        // Core Lost in space identification loop
        let mut j = 0;
        let mut k = 0;

        let n = centroids.len();
        for dj in 1..n-1 {
            for dk in 1..n-dj {
                for i in 0..(n-dj-dk) {
                    j = i + dj;
                    k = j + dk;
                    if i == j || j == k || i == k {
                        continue;
                    }
                    // We take the unit vectors relative to the center of the camer, of 3 centroids
                    let b1 = centroids[i].unit_loc;
                    let b2 = centroids[j].unit_loc;
                    let b3 = centroids[k].unit_loc;

                    // We compute their "legs", cosine theta values that can be used to represent the angular distance between
                    // a pair of stars. Found by the dot product of two unit vectors
                    let c_theta_1 = b1.dot(&b2);
                    let c_theta_2 = b1.dot(&b3);
                    let c_theta_3 = b2.dot(&b3);

                    match &self.query_triangle_topology(&[c_theta_1, c_theta_2, c_theta_3]) {
                        Ok(triangles) => {
                            // Unique Solution
                            let n_triangles = triangles.len();
                            if n_triangles == 1 {

                                let triangle = triangles[0];
                                let mut reference_vectors: Vec<Vector<f64,3>> = triangle
                                                                .iter()
                                                                .map(|id| 
                                                                        self.retrieve_unit_vector(&id).vector)
                                                                .collect();
                                let body_vectors = vec![b1, b2, b3];

                                return Ok((reference_vectors, body_vectors));
                            } else if n_triangles > 1 {
                                for r in 0..n {
                                    if r == i || r == j || r == k {continue;}

                                    let b4: Vector<f64, 3> = centroids[r].unit_loc;
                                    let confirmation_leg_1 = b1.dot(&b4);
                                    let confirmation_leg_2 = b2.dot(&b4);
                                    let confirmation_leg_3 = b3.dot(&b4);
                                    for triangle in triangles {
                                        match &self.pyramid_confirmation(
                                        &[confirmation_leg_1, confirmation_leg_2, confirmation_leg_3], triangle) {
                                        Ok(star_r_index) => {
                                            let mut reference_vectors: Vec<Vector<f64,3>> = triangle
                                                                .iter()
                                                                .map(|id| 
                                                                        self.retrieve_unit_vector(&id).vector)
                                                                .collect();
                                            reference_vectors.push(self.retrieve_unit_vector(&star_r_index).vector);

                                            let body_vectors = vec![b1, b2, b3, b4];
                                            
                                            return Ok((reference_vectors, body_vectors));

                                        }
                                        Err(startracker_err) => {
                                            // eprintln!("{}",startracker_err)
                                        }
                                    }  
                                    }
                                    
                                }
                            }
                            
                        }
                        Err(startracker_err) => {
                            // eprintln!("{}",startracker_err)
                        }
                    }
                    
                }
            }
        }
        Err(StartrackerError::NoSolution)
    }
}


use crate::startrack::quest::quest;
use aether::attitude::Quaternion;
use aether::reference_frame::Body;
use aether::reference_frame::ICRF;
struct Pyramid {
    reference_vectors: [Vector<f64,3>;4],
    body_vectors: [Vector<f64,3>;4],
    quaternion: Quaternion<f64,ICRF<f64>,Body<f64>>
}


// A Debug method to get a many solves for stars in the same image - need to fix
impl Startracker {
    pub fn exhaustive_solve(&self, centroids: Vec<Centroid>, mut n_pyramids: usize, camera_model: CameraModel) -> Result<(Vec<Vector<f64,3>>, Vec<Vector<f64,3>>), StartrackerError> {
        let mut reference_vectors: Vec<Vector<f64,3>> = Vec::new();
        let mut body_vectors: Vec<Vector<f64,3>> = Vec::new();

        // Core Lost in space identification loop
        let mut j = 0;
        let mut k = 0;

        let n = centroids.len();
        for dj in 1..n-1 {
            for dk in 1..n-dj {
                'inner: for i in 0..(n-dj-dk) {
                    j = i + dj;
                    k = j + dk;
                    if i == j || j == k || i == k {
                        continue;
                    }
                    // We take the unit vectors relative to the center of the camer, of 3 centroids
                    let b1 = centroids[i].unit_loc;
                    let b2 = centroids[j].unit_loc;
                    let b3 = centroids[k].unit_loc;

                    // We compute their "legs", cosine theta values that can be used to represent the angular distance between
                    // a pair of stars. Found by the dot product of two unit vectors
                    let c_theta_1 = b1.dot(&b2);
                    let c_theta_2 = b1.dot(&b3);
                    let c_theta_3 = b2.dot(&b3);

                    match &self.query_triangle_topology(&[c_theta_1, c_theta_2, c_theta_3]) {
                        Ok(triangles) => {
                            // Unique Solution
                            let n_triangles = triangles.len();
                            if n_triangles == 1 {
                                let star = triangles[0][0];
                                println!("Star Identified in initial triangle {}", star);
                                break;
                            } else if n_triangles > 1 {
                                for r in 0..n {
                                    if r == i || r == j || r == k {continue;}

                                    let b4: Vector<f64, 3> = centroids[r].unit_loc;
                                    let confirmation_leg_1 = b1.dot(&b4);
                                    let confirmation_leg_2 = b2.dot(&b4);
                                    let confirmation_leg_3 = b3.dot(&b4);
                                    for triangle in triangles {
                                        match &self.pyramid_confirmation(
                                        &[confirmation_leg_1, confirmation_leg_2, confirmation_leg_3], triangle) {
                                        Ok(star_r_index) => {
                                            // let mut reference_vectors: Vec<Vector<f64,3>> = triangle
                                            //                     .iter()
                                            //                     .map(|id| 
                                            //                             self.retrieve_unit_vector(&id).vector)
                                            //                     .collect();
                                            for id in triangle.iter() {
                                                reference_vectors.push(self.retrieve_unit_vector(&id).vector);
                                            }
                                            reference_vectors.push(self.retrieve_unit_vector(&star_r_index).vector);

                                            // let body_vectors = vec![b1, b2, b3, b4];
                                            body_vectors.push(b1);
                                            body_vectors.push(b2);
                                            body_vectors.push(b3);
                                            body_vectors.push(b4);
                                            
                                            if n_pyramids == 1 {
                                                return Ok((reference_vectors, body_vectors));
                                            } else {
                                                let q = quest(&reference_vectors, &body_vectors);

                                                for (reference, body) in reference_vectors.iter().zip(body_vectors.iter()) {
                                                    let star = q.rotate_vector(*reference);

                                                    if let Some((cat_x, cat_y)) = camera_model.project_vector(star) {
                                                        if let Some((body_x, body_y)) = camera_model.project_vector(*body) {
                                                            println!("({:.5}, {:.5}),", cat_x, cat_y);
                                                            println!("({:.5}, {:.5}),", body_x, body_y);
                                                            // println!("Check Coords");
                                                            // println!("------------");
                                                            // println!("Cat:  ({:.2}, {:.2})", cat_x, cat_y);
                                                            // println!("Body: ({:.2}, {:.2})", body_x, body_y);
                                                            // println!("---");
                                                        } else {
                                                            eprintln!("Measured star body vector failed projection (should be impossible?)");
                                                        }
                                                    } else {
                                                        eprintln!("Solved star is behind camera (Bad Quaternion?)");
                                                    }
                                                }
                                                reference_vectors.clear();
                                                body_vectors.clear();
                                                n_pyramids -= 1;
                                                break 'inner;
                                            }
                                            

                                        }
                                        Err(startracker_err) => {
                                            // eprintln!("{}",startracker_err)
                                        }
                                    }  
                                    }
                                    
                                }
                            }
                            
                        }
                        Err(startracker_err) => {
                            // eprintln!("{}",startracker_err)
                        }
                    }
                    
                }
            }
        }
        Err(StartrackerError::NoSolution)
        
    }
}
