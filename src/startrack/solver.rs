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

        if s0 >= e0 || s1 >= e1 || s2 >= e2 {
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



use crate::startrack::quest::quest;
use aether::attitude::Quaternion;
use aether::reference_frame::Body;
use aether::reference_frame::ICRF;




impl Startracker {

    /// Project all captured centroids and accept ones that pass the threshold of the tolerance
    /// cosine or the best of centroid angular distance precision as inliers
    /// return inlier refs (catalog) and bodies
    fn score_quaternion(
        &self, 
        q: &Quaternion<f64, ICRF<f64>, Body<f64>>, 
        centroids: &[Centroid],
        tolerance_cos: f64 
    ) -> (Vec<Vector<f64, 3>>, Vec<Vector<f64, 3>>) {
        let mut inlier_refs = Vec::with_capacity(centroids.len());
        let mut inlier_bodies = Vec::with_capacity(centroids.len());
        let mut claimed_cat_indices = Vec::with_capacity(centroids.len());

        for centroid in centroids {
            let v_celestial = q.rotate_vector(centroid.unit_loc);
            let mut best_dot = tolerance_cos;
            let mut best_match_idx = None;

            for (idx, cat_star) in self.star_cat.iter().enumerate() {
                let dot = v_celestial.dot(&cat_star.vector);
                if dot > best_dot && !claimed_cat_indices.contains(&idx) {
                    best_dot = dot;
                    best_match_idx = Some(idx);
                }
            }

            if let Some(idx) = best_match_idx {
                claimed_cat_indices.push(idx);
                inlier_refs.push(self.star_cat[idx].vector);
                inlier_bodies.push(centroid.unit_loc);
            }
        }
        (inlier_refs, inlier_bodies)
    }


    /// Hybrid between Pyramid and Progressive Sample Consensus
    /// Basically, we look for a valid pyramid of 4 stars with an angular distance
    /// configuration that could befound in the database, like with the regular scheme 
    /// of the pyramid algorithm.
    /// However after this, it still could be false, as our camera is not of the 
    /// quality of typical startrackers, so we generate a quaternion from it,
    /// then see if that quaternion generates enough 
    pub fn pyramid_solve(&self, centroids: Vec<Centroid>, max_iters: u32, strict_threshold: f64) -> Result<(Vec<Vector<f64,3>>, Vec<Vector<f64,3>>), StartrackerError> {
        let n = centroids.len();
        if n < 4 {
            return Err(StartrackerError::NoSolution);
        }

        let mut best_inliers = 0;
        let mut best_solution: Option<(Vec<Vector<f64,3>>, Vec<Vector<f64,3>>)> = None;
        let mut counter = 0;

        // Smart loop to start from brightest stars, but swap around indeces
        // such that we never keep an invalid star for too long
        for dj in 1..n-1 {
            for dk in 1..n-dj {
                for i in 0..(n-dj-dk) {
                    let j = i + dj;
                    let k = j + dk;
                    if i == j || j == k || i == k { continue; }

                    let b1 = centroids[i].unit_loc;
                    let b2 = centroids[j].unit_loc;
                    let b3 = centroids[k].unit_loc;

                    let c_theta_1 = b1.dot(&b2);
                    let c_theta_2 = b1.dot(&b3);
                    let c_theta_3 = b2.dot(&b3);

                    if let Ok(triangles) = self.query_triangle_topology(&[c_theta_1, c_theta_2, c_theta_3]) {
                        
                        counter += 1;
                        if counter >= max_iters {
                            return Err(StartrackerError::LoserBounds); 
                        }

                        for triangle in triangles {
                            for r in 0..n {
                                if r == i || r == j || r == k { continue; }

                                let b4 = centroids[r].unit_loc;
                                let confirmation_leg_1 = b1.dot(&b4);
                                let confirmation_leg_2 = b2.dot(&b4);
                                let confirmation_leg_3 = b3.dot(&b4);

                                if let Ok(star_r_index) = self.pyramid_confirmation(
                                    &[confirmation_leg_1, confirmation_leg_2, confirmation_leg_3], &triangle) 
                                {
                                    let mut reference_vectors: Vec<Vector<f64,3>> = triangle
                                        .iter()
                                        .map(|id| self.retrieve_unit_vector(id).vector)
                                        .collect();
                                    reference_vectors.push(self.retrieve_unit_vector(&star_r_index).vector);
                                    let body_vectors = vec![b1, b2, b3, b4];

                                    let q_hyp = quest(&reference_vectors, &body_vectors);
                                    
                                    // Accept a limited amount of inlier stars, within a loose degree threshold (2.5 degree)
                                    let (pass1_refs, pass1_bodies) = self.score_quaternion(&q_hyp, &centroids, 0.999);
                                    let pass1_count = pass1_refs.len();     
                                    
                                    // If we capture enough inliers (more than 4)
                                    if pass1_count >= 4 && pass1_count > best_inliers {
                                        let q_stable = quest(&pass1_refs, &pass1_bodies);

                                        // Accept inliers that are of a higher degree of precision (driven by variable, but generally trying to capture around 2 pixel error)
                                        let (pass2_refs, pass2_bodies) = self.score_quaternion(&q_stable, &centroids, strict_threshold);
                                        let pass2_count = pass2_refs.len();

                                        if pass2_count >= 3 {
                                            best_inliers = pass1_count; 
                                            best_solution = Some((pass2_refs, pass2_bodies));

                                            if pass2_count >= 4 || pass1_count >= 5 {
                                                return Ok(best_solution.unwrap());
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if best_inliers >= 4 {
            if let Some(solution) = best_solution {
                return Ok(solution);
            }
        }

        Err(StartrackerError::NoSolution)
    }


    /// Method to try solving twice, one pass for a high accurracy solve
    /// with lower chance of succeeding, and one for a lower accuracy solve
    /// with a high change of succeeding
    pub fn adaptive_pyramid_solve(
        &self, 
        centroids: Vec<Centroid>
    ) -> Result<(Vec<Vector<f64, 3>>, Vec<Vector<f64, 3>>), StartrackerError> {
        
        // Tight threshold (0.08 deg), low iteration limit (Fast fail)                                               291 arcsec
        if let Ok(solution) = self.pyramid_solve(centroids.clone(), 500, 0.999999) {
            return Ok(solution);
        }

        // Loose threshold (0.25 deg), high iteration limit (Deep search)
        if let Ok(solution) = self.pyramid_solve(centroids, 100, 0.999998) {
            return Ok(solution);
        }

        // If both phases exhaust their limits, the image is truly unsolvable.
        Err(StartrackerError::NoSolution)
    }

}


impl Startracker {
    pub fn pyramid_solve_old(&self, centroids: Vec<Centroid> ) -> Result<(Vec<Vector<f64,3>>, Vec<Vector<f64,3>>), StartrackerError> {

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



