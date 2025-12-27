use usno_gnc_catalog::{GncCatalogReader, IcrsPropagator};
// use aether::
use serde::{Deserialize, Serialize};

use aether::math::Vector;

use bincode::{
    config::standard,
    serde::{decode_from_std_read, encode_into_std_write},
};
mod logger;
use logger::{
    ObjectWriter,
    ObjectReader
};
mod stars;

fn main() {
    
}

// Create the star table (Set of stars within a certain brightness)
// Create the star pair vector (set of all stars)
fn init_data() {
    let gnc = GncCatalogReader::from_csv("../angel/catalog/gnc_v1_1_mar_2_2023.csv").unwrap();
    let mut star_cat_saver = ObjectWriter::new("stars");
    let mut star_pair_saver= ObjectWriter::new("star_pairs");

    let mut stars: Vec<Star> = Vec::with_capacity(4424);
    let cam_config = CamConfig::default();

    let mut pairs: Vec<StarPair> = Vec::with_capacity(1_024_127);
    let min_cos = cam_config.fov.cos();
    let catalog = gnc.records;
    let epoch = 2025.9;
    let mut record_count = 0;

    let mut valid_stars_count = 0;

    for record in catalog {
        record_count += 1;
        println!("Record count: {record_count}");
        let Some(brightness) = record.bpmag.filter(|&mag| mag <= 6.0) else {
            continue;
        };
   
        let star =  record.icrs_sample(epoch);

        let mut star_vec = Vector::new(star.icrs_position_au);
        star_vec = star_vec.normalize();
        // star_vec.dot(rhs)
        stars.push(Star::new(star_vec, brightness));  
        valid_stars_count += 1;
        //    let mag = star.icrs_position_au.iter().map(|v_i| v_i.powi(2)).sum::<f64>().sqrt();
        //    let normalized_vectors: [f64 ; 3] = star.icrs_position_au.map(|v_i| v_i / mag);
       
    }
    println!("Valid stars count: {}, Total records: {}", valid_stars_count, record_count);
    star_cat_saver.write_obj(&stars).unwrap();


    let mut valid_pairs_count = 0;
    let n = 4424;
    for i in 0..n {
        for j in (i + 1)..n {
            let s1 = &stars[i];
            let s2 = &stars[j];
            

            let dot = s1.vector.dot(&s2.vector);

            if dot >= min_cos {

                pairs.push(StarPair {
                    cos_theta: dot,
                    id_1: i,
                    id_2: j,
                });
                valid_pairs_count += 1;
            }
        }
    }
    pairs.sort_by(|a,b| a.cos_theta.partial_cmp(&b.cos_theta).unwrap());
    println!("Valid Paris computed: {}", valid_pairs_count);
    println!("Pairs len{}", pairs.len());
    star_pair_saver.write_obj(&pairs).unwrap();

}

fn init_k_vector() {
    // init_data();
    let mut stars_reader = ObjectReader::new("stars");
    let mut stars = stars_reader.load_obj::<Vec<Star>>().unwrap();

    let mut pairs_reader = ObjectReader::new("star_pairs");
    let mut pairs = pairs_reader.load_obj::<Vec<StarPair>>().unwrap();

    println!("Stars Loaded: {}, Pairs Loaded: {}", stars.len(), pairs.len());

    // Construct k_vector table

    let K = 10000;

    let mut k_vector = vec![0usize; K];

    let cos_min = pairs.first().unwrap().cos_theta;
    let cos_max = pairs.last().unwrap().cos_theta;
    // Change in index given cosine
    let m = (K - 1) as f64 / (cos_max - cos_min);
    let mut current_k = 0;
    let mut k_index = 0;
    let mut y = 0;
    for (p_index, pair) in pairs.iter().enumerate() {
        // Calculate the index given the current cosign
        k_index = ((pair.cos_theta - cos_min) * m).floor() as usize;
        
        // Fill the k vector table up to that point with pointers to the first star pair that
        // creates this current k index, given it's cosine value
        while current_k <= k_index {
            k_vector[current_k] = p_index;
            current_k += 1;
        }
    }

    // Fill the last set of indeces
    let pairs_len = pairs.len();
    while current_k < K {
        k_vector[current_k] = pairs_len;
        current_k += 1;
    }

    let mut k_writer = ObjectWriter::new("k_vector");
    k_writer.write_obj(&k_vector).unwrap();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_k_vector_mapping_integrity() {
        // 1. Load the generated data
        let mut pairs_reader = ObjectReader::new("star_pairs");
        let mut k_reader = ObjectReader::new("k_vector");
        
        let pairs = pairs_reader.load_obj::<Vec<StarPair>>()
            .expect("Failed to load star_pairs");
        let k_table = k_reader.load_obj::<Vec<usize>>()
            .expect("Failed to load k_vector");

        let k_size = k_table.len() - 1; // K = 10000
        let cos_min = pairs.first().unwrap().cos_theta;
        let cos_max = pairs.last().unwrap().cos_theta;

        // 2. Reconstruct the line parameters used during init_data
        let m = (k_size as f64 - 1.0) / (cos_max - cos_min);

        // 3. Verify specific bins
        // Pick a few bins to check (e.g., bin 0, middle bin, and last bin)
        let bins_to_check = [0, k_size / 2, k_size - 1];

        for &bin_idx in &bins_to_check {
            let start_pair_idx = k_table[bin_idx];
            let end_pair_idx = k_table[bin_idx + 1];

            // If the bin isn't empty, check the first pair in that bin
            if start_pair_idx < end_pair_idx && start_pair_idx < pairs.len() {
                let pair_cos = pairs[start_pair_idx].cos_theta;
                
                // Reverse calculation: which bin SHOULD this cosine belong to?
                let calculated_bin = (m * (pair_cos - cos_min)).floor() as usize;
                
                // Allowing for minor floating point rounding, the calculated bin 
                // should match the bin_idx we are checking.
                assert!(
                    calculated_bin == bin_idx || calculated_bin == bin_idx - 1 || calculated_bin == bin_idx + 1,
                    "K-vector bin mismatch. Bin {} points to pair with cos {}, which belongs in bin {}",
                    bin_idx, pair_cos, calculated_bin
                );
            }
        }
    }

    #[test]
    fn test_k_vector_monotonicity() {
        let mut k_reader = ObjectReader::new("k_vector");
        let k_table = k_reader.load_obj::<Vec<usize>>().unwrap();

        // The k_vector indices MUST be non-decreasing
        for i in 0..k_table.len() - 1 {
            assert!(
                k_table[i] <= k_table[i+1],
                "K-vector is not monotonic at index {}: {} > {}",
                i, k_table[i], k_table[i+1]
            );
        }
    }

    #[test]
    fn test_catalog_integrity_counts() {
        // Initialize readers for the saved binary objects
        let mut stars_reader = ObjectReader::new("stars");
        let mut pairs_reader = ObjectReader::new("star_pairs");

        // Load the objects and handle potential IO/Decoding errors
        let stars = stars_reader.load_obj::<Vec<Star>>()
            .expect("Failed to load stars database");
        let star_pairs = pairs_reader.load_obj::<Vec<StarPair>>()
            .expect("Failed to load star_pairs database");

        // Verify Star Table Count (Mag <= 6.0)
        assert_eq!(
            stars.len(), 
            4424, 
            "Star catalog count mismatch. Expected 4424 stars brighter than Mag 6.0"
        );

        // Verify Star Pair Count (Angles within 62.2 deg FOV)
        assert_eq!(
            star_pairs.len(), 
            1024127, 
            "Star pair count mismatch. Expected 1024127 pairs within the 62.2 degree FOV"
        );

        // Verify ascending order of cos_theta
        for i in 0..star_pairs.len() - 1 {
            assert!(
                star_pairs[i].cos_theta <= star_pairs[i + 1].cos_theta,
                "Star pairs are not in ascending order at index {}: {} > {}",
                i,
                star_pairs[i].cos_theta,
                star_pairs[i + 1].cos_theta
            );
        }
    }

    #[test]
    fn test_pair_indices_validity() {
        let mut stars_reader = ObjectReader::new("stars");
        let mut pairs_reader = ObjectReader::new("star_pairs");

        let stars = stars_reader.load_obj::<Vec<Star>>().unwrap();
        let star_pairs = pairs_reader.load_obj::<Vec<StarPair>>().unwrap();

        // Check a sample to ensure indices don't point out of bounds
        for pair in star_pairs.iter().take(100) {
            assert!(pair.id_1 < stars.len(), "Index id_1 is out of bounds");
            assert!(pair.id_2 < stars.len(), "Index id_2 is out of bounds");
        }
    }
}