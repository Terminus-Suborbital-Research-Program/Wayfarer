use crate::logger::{
    ObjectWriter,
    ObjectReader
};

use crate::startrack::stardat::{
    // CamConfig,
    Star,
    StarPair,
};
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_k_vector_mapping_integrity() {
        // Load the generated data
        let mut pairs_reader = ObjectReader::new("star_pairs");
        let mut k_reader = ObjectReader::new("k_vector");
        
        let pairs = pairs_reader.load_obj::<Vec<StarPair>>()
            .expect("Failed to load star_pairs");
        let k_table = k_reader.load_obj::<Vec<usize>>()
            .expect("Failed to load k_vector");

        let k_size = k_table.len() - 1; // K = 10000
        let cos_min = pairs.first().unwrap().cos_theta;
        let cos_max = pairs.last().unwrap().cos_theta;

        //Reconstruct the line parameters used during init_data
        let m = (k_size as f64 - 1.0) / (cos_max - cos_min);

        // Verify specific bins
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

