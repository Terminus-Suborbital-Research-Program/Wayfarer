
use crate::stars::{
    CamConfig,
    Star,
    StarPair,
    K_Vector,
};
use crate::logger::{
    ObjectWriter,
    ObjectReader
};
use usno_gnc_catalog::{GncCatalogReader, IcrsPropagator};
use aether::math::Vector;

// Create the star table (Set of stars within a certain brightness)
// Create the star pair vector (set of all stars)
pub fn init_data() {
    let gnc = GncCatalogReader::from_csv("../angel/catalog/gnc_v1_1_mar_2_2023.csv").unwrap();
    let mut star_cat_saver = ObjectWriter::new("stars.dat");
    let mut star_pair_saver= ObjectWriter::new("star_pairs.dat");

    let mut stars: Vec<Star> = Vec::with_capacity(4424);
    let cam_config = CamConfig::default();

    let mut pairs: Vec<StarPair> = Vec::with_capacity(1_024_127);
    let min_cos = cam_config.fov.cos();
    let catalog = gnc.records;
    let epoch = 2025.9;
    let record_count = catalog.len();

    let mut valid_stars_count = 0;

    for record in catalog {
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
    star_pair_saver.write_obj(&pairs).unwrap();

}

pub fn init_k_vector() {
    // init_data();
    let mut stars_reader = ObjectReader::new("stars.dat");
    let mut stars = stars_reader.load_obj::<Vec<Star>>().unwrap();

    let mut pairs_reader = ObjectReader::new("star_pairs.dat");
    let mut pairs = pairs_reader.load_obj::<Vec<StarPair>>().unwrap();

    println!("Stars Loaded: {}, Pairs Loaded: {}", stars.len(), pairs.len());

    // Construct k_vector table

    let K = 10000;

    // Might change back later but k+1 now for the case that we accessed the last item in the array with the line equation.
    let mut k_vector = vec![0usize; K + 1];

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

    let k_vec = K_Vector::new(k_vector, cos_min, m);

    let mut k_writer = ObjectWriter::new("k_vector.dat");
    k_writer.write_obj(&k_vec).unwrap();
}
