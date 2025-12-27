#[derive(Serialize, Deserialize, Debug)]
struct Star {
    vector: Vector<f64,3>,
    // k_vectors: [f64; 3],
    mag: f32,
}

impl Star {
    fn new(vector: Vector<f64, 3>, mag: f32) -> Self {
        Self { vector, mag }
    }
}

#[derive(Serialize, Deserialize, Debug)]
struct StarPair {
    cos_theta: f64,
    id_1: usize,
    id_2: usize,
}

struct CamConfig {
    fov: f64,
}

impl  Default for  CamConfig {
    fn default() -> Self {
        Self {
            fov: 62.2
        }
    }
}