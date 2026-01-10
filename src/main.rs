#![allow(warnings)]

use aether::math::{Matrix, Vector};

use bincode::{
    config::standard,
    serde::{decode_from_std_read, encode_into_std_write},
};
mod logger;
use logger::{
    ObjectWriter,
    ObjectReader
};
mod startrack;
use startrack::{
    solver::Startracker
};
mod tests;
mod init;
use init::{init_data, init_k_vector};

// mod image_tools;
mod perception;


use perception::{
    camera_model::CameraModel,
    centroiding::{
        Starfinder,
        Centroid,
    }
};
use image::{GrayImage, ImageReader, Luma, RgbImage};
use image::DynamicImage;



fn main() {
    // init_data();
    // init_k_vector();

    let path = "/home/supergoodname77/Desktop/Elara/startracking/images/set1/bright-VISNIR-310ms_24d1g_50ict_0bl_0d80gam_set1_1.tiff";

    let img = ImageReader::open(path).unwrap().decode().unwrap();
    let mut gray_image = GrayImage::from(img.clone());

    let mut starfinder = Starfinder::default();

    let mut centroids = starfinder.star_find(&mut gray_image);

    let camera_view_centroids: Vec<Centroid> = centroids.iter().cloned().collect();

    let camera_model = CameraModel::default();

    camera_model.undistort_centroids(&mut centroids);

    let startracker = Startracker::default();

    
    match startracker.pyramid_solve(centroids) {
        Ok((reference_vectors, body_vectors)) => {
            // Need to confirm that reference vectors and body vectors are the same length
            // and definitely returned properly from pyramid solve
            // This should be fine but it would be better to be compiler enforced,
            // however going with raw vectors right now instead of cartesian
            let weights = reference_vectors.len();
            let mut B: Matrix<f64, 3, 3>  = Matrix::zeros();
            // let mut Z: Vector<f64,3> = Vector::zeros();
            // This is an assumption that all stars are equally trusted.
            // If this is made more sophisticated later, weights may vary and can be collected in an iterable 
            // data structure, and then each outer product of each star is multipied by the amount we trust
            // that specific star
            let weight = 1.0 / weights as f64;
            for (r, b) in reference_vectors.iter().zip(body_vectors.iter()) {
                let m = Matrix::outer(b, r);
                B = B + (m * weight);

                // let v = b.cross(r);
                // Z = Z + (v * weight)
            }

            let Z = Vector::new([
                B[(1, 2)] - B[(2, 1)],
                B[(2, 0)] - B[(0, 2)],
                B[(0, 1)] - B[(1, 0)]
            ]);

            let sigma = B.trace();
            // Symmetric MAtrix
            let S = B + B.transpose();

            // We only need the trace of the adjoint, which is the sum of the diagonal of the adjoint
            // The diagonal of the adjoint is the same as the diagnoal of the cofactor, so we can just
            // only calculate the diagonal cofactor values, and add them to get the same result
            let adj_00 = S[1][1] * S[2][2] - S[1][2] * S[2][1];
            let adj_11 = S[0][0] * S[2][2] - S[0][2] * S[2][0];
            let adj_22 = S[0][0] * S[1][1] - S[0][1] * S[1][0];
            let k = adj_00 + adj_11 + adj_22;

            // For the newton raphson optimized lambda we need alpha, beta, delta, and ceta
            let a = sigma.powi(2) - k;

            let b = sigma.powi(2) + Z.dot(&Z);

            let c = S.determinant() + Z.dot(&(S * Z));
            // let d = S.determinant() + Z.dot(&(S *(S * Z)));
            let d = Z.dot(&(S *(S * Z)));


            let lambda_guess = 1.0;
            let lambda_max = newton_raphson(a, b, c, d, sigma, lambda_guess);

            let alpha = lambda_max + sigma;
            let I: Matrix<f64, 3, 3> = Matrix::identity();
            let X = (I * alpha) - S;

            let p = cofactor(&X).transpose() * Z;

            let w = X.determinant();

            // let norm = 1.0 / (p.magnitude().powi(2) + w.powi(2)).sqrt();
            let norm = 1.0 / (p.dot(&p) + w.powi(2)).sqrt();

            // let q = 1 / sqr
            let q: Vector<f64,4> = Vector::new([p.data[0], p.data[1], p.data[2], w]) * norm;
            println!("Q: {:?}", q);
            println!("--- VERIFICATION ---");
            let mut total_error = 0.0;
            for (r, b) in reference_vectors.iter().zip(body_vectors.iter()) {
                let b_predicted = rotate_vector(&q, r);
                
                // Dot product gives cosine of angle between them
                let dot = b.dot(&b_predicted).min(1.0).max(-1.0); // Clamp for safety
                let angle_rad = dot.acos();
                let angle_deg = angle_rad.to_degrees();
                
                println!("Star Error: {:.5} degrees", angle_deg);
                total_error += angle_deg;
            }
            println!("Average Error: {:.5} degrees", total_error / reference_vectors.len() as f64);
        }

        Err(e) => {
            eprintln!("{}",e);
        }
    }
}

fn quest(reference_vectors: Vec<Vector<f64>>, body_vectors: Vec<Vector<f64>>) -> Vector<f64, 4> {
    
}

// Find the optimal lambda based on an intial guess lambda
fn newton_raphson(a: f64, b: f64, c: f64, d: f64, sigma: f64, mut lambda: f64) -> f64 {
    for i in 0..3 {
        let f = lambda.powi(4) - (a + b) * lambda.powi(2) - c * lambda + (a * b + c * sigma - d);
        let f_prime = 4.0 * lambda.powi(3) - 2.0 * (a + b) * lambda - c;
        // No divide by zero
        if f_prime.abs() < 1e-12 {
            break
        }
        lambda = lambda - (f / f_prime);
    }

    lambda
}

pub fn cofactor(m: &Matrix<f64, 3, 3>) -> Matrix<f64, 3, 3> {
    // let m: Matrix<T,N,N> = Matrix::zeros();
    // let m = *self;
    let mut res: Matrix<f64, 3, 3> = Matrix::zeros();

    let r_len = m.data.len();
    let c_len = m.data[0].len();
    for i in 0..r_len{
        for j in 0..c_len {
            // The above selects the row and col to avoid
            let det = sub_det(m, i, j);
            if (i + j) % 2 == 0 {
                res[(i,j)] = det;
            } else {
                res[(i,j)] = -det;
            }
        }
    }
    res
}

pub fn sub_det(m: &Matrix<f64, 3, 3>, row_excluded: usize, column_excluded: usize) -> f64 {
    let mut left = true;
    let mut left_diag = 1.0;
    let mut right_diag = 1.0;
    let mut c = 0;
    for i in 0..3 {
        for j in 0..3 {
            if i == row_excluded || j == column_excluded {continue;}
            if left {
                left_diag *= m[(i,j)];
                left = false;
            } else {
                right_diag *= m[(i,j)];
                c+= 1;
                if c == 2 {
                    left = true;
                }
            }
        }
    }
    left_diag - right_diag
}

fn rotate_vector(q: &Vector<f64, 4>, v: &Vector<f64, 3>) -> Vector<f64, 3> {
    let p = Vector::new([q[0], q[1], q[2]]);
    let w = q[3];
    
    let p_cross_v = p.cross(v);
    let p_cross_p_cross_v = p.cross(&p_cross_v);
    
    // v + 2*w*(p x v) + 2*(p x (p x v))
    v.clone() + (p_cross_v * (2.0 * w)) + (p_cross_p_cross_v * 2.0)
}
/*
Q: Vector { data: [0.22732334822563513, 0.16095940395572375, -0.26835912918251825, 0.9221711031116218] }


Q: Vector { data: [0.22732334822563513, 0.16095940395572375, -0.26835912918251825, 0.9221711031116218] }

*/

// tests/quest_sanity.rs
#[cfg(test)]
mod tests {
    use aether::math::{Vector, Matrix};
    // Import other necessary functions from your main or lib

    /// Helper to run the QUEST logic without the full Startracker struct
    /// (Copy-paste the math logic from your main.rs into a function 'solve_quest_math')
    fn run_quest_on_vectors(ref_vecs: &[Vector<f64, 3>], body_vecs: &[Vector<f64, 3>]) -> Vector<f64, 4> {
        // ... Insert your B-matrix, Z-vector, Newton-Raphson, and Quaternion logic here ...
        // Return the calculated q
    }

    #[test]
    fn test_quest_identity_case() {
        // SCENARIO 1: The "Perfect Alignment"
        // Camera is perfectly aligned with the Universe.
        // Body Vectors == Reference Vectors.
        let r1 = Vector::new([1.0, 0.0, 0.0]);
        let r2 = Vector::new([0.0, 1.0, 0.0]);
        let r3 = Vector::new([0.0, 0.0, 1.0]);

        let refs = vec![r1, r2, r3];
        let body = vec![r1, r2, r3]; // Identity: Body sees exactly what Catalog has

        let q = run_quest_on_vectors(&refs, &body);

        println!("Identity Q: {:?}", q);

        // EXPECTED: [0, 0, 0, 1] (or [0, 0, 0, -1])
        // The scalar part (w) should be 1.0. Vector part (xyz) should be 0.
        assert!((q[3].abs() - 1.0).abs() < 1e-6, "Scalar part should be 1.0 for identity");
        assert!(q[0].abs() < 1e-6, "X should be 0");
    }

    #[test]
    fn test_quest_90_degree_yaw() {
        // SCENARIO 2: 90 Degree Rotation around Z-axis
        // A star at X=1 (Ref) should appear at Y=-1 (Body) if we rotated 90 deg Left? 
        // Let's define: Body is rotated +90 deg around Z relative to Ref.
        // Rotation Matrix: [0 -1 0; 1 0 0; 0 0 1]
        
        let r1 = Vector::new([1.0, 0.0, 0.0]);
        let r2 = Vector::new([0.0, 1.0, 0.0]);
        let r3 = Vector::new([0.0, 0.0, 1.0]);

        // Body vectors are 'r' vectors rotated by 90 deg Z
        let b1 = Vector::new([0.0, 1.0, 0.0]);  // Ref X -> Body Y
        let b2 = Vector::new([-1.0, 0.0, 0.0]); // Ref Y -> Body -X
        let b3 = Vector::new([0.0, 0.0, 1.0]);  // Ref Z -> Body Z

        let refs = vec![r1, r2, r3];
        let body = vec![b1, b2, b3];

        let q = run_quest_on_vectors(&refs, &body);
        
        println!("90 Deg Z Q: {:?}", q);

        // Expected Quaternion for 90 deg Z: [0, 0, 0.707, 0.707]
        assert!(q[2].abs() > 0.7, "Should have strong Z component");
        assert!(q[3].abs() > 0.7, "Should have strong W component");
    }
}