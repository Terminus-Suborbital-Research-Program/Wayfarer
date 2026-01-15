use aether::math::{Vector,Matrix};
use aether::coordinate::Cartesian;
use aether::attitude::Quaternion;

pub fn quest(reference_vectors: &Vec<Vector<f64,3>>, body_vectors: &Vec<Vector<f64,3>>) -> Quaternion<f64> {
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

        // let norm = 1.0 / (p.dot(&p) + w.powi(2)).sqrt();

        // Shuster's algorithm, so negate rotational components
        let q = Quaternion::new(w, -p.data[0], -p.data[1], -p.data[2]).normalized();
        
        // let q: Vector<f64,4> = Vector::new([-p.data[0], -p.data[1], -p.data[2], w]) * norm;
        // println!("Q: {:?}", q);
        // println!("");
        // println!("Check Angle");
        // let mut total_error = 0.0;
        // for (r, b) in reference_vectors.iter().zip(body_vectors.iter()) {
            
        //     let b_predicted = q.rotate_vector(*r);
            
            
        //     let dot = b.dot(&b_predicted).min(1.0).max(-1.0); // Clamp for safety
        //     let angle_rad = dot.acos();
        //     let angle_deg = angle_rad.to_degrees();
            
        //     println!("Star Error: {:.5} degrees", angle_deg);
        //     total_error += angle_deg;
        // }
        // println!("Average Error: {:.5} degrees", total_error / reference_vectors.len() as f64);
        // println!("");


        return q;
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


