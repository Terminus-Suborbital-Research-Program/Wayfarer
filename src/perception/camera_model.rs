
use crate::perception::centroiding::Centroid;
use aether::math::Vector;

#[derive(Debug, Clone)]
pub struct CameraModel {
    pub fov: f64,
    // Intrinsics 
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    // Distortion (Brown-Conrady)
    pub k1: f64, pub k2: f64, pub p1: f64, pub p2: f64, pub k3: f64,
}

impl Default for CameraModel {
    fn default() -> Self {
        Self {
            fov: 27.2,
            // Horizontal Scaling
            fx: 5424.41488,
            // Vertical Scaling
            fy: 5424.35391,
            // Horizontal Center
            cx: 1295.5,
            // Vertical Center
            cy: 971.5,
            // k 1..3 are coefficients that account for warping around the edges / corners (Radial Coefficients)
            k1: 0.51059,
            k2: -19.48679,
            k3: 171.20994,
            // p1 & 2 are coefficients that account for lens crookedness (tilt) relative to the sensor
            p1: -0.00291,
            p2: 0.0000206,
        }
    }
}
impl CameraModel {

    pub fn new(
        fov: f64,
        fx: f64, 
        fy: f64, 
        cx: f64, 
        cy: f64, 
        k1: f64, 
        k2: f64, 
        k3: f64, 
        p1: f64, 
        p2: f64
    ) -> Self {
        Self { fov,fx, fy, cx, cy, k1, k2, k3, p1, p2 }
    }

    /// The "Fundamental" Operation: Pixel (x, y) -> Unit Vector (x, y, z)
    /// This runs the iterative solver to remove distortion.
    pub fn undistort_centroid(&self, centroid: &mut Centroid) {
        let x = centroid.unit_loc[0];
        let y = centroid.unit_loc[1];
        // Normalize pixels to "Camera Coordinates" (centered, scale-invariant)
        // These are effectively "Distorted Slopes"
        let y_distorted = (y - self.cy) / self.fy;
        let x_distorted = (x - self.cx) / self.fx;

        // Iteratively Undistort (Fixed-Point Iteration)
        // We want to find the true (ideal) x and y such that applying distortion
        // yields x_distorted and y_distorted.
        let mut x_ideal = x_distorted;
        let mut y_ideal = y_distorted;

        // 5 iterations is usually sufficient for sub-pixel precision
        for _ in 0..5 {
            let r2 = x_ideal * x_ideal + y_ideal * y_ideal;
            let r4 = r2 * r2;
            let r6 = r2 * r4;

            // Radial component
            let k_radial = 1.0 + self.k1 * r2 + self.k2 * r4 + self.k3 * r6;

            // Tangential component
            let delta_x = 2.0 * self.p1 * x_ideal * y_ideal + self.p2 * (r2 + 2.0 * x_ideal * x_ideal);
            let delta_y = self.p1 * (r2 + 2.0 * y_ideal * y_ideal) + 2.0 * self.p2 * x_ideal * y_ideal;

            // The Inverse Step:
            // We know: x_distorted = x_ideal * k_radial + delta_x
            // Therefore: x_ideal = (x_distorted - delta_x) / k_radial
            x_ideal = (x_distorted - delta_x) / k_radial;
            y_ideal = (y_distorted - delta_y) / k_radial;
        }

        // Convert to Unit Vector
        // We now have the ideal slope (X/Z, Y/Z). We assume Z=1 to make a vector.
        let mut raw_vector = Vector::new([x_ideal, y_ideal, 1.0]);
        
        raw_vector = raw_vector.normalize();

        centroid.unit_loc.data = raw_vector.data;
    }

    pub fn undistort_centroids(&self, centroids: &mut [Centroid]){
        for centroid in centroids {
            self.undistort_centroid(centroid);
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