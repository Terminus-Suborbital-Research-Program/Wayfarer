#[cfg(test)]
mod tests {
    use aether::math::{Vector, Matrix};

    use crate::startrack::quest::quest;


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

        let q = quest(&refs, &body);

        println!("Identity Q: {:?}", q);

        // EXPECTED: [0, 0, 0, 1] (or [0, 0, 0, -1])
        // The scalar part (w) should be 1.0. Vector part (xyz) should be 0.
        assert!((q.w().abs() - 1.0).abs() < 1e-6, "Scalar part should be 1.0 for identity");
        assert!(q.i().abs() < 1e-6, "i should be 0");
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

        let q = quest(&refs, &body);
        
        println!("90 Deg Z Q: {:?}", q);

        // Expected Quaternion for 90 deg Z: [0, 0, 0.707, 0.707]
        assert!(q.k().abs() > 0.7, "Should have strong Z component");
        assert!(q.w().abs() > 0.7, "Should have strong W component");
    }
}