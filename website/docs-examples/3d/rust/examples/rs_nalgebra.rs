#[rustfmt::skip]
fn main() {
    // DOCUSAURUS: Vectors1 start
    use nalgebra::{Vector3, vector};
    let v = Vector3::new(1.0, 2.0, 3.0); // Basic constructor.
    let v = vector![1.0, 2.0, 3.0];      // Macro.
    let v: Point3<_> = [1.0, 2.0, 3.0].into();      // From<[T; 3]> conversion.
    // DOCUSAURUS: Vectors1 stop

    // DOCUSAURUS: Vectors2 start
    let v = vector![1.0, 2.0, 3.0]; // 3D vector.
    assert_eq!(v.x, 1.0);  assert_eq!(v.y, 2.0);  assert_eq!(v.z, 3.0);
    assert_eq!(v[0], 1.0); assert_eq!(v[1], 2.0); assert_eq!(v[2], 3.0);
    // DOCUSAURUS: Vectors2 stop

    // DOCUSAURUS: Points start
    use nalgebra::{Point3, point};
    let pt = Point3::new(1.0, 2.0, 3.0); // Basic constructor.
    let pt = point![1.0, 2.0, 3.0];      // Macro.
    let pt: Point3<_> = [1.0, 2.0, 3.0].into();     // From<[T; 3]> conversion.
    
    assert_eq!(pt.x, 1.0);  assert_eq!(pt.y, 2.0);  assert_eq!(pt.z, 3.0);
    assert_eq!(pt[0], 1.0); assert_eq!(pt[1], 2.0); assert_eq!(pt[2], 3.0);
    // DOCUSAURUS: Points stop

    {
    // DOCUSAURUS: Isometries start
    use nalgebra::{Isometry3, vector};
    let iso = Isometry3::translation(1.0, 2.0, 3.0);
    let iso = Isometry3::rotation(vector![0.1, 0.5, 0.7]); // The vector is the rotation in axis-angle format.
    let iso = Isometry3::new(vector![1.0, 2.0, 3.0], vector![0.1, 0.5, 0.7]);
    assert_eq!(iso.rotation.scaled_axis(), vector![0.1, 0.5, 0.7]);
    assert_eq!(iso.translation.vector, vector![1.0, 2.0, 3.0]);
    // DOCUSAURUS: Isometries stop

    {
    // DOCUSAURUS: Glam start
    // Here are just a few examples.
    use nalgebra::{vector, Point2, Isometry3};
    use glam::{Quat, Vec2, Vec3};

    let na_vector = vector![1.0, 2.0, 3.0];
    let glam_vector: Vec3 = na_vector.into();

    let glam_vector = Vec2::new(1.0, 2.0);
    let na_point: Point2<f32> = glam_vector.into();

    let na_isometry: Isometry3<f32> = 
        (Vec3::new(0.1, 2.0, 0.0), Quat::from_rotation_x(0.4)).into();
    // DOCUSAURUS: Glam stop
    }
    }
}
