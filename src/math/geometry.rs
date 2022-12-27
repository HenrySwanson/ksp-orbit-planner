use nalgebra::{Rotation3, Vector3};

use std::f64::consts::PI;

pub fn reject(u: &Vector3<f64>, v: &Vector3<f64>) -> Vector3<f64> {
    // Computes the vector rejection of u from v. v must be non-zero.
    let u_proj_v = u.dot(v) * v / v.norm_squared();
    u - u_proj_v

    // TODO this could also be done by computing v x (u x v) and normalizing
}

pub fn directed_angle(u: &Vector3<f64>, v: &Vector3<f64>, up: &Vector3<f64>) -> f64 {
    // Returns the angle between u and v, measured as a positive angle around 'up'.
    let theta = u.angle(v);
    if u.cross(v).dot(up) >= 0.0 {
        theta
    } else {
        2.0 * PI - theta
    }
}

/// Returns a rotation R that sends the z- and x- axes to point in the specified directions.
/// The orthogonality of new_z and new_x is not checked.
/// If new_z or new_x is sufficiently close to zero, then some semi-canonical choices will
/// be made. Unfortunately, the hairy ball theorem prevents us from doing so in a completely
/// canonical way.
///
/// The specific choices we make are:
/// - if new_z is small:
///   - R(z) will point as much along the z-axis as possible, while remaining perpendicular
///     to R(x) = new_x
///   - if this is ill-defined (new_x ~= z), then R(z) = y
/// - similarly, if new_x is small:
///   - R(x) will point as much along the x-axis as possible, while remaining perpendicular
///     to R(z) = new_z
///   - if this is ill-defined (new_z ~= x), then R(x) = -y
/// - if both new_z and new_x are small, then this returns the identity
pub fn always_find_rotation(
    new_z: &Vector3<f64>,
    new_x: &Vector3<f64>,
    tolerance: f64,
) -> Rotation3<f64> {
    let z_large_enough = new_z.norm() >= tolerance;
    let x_large_enough = new_x.norm() >= tolerance;

    let (new_z, new_x) = match (z_large_enough, x_large_enough) {
        // Both are good; the easy case
        (true, true) => (*new_z, *new_x),
        // z is too small
        (false, true) => {
            // Rejecting the z axis from new_x gives us the most-z-like vector
            // that's perpendicular to new_x. If it's too small, we just pick
            // our fallback choice.
            let mut best_new_z = reject(&Vector3::z(), new_x);
            if best_new_z.norm() < tolerance {
                best_new_z = Vector3::y();
            };
            (best_new_z, *new_x)
        }
        // x is too small
        (true, false) => {
            // Same thing as above, with z and x switched.
            let mut best_new_x = reject(&Vector3::x(), new_z);
            if best_new_x.norm() < tolerance {
                best_new_x = -Vector3::y();
            };
            (*new_z, best_new_x)
        }
        // Easy case, early return here.
        (false, false) => return Rotation3::identity(),
    };

    // Unfortunately, the Rotation::face_towards call takes new-z and new-y as arguments,
    // so we prepend a 90-degree rotation around z (e.g., one taking x to y).
    let mut rotation = Rotation3::face_towards(&new_z, &new_x);
    rotation *= Rotation3::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
    rotation.renormalize();
    rotation
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reject() {
        approx::assert_relative_eq!(
            reject(&Vector3::new(4.0, 5.0, 7.0), &Vector3::new(1.0, 2.0, 3.0)),
            Vector3::new(1.5, 0.0, -0.5),
        );

        approx::assert_relative_eq!(
            reject(
                &Vector3::new(5.0, 8.0, -2.0),
                &Vector3::new(1.0, -2.0, -1.0)
            ),
            Vector3::new(6.5, 5.0, -3.5),
        );

        approx::assert_relative_eq!(
            reject(&Vector3::new(0.0, 0.0, 0.0), &Vector3::new(1.0, 2.0, 3.0)),
            Vector3::new(0.0, 0.0, 0.0),
        );
    }

    #[test]
    fn test_rotation_code() {
        fn test_rotation(r: Rotation3<f64>, expected_z: &Vector3<f64>, expected_x: &Vector3<f64>) {
            approx::assert_relative_eq!(
                r * Vector3::z(),
                expected_z.normalize(),
                max_relative = 1e-15
            );
            approx::assert_relative_eq!(
                r * Vector3::x(),
                expected_x.normalize(),
                max_relative = 1e-15
            );
        }

        // Common vectors
        let u = Vector3::new(1.0, 2.0, 3.0);
        let v = Vector3::new(2.0, 2.0, -2.0);

        // Normal
        test_rotation(always_find_rotation(&u, &v, 1e-20), &u, &v);

        // new-z is too small
        test_rotation(
            always_find_rotation(&Vector3::zeros(), &v, 1e-20),
            &Vector3::new(1.0, 1.0, 2.0),
            &v,
        );

        // new-z is too small, and new-x points along z
        // TODO should we treat new-x = kz and new-x = -kz differently?
        test_rotation(
            always_find_rotation(&Vector3::zeros(), &Vector3::z(), 1e-20),
            &Vector3::y(),
            &Vector3::z(),
        );

        // new-x is too small
        test_rotation(
            always_find_rotation(&u, &Vector3::zeros(), 1e-20),
            &u,
            &Vector3::new(13.0, -2.0, -3.0),
        );

        // new-x is too small, and new-z points along x
        test_rotation(
            always_find_rotation(&Vector3::x(), &Vector3::zeros(), 1e-20),
            &Vector3::x(),
            &-Vector3::y(),
        );

        // both are small
        test_rotation(
            always_find_rotation(&Vector3::zeros(), &Vector3::zeros(), 1e-20),
            &Vector3::z(),
            &Vector3::x(),
        );
    }
}
