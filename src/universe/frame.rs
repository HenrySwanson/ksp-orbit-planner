use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use simba::scalar::{RealField, SubsetOf, SupersetOf};

// See /latex/frame-transform.tex for the derivation of these formulas, and also
// for the variable names.

// TODO: would it make more sense for `angular_velocity` to measure
// the rotation of the target frame in its own basis?
// It breaks the analogy with `relative_velocity`, but it'd make
// a lot of sense for something like a Kerbin-fixed frame, where we'd
// always want to rotate around z (i.e., the polar axis).

pub struct FrameTransform<T: RealField> {
    /// Isometry taking source coordinates to target coordinates
    isometry: Isometry3<T>, // TODO break this apart? :\
    /// Relative velocity of target frame wrt source frame
    relative_velocity: Vector3<T>,
    /// Angular velocity of target frame wrt source frame
    angular_velocity: Vector3<T>,
}

impl<T: RealField> FrameTransform<T> {
    /// Creates a FrameTransform between the standard basis, and the basis given by applying
    /// `rotation`, followed by `translation`, to the standard basis vectors.
    /// The parameters `relative_velocity` and `angular_velocity` have the same meaning as
    /// they do in the struct.
    pub fn from_active(
        rotation: UnitQuaternion<T>,
        translation: Vector3<T>, // TODO should this be a Translation3?
        relative_velocity: Vector3<T>,
        angular_velocity: Vector3<T>,
    ) -> Self {
        // gotta invert the transformation to go from active to passive
        let active_isometry = Isometry3::from_parts(Translation3::from(translation), rotation);
        FrameTransform {
            isometry: active_isometry.inverse(),
            relative_velocity,
            angular_velocity,
        }
    }

    pub fn identity() -> Self {
        FrameTransform {
            isometry: Isometry3::identity(),
            relative_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
        }
    }

    pub fn inverse(&self) -> Self {
        // To get the relative velocity of the source frame wrt the target frame,
        // just convert from what it is in the source frame (e.g., zero).
        let relative_velocity = self.convert_velocity(&Point3::origin(), &Vector3::zeros());

        // Angular velocity transforms like a vector, and we need to reverse it.
        let angular_velocity = -self.convert_vector(&self.angular_velocity);

        FrameTransform {
            isometry: self.isometry.inverse(),
            relative_velocity,
            angular_velocity,
        }
    }

    pub fn append_transformation(&self, other: &Self) -> Self {
        // Say `self` transforms from frame A to B, and `other` from B to C.

        // The relative velocity is the velocity of C's origin in A's coordinates, and this
        // can be delegated to our helper functions.
        let relative_velocity = self.inverse_convert_velocity(
            &other.inverse_convert_point(&Point3::origin()),
            &other.relative_velocity,
        );

        // Angular velocities add, but first we have to convert both vectors to A's coordinates.
        let angular_velocity =
            self.angular_velocity + self.inverse_convert_vector(&other.angular_velocity);

        FrameTransform {
            isometry: other.isometry * self.isometry,
            relative_velocity,
            angular_velocity,
        }
    }

    pub fn prepend_transformation(&self, other: &Self) -> Self {
        other.append_transformation(self)
    }

    /// Converts the point from source coordinates to target coordinates
    pub fn convert_point(&self, pt: &Point3<T>) -> Point3<T> {
        self.isometry.transform_point(pt)
    }

    /// Converts the given point from target coordinates to source coordinates.
    pub fn inverse_convert_point(&self, pt: &Point3<T>) -> Point3<T> {
        self.isometry.inverse_transform_point(pt)
    }

    /// Converts the given vector from source coordinates to target coordinates.
    /// Unlike transform_point, this ignores the displacement of the two frames.
    pub fn convert_vector(&self, vector: &Vector3<T>) -> Vector3<T> {
        self.isometry.transform_vector(vector)
    }

    /// Converts the given vector from target coordinates to source coordinates.
    pub fn inverse_convert_vector(&self, vector: &Vector3<T>) -> Vector3<T> {
        self.isometry.inverse_transform_vector(vector)
    }

    /// Given an object's position and velocity in source coordinates, returns the velocity
    /// converted to target coordinates.
    pub fn convert_velocity(&self, position: &Point3<T>, velocity: &Vector3<T>) -> Vector3<T> {
        let rb_src = position - self.inverse_convert_point(&Point3::origin());
        let vb_src = velocity - self.relative_velocity - self.angular_velocity.cross(&rb_src);
        self.convert_vector(&vb_src)
    }

    /// Given an object's position and velocity in target coordinates, returns the velocity
    /// converted to source coordinates.
    pub fn inverse_convert_velocity(
        &self,
        position: &Point3<T>,
        velocity: &Vector3<T>,
    ) -> Vector3<T> {
        let vb_src = self.inverse_convert_vector(&velocity);
        let rb_src = self.inverse_convert_vector(&position.coords);
        vb_src + self.relative_velocity + self.angular_velocity.cross(&rb_src)
    }
}

impl<T1, T2> SubsetOf<FrameTransform<T2>> for FrameTransform<T1>
where
    T1: RealField,
    T2: RealField + SupersetOf<T1>,
{
    #[inline]
    fn to_superset(&self) -> FrameTransform<T2> {
        FrameTransform {
            isometry: self.isometry.to_superset(),
            relative_velocity: self.relative_velocity.to_superset(),
            angular_velocity: self.angular_velocity.to_superset(),
        }
    }

    #[inline]
    fn is_in_subset(transform: &FrameTransform<T2>) -> bool {
        nalgebra::is_convertible::<_, Isometry3<T1>>(&transform.isometry)
            && nalgebra::is_convertible::<_, Vector3<T1>>(&transform.relative_velocity)
            && nalgebra::is_convertible::<_, Vector3<T1>>(&transform.angular_velocity)
    }

    #[inline]
    fn from_superset_unchecked(transform: &FrameTransform<T2>) -> Self {
        FrameTransform {
            isometry: transform.isometry.to_subset_unchecked(),
            relative_velocity: transform.relative_velocity.to_subset_unchecked(),
            angular_velocity: transform.angular_velocity.to_subset_unchecked(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    // TODO why do i need `epsilon = 1e-14` everywhere? Is PI/2 not good enough?

    fn check_equality(lhs: FrameTransform<f64>, rhs: FrameTransform<f64>) {
        let epsilon = 1e-14;
        assert_relative_eq!(lhs.isometry, rhs.isometry, epsilon = epsilon);
        assert_relative_eq!(
            lhs.relative_velocity,
            rhs.relative_velocity,
            epsilon = epsilon
        );
        assert_relative_eq!(
            lhs.angular_velocity,
            rhs.angular_velocity,
            epsilon = epsilon
        );
    }

    #[test]
    fn test_transform() {
        // New frame is rotated by 90 degrees around z, and 3 units lower down.
        let xfm = FrameTransform::from_active(
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0),
            Vector3::new(0.0, 0.0, -3.0),
            Vector3::zeros(),
            Vector3::zeros(),
        );

        let pt = Point3::new(1.0, 2.0, 3.0);
        let dir = pt.coords;

        let expected_pt = Point3::new(2.0, -1.0, 6.0);
        let expected_dir = Vector3::new(2.0, -1.0, 3.0);

        assert_relative_eq!(xfm.convert_point(&pt), expected_pt, max_relative = 1e-15,);
        assert_relative_eq!(xfm.convert_vector(&dir), expected_dir, epsilon = 1e-14,);

        // Adding in a relative velocity won't change the positional data, but it will change
        // the velocity.
        let xfm_with_velocity = FrameTransform {
            isometry: xfm.isometry,
            relative_velocity: Vector3::new(5.0, 0.0, 0.0),
            angular_velocity: Vector3::new(4.0, 2.0, 0.0),
        };

        assert_relative_eq!(
            xfm_with_velocity.convert_point(&pt),
            expected_pt,
            epsilon = 1e-14,
        );
        assert_relative_eq!(
            xfm_with_velocity.convert_vector(&dir),
            expected_dir,
            epsilon = 1e-14,
        );
    }

    #[test]
    fn test_velocity_transform() {
        let sqrt_2 = 2.0_f64.sqrt();

        // New frame is rotated by 45 degrees around z, and is spinning about its x axis.
        let xfm = FrameTransform::from_active(
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 4.0),
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::new(1.0, 1.0, 0.0) / sqrt_2,
        );

        // At the origin, velocity transforms like a vector does. Furthermore, this is true for all
        // points on the rotation axis.
        let velocity = Vector3::new(1.0, 3.0, 8.0);
        assert_relative_eq!(
            xfm.convert_velocity(&Point3::origin(), &velocity),
            xfm.convert_vector(&velocity),
        );
        assert_relative_eq!(
            xfm.convert_velocity(&Point3::new(-4.0, -4.0, 0.0), &velocity),
            xfm.convert_vector(&velocity),
        );

        // Consider some points fixed in the source frame
        assert_relative_eq!(
            xfm.convert_velocity(&Point3::new(1.0, 0.0, 0.0), &Vector3::zeros()),
            Vector3::z() / sqrt_2,
        );
        assert_relative_eq!(
            xfm.convert_velocity(&Point3::new(0.0, 1.0, 0.0), &Vector3::zeros()),
            -Vector3::z() / sqrt_2,
        );
        assert_relative_eq!(
            xfm.convert_velocity(&Point3::new(0.0, 0.0, 1.0), &Vector3::zeros()),
            Vector3::y(),
        );

        // And for good measure, one moving point
        assert_relative_eq!(
            xfm.convert_velocity(&Point3::new(1.0, 0.0, 0.0), &Vector3::x()),
            Vector3::new(1.0 / sqrt_2, -1.0 / sqrt_2, 1.0 / sqrt_2),
        );

        // Now repeat with some linear velocity added
        let xfm2 = FrameTransform::from_active(
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 4.0),
            Vector3::zeros(),
            Vector3::new(0.0, 5.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0) / sqrt_2,
        );
        let extra_linear_velocity = -Vector3::new(1.0, 1.0, 0.0) * 5.0 / sqrt_2;
        assert_relative_eq!(
            xfm2.convert_velocity(&Point3::new(1.0, 0.0, 0.0), &Vector3::zeros()),
            Vector3::z() / sqrt_2 + extra_linear_velocity,
        );
        assert_relative_eq!(
            xfm2.convert_velocity(&Point3::new(0.0, 1.0, 0.0), &Vector3::zeros()),
            -Vector3::z() / sqrt_2 + extra_linear_velocity,
        );
        assert_relative_eq!(
            xfm2.convert_velocity(&Point3::new(0.0, 0.0, 1.0), &Vector3::zeros()),
            Vector3::y() + extra_linear_velocity,
        );
        assert_relative_eq!(
            xfm2.convert_velocity(&Point3::new(1.0, 0.0, 0.0), &Vector3::x()),
            Vector3::new(1.0 / sqrt_2, -1.0 / sqrt_2, 1.0 / sqrt_2) + extra_linear_velocity,
        );

        // Last one, same as before, but offset from the origin
        let xfm3 = FrameTransform::from_active(
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 4.0),
            Vector3::new(5.0, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::new(1.0, 1.0, 0.0) / sqrt_2,
        );

        // Vectors at the origin of the target frame don't get extra boosted
        assert_relative_eq!(
            xfm3.convert_velocity(&Point3::new(5.0, 0.0, 0.0), &Vector3::zeros()),
            Vector3::zeros()
        );
        assert_relative_eq!(
            xfm3.convert_velocity(&Point3::new(5.0, 0.0, 0.0), &Vector3::x()),
            Vector3::new(1.0, -1.0, 0.0) / sqrt_2
        );

        // Vectors away from the origin of the target may acquire velocity
        assert_relative_eq!(
            xfm3.convert_velocity(&Point3::origin(), &Vector3::zeros()),
            Vector3::z() * -5.0 / sqrt_2
        );
    }

    #[test]
    fn test_inverse_transforms() {
        // Arbitrary
        let xfm = FrameTransform::from_active(
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 1.0),
            Vector3::new(4.0, 0.0, 3.0),
            Vector3::new(1.0, -2.0, 0.0),
            Vector3::new(2.0, 3.0, 1.0),
        );

        let test_point = Point3::new(2.0, 2.0, 5.0);
        let test_dir = test_point.coords;

        // Doing and undoing a conversion should return the same point
        assert_relative_eq!(
            xfm.inverse_convert_point(&xfm.convert_point(&test_point)),
            test_point,
            epsilon = 1e-14,
        );
        assert_relative_eq!(
            xfm.convert_point(&xfm.inverse_convert_point(&test_point)),
            test_point,
            epsilon = 1e-14,
        );

        assert_relative_eq!(
            xfm.inverse_convert_vector(&xfm.convert_vector(&test_dir)),
            test_dir,
            epsilon = 1e-14,
        );
        assert_relative_eq!(
            xfm.convert_vector(&xfm.inverse_convert_vector(&test_dir)),
            test_dir,
            epsilon = 1e-14,
        );

        assert_relative_eq!(
            xfm.inverse_convert_velocity(
                &xfm.convert_point(&test_point),
                &xfm.convert_velocity(&test_point, &test_dir)
            ),
            test_dir,
            epsilon = 1e-14,
        );
        assert_relative_eq!(
            xfm.convert_velocity(
                &xfm.inverse_convert_point(&test_point),
                &xfm.inverse_convert_velocity(&test_point, &test_dir)
            ),
            test_dir,
            epsilon = 1e-14,
        );
    }

    #[test]
    fn test_composition() {
        // Two arbitrary transformations
        let xfm1 = FrameTransform::from_active(
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 2.0),
            Vector3::new(-1.0, 4.0, 3.0),
            Vector3::new(1.0, 0.0, 5.0),
            Vector3::new(2.0, 0.0, 8.0),
        );
        let xfm2 = FrameTransform::from_active(
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -3.5),
            Vector3::new(10.0, 3.0, 0.0),
            Vector3::new(0.0, -5.0, 8.0),
            Vector3::new(1.0, 1.0, 1.0),
        );

        // Appending and prepending should do the same thing
        check_equality(
            xfm1.append_transformation(&xfm2),
            xfm2.prepend_transformation(&xfm1),
        );

        // Applying two transformations should be the same thing as applying their composition
        let test_point = Point3::origin(); // Point3::new(1.0, 2.0, 4.0);
        let test_dir = 0.0 * Vector3::new(4.0, 1.0, 2.0);

        assert_relative_eq!(
            xfm2.prepend_transformation(&xfm1)
                .convert_point(&test_point),
            xfm2.convert_point(&xfm1.convert_point(&test_point)),
            epsilon = 1e-14,
        );
        assert_relative_eq!(
            xfm2.prepend_transformation(&xfm1).convert_vector(&test_dir),
            xfm2.convert_vector(&xfm1.convert_vector(&test_dir)),
            epsilon = 1e-14,
        );
        assert_relative_eq!(
            xfm2.prepend_transformation(&xfm1)
                .convert_velocity(&test_point, &test_dir),
            xfm2.convert_velocity(
                &xfm1.convert_point(&test_point),
                &xfm1.convert_velocity(&test_point, &test_dir)
            ),
            epsilon = 1e-13,
        );

        // Composing with the inverse should give the identity
        check_equality(
            xfm1.append_transformation(&xfm1.inverse()),
            FrameTransform::identity(),
        );
        check_equality(
            xfm1.inverse().append_transformation(&xfm1),
            FrameTransform::identity(),
        );
    }
}
