pub mod astro;
pub mod file;
pub mod gui;
pub mod math;
pub mod model;

// TODO: move this out of test?
#[cfg(test)]
pub mod consts;

#[cfg(test)]
#[macro_use]
mod testing_utils {
    use nalgebra::Vector3;

    macro_rules! assert_very_large {
        ($exp:expr) => {
            approx::assert_relative_eq!($exp.recip(), 0.0)
        };
    }

    pub(crate) use assert_very_large;

    // We'll count this as a success if the difference between the vectors is small,
    // relative to the length of the expected vector.
    //
    // This differs from approx::assert_relative_eq, which works component-wise
    pub fn assert_vectors_close(expected: &Vector3<f64>, actual: &Vector3<f64>, tolerance: f64) {
        let difference = actual - expected;
        if difference.norm() >= tolerance * expected.norm() {
            panic!(
                "Vectors were not as close as expected!\n
                Expected: {}\n
                Received: {}\n
                Difference: {}\n
                Relative difference: {:e}",
                expected,
                actual,
                difference,
                difference.norm() / expected.norm(),
            );
        }
    }
}
