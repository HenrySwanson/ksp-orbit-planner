use super::intervals::Interval;

/// Very primitive way to construct a bracket for future root-finding.
/// Simply doubles the radius until a bracket with opposite signs at the
/// endpoints is found.
pub fn find_root_bracket(
    f: impl Fn(f64) -> f64,
    center: f64,
    mut radius: f64,
    num_iterations: usize,
) -> Interval {
    let initial_radius = radius;
    for _ in 0..num_iterations {
        let a = center - radius;
        let b = center + radius;

        if f(a) * f(b) < 0.0 {
            return Interval::new(a, b);
        }

        // Double the search radius
        radius *= 2.0;
    }

    panic!(
        "Unable to find two points of opposite sign, starting at {} with radius {}",
        center, initial_radius
    );
}

#[allow(clippy::float_cmp)]
pub fn bisection(f: impl Fn(f64) -> f64, mut interval: Interval, num_iterations: usize) -> f64 {
    // We need to determine which way f is oriented in our interval.
    let lo_is_neg = f(interval.lo()) < 0.0;

    for _ in 0..num_iterations {
        // Check right in the middle of the interval
        let guess = interval.midpoint();

        // If the interval is too small, we've converged.
        if guess == interval.lo() || guess == interval.hi() {
            return guess;
        }

        // Check the value and update the interval
        let value = f(guess);
        interval = match (lo_is_neg, value < 0.0) {
            (true, true) => interval.split_right(guess),   // - - +
            (true, false) => interval.split_left(guess),   // - + +
            (false, true) => interval.split_left(guess),   // + - -
            (false, false) => interval.split_right(guess), // + + -
        }
    }

    panic!(
        "Hit max iterations ({}) when trying to find a root in {}",
        num_iterations, interval
    );
}

// Adapted from `rtsafe` in http://www.grad.hr/nastava/gs/prg/NumericalRecipesinC.pdf
#[allow(clippy::float_cmp)]
pub fn newton_plus_bisection(
    f_and_f_prime: impl Fn(f64) -> (f64, f64),
    mut interval: Interval,
    num_iterations: usize,
) -> f64 {
    // Initial setup: we guess right in the middle of the interval. Also, we need to
    // determine which way f is oriented.
    let mut guess = interval.midpoint();
    let lo_is_neg = f_and_f_prime(interval.lo()).0 < 0.0;

    for _ in 0..num_iterations {
        let (f, f_prime) = f_and_f_prime(guess);

        // Update the bracket
        interval = match (lo_is_neg, f < 0.0) {
            (true, true) => interval.split_right(guess),   // - - +
            (true, false) => interval.split_left(guess),   // - + +
            (false, true) => interval.split_left(guess),   // + - -
            (false, false) => interval.split_right(guess), // + + -
        };

        // If the interval is too small, return
        let midpoint = interval.midpoint();
        if midpoint == interval.lo() || midpoint == interval.hi() {
            return guess;
        }

        // What's our next guess? Let's try one from Newton's method
        let newton_guess = guess - f / f_prime;

        // If it's outside the interval (or on the edge); discard it. It won't help
        // us shrink our search space.
        guess = if interval.contains(newton_guess)
            && newton_guess != interval.lo()
            && newton_guess != interval.hi()
        {
            newton_guess
        } else {
            // If we can't use Newton, use the midpoint
            interval.midpoint()
        };
    }

    panic!(
        "Hit max iterations ({}) when trying to find a root in {}",
        num_iterations, interval
    );
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn test_bisection() {
        // Find the root of x^3 - a for several a
        // TODO: this fails if a = 0, but not for convergence reasons, just
        // because lots of floats are near zero
        for a in [2.0, 50.0, -1.0, 0.1].iter() {
            let root = bisection(|x| x * x * x - a, Interval::new(-100.0, 100.0), 100);
            assert_relative_eq!(root, a.cbrt());
        }

        // There are three roots to x^3 - 4x^2 - 7x + 10: -2, 1, 5
        let f = |x| 10.0 + x * (-7.0 + x * (-4.0 + x));
        let x1 = bisection(f, Interval::new(-3.0, 0.0), 100);
        assert_relative_eq!(x1, -2.0);
        let x2 = bisection(f, Interval::new(0.0, 4.0), 100);
        assert_relative_eq!(x2, 1.0);
        let x3 = bisection(f, Interval::new(4.0, 10.0), 100);
        assert_relative_eq!(x3, 5.0);
    }

    #[test]
    fn test_cubics() {
        // Find the root of x^3 - a for several a
        // TODO: this actually fails to converge when a = 0!
        // Can we detect when Newton is converging slowly?
        for a in [2.0, 50.0, -1.0, 0.1].iter() {
            let root = newton_plus_bisection(
                |x| (x * x * x - a, 3.0 * x * x),
                Interval::new(-100.0, 100.0),
                100,
            );
            assert_relative_eq!(root, a.cbrt());
        }

        // There are three roots to x^3 - 4x^2 - 7x + 10: -2, 1, 5
        let f = |x| 10.0 + x * (-7.0 + x * (-4.0 + x));
        let f_ = |x| -7.0 + x * (-8.0 + x * 3.0);
        let x1 = newton_plus_bisection(|x| (f(x), f_(x)), Interval::new(-3.0, 0.0), 100);
        assert_relative_eq!(x1, -2.0);
        let x2 = newton_plus_bisection(|x| (f(x), f_(x)), Interval::new(0.0, 4.0), 100);
        assert_relative_eq!(x2, 1.0);
        let x3 = newton_plus_bisection(|x| (f(x), f_(x)), Interval::new(4.0, 10.0), 100);
        assert_relative_eq!(x3, 5.0);
    }

    #[test]
    fn test_trig() {
        // There's a unique fixed point cos(x) = x
        let root = newton_plus_bisection(
            |x| (x.cos() - x, -x.sin() - 1.0),
            Interval::new(-1.0, 1.0),
            100,
        );
        assert_relative_eq!(root, 0.73908513321516064);
    }
}
