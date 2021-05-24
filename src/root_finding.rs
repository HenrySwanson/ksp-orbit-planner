// Invariant: lo <= hi
pub struct Bracket {
    lo: f64,
    hi: f64,
}

impl Bracket {
    pub fn new(mut lo: f64, mut hi: f64) -> Self {
        if lo > hi {
            std::mem::swap(&mut lo, &mut hi);
        }
        Bracket { lo, hi }
    }

    pub fn contains(&self, x: f64) -> bool {
        self.lo <= x && x <= self.hi
    }
}

impl std::fmt::Debug for Bracket {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "[{}, {}]", self.lo, self.hi)
    }
}

pub fn find_root_bracket(f: impl Fn(f64) -> f64, x: f64, mut r: f64) -> Bracket {
    for _ in 0..100 {
        let a = x - r;
        let b = x + r;

        if f(a) * f(b) < 0.0 {
            return Bracket::new(a, b);
        }

        // Double the search radius
        r *= 2.0;
    }

    panic!(
        "Unable to find two points of opposite sign, starting at {} with radius {}",
        x, r
    );
}

// Adapted from `rtsafe` in http://www.grad.hr/nastava/gs/prg/NumericalRecipesinC.pdf
pub fn newton_plus_bisection(
    f_and_f_prime: impl Fn(f64) -> (f64, f64),
    mut bracket: Bracket,
    num_iterations: usize,
) -> f64 {
    // Initial setup: we guess right in the middle of the bracket. Also, we need to determine
    // which way our bracket is oriented.
    let mut guess = (bracket.lo + bracket.hi) / 2.0;
    let lo_is_neg = f_and_f_prime(bracket.lo).0 < 0.0;

    for _ in 0..num_iterations {
        let (f, f_prime) = f_and_f_prime(guess);

        // Update the bracket
        match (lo_is_neg, f < 0.0) {
            (true, true) => bracket.lo = guess,   // - - +
            (true, false) => bracket.hi = guess,  // - + +
            (false, true) => bracket.hi = guess,  // + - -
            (false, false) => bracket.lo = guess, // + + -
        }

        // TODO what if f_prime is zero?
        let newton_guess = guess - f / f_prime;

        // Use the Newton's method guess if possible, otherwise fall back on bisection.
        guess = if bracket.contains(newton_guess) {
            // If the guess hasn't changed any, we've converged
            if newton_guess == guess {
                return guess;
            }
            newton_guess
        } else {
            // If the bracket is too small, we've converged.
            let midpoint = (bracket.lo + bracket.hi) / 2.0;
            if midpoint == bracket.lo || midpoint == bracket.hi {
                return guess;
            }
            midpoint
        };
    }

    panic!(
        "Hit max iterations ({}) when trying to find a root in {:?}",
        num_iterations, bracket
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    use approx::assert_relative_eq;

    #[test]
    fn test_cubics() {
        // Find the root of x^3 - a for several a
        // TODO: this actually fails to converge when a = 0!
        // Can we detect when Newton is converging slowly?
        for a in [2.0, 50.0, -1.0, 0.1].iter() {
            let root = newton_plus_bisection(
                |x| (x * x * x - a, 3.0 * x * x),
                Bracket::new(-100.0, 100.0),
                100,
            );
            assert_relative_eq!(root, a.cbrt());
        }

        // There are three roots to x^3 - 4x^2 - 7x + 10: -2, 1, 5
        let f = |x| 10.0 + x * (-7.0 + x * (-4.0 + x));
        let f_ = |x| -7.0 + x * (-8.0 + x * 3.0);
        let x1 = newton_plus_bisection(|x| (f(x), f_(x)), Bracket::new(-3.0, 0.0), 100);
        assert_relative_eq!(x1, -2.0);
        let x2 = newton_plus_bisection(|x| (f(x), f_(x)), Bracket::new(0.0, 4.0), 100);
        assert_relative_eq!(x2, 1.0);
        let x3 = newton_plus_bisection(|x| (f(x), f_(x)), Bracket::new(4.0, 10.0), 100);
        assert_relative_eq!(x3, 5.0);
    }

    #[test]
    fn test_trig() {
        // There's a unique fixed point cos(x) = x
        let root = newton_plus_bisection(
            |x| (x.cos() - x, -x.sin() - 1.0),
            Bracket::new(-1.0, 1.0),
            100,
        );
        assert_relative_eq!(root, 0.73908513321516064);
    }
}
