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

// Adapted from http://www.grad.hr/nastava/gs/prg/NumericalRecipesinC.pdf
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
        let newton_guess = guess - f / f_prime;

        let new_guess = if bracket.contains(newton_guess) {
            // Use the Newton's method guess if possible
            newton_guess
        } else {
            // If the Newton's method guess is out of the interval, fall back on bisection.
            (bracket.lo + bracket.hi) / 2.0
        };

        // If the change would be negligible, we've converged to machine precision and should
        // return.
        if new_guess == guess {
            return guess;
        }
        guess = new_guess;

        // Update the bracket
        match (lo_is_neg, f < 0.0) {
            (true, true) => bracket.lo = guess,   // - - +
            (true, false) => bracket.hi = guess,  // - + +
            (false, true) => bracket.hi = guess,  // + - -
            (false, false) => bracket.lo = guess, // + + -
        }
    }

    panic!(
        "Hit max iterations ({}) when trying to find a root in {:?}",
        num_iterations, bracket
    );
}
