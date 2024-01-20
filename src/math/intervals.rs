use std::fmt::Display;

#[derive(Debug, Clone, Copy)]
pub struct Interval {
    lo: f64,
    hi: f64,
}

impl Interval {
    pub fn new(lo: f64, hi: f64) -> Interval {
        if lo <= hi {
            Self { lo, hi }
        } else {
            Self { lo: hi, hi: lo }
        }
    }

    fn new_unchecked(lo: f64, hi: f64) -> Interval {
        debug_assert!(lo <= hi);
        Self { lo, hi }
    }

    pub fn point(val: f64) -> Self {
        Self::new_unchecked(val, val)
    }

    pub fn lo(&self) -> f64 {
        self.lo
    }

    pub fn hi(&self) -> f64 {
        self.hi
    }

    pub fn split_left(&self, mid: f64) -> Self {
        assert!(self.contains(mid));
        Self::new_unchecked(self.lo, mid)
    }

    pub fn split_right(&self, mid: f64) -> Self {
        assert!(self.contains(mid));
        Self::new_unchecked(mid, self.hi)
    }

    pub fn width(&self) -> f64 {
        debug_assert!(self.lo <= self.hi);
        self.hi - self.lo
    }

    pub fn midpoint(&self) -> f64 {
        (self.lo + self.hi) / 2.0
    }

    pub fn widen(self, value: f64) -> Self {
        self + Self::new_unchecked(-value.abs(), value.abs())
    }

    pub fn norm(&self) -> f64 {
        self.lo.abs().max(self.hi.abs())
    }

    pub fn bisect(&self) -> (Self, Self) {
        let mid = self.midpoint();
        (
            Self::new_unchecked(self.lo, mid),
            Self::new_unchecked(mid, self.hi),
        )
    }

    pub fn intersect(&self, other: &Self) -> Option<Self> {
        let new_lo = self.lo.max(other.lo);
        let new_hi = self.hi.min(other.hi);
        if new_lo <= new_hi {
            Some(Self::new_unchecked(new_lo, new_hi))
        } else {
            None
        }
    }

    pub fn is_subset_of(&self, other: &Self) -> bool {
        other.lo <= self.lo && self.hi <= other.hi
    }

    pub fn monotone_map(&self, f: impl Fn(f64) -> f64) -> Self {
        // The map could be monotone decreasing, so we can't use unchecked
        Self::new(f(self.lo), f(self.hi))
    }

    pub fn contains(&self, value: f64) -> bool {
        self.lo <= value && value <= self.hi
    }

    pub fn include(&mut self, value: f64) {
        if value < self.lo {
            self.lo = value;
        }
        if value > self.hi {
            self.hi = value;
        }
    }

    /// Returns true if the interval contains an integer of the form mk + a
    pub fn contains_integer_with_mod_constraint(&self, m: u32, a: u32) -> bool {
        // Round the bottom up to the nearest integer
        let lo_int = self.lo.ceil() as u32;

        // Find the next integer higher than this that could fit the criteria
        let b = lo_int % m;
        let next_valid_int = if b <= a {
            lo_int + (a - b)
        } else {
            lo_int + (m + a - b)
        };

        self.contains(next_valid_int as f64)
    }
}

impl std::ops::Add<Interval> for Interval {
    type Output = Interval;

    fn add(self, rhs: Self) -> Self {
        Self::new_unchecked(self.lo + rhs.lo, self.hi + rhs.hi)
    }
}

impl std::ops::Neg for Interval {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            lo: self.hi,
            hi: self.lo,
        }
    }
}

impl std::ops::Sub for Interval {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self::new(self.lo - other.hi, self.hi - other.lo)
    }
}

impl std::ops::Mul for Interval {
    type Output = Interval;

    fn mul(self, other: Self) -> Self::Output {
        let mut output = Self::new(self.lo * other.lo, self.hi * other.hi);
        output.include(self.lo * other.hi);
        output.include(other.lo * self.hi);
        output
    }
}

// Division is special, we want to avoid division by zero, so we only allow
// division by scalars
// TODO: do we want to allow it anyways and panic if there's a zero?
impl std::ops::Div<f64> for Interval {
    type Output = Interval;

    fn div(self, rhs: f64) -> Self::Output {
        Self::new(self.lo / rhs, self.hi / rhs)
    }
}

impl Display for Interval {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}, {}]", self.lo, self.hi)
    }
}

// Extend arithmetic operations to scalar quantities
macro_rules! extend_to_scalar {
    ($($trait_path:ident)::*, $fn_name: ident) => {
        impl $($trait_path)::*<f64> for Interval {
            type Output = Interval;

            fn $fn_name(self, rhs: f64) -> Self::Output {
                self.$fn_name(Interval::point(rhs))
            }
        }

        impl $($trait_path)::*<Interval> for f64 {
            type Output = Interval;

            fn $fn_name(self, rhs: Interval) -> Self::Output {
                Interval::point(self).$fn_name(rhs)
            }
        }
    };
}

extend_to_scalar!(std::ops::Add, add);
extend_to_scalar!(std::ops::Sub, sub);
extend_to_scalar!(std::ops::Mul, mul);
