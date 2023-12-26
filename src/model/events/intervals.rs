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

    pub fn lo(&self) -> f64 {
        self.lo
    }

    pub fn hi(&self) -> f64 {
        self.hi
    }

    pub fn width(&self) -> f64 {
        debug_assert!(self.lo <= self.hi);
        self.hi - self.lo
    }

    pub fn bisect(&self) -> (Self, Self) {
        let mid = (self.lo + self.hi) / 2.0;
        (Self::new(self.lo, mid), Self::new(mid, self.hi))
    }

    pub fn monotone_map(&self, f: impl Fn(f64) -> f64) -> Self {
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

    pub fn separated_by(&self, other: &Self, threshold: f64) -> bool {
        debug_assert!(threshold >= 0.0);
        // They're sufficiently far away if either:
        // - self.hi << other.lo
        // - other.hi << self.lo
        (self.hi + threshold < other.lo) || (other.hi + threshold < self.lo)
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

impl std::ops::Add for Interval {
    type Output = Interval;

    fn add(self, other: Self) -> Self {
        Self::new(self.lo + other.lo, self.hi + other.hi)
    }
}

impl Display for Interval {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}, {}]", self.lo, self.hi)
    }
}

pub struct BoundingBox(pub [Interval; 3]);

impl BoundingBox {
    pub fn separated_by(&self, other: &Self, threshold: f64) -> bool {
        debug_assert!(threshold >= 0.0);
        self.0
            .iter()
            .zip(&other.0)
            .any(|(i1, i2)| i1.separated_by(i2, threshold))
    }
}
