//! A module for computing simple Keplerian orbits.
//!
//! The two important structs in this module are:
//! - [OrbitBase], which represents an orbit of a primary body around a
//!   secondary. It is possible to customize the primary and secondary bodies
//!   via the type parameters.
//!   - There are several useful typedefs, including [Orbit], [BareOrbit], and
//!     [TimedOrbit]
//! - [CartesianState], which represents a position and velocity

mod orbit;
mod orbit_methods;
mod state;

// Newton's gravitational constant, in N m^2 / kg^2
pub const NEWTON_G: f64 = 6.6743015e-11;

pub use orbit::{BareOrbit, Orbit, OrbitBase, PhysicalOrbit, TimedOrbit};
pub use state::CartesianState;

/// A point mass with no other physical properties.
///
/// Useful for satisfying a [HasMass] trait bound.
#[derive(Debug, Clone, Copy)]
pub struct PointMass(f64);

/// A trait indicating this object can be used in physical computations that
/// require a massive body.
pub trait HasMass {
    /// The standard gravitational parameter of this object
    fn mu(&self) -> f64;

    /// The mass of this object
    fn mass(&self) -> f64 {
        self.mu() / NEWTON_G
    }

    fn to_point_mass(&self) -> PointMass {
        PointMass::with_mu(self.mu())
    }
}

impl PointMass {
    /// Constructs a new mass
    pub fn with_mu(mu: f64) -> Self {
        Self(mu)
    }
}

impl HasMass for PointMass {
    fn mu(&self) -> f64 {
        self.0
    }
}

impl<T> HasMass for &T
where
    T: HasMass,
{
    fn mu(&self) -> f64 {
        (*self).mu()
    }
}
