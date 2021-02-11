
#![deny(missing_docs)]

//! Library for boolean operations on polygons.

mod intersection;
mod sweep_event;
mod compare_segments;
mod connect_edges;
mod possible_intersection;

// API exports.
pub use intersection::{boolean_op, boolean_multi_op};
pub use intersection::{edge_intersection_float, edge_intersection_integer, edge_intersection_rational};

/// Type of boolean operation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Operation {
    /// Compute the boolean AND.
    Intersection,
    /// Compute the boolean difference `A & (not B)`.
    Difference,
    /// Compute the boolean OR.
    Union,
    /// Compute the boolean XOR.
    Xor,
}
