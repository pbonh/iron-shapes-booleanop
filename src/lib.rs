extern crate num_traits;
extern crate num_rational;
extern crate itertools;

extern crate iron_shapes;

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
    Intersection,
    Difference,
    Union,
    Xor,
}