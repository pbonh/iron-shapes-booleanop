pub mod intersection;
pub mod sweep_event;
mod compare_segments;
mod connect_edges;
mod possible_intersection;

/// Type of boolean operation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Operation {
    Intersection,
    Difference,
    Union,
    Xor,
}