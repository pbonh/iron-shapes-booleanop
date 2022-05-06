/*
 * Copyright (c) 2018-2021 Thomas Kramer.
 *
 * This file is part of LibrEDA 
 * (see https://codeberg.org/libreda).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#![deny(missing_docs)]

//! Library for boolean operations on polygons.
//!
//! # Example
//!
//! ```
//! use iron_shapes::prelude::*;
//! use iron_shapes_booleanop::BooleanOp;
//!
//! // Create two polygons.
//! let p1 = Polygon::from(vec![(0., 0.), (2., 0.), (2., 1.), (0., 1.)]);
//! let p2 = p1.translate((1., 0.).into()); // Shift p1 by (1, 0).
//!
//! // Compute the boolean intersection of the two squares.
//! let intersection = p1.intersection(&p2);
//! assert_eq!(intersection.polygons.len(), 1);
//! assert_eq!(intersection.polygons[0], Polygon::from(vec![(1., 0.), (2., 0.), (2., 1.), (1., 1.)]));
//!
//! // Compute the boolean exclusive-or of the two squares.
//! // This results in two unconnected polygons. This demonstrates why boolean operations return always
//! // a `MultiPolygon`.
//! let intersection = p1.xor(&p2);
//! assert_eq!(intersection.polygons.len(), 2);
//! ```

mod intersection;
mod sweep_event;
mod compare_segments;
mod connect_edges;
mod possible_intersection;
mod naive_scanline;

use num_rational::{Rational32, Rational64};

// API exports.
pub use intersection::boolean_op;
pub use intersection::{edge_intersection_float, edge_intersection_integer, edge_intersection_rational};

use iron_shapes::prelude::{CoordinateType, Polygon, MultiPolygon};

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

/// Define the 'inside' of a polygon. Significant for self-overlapping polygons.
///
/// * `Union`: A point `p` is inside the polygon if the winding number is larger than `0`.
/// This means that if a polygon overlaps with itself or multiple polygons overlap, the overlapping
/// area is always 'inside'.
/// * `XOR`: A point `p` is inside the polygon if the winding number modulo 2 is larger than `0`.
/// This means that if an odd number of polygons overlap, the overlapping area is 'inside' the polygon.
/// In case of an even number of overlaps, the overlapping area is 'outside'.
///
/// This plays an important role for self-overlapping polygons and self-overlapping multi-polygons.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PolygonSemantics {
    /// A point `p` is inside the polygon if the winding number is larger than `0`.
    Union,
    /// A point `p` is inside the polygon if the winding number modulo 2 is larger than `0`.
    XOR,
}

/// Trait for geometric primitives that support boolean operations.
pub trait BooleanOp<T: CoordinateType> {
    /// Compute the boolean operation of `self` and `other`.
    ///
    /// # Parameters
    ///
    /// * `operation`: The type of boolean operation to be computed (intersection, union, difference, XOR).
    /// * `other`: The other operand. A geometric object of the same type as `self`.
    /// * `polygon_semantics`: Define the 'inside' of a polygon. In case of doubt `Union`-semantics
    /// could be a good choice.
    fn boolean_op(&self, operation: Operation, other: &Self, polygon_semantics: PolygonSemantics) -> MultiPolygon<T>;

    /// Compute the boolean intersection `self & other`.
    ///
    /// Union semantics are used for self-overlapping polygons.
    fn intersection(&self, other: &Self) -> MultiPolygon<T> {
        self.boolean_op(Operation::Intersection, other, PolygonSemantics::Union)
    }

    /// Compute the boolean difference `self - other`.
    ///
    /// Union semantics are used for self-overlapping polygons.
    fn difference(&self, other: &Self) -> MultiPolygon<T> {
        self.boolean_op(Operation::Difference, other, PolygonSemantics::Union)
    }

    /// Compute the boolean union `self | other`.
    ///
    /// Union semantics are used for self-overlapping polygons.
    fn union(&self, other: &Self) -> MultiPolygon<T> {
        self.boolean_op(Operation::Union, other, PolygonSemantics::Union)
    }

    /// Compute the boolean exclusive OR `self ^ other`.
    ///
    /// Union semantics are used for self-overlapping polygons.
    fn xor(&self, other: &Self) -> MultiPolygon<T> {
        self.boolean_op(Operation::Xor, other, PolygonSemantics::Union)
    }
}

/// Implement the `BooleanOp` trait for `MultiPolygon<...>`.
macro_rules! impl_booleanop_multipolygon {
 ($coord:ty, $edge_intersection:ident) => {
     impl BooleanOp<$coord> for MultiPolygon<$coord> {
        fn boolean_op(&self, operation: Operation, other: &Self, polygon_semantics: PolygonSemantics)
         -> MultiPolygon<$coord> {
            let subject = self.polygons.iter();
            let clipping = other.polygons.iter();
            boolean_op(
                &$edge_intersection,
                subject,
                clipping,
                operation,
                polygon_semantics
            )
        }
    }
 }
}


impl_booleanop_multipolygon!(f32, edge_intersection_float);
impl_booleanop_multipolygon!(f64, edge_intersection_float);
impl_booleanop_multipolygon!(i32, edge_intersection_integer);
impl_booleanop_multipolygon!(i64, edge_intersection_integer);

impl_booleanop_multipolygon!(Rational32, edge_intersection_rational);
impl_booleanop_multipolygon!(Rational64, edge_intersection_rational);

/// Implement the `BooleanOp` trait for `Polygon<...>`.
macro_rules! impl_booleanop_polygon {
 ($coord:ty, $edge_intersection:ident) => {
     impl BooleanOp<$coord> for Polygon<$coord> {
        fn boolean_op(&self, operation: Operation, other: &Self, polygon_semantics: PolygonSemantics)
        -> MultiPolygon<$coord> {
            let subject = std::iter::once(self);
            let clipping = std::iter::once(other);
            boolean_op(
                &$edge_intersection,
                subject,
                clipping,
                operation,
                polygon_semantics
            )
        }
    }
 }
}

impl_booleanop_polygon!(f32, edge_intersection_float);
impl_booleanop_polygon!(f64, edge_intersection_float);
impl_booleanop_polygon!(i32, edge_intersection_integer);
impl_booleanop_polygon!(i64, edge_intersection_integer);

impl_booleanop_polygon!(Rational32, edge_intersection_rational);
impl_booleanop_polygon!(Rational64, edge_intersection_rational);
