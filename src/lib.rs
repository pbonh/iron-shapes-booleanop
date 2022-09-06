// Copyright (c) 2018-2021 Thomas Kramer.
// SPDX-FileCopyrightText: 2022 Thomas Kramer
//
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Original licence until 2020: MIT
// SPDX-FileCopyrightText: 2020 Fabian Keller <github.100.fkeller@spamgourmet.com> (contributions under MIT licence)
// SPDX-FileCopyrightText: 2020 Bodo Junglas <junglas@objectcode.de> (contributions under MIT licence)
// SPDX-FileCopyrightText: 2020 Boyd Johnson (all contributions overwritten by now)
// SPDX-FileCopyrightText: 2020 Robert Sayre (all contributions overwritten by now)

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
//!
//! # References
//! * This work is originally loosely based: F. Martinez, A. Rueda, F. Feito, "A new algorithm for computing Boolean operations on polygons", 2013, doi:10.1016/j.advengsoft.2013.04.004
//!
//! The algorithm implemented here deviates from the reference paper. Most notably, the ordering of lines
//! 6-9 in Listing 2 is done differently to properly handle vertical overlapping edges.
//!
//! * More systematic approach: [PDF](https://www.boost.org/doc/libs/1_44_0/libs/polygon/doc/GTL_boostcon2009.pdf), [archived](https://web.archive.org/save/https://www.boost.org/doc/libs/1_44_0/libs/polygon/doc/GTL_boostcon2009.pdf)

extern crate iron_shapes;

mod connect_edges;
mod sweep_line;
mod booleanop;
pub mod connectivity_extraction;
mod init_events;

use num_rational::{Rational32, Rational64};

// API exports.
pub use booleanop::{boolean_op, edges_boolean_op};
pub use sweep_line::intersection::{edge_intersection_float, edge_intersection_integer, edge_intersection_rational};

use iron_shapes::prelude::{CoordinateType, Polygon, MultiPolygon};

// /// Abstraction of a line segment or 'edge'.
// pub trait Segment {
//     /// Numeric type used for coordinates.
//     type Coord: CoordinateType;
//
//     /// Get the starting point of the segment.
//     fn start(&self) -> Point<Self::Coord>;
//
//     /// Get the end point of the segment.
//     fn end(&self) -> Point<Self::Coord>;
//
//     /// If start and end point are equal.
//     fn is_degenerate(&self) -> bool {
//         self.start() == self.end()
//     }
// }

/// Type of boolean operation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Operation {
    /// Compute the boolean AND.
    Intersection,
    /// Compute the boolean difference `A & (not B)`.
    Difference,
    /// Compute the boolean OR.
    Union,
    /// Compute the boolean XOR (symmetric difference).
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
            let subject = self.all_edges_iter();
            let clipping = other.all_edges_iter();
            edges_boolean_op(
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
            let subject = self.all_edges_iter();
            let clipping = other.all_edges_iter();
            edges_boolean_op(
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
