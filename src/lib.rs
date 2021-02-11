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
