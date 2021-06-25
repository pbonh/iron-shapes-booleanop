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
use std::cell::RefCell;
use std::rc::{Rc, Weak};
use iron_shapes::point::Point;
use iron_shapes::edge::{Edge, Side};
use iron_shapes::CoordinateType;

use std::cmp::Ordering;
use crate::PolygonSemantics;

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum PolygonType {
    Subject,
    Clipping,
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum EdgeType {
    Normal,
    /// No contribution to the result.
    NonContributing,
    /// The edge represents two collinear edges of different polygons. Both have
    /// the 'inside' of the polygon on the *same* side.
    SameTransition,
    /// The edge represents two collinear edges of different polygons. They have
    /// the 'inside' of the polygon on *different* sides.
    DifferentTransition,
}

/// Mutable data of a sweep event.
#[derive(Debug, Clone)]
struct MutablePart<T: CoordinateType> {
    /// Reference to the event associated with the other endpoint of the edge.
    other_event: Weak<SweepEvent<T>>,
    /// Edge below this event. This is used to find polygon-hole relationships.
    prev: Weak<SweepEvent<T>>,
    /// Is p the left endpoint of the edge (p, other.p)?
    is_left_event: bool,
    /// Counts the parity of all edges of the same polygon type (clipping/subject) below this edge.
    /// Lower boundaries are counted as 1, upper boundaries are counted as -1.
    edge_count: i32,
    /// Counts the parity of all edges of the other polygon type (clipping/subject) below this edge.
    /// Lower boundaries are counted as 1, upper boundaries are counted as -1.
    other_edge_count: i32,
    ///
    edge_type: EdgeType,
    /// Index of this event in an array.
    /// In a later step of the algorithm this will hold the index of the other event.
    pos: usize,
    /// Unique ID of the edge. Used to break ties and guarantee ordering for overlapping edges.
    edge_id: usize,
}

#[derive(Debug, Clone)]
pub struct SweepEvent<T: CoordinateType> {
    /// Mutable part of the sweep event. Borrow checking happens at runtime.
    mutable: RefCell<MutablePart<T>>,
    /// Point associated with the event. Starting point or end point of the edge.
    pub p: Point<T>,
    /// Type of polygon: either SUBJECT or CLIPPING.
    pub polygon_type: PolygonType,
    /// Is this edge an upper boundary of the input polygon?
    pub is_upper_boundary: bool,
}


impl<T: CoordinateType> SweepEvent<T> {
    /// Create a new sweep event wrapped into a `Rc`.
    pub fn new_rc(
        edge_id: usize,
        point: Point<T>,
        is_left_event: bool,
        other_event: Weak<SweepEvent<T>>,
        polygon_type: PolygonType,
        edge_type: EdgeType,
        is_upper_boundary: bool,
    ) -> Rc<SweepEvent<T>> {
        Rc::new(SweepEvent {
            mutable: RefCell::new(MutablePart {
                other_event,
                prev: Weak::new(),
                is_left_event,
                edge_count: 0,
                edge_type,
                other_edge_count: 0,
                pos: 0,
                edge_id,
            }),
            p: point,
            polygon_type,
            is_upper_boundary,
        })
    }

    pub fn is_left_event(&self) -> bool {
        self.mutable.borrow().is_left_event
    }

    // pub fn set_left_event(&self, left: bool) {
    //     self.mutable.borrow_mut().is_left_event = left
    // }

    /// Get the event that represents the other end point of this segment.
    pub fn get_other_event(&self) -> Option<Rc<SweepEvent<T>>> {
        self.mutable.borrow().other_event.upgrade()
    }

    /// Set the event that represents the other end point of this segment.
    pub fn set_other_event(&self, other_event: &Rc<SweepEvent<T>>) {
        debug_assert_ne!(self.is_left_event(), other_event.is_left_event());
        self.mutable.borrow_mut().other_event = Rc::downgrade(other_event);
    }

    /// Get the segment associated with the event.
    /// The `start` point will be the point of the first event,
    /// `end` will be the point of the second event.
    pub fn get_edge(&self) -> Option<Edge<T>> {
        self.get_other_event().map(|other| {
            let p1 = self.p;
            let p2 = other.p;

            debug_assert!(self.is_left_event() ^ other.is_left_event());

            Edge::new(p1, p2)
        })
    }

    /// Get the segment associated with the event.
    /// The endpoints are sorted such that the left endpoint is `start`.
    pub fn get_edge_left_right(&self) -> Option<Edge<T>> {
        self.get_other_event().map(|other| {
            let p1 = self.p;
            let p2 = other.p;

            debug_assert!(self.is_left_event() ^ other.is_left_event());

            if self.is_left_event() {
                Edge::new(p1, p2)
            } else {
                Edge::new(p2, p1)
            }
        })
    }

    pub fn get_edge_type(&self) -> EdgeType {
        self.mutable.borrow().edge_type
    }

    pub fn set_edge_type(&self, edge_type: EdgeType) {
        self.mutable.borrow_mut().edge_type = edge_type
    }

    pub fn edge_count(&self) -> i32 {
        self.mutable.borrow().edge_count
    }


    /// Is this event outside of the other polygon?
    pub fn other_edge_count(&self) -> i32 {
        self.mutable.borrow().other_edge_count
    }

    pub fn set_edge_count(&self, edge_count: i32, other_edge_count: i32) {
        let mut mutable = self.mutable.borrow_mut();

        mutable.edge_count = edge_count;
        mutable.other_edge_count = other_edge_count;
    }

    /// Check if this event lies outside the other polygon.
    pub fn is_outside_other(&self, polygon_semantics: PolygonSemantics) -> bool {
        let edge_count = self.other_edge_count();
        match polygon_semantics {
            PolygonSemantics::Union => edge_count == 0,
            PolygonSemantics::XOR => edge_count % 2 == 0
        }
    }

    /// Check if the edge that belongs to this event is an upper boundary of this polygon.
    pub fn is_upper_boundary(&self, polygon_semantics: PolygonSemantics) -> bool {
        let edge_count = self.edge_count();
        match polygon_semantics {
            PolygonSemantics::Union => edge_count == 0,
            PolygonSemantics::XOR => edge_count % 2 == 0
        }
    }

    pub fn get_pos(&self) -> usize {
        self.mutable.borrow().pos
    }

    pub fn set_pos(&self, pos: usize) {
        self.mutable.borrow_mut().pos = pos
    }


    pub fn get_edge_id(&self) -> usize {
        self.mutable.borrow().edge_id
    }

    pub fn set_edge_id(&self, edge_id: usize) {
        self.mutable.borrow_mut().edge_id = edge_id
    }


    /// Check if the corresponding line segment is vertical.
    pub fn is_vertical(&self) -> bool {
        match self.get_other_event() {
            Some(ref other_event) => self.p.x == other_event.p.x,
            None => false,
        }
    }

    pub fn get_prev(&self) -> Weak<SweepEvent<T>> {
        self.mutable.borrow().prev.clone()
    }

    pub fn set_prev(&self, prev: Weak<SweepEvent<T>>) {
        self.mutable.borrow_mut().prev = prev;
    }

    /// Lower boundaries have a weight `1`, upper boundaries have a weight `-1`.
    pub fn edge_weight(&self) -> i32 {
        if self.is_upper_boundary {
            -1
        } else {
            1
        }
    }
}

impl<'a, T> PartialEq for SweepEvent<T>
    where T: CoordinateType {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}


impl<T> Eq for SweepEvent<T>
    where T: CoordinateType {}


impl<T> PartialOrd for SweepEvent<T>
    where T: CoordinateType {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<'a, T> Ord for SweepEvent<T>
    where T: CoordinateType {
    fn cmp(&self, other: &Self) -> Ordering {
        // Note that the order is reversed because this is used in a max-heap.
        let point_ordering = other.p.partial_cmp(&self.p).unwrap();

        match point_ordering {
            Ordering::Equal => {
                debug_assert!(self.p == other.p);

                // Points are equal. Break the tie!
                // Prefer right events over left events (This is needed to efficiently connect the edges later on).
                match other.is_left_event().cmp(&self.is_left_event()) {
                    Ordering::Equal => {

                        // Break the tie by the edges.
                        let edge1 = self.get_edge_left_right().unwrap();
                        let edge2 = other.get_edge_left_right().unwrap();

                        debug_assert!(edge1.start == edge2.start || edge1.end == edge2.end);

                        let reference_point = if other.is_left_event() {
                            edge2.end
                        } else {
                            edge2.start
                        };

                        match edge1.side_of(reference_point) {
                            // Prefer the lower edge (which has the other end point on the left side).
                            Side::Left => {
                                debug_assert!(!edge1.is_collinear(&edge2));
                                Ordering::Greater
                            }
                            Side::Right => {
                                debug_assert!(!edge1.is_collinear(&edge2));
                                Ordering::Less
                            }
                            Side::Center => {
                                debug_assert!(edge1.is_collinear(&edge2));
                                // // Break the tie by the polygon type and then the edge_id.
                                // other.get_edge_id().cmp(&self.get_edge_id())

                                // Take lower boundaries before upper boundaries, break the tie by the edge_id.
                                other.is_upper_boundary.cmp(&self.is_upper_boundary)
                                    .then_with(|| other.get_edge_id().cmp(&self.get_edge_id()))

                            }
                        }
                    }
                    less_or_greater => less_or_greater
                }
            }
            less_or_greater => less_or_greater
        }
    }
}


#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_prefer_right_events_over_left_events() {
        let left = SweepEvent::new_rc(
            0,
            (0, 0).into(),
            true, // left
            Weak::new(),
            PolygonType::Subject,
            EdgeType::Normal,
            false,
        );
        let right = SweepEvent::new_rc(
            0,
            (0, 0).into(),
            false, // right
            Weak::new(),
            PolygonType::Subject,
            EdgeType::Normal,
            false,
        );

        assert!(right > left);
    }

    #[test]
    fn test_on_equal_x_sort_y() {
        let lower = SweepEvent::new_rc(
            0,
            (0, 0).into(),
            true, // left
            Weak::new(),
            PolygonType::Subject,
            EdgeType::Normal,
            false,
        );
        let upper = SweepEvent::new_rc(
            0,
            (0, 1).into(),
            false, // right
            Weak::new(),
            PolygonType::Subject,
            EdgeType::Normal,
            false,
        );

        assert!(lower > upper);
    }
}
