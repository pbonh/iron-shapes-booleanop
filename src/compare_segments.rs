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

//! Compute the ordering among edges. This is used to keep sweep events in the right ordering
//! in the scan line.

use std::rc::Rc;
use iron_shapes::edge::{Edge, Side};
use std::cmp::Ordering;
use iron_shapes::CoordinateType;
use super::sweep_event::*;

use std::fmt::Debug;

/// Compare two edges.
/// Returns `Less` if the starting point of `second` is below `first` and `Greater` if it is above.
/// If the starting point of `second` lies on the edge `first` then the end point is used to break the tie.
fn compare_edges<T: CoordinateType>(first: &Edge<T>, second: &Edge<T>) -> Ordering {
    debug_assert!(first.start != second.start);

    debug_assert!(first.start.x <= first.end.x);
    debug_assert!(second.start.x <= second.end.x);

    // Check if it makes sense to compare those two edges.
    debug_assert!(
        !(first.start.x > second.end.x || second.start.x > first.end.x),
        "The edges must overlap in their x-coordinates to be in the scan-line at the same point in time."
    );

    match first.side_of(second.start) {
        Side::Left => Ordering::Less,
        Side::Right => Ordering::Greater,
        Side::Center => {
            match first.side_of(second.end) {
                Side::Left => Ordering::Less,
                Side::Right => Ordering::Greater,
                Side::Center => Ordering::Equal
            }
        }
    }
}

/// Compute the ordering of two sweep events. This is used to keep the sweep events
/// sorted in the scan line.
///
/// The two sweep events MUST be left events. Otherwise the function panics in debug mode.
///
/// Also the two edges defined by the sweep events must overlap when projected on the x-axis.
/// (Otherwise they would not both intersect with the scan line.)
///
/// When used correctly the sweep events are sorted by the ascending y-coordinate of their
/// intersection point with the scan line.
pub fn compare_events_by_segments<T>(le1: &Rc<SweepEvent<T>>,
                                     le2: &Rc<SweepEvent<T>>) -> Ordering
    where
        T: CoordinateType + Debug,
{
//    let tolerance = 1e-12;

    debug_assert!(le1.is_left_event());
    debug_assert!(le2.is_left_event());

    // Check if pointers point to same event.
    if Rc::ptr_eq(&le1, &le2) {
        return Ordering::Equal;
    }

    let edge1 = le1.get_edge().unwrap();
    let edge2 = le2.get_edge().unwrap();

    debug_assert!(edge1.start.x <= edge1.end.x);
    debug_assert!(edge2.start.x <= edge2.end.x);

    debug_assert!(!edge1.is_degenerate());
    debug_assert!(!edge2.is_degenerate());

    // Check if it makes sense to compare those two events.
    debug_assert!(
        !(edge1.start.x > edge2.end.x || edge2.start.x > edge1.end.x),
        "The edges must overlap in their x-coordinates to be in the scan-line at the same point in time."
    );

//       TODO: if edge1.is_collinear_approx(edge2, tolerance) {
    if edge1.is_collinear(&edge2) {
        // Segments are collinear, thus they intersect the scan line in the same point.
        // Break the tie by the edge_id.

        le1.get_edge_id().cmp(&le2.get_edge_id())
    } else {
        // Segments are not collinear.

        if edge1.start == edge2.start {
            // If they share their left endpoint use the right endpoint to sort

            match edge1.side_of(edge2.end) {
                Side::Left => Ordering::Less,
                Side::Right => Ordering::Greater,
                Side::Center => {
                    // This is collinear! Should never happen.
                    // TODO: Remove collinearity check from above (if) and put code here instead.
                    panic!("This edges are collinear! Should not happen.");
                }
            }
        } else if edge1.start.x == edge2.start.x {
            // Different left endpoint: use the left endpoint to sort
            debug_assert!(edge1.start.y != edge2.start.y,
                          "Case of equality must be handled before.");

            if edge1.start.y < edge2.start.y {
                Ordering::Less
            } else {
                Ordering::Greater
            }
        } else {
            // Left end points differ in both x and y coordinates.

            debug_assert!(edge1.start.x != edge2.start.x, "x-coordinate should not be equal here.");

            let edge_cmp_result = if edge1.start.x < edge2.start.x {
                compare_edges(&edge1, &edge2)
            } else {
                compare_edges(&edge2, &edge1).reverse()
            };
            debug_assert!(edge_cmp_result != Ordering::Equal);
            edge_cmp_result
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use std::rc::Weak;

    fn make_event_pair(
        event_id: usize,
        left: (f64, f64),
        right: (f64, f64),
        polygon_type: PolygonType,
    ) -> (Rc<SweepEvent<f64>>, Rc<SweepEvent<f64>>) {
        let other = SweepEvent::new_rc(
            event_id,
            right.into(),
            false,
            Weak::new(),
            polygon_type,
            EdgeType::Normal,
        );
        let event = SweepEvent::new_rc(
            event_id,
            left.into(),
            true,
            Rc::downgrade(&other),
            polygon_type,
            EdgeType::Normal,
        );
        other.set_other_event(&event);

        (event, other)
    }

    fn simple_event_pair(
        left: (f64, f64),
        right: (f64, f64)) -> (Rc<SweepEvent<f64>>, Rc<SweepEvent<f64>>) {
        make_event_pair(0, left, right, PolygonType::Clipping)
    }

    #[test]
    fn not_collinear_shared_left_right_first() {
        let (se1, _other1) =
            simple_event_pair((0.0, 0.0), (1.0, 1.0));
        let (se2, _other2) =
            simple_event_pair((0.0, 0.0), (2.0, 2.1));

        assert_eq!(compare_events_by_segments(&se1, &se2), Ordering::Less);
        assert_eq!(compare_events_by_segments(&se2, &se1), Ordering::Greater);
    }

    #[test]
    fn not_collinear_different_left_right_sort_y() {
        let (se1, _other1) =
            simple_event_pair((0.0, 1.0), (1.0, 1.0));
        let (se2, _other2) =
            simple_event_pair((0.0, 2.0), (2.0, 3.0));

        assert_eq!(compare_events_by_segments(&se1, &se2), Ordering::Less);
        assert_eq!(compare_events_by_segments(&se2, &se1), Ordering::Greater);
    }


    #[test]
    fn not_collinear_order_in_sweep_line() {
        let (se1, _other1) = simple_event_pair((0.0, 1.0), (2.0, 1.0));
        let (se2, _other2) = simple_event_pair((-1.0, 0.0), (2.0, 3.0));

        let (se3, _other3) = simple_event_pair((0.0, 1.0), (3.0, 4.0));
        let (se4, _other4) = simple_event_pair((-1.0, 0.0), (3.0, 1.0));

        assert_eq!(se1.cmp(&se2), Ordering::Less);
        assert_eq!(compare_events_by_segments(&se1, &se2), Ordering::Less);
        assert_eq!(compare_events_by_segments(&se2, &se1), Ordering::Greater);

        assert_eq!(se3.cmp(&se4), Ordering::Less);
        assert_eq!(compare_events_by_segments(&se3, &se4), Ordering::Greater);
        assert_eq!(compare_events_by_segments(&se4, &se3), Ordering::Less);
    }

    #[test]
    fn test_vertical_intersection_in_lower_endpoint() {
        // "If a non-vertical edge intersects the sweep-line at the lower
        // endpoint of a vertical edge, then the vertical edge is placed in S after the
        // non-vertical edge"

        // vertical
        let (se1, _other1) = simple_event_pair((0.0, 0.0), (0.0, 1.0));
        // non-vertical
        let (se2, _other2) = simple_event_pair((0.0, 0.0), (1.0, 1.0));

        assert_eq!(compare_events_by_segments(&se1, &se2), Ordering::Greater);
        assert_eq!(compare_events_by_segments(&se2, &se1), Ordering::Less);
    }

    #[test]
    fn test_both_vertical_same_start() {
        // Two verticals with same start point should be ordered by event_id.


        let (se1, _other1) =
            make_event_pair(0, (0.0, 0.0), (0.0, 1.0), PolygonType::Clipping);
        let (se2, _other2) =
            make_event_pair(1, (0.0, 0.0), (0.0, 2.0), PolygonType::Clipping);

        // Break tie by event_id.
        assert_eq!(compare_events_by_segments(&se1, &se2), Ordering::Less);
        assert_eq!(compare_events_by_segments(&se2, &se1), Ordering::Greater);

        // Swap event ids.
        let (se1, _other1) =
            make_event_pair(1, (0.0, 0.0), (0.0, 1.0), PolygonType::Clipping);
        let (se2, _other2) =
            make_event_pair(0, (0.0, 0.0), (0.0, 2.0), PolygonType::Clipping);

        // Break tie by event_id.
        assert_eq!(compare_events_by_segments(&se1, &se2), Ordering::Greater);
        assert_eq!(compare_events_by_segments(&se2, &se1), Ordering::Less);
    }
}