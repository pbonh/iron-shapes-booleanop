// Copyright (c) 2018-2021 Thomas Kramer.
// SPDX-FileCopyrightText: 2022 Thomas Kramer
// SPDX-FileCopyrightText: 2020 Fabian Keller <github.100.fkeller@spamgourmet.com> (contributions under MIT licence)
// SPDX-FileCopyrightText: 2020 Bodo Junglas <junglas@objectcode.de> (contributions under MIT licence)
//
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::binary_heap::BinaryHeap;

use std::rc::Rc;
use iron_shapes::point::Point;
use iron_shapes::edge::{Edge, EdgeIntersection};

use iron_shapes::CoordinateType;
use super::sweep_event::*;
use std::fmt::Debug;
use crate::compare_segments::compare_events_by_segments;
use std::cmp::Ordering;

/// Split a segment into two segments at the intersection point `inter` and push the new events into the queue.
fn divide_segment<T>(event: &Rc<SweepEvent<T>>,
                     inter: Point<T>,
                     queue: &mut BinaryHeap<Rc<SweepEvent<T>>>)
    where
        T: CoordinateType + Debug
{
    debug_assert!(event.is_left_event());

    if let Some(other_event) = event.get_other_event() {
        debug_assert!({
                          // Check that the point where the segment is split is really on the segment.
                          let edge = event.get_edge().unwrap();

                          // Calculate area of rhomboid spanned by edge and intersection point.
                          let a = edge.vector();
                          let b = inter - edge.start;
                          let area = b.cross_prod(a);

                          let tol = T::one();
                          area >= T::zero() - tol && area <= tol
                      }, "`inter` is not an intersection point.");

        debug_assert!({
                          // Check that the edge is well defined.
                          let edge = event.get_edge().unwrap();
                          !edge.is_degenerate()
                      }, "Degenerate edge detected.");

        debug_assert!(event.p != inter && other_event.p != inter,
                      "Intersection point must not lie on the end-points.");

        debug_assert!(event.p < inter && inter < other_event.p,
                      "Intersection point must lie on the edge but not on the end-points.");

        let r = SweepEvent::new_rc(
            event.get_edge_id(),
            inter,
            event.p,
            false,
            Rc::downgrade(&event),
            event.polygon_type,
            event.is_upper_boundary);

        let l = SweepEvent::new_rc(
            event.get_edge_id(),
            inter,
            other_event.p,
            true,
            Rc::downgrade(&other_event),
            event.polygon_type,
            event.is_upper_boundary,
        );

        debug_assert!(event.p <= r.p);
        debug_assert!(r.p == l.p);
        debug_assert!(l.p <= other_event.p);

        other_event.set_other_event(&l);
        event.set_other_event(&r);

        // Check that all edges are well defined.
        debug_assert!(!l.get_edge().unwrap().is_degenerate());
        debug_assert!(!r.get_edge().unwrap().is_degenerate());
        debug_assert!(!event.get_edge().unwrap().is_degenerate());
        debug_assert!(!other_event.get_edge().unwrap().is_degenerate());

        // Check that for each event pair there is always a left and a right event.
        debug_assert!(l.is_left_event() ^ l.get_other_event().unwrap().is_left_event());
        debug_assert!(r.is_left_event() ^ r.get_other_event().unwrap().is_left_event());
        debug_assert!(event.is_left_event() ^ event.get_other_event().unwrap().is_left_event());
        debug_assert!(other_event.is_left_event() ^ other_event.get_other_event().unwrap().is_left_event());

        queue.push(l);
        queue.push(r);

    }
}


/// Check two neighboring events for intersection and make necessary modifications to them and the queue.
///
/// `event1` must appear before `event2` in the scan line.
///
/// Returns: `true` if there was an intersection and modification to the queue.
pub fn possible_intersection<F, I>(
    // Function to compute edge intersections.
    edge_intersection_fn: I,
    // Previous event.
    event1: &Rc<SweepEvent<F>>,
    // Next event.
    event2: &Rc<SweepEvent<F>>,
    // Event queue.
    queue: &mut BinaryHeap<Rc<SweepEvent<F>>>,
) -> bool
    where
        F: CoordinateType + Debug,
        I: Fn(&Edge<F>, &Edge<F>) -> EdgeIntersection<F, F, Edge<F>>
{
    debug_assert!(event1.is_left_event());
    debug_assert!(event2.is_left_event());

    let edge1 = event1.get_edge().unwrap();
    let edge2 = event2.get_edge().unwrap();

    debug_assert!(!edge1.is_degenerate());
    debug_assert!(!edge2.is_degenerate());

    // Check that edges are oriented left to right.
    debug_assert!(edge1.start <= edge1.end);
    debug_assert!(edge2.start <= edge2.end);

    // event1 must come before event2 in the scan line.
    debug_assert_eq!(compare_events_by_segments(event1, event2), Ordering::Less,
                     "Wrong ordering.");

    match edge_intersection_fn(&edge1, &edge2) {
        EdgeIntersection::None => false,
        EdgeIntersection::Point(p) => {
            divide_segment(event1, p, queue);
            divide_segment(event2, p, queue);
            true
        }
        EdgeIntersection::EndPoint(p) => {
            debug_assert!(edge1.start == p || edge1.end == p || edge2.start == p || edge2.end == p,
                          "`p` is expected to be an end-point but is not.");

            debug_assert!({
                              p.x >= edge1.start.x && p.x >= edge2.start.x
                          },
                          "Intersection lies left of both edges.");

            if p != edge1.start && p != edge1.end {
                // `p` is not an endpoint of event1.
                divide_segment(event1, p, queue);
                true
            } else if p != edge2.start && p != edge2.end {
                // `p` is not an endpoint of event2.
                divide_segment(event2, p, queue);
                true
            } else {
                // `p` is an endpoint of both edges, therefore no edge is divided.
                false
            }
        }

        EdgeIntersection::Overlap(overlap) => {
            // The segments overlap.
            debug_assert!(edge1.start <= edge1.end);
            debug_assert!(edge2.start <= edge2.end);
            debug_assert!(overlap.start <= overlap.end);

            let left_coincide = edge1.start == edge2.start;
            let right_coincide = edge1.end == edge2.end;

            if left_coincide {
                if right_coincide {
                    // Edges are equal. No need to split any.
                    false
                } else {
                    // Left points coincide but not right.
                    debug_assert!(edge1.end != edge2.end);
                    if edge1.end < edge2.end {
                        // Split edge2 at edge1.end.
                        divide_segment(event2, edge1.end, queue)
                    } else {
                        // Split edge1 at edge2.end.
                        divide_segment(event1, edge2.end, queue)
                    }
                    true
                }
            } else {
                // Edges don't have the same left point.
                // So one edge will be split by left point of the other.
                debug_assert!(edge1.start != edge2.start);
                if edge1.start < edge2.start {
                    // Split edge1 at edge2.start.
                    divide_segment(event1, edge2.start, queue);
                } else {
                    // Split edge2 at edge1.start.
                    divide_segment(event2, edge1.start, queue);
                }
                true
            }
        }
    }
}
