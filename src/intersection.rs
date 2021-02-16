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
use std::collections::binary_heap::BinaryHeap;

use std::rc::{Rc, Weak};
use iron_shapes::polygon::{Polygon, SimplePolygon};
use iron_shapes::edge::{Edge, EdgeIntersection};
use num_rational::Ratio;
use iron_shapes::multi_polygon::MultiPolygon;

use iron_shapes::CoordinateType;
use libreda_splay::SplaySet;
use super::sweep_event::*;
use super::Operation;
use num_traits::{Float, PrimInt};
use super::compare_segments::compare_events_by_segments;
use super::connect_edges::connect_edges;
use super::possible_intersection::possible_intersection;
use std::fmt::Debug;
use std::ops::RangeFrom;
use std::cmp::Ordering;
use itertools::Itertools;
use std::usize;
use num_integer::Integer;
use crate::PolygonSemantics;

/// Insert the edges of the polygons into the event queue.
fn fill_queue<'a, T, S, C>(subject: S,
                           clipping: C) -> BinaryHeap<Rc<SweepEvent<T>>>
    where S: Iterator<Item=&'a Polygon<T>>,
          C: Iterator<Item=&'a Polygon<T>>,
          T: CoordinateType + 'a {
    let mut event_queue = BinaryHeap::new();

    /// Add edges of a polygon to the event queue.
    fn process_polygon<T: CoordinateType>(event_queue: &mut BinaryHeap<Rc<SweepEvent<T>>>,
                                          poly: &SimplePolygon<T>,
                                          polygon_type: PolygonType) {
        for edge in poly.edges() {
            // Skip degenerate edges.
            if !edge.is_degenerate() {
//                let edge_id = edge_ids.next().unwrap();
                let edge_id = usize::MAX;
                let event_a_is_left = edge.start < edge.end;

                // Upper boundary edges are directed from right to left.
                let is_upper_boundary = edge.end < edge.start;

                let event_a = SweepEvent::new_rc(
                    edge_id,
                    edge.start,
                    event_a_is_left,
                    Weak::new(),
                    polygon_type,
                    EdgeType::Normal,
                    is_upper_boundary,
                );
                let event_b = SweepEvent::new_rc(
                    edge_id,
                    edge.end,
                    !event_a_is_left,
                    Rc::downgrade(&event_a),
                    polygon_type,
                    EdgeType::Normal,
                    is_upper_boundary,
                );


                event_a.set_other_event(&event_b);

                event_queue.push(event_a);
                event_queue.push(event_b);
            }
        }
    }
    ;

    // Subject polygons.
    for p in subject {
        process_polygon(&mut event_queue, &p.exterior, PolygonType::Subject);
        for i in &p.interiors {
            // Holes
            process_polygon(&mut event_queue, i, PolygonType::Subject);
        }
    }

    // Clipping polygons.
    for p in clipping {
        process_polygon(&mut event_queue, &p.exterior, PolygonType::Clipping);
        for i in &p.interiors {
            // Holes
            process_polygon(&mut event_queue, i, PolygonType::Clipping);
        }
    }

    event_queue
}


/// Compute flags and fields for a segment based on its predecessor in the scan line (if there is one).
pub fn compute_fields<T>(event: &Rc<SweepEvent<T>>,
                         maybe_prev: Option<&Rc<SweepEvent<T>>>)
    where
        T: CoordinateType,
{
    if let Some(prev) = maybe_prev {
        let is_same_type = event.polygon_type == prev.polygon_type;

        // let upper_boundary = match (event.is_vertical(), is_same_type) {
        //     (false, false) => !prev.other_edge_count(),
        //     (false, true) => !prev.edge_count(),
        //
        //     (true, false) => prev.other_edge_count(),
        //     (true, true) => prev.edge_count(),
        // };
        //
        // let is_outside_other = match is_same_type {
        //     false => prev.edge_count(),
        //     true => prev.other_edge_count(),
        // };


        let edge_count = if is_same_type {
            prev.edge_count()
        } else {
            prev.other_edge_count()
        };

        // Update the edge count of the current type.
        let edge_count = if event.is_vertical() {
            edge_count
        } else {
            edge_count + event.edge_weight()
        };

        let other_edge_count = if is_same_type {
            prev.other_edge_count()
        } else {
            prev.edge_count()
        };

        event.set_edge_count(edge_count, other_edge_count);

        // Remember the previous segment for contour-hole attribution.
        // Note: What matters in the end is the previous segment that also contributes to the result
        // Hence there is post-processing necessary.
        event.set_prev(Rc::downgrade(prev));
    } else {
        // This is the first event in the scan line.
        if event.is_vertical() {
            event.set_edge_count(0, 0); // TODO: Is this correct?
        } else {
            // First event in the scan line, it is not vertical.
            // Treat it as a lower boundary that is outside of the other polygon.

            // It is always a lower boundary and outside of the other polygon.
            // debug_assert!(!event.is_upper_boundary); // Not necessarily true for instance in a hour-glass shape with a self-intersecting polygon.

            event.set_edge_count(event.edge_weight(), 0);
        }
    }
}

/// Perform boolean operation.
///
/// # Example
/// ```
/// use iron_shapes_booleanop::*;
/// use iron_shapes::prelude::*;
/// let p1 = Polygon::from(vec![(0., 0.), (2., 0.), (2., 2.), (0., 2.)]);
/// let p2 = p1.translate((1., 1.).into());
/// let expected_union = Polygon::from(vec![(0., 0.), (2., 0.), (2., 1.), (3., 1.),
///                                                 (3., 3.), (1., 3.), (1., 2.), (0., 2.)]);
///
/// let i = boolean_op(edge_intersection_float, vec![&p1], vec![&p2], Operation::Union, PolygonSemantics::XOR);
///
/// assert_eq!(i.len(), 1);
/// assert_eq!(i.polygons[0], expected_union);
/// ```
pub fn boolean_op<'a, I, T, S, C>(edge_intersection: I,
                                  subject: S,
                                  clipping: C,
                                  operation: Operation,
                                  polygon_semantics: PolygonSemantics) -> MultiPolygon<T>
    where I: Fn(&Edge<T>, &Edge<T>) -> EdgeIntersection<T, T>,
          T: CoordinateType + Debug + 'a,
          S: IntoIterator<Item=&'a Polygon<T>>,
          C: IntoIterator<Item=&'a Polygon<T>>,
{
    let mut event_id_generator = (1..).into_iter();

    // Prepare the event queue.
    let mut event_queue = fill_queue(
        subject.into_iter(),
        clipping.into_iter(),
    );

    // Compute the edge intersections, the result is a set of sorted non-intersecting edges stored
    // as events.
    let sorted_events = subdivide_segments(
        edge_intersection,
        &mut event_queue,
        &mut event_id_generator,
        polygon_semantics,
    );


    dbg!(&sorted_events);

    // Connect the edges into polygons.
    let r = connect_edges(&sorted_events, operation, polygon_semantics);
    MultiPolygon::new(r)
}


/// Compute approximate intersection point of two edges in floating point coordinates.
pub fn edge_intersection_float<F: Float>(e1: &Edge<F>, e2: &Edge<F>) -> EdgeIntersection<F, F> {
    e1.edge_intersection_approx(e2, F::from(1e-8).unwrap())
}

/// Compute the intersection of edges with rational coordinates.
/// In rational coordinates intersections can be computed exactly.
pub fn edge_intersection_rational<T>(e1: &Edge<Ratio<T>>, e2: &Edge<Ratio<T>>)
                                     -> EdgeIntersection<Ratio<T>, Ratio<T>>
    where T: CoordinateType + Integer {
    e1.edge_intersection_rational(e2)
}

/// Compute intersection of edges in integer coordinates.
/// For edges that are parallel to the x or y axis the intersection can be computed exactly.
/// For others it will be rounded.
pub fn edge_intersection_integer<T: PrimInt + Debug>(e1: &Edge<T>, e2: &Edge<T>) -> EdgeIntersection<T, T> {
    e1.edge_intersection_rounded(e2)
}


/// Find all intersecting segments and subdivide them such that the set of resulting segments contains
/// no intersecting segments anymore.
/// The resulting events are sorted by their coordinates.
fn subdivide_segments<T: CoordinateType + Debug, I>(
    edge_intersection: I,
    event_queue: &mut BinaryHeap<Rc<SweepEvent<T>>>,
    event_id_generator: &mut RangeFrom<usize>,
    polygon_semantics: PolygonSemantics,
) -> Vec<Rc<SweepEvent<T>>>
    where I: Fn(&Edge<T>, &Edge<T>) -> EdgeIntersection<T, T> {
    let mut sorted_events = Vec::new();
    // Reserve the minimum amount of storage necessary.
    sorted_events.reserve(event_queue.len());

    let mut scan_line = SplaySet::new(compare_events_by_segments);

    let mut scan_line_position = None;

    while let Some(event) = event_queue.pop() {
        debug_assert!(event.is_left_event() ^ event.get_other_event().unwrap().is_left_event());

        let other_event = event.get_other_event().unwrap();

        if let Some(pos) = scan_line_position {
            debug_assert!(event.p.x >= pos, "Events are not ordered by x-coordinate.");
        }
        scan_line_position = Some(event.p.x);

        // If the event does not come from subdividing a segment
        // it must be assigned a new ID such that the ordering of collinear segments
        // will be preserved.
        if event.get_edge_id() == usize::MAX {
            debug_assert!(event.is_left_event(), "Right events should have the ID already assigned.");
            let edge_id = event_id_generator.next().unwrap();
            event.set_edge_id(edge_id);
            other_event.set_edge_id(edge_id);
        }

        // Insert the event such that the list remains sorted.
        // This is necessary in the rare case of overlapping edges the ordering
        // can be messed up.
        // Search position for the new event:
        let len = sorted_events.len();

        // TODO use something faster like galloping search?
        let pos_back: isize = sorted_events.iter()
            .rev() // Search from end of vector.
            .find_position(|&e: &&Rc<SweepEvent<T>>| e >= &event)
            .map(|(index, _value)| index as isize)
            .unwrap_or(len as isize); // If nothing is found, then place it at the beginning.

        let pos = (len as isize - pos_back) as usize;

        debug_assert_eq!(pos, sorted_events.iter()
            .find_position(|&e: &&Rc<SweepEvent<T>>| e <= &event)
            .map(|(index, _value)| index)
            .unwrap_or(len)
        );

        // Insert event at found position.
        sorted_events.insert(pos, event.clone());

        if event.is_left_event() {
            debug_assert!(!scan_line.contains(&event), "Event is already in the scan line.");
            // println!("insert {:?}", event.get_edge_id());

            scan_line.insert(event.clone());

            let maybe_next = scan_line.next(&event);
            if let Some(next) = maybe_next {
                debug_assert_ne!(compare_events_by_segments(&event, next), Ordering::Greater);
            }

            let maybe_prev = scan_line.prev(&event);
            if let Some(prev) = maybe_prev {
                // Debug-assert that the ordering of scan line elements is correct.
                debug_assert_ne!(compare_events_by_segments(&event, prev), Ordering::Less);
            }

            compute_fields(&event, maybe_prev);


            if let Some(next) = maybe_next {
                // Debug-assert that the ordering of scan line elements is correct.
                debug_assert!(compare_events_by_segments(&event, next) != Ordering::Greater);
                possible_intersection(&edge_intersection,
                                      &event, &next, event_queue);
            }

            if let Some(prev) = maybe_prev {
                // Debug-assert that the ordering of scan line elements is correct.
                debug_assert_ne!(compare_events_by_segments(&event, prev), Ordering::Less);
                possible_intersection(&edge_intersection,
                                      &prev, &event, event_queue);
            }
        } else {
            let left_event = other_event;

            // `event` must be a right event.
            debug_assert!(!event.is_left_event());
            debug_assert!(left_event.is_left_event());

            debug_assert!(scan_line.contains(&left_event), "Left event is not in scan line.");

            if scan_line.contains(&left_event) {
                let maybe_prev = scan_line.prev(&left_event).cloned();
                let maybe_next = scan_line.next(&left_event).cloned();

                // println!("remove {:?}", left_event.get_edge_id());
                scan_line.remove(&left_event);

                // prev and next are possibly new neighbours. Check for intersection.
                if let (Some(prev), Some(next)) = (maybe_prev, maybe_next) {
                    debug_assert!(compare_events_by_segments(&next, &prev) != Ordering::Less);
                    possible_intersection(&edge_intersection,
                                          &prev, &next, event_queue);
                }
            }
        }
    }

    debug_assert!(event_queue.is_empty(), "Not all events have been processed.");
    debug_assert!(scan_line.is_empty(), "Scan line still contains segments.");

    debug_assert!(sorted_events.windows(2).all(|w| w[0].p <= w[1].p),
                  "Events are not sorted by points.");

    debug_assert!(sorted_events.windows(2).all(|w| w[0] >= w[1]),
                  "Events are not sorted.");

    // Merge duplicated edges.
    (&sorted_events).into_iter()
        // Process left events only.
        .filter(|e| e.is_left_event())
        // Make groups of events that have the same edge.
        .group_by(|e| e.get_edge()).into_iter()
        // Reduce each group to a single contributing edge.
        .for_each(|(edge, event_group)|
            merge_edges(edge.unwrap(), event_group.collect(), polygon_semantics)
        );

    sorted_events
}


/// Merge a group of equal edges into a single edge.
/// Expects to get only left events.
///
/// TODO: Can this be done incrementally in `compute_fields`?
///
/// # Parameters
/// `edge`: An `Edge`.
/// `events`: All events that describe an equal edge as `edge`.
fn merge_edges<T: CoordinateType + Debug>(edge: Edge<T>,
                                          events: Vec<&Rc<SweepEvent<T>>>,
                                          polygon_semantics: PolygonSemantics,
) -> () {
    debug_assert!(&events.iter().all(|e| e.get_edge().unwrap() == edge),
                  "All group items must share the same edge.");

    // Compute the parity of the edges.
    // Two edges of the same polygon type (subject or clipping) cancel each other out.
    // Therefore for each polygon type at most one edge will remain.

    // Get previous edge below the set overlapping edges.
    let prev = events.first()
        .map(|e| e.get_prev())
        .unwrap_or(Weak::new());

    let mut last_subject: Option<&Rc<SweepEvent<T>>> = None;
    let mut sum_subject = 0;
    let mut last_clipping: Option<&Rc<SweepEvent<T>>> = None;
    let mut sum_clipping = 0;

    for &e in &events {
        debug_assert!(e.is_left_event());
        e.set_edge_type(EdgeType::Normal);
        match e.polygon_type {
            PolygonType::Subject => {
                sum_subject += e.edge_weight();
                // last_subject = match last_subject {
                //     None => Some(e),
                //     Some(last) => {
                //         // last and e cancel out.
                //         e.set_edge_type(EdgeType::NonContributing);
                //         last.set_edge_type(EdgeType::NonContributing);
                //         None
                //     }
                // };
                if let Some(last) = last_subject {
                    last.set_edge_type(EdgeType::NonContributing);
                }
                last_subject = Some(e);
            }
            PolygonType::Clipping => {
                sum_clipping += e.edge_weight();
                // last_clipping = match last_clipping {
                //     None => Some(e),
                //     Some(last) => {
                //         // last and e cancel out.
                //         e.set_edge_type(EdgeType::NonContributing);
                //         last.set_edge_type(EdgeType::NonContributing);
                //         None
                //     }
                // };
                if let Some(last) = last_clipping {
                    last.set_edge_type(EdgeType::NonContributing);
                }
                last_clipping = Some(e);
            }
        }
    }

    let is_last_subject_contributing = match polygon_semantics {
        PolygonSemantics::Union => sum_subject != 0,
        PolygonSemantics::XOR => sum_subject % 2 != 0
    };

    let is_last_clipping_contributing = match polygon_semantics {
        PolygonSemantics::Union => sum_clipping != 0,
        PolygonSemantics::XOR => sum_clipping % 2 != 0
    };

    if !is_last_clipping_contributing {
        if let Some(last) = last_clipping {
            last.set_edge_type(EdgeType::NonContributing);
        }
        last_clipping = None;
    }

    if !is_last_subject_contributing{
        if let Some(last) = last_subject {
            last.set_edge_type(EdgeType::NonContributing);
        }
        last_subject = None;
    }

    if let (Some(s), Some(c)) = (last_subject, last_clipping) {
        // Reduce to a single edge of type DifferentTransition or SameTransition.
        // The `outside` flag does not matter to the `connect_edge` stage in this case.
        s.set_edge_type(EdgeType::NonContributing);
        c.set_edge_type(
            match s.is_upper_boundary(polygon_semantics) == c.is_upper_boundary(polygon_semantics) {
                false => EdgeType::DifferentTransition,
                true => EdgeType::SameTransition
            }
        );
        // Update predecessor.
        c.set_prev(prev);
    } else if let Some(s) = last_subject {
        s.set_prev(prev);
    } else if let Some(c) = last_clipping {
        c.set_prev(prev);
    }

    // Compute the correct `outside` field for the chosen edge.
    // This is done by imagining to strip all previous edges away and update
    // the `outside` field accordingly.
    if !edge.is_vertical() {

        // Compute the correct `outside` field for the chosen edge.
        // This is done by imagining to strip all previous edges away and update
        // the `outside` field accordingly.
        // This is not necessary for vertical edges because `compute_fields` does behave differently
        // for verticals.
        match (last_subject, last_clipping) {
            (Some(edge), None) | (None, Some(edge)) => {
                // Count how many edges of the other polygon type appear before the remaining edge.
                let sum_other_edges: i32 = events.iter()
                    .take_while(|e| !Rc::ptr_eq(edge, e))
                    .filter(|e| e.polygon_type != edge.polygon_type)
                    .map(|e| e.edge_weight())
                    .sum();

                // Update the edge count.
                edge.set_edge_count(edge.edge_count(),
                                    edge.other_edge_count() - sum_other_edges);
            }
            _ => ()
        }
    }
}

#[cfg(test)]
mod test {
    extern crate rand;

    use crate::*;
    use iron_shapes::prelude::*;
    use iron_shapes::traits::{Translate, Scale};
    use num_rational::Rational;
    use self::rand::distributions::{Uniform, Distribution};
    use self::rand::rngs::StdRng;
    use self::rand::SeedableRng;

    #[test]
    fn test_degenerate_polygons() {
        let p1 = Polygon::from(vec![(0., 0.), (1., 1.)]);
        let p2 = Polygon::from(vec![(1., 0.), (0., 1.)]);

        let i = p1.intersection(&p2);

        assert_eq!(i.len(), 0);
    }

//    #[test]
//    fn test_almost_degenerate() {
//        let p1 = Polygon::from(vec![(0., 0.), (1., 1.), (1., 1.0001)]);
//        let p2 = Polygon::from(vec![(1., 0.), (0., 1.), (0., 1.0001)]);
//
//        let i = boolean_op(
//            edge_intersection_float,
//            &[&p1],
//            &[&p2],
//            Operation::Intersection,
// PolygonSemantics::XOR
//        );
//        println!("{:?}", i);
//        assert_eq!(i.len(), 1);
//        assert_eq!(i.polygons[0].len(), 4);
//    }

    #[test]
    fn test_boolean_op() {
        let p1 = Polygon::from(vec![(0., 0.), (2., 0.), (2., 2.), (0., 2.)]);
        let p2 = p1.translate((1., 1.).into());
//        let p1 = Polygon::from(vec![(1., 0.), (2., 1.), (1., 2.), (0., 1.)]);
//        let p2 = p1.translate((1., 0.).into());

        let expected_union = Polygon::from(vec![(0., 0.), (2., 0.), (2., 1.), (3., 1.),
                                                (3., 3.), (1., 3.), (1., 2.), (0., 2.)]);

        let i = p1.union(&p2);

        assert_eq!(i.len(), 1);
        assert_eq!(i.polygons[0], expected_union);
    }

    #[test]
    fn test_simple_union_integer() {
        let p1 = Polygon::from(vec![(0, 0), (2, 0), (2, 2), (0, 2)]);
        let p2 = p1.translate((1, 1).into());

        let expected_union = Polygon::from(vec![(0, 0), (2, 0), (2, 1), (3, 1),
                                                (3, 3), (1, 3), (1, 2), (0, 2)]);

        let i = boolean_op(
            edge_intersection_integer, &[p1], &[p2],
            Operation::Union,
            PolygonSemantics::XOR);

        assert_eq!(i.len(), 1);
        assert_eq!(i.polygons[0], expected_union);
    }

    #[test]
    fn test_boolean_op_same_polygon() {
        let p1 = Polygon::from(vec![(0., 0.), (2., 0.), (2., 2.)]);

        let i = boolean_op(
            edge_intersection_float, vec![&p1], vec![&p1], Operation::Union,
            PolygonSemantics::XOR);

        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);
    }


    #[test]
    fn test_boolean_op_duplicate_union_semantics() {

        // Use the same polygon multiple times as a subject or clipping polygon.
        // Union semantics should reduce multiple overlapping polygons into one polygon.

        let p1 = Polygon::from(vec![(0., 0.), (2., 0.), (2., 2.)]);

        let i = boolean_op(
            edge_intersection_float, vec![], vec![&p1, &p1],
            Operation::Union,
            PolygonSemantics::Union);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);

        let i = boolean_op(
            edge_intersection_float, vec![&p1, &p1], vec![],
            Operation::Union,
            PolygonSemantics::Union);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);

        let i = boolean_op(
            edge_intersection_float, vec![&p1, &p1], vec![&p1, &p1],
            Operation::Union,
            PolygonSemantics::Union);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);

        let i = boolean_op(
            edge_intersection_float, vec![&p1, &p1, &p1, &p1, &p1], vec![&p1, &p1, &p1],
            Operation::Union,
            PolygonSemantics::Union);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);
    }

    #[test]
    fn test_xor_same_polygon() {
        let p1 = Polygon::from(vec![(0., 0.), (2., 0.), (2., 2.)]);

        let i = boolean_op(
            edge_intersection_float, vec![&p1], vec![&p1],
            Operation::Xor,
            PolygonSemantics::XOR);

        assert_eq!(i.len(), 0);

        let i = boolean_op(
            edge_intersection_float, vec![&p1, &p1], vec![&p1],
            Operation::Xor,
            PolygonSemantics::XOR);

        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);


        let i = boolean_op(
            edge_intersection_float, vec![&p1], vec![&p1, &p1],
            Operation::Xor,
            PolygonSemantics::XOR);

        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);
    }

    #[test]
    fn test_holes() {
        let little_square = Polygon::from(vec![(0., 0.), (1., 0.), (1., 1.), (0., 1.)]);
        let big_square = little_square.scale(4.);
        let little_square_outside = little_square.translate((1., 5.).into());
        let little_square_inside = little_square.translate((2., 1.).into());

        let i = boolean_op(
            edge_intersection_float,
            &[big_square],
            &[little_square_inside, little_square_outside],
            Operation::Xor,
            PolygonSemantics::XOR,
        );

        assert!(i.polygons.iter().any(|p| p.interiors.len() == 1));
        assert!(i.polygons.iter().any(|p| p.interiors.len() == 0));

        assert!(i.contains_point((0.1, 0.1).into()));
        assert!(!i.contains_point((2.1, 1.1).into()));
        assert!(i.contains_point((1.1, 5.1).into()));
        assert!(!i.contains_point((100., 100.).into()));
    }

    #[test]
    fn test_separate_polygons_or_hole() {
        let little_square = Polygon::from(vec![(0., 0.), (1., 0.), (1., 1.), (0., 1.)]);
        let square1 = little_square.scale(2.);
        let square2 = square1.translate((1., 1.).into());

        let result = boolean_op(
            edge_intersection_float,
            vec![&square1],
            vec![&square2],
            Operation::Xor,
            PolygonSemantics::XOR,
        );

        let probe_points = [(0., 0.), (0.5, 0.5), (1., 1.), (1.5, 1.5), (2.5, 2.5)];

        // Create grid of probe points.
//        let n = 32;
//        let scale = 3.0 * n as f64;
//        let grid = (0..n).into_iter()
//            .flat_map(|x|
//                (0..n).into_iter()
//                    .map(move |y| (x,y))
//            ).map(|(x,y)| (x as f64 * scale, y as f64 * scale));

        for p in &probe_points {
            let p = p.into();
            let in1 = square1.contains_point(p);
            let in2 = square2.contains_point(p);

            let expected = in1 ^ in2; // Xor

            assert_eq!(result.contains_point(p), expected);
        }
    }
//
//    #[test]
//    fn test_random1() {
//        let seed = 2u8;
//        let seed1 = [seed + 0; 32];
//        let seed2 = [seed + 1; 32];
//
//        let between = Uniform::from(0..8);
//        let mut rng = ChaChaRng::from_seed(seed1);
//
//        let mut rand_polygon = |n_points: usize| -> Polygon<f64> {
//            let points: Vec<(f64, f64)> = (0..n_points).into_iter()
//                .map(|_| (between.sample(&mut rng) as f64, between.sample(&mut rng) as f64))
//                .collect();
//
//            Polygon::new(&points)
//        };
//
//        for i in 0..4000 {
//            println!("{:?}", i);
//
//            let a = rand_polygon(3);
//            let b = rand_polygon(3);
//
//            println!("a {:?}", a);
//            println!("b {:?}", b);
//
//            let result = boolean_op(
//                edge_intersection_float,
//                &[&a],
//                &[&b],
//                Operation::Intersection,
//            );
//
//            println!("result {:?}", result);
//
//            let a_in_b = a.exterior.iter().all(|&p| b.contains_point_non_oriented(p));
//            let b_in_a = b.exterior.iter().all(|&p| a.contains_point_non_oriented(p));
//
//            // Skip cases where there will be a hole. Not supported yet.
//            if a_in_b || b_in_a {
//                continue;
//            }
//
//            // Create grid of probe points.
//            let mut rng = ChaChaRng::from_seed(seed2);
//            let num_probe_points = 100;
//            let probe_points = (0..num_probe_points).into_iter()
//                .map(|_| (between.sample(&mut rng) as f64, between.sample(&mut rng) as f64));
//
//            let mut num_checks = 0usize;
//            for p in probe_points {
//                let p = p.into();
//
//                let on_border_1 = a.exterior.edges().iter()
//                    .any(|e| e.contains_point(p).inclusive_bounds());
//                let on_border_2 = b.exterior.edges().iter()
//                    .any(|e| e.contains_point(p).inclusive_bounds());
//
//                if on_border_1 || on_border_2 {
//                    // Ignore cases where the probe point lies on the border of the polygon because,
//                    // there `contains_point` is known to be inconsistent with `boolean_op`.
//                    continue;
//                }
//
//                let in1 = a.contains_point_non_oriented(p);
//                let in2 = b.contains_point_non_oriented(p);
//
//                let expected = in1 & in2; // Xor
//
//                let result_contains_p = result.contains_point_non_oriented(p);
//                if result_contains_p != expected {
//                    println!("a {:?}", a);
//                    println!("b {:?}", b);
//                    println!("result {:?}", result);
//                }
//
//                assert_eq!(result_contains_p, expected);
//                num_checks += 1;
//            }
//
//            assert!(num_checks >= num_probe_points / 2);
//        }
//    }

    #[test]
    fn test_rational_collinear_edge() {
        // The edges on the x-axis are collinear and overlap.

        let rp = |a: isize, b: isize| Point::new(Rational::from(a), Rational::from(b));
        let r = |a: isize, b: isize| Rational::new(a, b);

        let a = Polygon::from(vec![rp(0, 0), rp(5, 0), rp(3, 1)]);
        let b = Polygon::from(vec![rp(1, 0), rp(5, 0), rp(3, 2)]);

        let result = boolean_op(
            edge_intersection_rational,
            vec![&a],
            vec![&b],
            Operation::Intersection,
            PolygonSemantics::XOR,
        );

        let expected = Polygon::from(vec![rp(1, 0), rp(5, 0),
                                          rp(3, 1), (r(3, 2), r(1, 2)).into()]);

        assert_eq!(result.polygons[0], expected);
    }

    #[test]
    fn test_rational_collinear_edge_2() {
        // The edges on the x-axis are collinear and overlap.

        let rp = |a: isize, b: isize| Point::new(Rational::from(a), Rational::from(b));
//        let rp = |a: isize, b: isize| Point::new(a, b);
        let r = |a: isize, b: isize| Rational::new(a, b);

        let scale = r(1, 1);
        let a = Polygon::from(vec![rp(3, 2), rp(7, 6), rp(2, 5)]).scale(scale);
        let b = Polygon::from(vec![rp(1, 0), rp(7, 6), rp(3, 3)]).scale(scale);

        let result = boolean_op(
            edge_intersection_rational,
//            edge_intersection_integer,
            vec![&a],
            vec![&b],
            Operation::Intersection,
            PolygonSemantics::XOR,
        );

        dbg!(&result);

//        let expected = Polygon::from(vec![rp(1, 0), rp(5, 0),
//                                          rp(3, 1), (r(3,2), r(1,2)).into()]);

        assert_eq!(result.polygons.len(), 1);
        assert_eq!(result.polygons[0].len(), 4);
    }

    #[test]
    fn test_integer_collinear_edge() {
        // The edges on the x-axis are collinear and overlap.

        let p = |a: isize, b: isize| Point::new(a, b);

        let a = Polygon::from(vec![p(0, 0), p(10, 0), p(6, 2)]);
        let b = Polygon::from(vec![p(2, 0), p(10, 0), p(6, 4)]);

        let result = boolean_op(
            edge_intersection_integer,
            vec![&a],
            vec![&b],
            Operation::Intersection,
            PolygonSemantics::XOR,
        );

        let expected = Polygon::from(vec![p(2, 0), p(10, 0),
                                          p(6, 2), p(3, 1).into()]);

        assert_eq!(result.polygons.len(), 1);
        assert_eq!(result.polygons[0], expected);
    }

    #[test]
    fn test_vertical_degenerate_polygon() {
        // The edges on the x-axis are collinear and overlap.

        let p = |a: isize, b: isize| Point::new(Rational::from(a), Rational::from(b));
//        let p = |a: isize, b: isize| Point::new(a, b);


        let a = Polygon::from(vec![p(0, 0), p(0, 2)]);
//        let a = Polygon::from(vec![p(0, 0), p(0, 1), p(0, 3), p(0, 1)]);
        let b = Polygon::from(vec![p(0, 1), p(1, 1), p(1, 2)]);


//        // similar but no verticals (works)
//        let a = Polygon::from(vec![p(0, 0), p(2, 4)]);
//        let b = Polygon::from(vec![p(1, 2), p(3, 2), p(3, 4)]);

        let result = boolean_op(
            edge_intersection_rational,
            vec![&a],
            vec![&b],
            Operation::Intersection,
            PolygonSemantics::XOR,
        );

        dbg!(&result);

        assert_eq!(result.polygons.len(), 0);
    }

    #[test]
    fn test_rational_collinear_edge3() {
        // The edges on the x-axis are collinear and overlap.

//        let p = |a: isize, b: isize| Point::new(a, b);
        let p = |a: isize, b: isize| Point::new(Rational::from(a), Rational::from(b));

        // Some random translation
//        let t = Point::new(Rational::new(100, 7), Rational::new(5, 23));
        let t = Vector::new(Rational::new(0, 1), Rational::new(0, 1));

        let a = Polygon::from(vec![p(0, 0), p(11, 11), p(4, 8)]).translate(t);
        let b = Polygon::from(vec![p(4, 4), p(11, 11), p(1, 10)]).translate(t);

        let result = boolean_op(
//            edge_intersection_integer,
edge_intersection_rational,
vec![&a],
vec![&b],
Operation::Intersection,
PolygonSemantics::XOR,
        );

        let expected = Polygon::from(vec![p(4, 4), p(11, 11),
                                          p(4, 8), p(3, 6).into()]).translate(t);

        assert_eq!(result.polygons[0], expected);
    }

    /// Test boolean operations on random polygons in rational coordinates.
    #[test]
    fn test_random_rational() {
        let seed = 3u8;
        let seed1 = [seed + 0; 32];
        let seed2 = [seed + 1; 32];

        assert!(Rational::new(1, 2) == Rational::new(2, 4));

        let max_coordinate = 4;
        let between = Uniform::from(0..max_coordinate);
        let mut rng = StdRng::from_seed(seed1);

        let mut rand_polygon = |n_points: usize| -> Polygon<Rational> {
            let points: Vec<_> = (0..n_points).into_iter()
                .map(|_| (Rational::new(between.sample(&mut rng), 1),
                          Rational::new(between.sample(&mut rng), 1)))
                .collect();

            Polygon::new(&points)
        };

        for _i in 0..100 {
            let a = rand_polygon(5);
            let b = rand_polygon(4);

            let results: Vec<_> = vec![Operation::Intersection,
                                       Operation::Union,
                                       Operation::Difference,
                                       Operation::Xor]
                .into_iter()
                .map(|operation| boolean_op(
                    edge_intersection_rational,
                    vec![&a],
                    vec![&b],
                    operation,
                    PolygonSemantics::XOR,
                )).collect();


            // Create grid of probe points.
            let mut rng = StdRng::from_seed(seed2);
            let num_probe_points = 10;
            let denominator = 100;
            let between = Uniform::from(0..max_coordinate * denominator);

            // Create random probe points.
            let probe_points = (0..num_probe_points).into_iter()
                .map(|_| (Rational::new(between.sample(&mut rng), denominator),
                          Rational::new(between.sample(&mut rng), denominator)));

            let mut num_checks = 0usize;
            for p in probe_points {
                let p = p.into();

                let on_border_1 = a.exterior.edges().iter()
                    .any(|e| e.contains_point(p).inclusive_bounds());
                let on_border_2 = b.exterior.edges().iter()
                    .any(|e| e.contains_point(p).inclusive_bounds());

                if on_border_1 || on_border_2 {
                    // Ignore cases where the probe point lies on the border of the polygon because,
                    // there `contains_point` is known to be inconsistent with `boolean_op`.
                    continue;
                }

                let in1 = a.contains_point(p);
                let in2 = b.contains_point(p);

                let expected = [
                    in1 & in2,
                    in1 | in2,
                    in1 & !in2,
                    in1 ^ in2,
                ];

                let result_contains_p = results.iter().map(|r|
                    r.contains_point(p)
                );

                let success = result_contains_p
                    .zip(expected.iter())
                    .all(|(actual, expected)| actual == *expected);

                if !success {
                    dbg!(&a);
                    dbg!(&b);
                    dbg!(&results);
                    dbg!(&p);
                }

                assert!(success);
                num_checks += 1;
            }

            assert!(num_checks >= num_probe_points / 2);
        }
    }
}