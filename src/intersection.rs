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
use num_rational::Rational;
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

/// Insert the edges of the polygons into the event queue.
fn fill_queue<T: CoordinateType>(subject: &[&Polygon<T>],
                                 clipping: &[&Polygon<T>]) -> BinaryHeap<Rc<SweepEvent<T>>> {
    let mut event_queue = BinaryHeap::new();

    /// Add edges of a polygon to the event queue.
    fn process_polygon<T: CoordinateType>(event_queue: &mut BinaryHeap<Rc<SweepEvent<T>>>,
                                          poly: &SimplePolygon<T>,
                                          polygon_type: PolygonType,
                                          contour_id: usize,
                                          is_hull: bool) {
        for edge in poly.edges() {
            // Skip degenerate edges.
            if !edge.is_degenerate() {
//                let edge_id = edge_ids.next().unwrap();
                let edge_id = usize::MAX;
                let event_a_is_left = edge.start < edge.end;
                let event_a = SweepEvent::new_rc(
                    contour_id,
                    edge_id,
                    edge.start,
                    event_a_is_left,
                    Weak::new(),
                    polygon_type,
                    EdgeType::Normal,
                    is_hull,
                );
                let event_b = SweepEvent::new_rc(
                    contour_id,
                    edge_id,
                    edge.end,
                    !event_a_is_left,
                    Rc::downgrade(&event_a),
                    polygon_type,
                    EdgeType::Normal,
                    is_hull,
                );


                event_a.set_other_event(&event_b);

                event_queue.push(event_a);
                event_queue.push(event_b);
            }
        }
    }
    ;

    let mut contour_id = 0;

    // Subject polygons.
    for p in subject {
        contour_id += 1;
        process_polygon(&mut event_queue, &p.exterior, PolygonType::Subject, contour_id, true);
        for i in &p.interiors {
            // Holes
            process_polygon(&mut event_queue, i, PolygonType::Subject, contour_id, false);
        }
    }

    // Clipping polygons.
    for p in clipping {
        process_polygon(&mut event_queue, &p.exterior, PolygonType::Clipping, contour_id, true);
        for i in &p.interiors {
            // Holes
            process_polygon(&mut event_queue, i, PolygonType::Clipping, contour_id, false);
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

        let upper_boundary = match (event.is_vertical(), is_same_type) {
            (false, false) => !prev.is_outside_other(),
            (false, true) => !prev.is_upper_boundary(),

            (true, false) => prev.is_outside_other(),
            (true, true) => prev.is_upper_boundary(),
        };

        let is_outside_other = match is_same_type {
            false => prev.is_upper_boundary(),
            true => prev.is_outside_other(),
        };

        event.set_upper_boundary(upper_boundary, is_outside_other);

        // TODO: Remember the previous segment for contour-hole attribution.

    } else {
        // This is the first event in the scan line.
        if event.is_vertical() {
            event.set_upper_boundary(true, true);
        } else {
            // It is always a lower boundary and outside of the other polygon.
            event.set_upper_boundary(false, true);
        }
    }
}

/// Perform boolean operation on multiple subject and clipping polygons.
/// # Parameters
/// `edge_intersection`: Function to compute the intersection of two edges.
/// `operations`: The boolean operations to be computed.
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
/// let result = boolean_multi_op(edge_intersection_float, &[&p1], &[&p2],
///     &[Operation::Union]);
///
/// assert_eq!(result[0].len(), 1);
/// assert_eq!(result[0].polygons[0], expected_union);
/// ```
pub fn boolean_multi_op<I, F>(edge_intersection: I,
                              subject: &[&Polygon<F>],
                              clipping: &[&Polygon<F>],
                              operations: &[Operation]) -> Vec<MultiPolygon<F>>
    where I: Fn(&Edge<F>, &Edge<F>) -> EdgeIntersection<F, F>,
          F: CoordinateType + Debug {
    let mut event_id_generator = (1..).into_iter();
    // Prepare the event queue.
    let mut event_queue = fill_queue(subject, clipping);

    // Compute the edge intersections, the result is a set of sorted non-intersecting edges stored
    // as events.
    let sorted_events = subdivide_segments(
        edge_intersection,
        &mut event_queue,
        &mut event_id_generator,
    );

    // Connect the edges into polygons.
    let results = operations.iter()
        .map(|&o| {
            let r = connect_edges(&sorted_events, o);
            MultiPolygon::new(r)
        }
        ).collect();

    results
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
/// let i = boolean_op(edge_intersection_float, &[&p1], &[&p2], Operation::Union);
///
/// assert_eq!(i.len(), 1);
/// assert_eq!(i.polygons[0], expected_union);
/// ```
pub fn boolean_op<I, F>(edge_intersection: I,
                        subject: &[&Polygon<F>],
                        clipping: &[&Polygon<F>],
                        operation: Operation) -> MultiPolygon<F>
    where I: Fn(&Edge<F>, &Edge<F>) -> EdgeIntersection<F, F>,
          F: CoordinateType + Debug {
    let mut result = boolean_multi_op(edge_intersection,
                                      subject,
                                      clipping,
                                      &[operation]);
    result.remove(0)
}

/// Compute approximate intersection point of two edges in floating point coordinates.
pub fn edge_intersection_float<F: Float>(e1: &Edge<F>, e2: &Edge<F>) -> EdgeIntersection<F, F> {
    e1.edge_intersection_approx(e2, F::from(1e-8).unwrap())
}

/// Compute the intersection of edges with rational coordinates.
/// In rational coordinates intersections can be computed exactly.
pub fn edge_intersection_rational(e1: &Edge<Rational>, e2: &Edge<Rational>) -> EdgeIntersection<Rational, Rational> {
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
            println!("insert {:?}", event.get_edge_id());

            scan_line.insert(event.clone());

            let maybe_next = scan_line.next(&event);
            if let Some(next) = maybe_next {
                debug_assert_ne!(compare_events_by_segments(&event, next), Ordering::Greater);
            }

            let maybe_prev = scan_line.prev(&event);
            if let Some(prev) = maybe_prev {
                debug_assert_ne!(compare_events_by_segments(&event, prev), Ordering::Less);
            }

            compute_fields(&event, maybe_prev);


            if let Some(next) = maybe_next {
                debug_assert!(compare_events_by_segments(&event, next) != Ordering::Greater);
                possible_intersection(&edge_intersection,
                                      &event, &next, event_queue);
            }

            if let Some(prev) = maybe_prev {
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

                println!("remove {:?}", left_event.get_edge_id());
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

    // Merge duplicated edges.j
    (&sorted_events).into_iter()
        // Process left events only.
        .filter(|e| e.is_left_event())
        // Make groups of events that have the same edge.
        .group_by(|e| e.get_edge()).into_iter()
        // Reduce each group to a single contributing edge.
        .for_each(|(edge, event_group)|
            merge_edges(edge.unwrap(), event_group.collect())
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
fn merge_edges<T: CoordinateType + Debug>(edge: Edge<T>, events: Vec<&Rc<SweepEvent<T>>>) -> () {
    debug_assert!(&events.iter().all(|e| e.get_edge().unwrap() == edge),
                  "All group items must share the same edge.");

    // Compute the parity of the edges.
    // Two edges of the same polygon type (subject or clipping) cancel each other out.
    // Therefore for each polygon type at most one edge will remain.

    let mut last_subject = None;
    let mut last_clipping = None;

    for e in &events {
        debug_assert!(e.is_left_event());
        e.set_edge_type(EdgeType::Normal);
        match e.polygon_type {
            PolygonType::Subject => {
                last_subject = match last_subject {
                    None => Some(e),
                    Some(last) => {
                        // last and e cancel out.
                        e.set_edge_type(EdgeType::NonContributing);
                        last.set_edge_type(EdgeType::NonContributing);
                        None
                    }
                }
            }
            PolygonType::Clipping => {
                last_clipping = match last_clipping {
                    None => Some(e),
                    Some(last) => {
                        // last and e cancel out.
                        e.set_edge_type(EdgeType::NonContributing);
                        last.set_edge_type(EdgeType::NonContributing);
                        None
                    }
                }
            }
        }
    }

    if let (Some(s), Some(c)) = (last_subject, last_clipping) {
        // Reduce to a single edge of type DifferentTransition or SameTransition.
        // The `outside` flag does not matter to the `connect_edge` stage in this case.
        s.set_edge_type(EdgeType::NonContributing);
        c.set_edge_type(
            match s.is_upper_boundary() == c.is_upper_boundary() {
                false => EdgeType::DifferentTransition,
                true => EdgeType::SameTransition
            }
        );
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
                let num_other_edges = &events.iter()
                    .take_while(|e| !Rc::ptr_eq(edge, e))
                    .filter(|e| e.polygon_type != edge.polygon_type)
                    .count();

                let flip_outside_flag = num_other_edges % 2 == 1;
                // Now flip the outside flag.
                edge.set_upper_boundary(edge.is_upper_boundary(),
                                        edge.is_outside_other() ^ flip_outside_flag)
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

        let i = boolean_op(
            edge_intersection_float,
            &[&p1],
            &[&p2],
            Operation::Intersection,
        );

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

        let i = boolean_op(
            edge_intersection_float, &[&p1], &[&p2], Operation::Union);

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
            edge_intersection_integer, &[&p1], &[&p2], Operation::Union);

        assert_eq!(i.len(), 1);
        assert_eq!(i.polygons[0], expected_union);
    }

    #[test]
    fn test_boolean_op_same_polygon() {
        let p1 = Polygon::from(vec![(0., 0.), (2., 0.), (2., 2.)]);

        let i = boolean_op(
            edge_intersection_float, &[&p1], &[&p1], Operation::Union);

        assert_eq!(i.len(), 1);
        assert_eq!(&p1, &p1);
    }

    #[test]
    fn test_holes() {
        let little_square = Polygon::from(vec![(0., 0.), (1., 0.), (1., 1.), (0., 1.)]);
        let big_square = little_square.scale(4.);
        let little_square_outside = little_square.translate((1., 5.).into());
        let little_square_inside = little_square.translate((2., 1.).into());

        let i = boolean_op(
            edge_intersection_float,
            &[&big_square],
            &[&little_square_inside, &little_square_outside],
            Operation::Xor,
        );

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
            &[&square1],
            &[&square2],
            Operation::Xor,
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
            &[&a],
            &[&b],
            Operation::Intersection,
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
            &[&a],
            &[&b],
            Operation::Intersection,
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
            &[&a],
            &[&b],
            Operation::Intersection,
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
            &[&a],
            &[&b],
            Operation::Intersection,
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
&[&a],
&[&b],
Operation::Intersection,
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

            let results = boolean_multi_op(
                edge_intersection_rational,
                &[&a],
                &[&b],
                &[Operation::Intersection,
                    Operation::Union,
                    Operation::Difference,
                    Operation::Xor],
            );

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