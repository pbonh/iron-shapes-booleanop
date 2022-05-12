// Copyright (c) 2018-2021 Thomas Kramer.
// SPDX-FileCopyrightText: 2022 Thomas Kramer
// SPDX-FileCopyrightText: 2020 Fabian Keller <github.100.fkeller@spamgourmet.com> (contributions under MIT licence)
// SPDX-FileCopyrightText: 2020 Bodo Junglas <junglas@objectcode.de> (contributions under MIT licence)
//
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::binary_heap::BinaryHeap;

use std::rc::{Rc, Weak};
use iron_shapes::polygon::{Polygon, SimplePolygon};
use iron_shapes::edge::{Edge, EdgeIntersection};
use num_rational::Ratio;
use iron_shapes::multi_polygon::MultiPolygon;

use iron_shapes::CoordinateType;
use super::sweep_event::*;
use super::Operation;
use num_traits::{Float, PrimInt};
use super::compare_segments::compare_events_by_segments;
use super::connect_edges::connect_edges;
use super::possible_intersection::possible_intersection;
use std::fmt::Debug;
use std::ops::{RangeFrom, Deref};
use std::cmp::Ordering;
use itertools::Itertools;
use num_integer::Integer;
use crate::PolygonSemantics;
use crate::splay_scanline::SplayScanLine;
use crate::btree_scanline::BTreeScanLine;
use crate::naive_scanline::NaiveScanLine;

/// Insert the edges of the polygons into the event queue.
fn fill_queue<'a, T, S, C>(
    subject: S,
    clipping: C,
) -> BinaryHeap<Rc<SweepEvent<T>>>
    where S: Iterator<Item=&'a Polygon<T>>,
          C: Iterator<Item=&'a Polygon<T>>,
          T: CoordinateType + 'a {
    let mut event_queue = BinaryHeap::new();
    let mut event_id_generator = (1..).into_iter();

    /// Add edges of a polygon to the event queue.
    fn process_polygon<T: CoordinateType>(event_queue: &mut BinaryHeap<Rc<SweepEvent<T>>>,
                                          poly: &SimplePolygon<T>,
                                          polygon_type: PolygonType,
                                          event_id_generator: &mut RangeFrom<usize>, ) {
        for edge in poly.edges() {
            // Skip degenerate edges.
            if !edge.is_degenerate() {
                let edge_id = event_id_generator.next().unwrap();
                let event_a_is_left = edge.start < edge.end;

                // Upper boundary edges are directed from right to left.
                let is_upper_boundary = edge.end < edge.start;

                let event_a = SweepEvent::new_rc(
                    edge_id,
                    edge.start,
                    edge.end,
                    event_a_is_left,
                    Weak::new(),
                    polygon_type,
                    is_upper_boundary,
                );
                let event_b = SweepEvent::new_rc(
                    edge_id,
                    edge.end,
                    edge.start,
                    !event_a_is_left,
                    Rc::downgrade(&event_a),
                    polygon_type,
                    is_upper_boundary,
                );

                event_a.set_other_event(&event_b);

                event_queue.push(event_a);
                event_queue.push(event_b);
            }
        }
    }

    // Subject polygons.
    for p in subject {
        process_polygon(&mut event_queue, &p.exterior, PolygonType::Subject, &mut event_id_generator);
        for i in &p.interiors {
            // Holes
            process_polygon(&mut event_queue, i, PolygonType::Subject, &mut event_id_generator);
        }
    }

    // Clipping polygons.
    for p in clipping {
        process_polygon(&mut event_queue, &p.exterior, PolygonType::Clipping, &mut event_id_generator);
        for i in &p.interiors {
            // Holes
            process_polygon(&mut event_queue, i, PolygonType::Clipping, &mut event_id_generator);
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
        debug_assert_eq!(event.is_left_event(), prev.is_left_event());


        let is_same_type = event.polygon_type == prev.polygon_type;

        let edge_count = if is_same_type {
            prev.edge_count()
        } else {
            prev.other_edge_count()
        };

        // Update the edge count of the current type.
        let edge_count = edge_count + event.edge_weight();

        let other_edge_count = if is_same_type {
            prev.other_edge_count()
        } else {
            prev.edge_count()
        };

        event.set_edge_count(edge_count, other_edge_count);

        // Remember the previous segment for contour-hole attribution.
        // Note: What matters in the end is the previous segment that also contributes to the result
        // Hence there is post-processing necessary (done in `connect_edges::filter_events()`).
        event.set_prev(Rc::downgrade(prev));
    } else {
        // This is the first event in the scan line.
        // First event in the scan line, it is not vertical.
        // Treat it as a lower boundary that is outside of the other polygon.

        // It is always a lower boundary and outside of the other polygon.
        // debug_assert!(!event.is_upper_boundary); // Not necessarily true for instance in a hour-glass shape with a self-intersecting polygon.

        // Verticals have an edge weight of 0.
        event.set_edge_count(event.edge_weight(), 0);
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
    where I: Fn(&Edge<T>, &Edge<T>) -> EdgeIntersection<T, T, Edge<T>>,
          T: CoordinateType + Debug + 'a,
          S: IntoIterator<Item=&'a Polygon<T>>,
          C: IntoIterator<Item=&'a Polygon<T>>,
{

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
    );

    // Connect the edges into polygons.
    let r = connect_edges(&sorted_events, operation, polygon_semantics);
    MultiPolygon::from_polygons(r)
}


/// Compute approximate intersection point of two edges in floating point coordinates.
pub fn edge_intersection_float<F: Float>(e1: &Edge<F>, e2: &Edge<F>) -> EdgeIntersection<F, F, Edge<F>> {
    e1.edge_intersection_approx(e2, F::from(1e-8).unwrap())
}

/// Compute the intersection of edges with rational coordinates.
/// In rational coordinates intersections can be computed exactly.
pub fn edge_intersection_rational<T>(e1: &Edge<Ratio<T>>, e2: &Edge<Ratio<T>>)
                                     -> EdgeIntersection<Ratio<T>, Ratio<T>, Edge<Ratio<T>>>
    where T: CoordinateType + Integer {
    e1.edge_intersection_rational(e2)
}

/// Compute intersection of edges in integer coordinates.
/// For edges that are parallel to the x or y axis the intersection can be computed exactly.
/// For others it will be rounded.
pub fn edge_intersection_integer<T: PrimInt + Debug>(e1: &Edge<T>, e2: &Edge<T>) -> EdgeIntersection<T, T, Edge<T>> {
    e1.edge_intersection_rounded(e2)
}

/// Wrap a SweepEvent in order to use another implementation of `Ord` which is needed for the scanline.
#[derive(Clone)]
struct ScanlineElement<T>(Rc<SweepEvent<T>>) where T: CoordinateType;

impl<T: PartialEq + CoordinateType> PartialEq for ScanlineElement<T> {
    fn eq(&self, other: &Self) -> bool {
        self.0.eq(&other.0)
    }
}

impl<T: PartialEq + CoordinateType> Eq for ScanlineElement<T> {}

impl<T> Deref for ScanlineElement<T>
    where T: CoordinateType {
    type Target = Rc<SweepEvent<T>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> Ord for ScanlineElement<T>
    where T: CoordinateType + Debug {
    fn cmp(&self, other: &Self) -> Ordering {
        compare_events_by_segments(self, other)
    }
}

impl<T> PartialOrd for ScanlineElement<T>
    where T: CoordinateType + Debug {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(compare_events_by_segments(self, other))
    }
}

/// Helper function to be used as comparator for the `SplayScanLine`.
fn compare_scanline_elements<T>(a: &ScanlineElement<T>, b: &ScanlineElement<T>) -> Ordering
    where T: CoordinateType + Debug {
    compare_events_by_segments(a, b)
}

/// Find all intersecting segments and subdivide them such that the set of resulting segments contains
/// no intersecting segments anymore.
/// The resulting events are sorted by their coordinates.
fn subdivide_segments<T: CoordinateType + Debug, I>(
    edge_intersection: I,
    event_queue: &mut BinaryHeap<Rc<SweepEvent<T>>>,
) -> Vec<Rc<SweepEvent<T>>>
    where I: Fn(&Edge<T>, &Edge<T>) -> EdgeIntersection<T, T, Edge<T>> {
    let mut sorted_events = Vec::new();
    // Reserve the minimum amount of storage necessary.
    sorted_events.reserve(event_queue.len());


    // let mut scan_line = SplayScanLine::new(compare_scanline_elements);
    let mut scan_line: BTreeScanLine<ScanlineElement<T>> = BTreeScanLine::new();
    // let mut scan_line = NaiveScanLine::new(compare_scanline_elements);

    #[cfg(debug)]
        let mut scan_line_position = None; // For sanity checks only.

    // Process all events.
    while let Some(event) = event_queue.pop() {
        debug_assert!(event.is_left_event() ^ event.get_other_event().unwrap().is_left_event());
        let event = ScanlineElement(event);

        let other_event = ScanlineElement(event.get_other_event().unwrap());

        #[cfg(debug)] {
            if let Some(pos) = scan_line_position {
                debug_assert!(event.p.x >= pos, "Events are not ordered by x-coordinate.");
            }
            scan_line_position = Some(event.p.x);
        }

        if event.is_left_event() {
            debug_assert!(!scan_line.contains(&event), "Event is already in the scan line.");

            {
                let maybe_next = scan_line.next(&event);
                if let Some(next) = maybe_next {
                    debug_assert_ne!(compare_events_by_segments(&event, next), Ordering::Greater);
                }


                if let Some(next) = maybe_next {
                    debug_assert!(
                        compare_events_by_segments(&event, next) != Ordering::Greater,
                        "ordering of elements in scanline is wrong"
                    );
                    let queue_modified = possible_intersection(&edge_intersection, &event, &next, event_queue);
                    if queue_modified {
                        scan_line.remove(&event);
                        event_queue.push(event.0);
                        continue;
                    }
                }

                let maybe_prev = scan_line.prev(&event);
                if let Some(prev) = maybe_prev {
                    // Debug-assert that the ordering of scan line elements is correct.
                    debug_assert_ne!(compare_events_by_segments(&event, prev), Ordering::Less);
                }

                if let Some(prev) = maybe_prev {
                    debug_assert_ne!(
                        compare_events_by_segments(&event, prev), Ordering::Less,
                        "ordering of elements in scanline is wrong"
                    );
                    let queue_modified = possible_intersection(&edge_intersection, &prev, &event, event_queue);
                    if queue_modified {
                        scan_line.remove(&event);
                        event_queue.push(event.0);
                        continue;
                    }
                }

                compute_fields(&event, maybe_prev.map(|e| e.deref()));
            }

            // Insert new event into the scanline.
            scan_line.insert(event.clone());
        } else {
            let left_event = other_event;

            // `event` must be a right event.
            debug_assert!(!event.is_left_event());
            debug_assert!(left_event.is_left_event());

            debug_assert!(scan_line.contains(&left_event), "Left event is not in scan line.");

            {
                let maybe_prev = scan_line.prev(&left_event).cloned();
                let maybe_next = scan_line.next(&left_event).cloned();

                scan_line.remove(&left_event);

                // prev and next are possibly new neighbours. Check for intersection.
                if let (Some(prev), Some(next)) = (maybe_prev, maybe_next) {
                    debug_assert!(compare_events_by_segments(&next, &prev) != Ordering::Less);
                    possible_intersection(&edge_intersection,
                                          &prev, &next, event_queue);
                }
            }
        }

        // Insert the event such that the list remains sorted.
        // This is necessary in the rare case of overlapping edges the ordering
        // can be messed up.
        // Search position for the new event:
        {
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
            sorted_events.insert(pos, event.0);
        }
    }

    debug_assert!(event_queue.is_empty(), "Not all events have been processed.");
    debug_assert!(scan_line.is_empty(), "Scan line still contains segments.");

    debug_assert!(sorted_events.windows(2).all(|w| w[0].p <= w[1].p),
                  "Events are not sorted by points.");

    debug_assert!(sorted_events.windows(2).all(|w| w[0] >= w[1]),
                  "Events are not sorted.");


    sorted_events
}
