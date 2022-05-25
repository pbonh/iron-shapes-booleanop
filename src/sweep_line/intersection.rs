// Copyright (c) 2018-2021 Thomas Kramer.
// SPDX-FileCopyrightText: 2022 Thomas Kramer
// SPDX-FileCopyrightText: 2020 Fabian Keller <github.100.fkeller@spamgourmet.com> (contributions under MIT licence)
// SPDX-FileCopyrightText: 2020 Bodo Junglas <junglas@objectcode.de> (contributions under MIT licence)
//
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::binary_heap::BinaryHeap;

use std::rc::Rc;
use iron_shapes::edge::{Edge, EdgeIntersection};
use num_rational::Ratio;

use iron_shapes::CoordinateType;
use num_traits::{Float, PrimInt};
use std::fmt::Debug;
use std::ops::Deref;
use std::cmp::Ordering;
use itertools::Itertools;
use num_integer::Integer;

use super::sweep_event::*;

use super::compare_segments::compare_events_by_segments;
use super::possible_intersection::possible_intersection;

// use super::splay_scanline::SplayScanLine;
use super::btree_scanline::BTreeScanLine;
// use super::naive_scanline::NaiveScanLine;


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
struct ScanlineElement<T, Ctr, P>(Rc<SweepEvent<T, Ctr, P>>) where T: CoordinateType;

impl<T: PartialEq + CoordinateType, Ctr, P> PartialEq for ScanlineElement<T, Ctr, P> {
    fn eq(&self, other: &Self) -> bool {
        self.0.eq(&other.0)
    }
}

impl<T: PartialEq + CoordinateType, Ctr, P> Eq for ScanlineElement<T, Ctr, P> {}

impl<T, Ctr, P> Deref for ScanlineElement<T, Ctr, P>
    where T: CoordinateType {
    type Target = Rc<SweepEvent<T, Ctr, P>>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T, Ctr, P> Ord for ScanlineElement<T, Ctr, P>
    where T: CoordinateType + Debug {
    fn cmp(&self, other: &Self) -> Ordering {
        compare_events_by_segments(self, other)
    }
}

impl<T, Ctr, P> PartialOrd for ScanlineElement<T, Ctr, P>
    where T: CoordinateType + Debug {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(compare_events_by_segments(self, other))
    }
}

/// Helper function to be used as comparator for the `SplayScanLine`.
#[allow(unused)]
fn compare_scanline_elements<T, Ctr, P>(a: &ScanlineElement<T, Ctr, P>, b: &ScanlineElement<T, Ctr, P>) -> Ordering
    where T: CoordinateType + Debug {
    compare_events_by_segments(a, b)
}

/// Find all intersecting segments and subdivide them such that the set of resulting segments contains
/// no intersecting segments anymore.
/// The resulting events are sorted by their coordinates.
pub fn subdivide_segments<T, I, Ctr, P, UpdateCtrFn>(
    edge_intersection: I,
    event_queue: &mut BinaryHeap<Rc<SweepEvent<T, Ctr, P>>>,
    update_counter: UpdateCtrFn,
) -> Vec<Rc<SweepEvent<T, Ctr, P>>>
    where I: Fn(&Edge<T>, &Edge<T>) -> EdgeIntersection<T, T, Edge<T>>,
          T: CoordinateType + Debug,
          Ctr: Clone + Default,
          P: Clone,
          UpdateCtrFn: Fn(&Rc<SweepEvent<T, Ctr, P>>, Option<&Rc<SweepEvent<T, Ctr, P>>>)
{
    let mut sorted_events = Vec::new();
    // Reserve the minimum amount of storage necessary.
    sorted_events.reserve(event_queue.len());


    // let mut scan_line = SplayScanLine::new(compare_scanline_elements);
    let mut scan_line: BTreeScanLine<ScanlineElement<T, Ctr, P>> = BTreeScanLine::new();
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

                update_counter(&event, maybe_prev.map(|e| e.deref()));

                if let Some(prev) = maybe_prev {
                    // Remember the previous segment for contour-hole attribution.
                    // Note: What matters in the end is the previous segment that also contributes to the result
                    // Hence there is post-processing necessary (done in `connect_edges::filter_events()`).
                    event.set_prev(Rc::downgrade(prev));
                }
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
                .find_position(|&e: &&Rc<SweepEvent<T, Ctr, P>>| e >= &event)
                .map(|(index, _value)| index as isize)
                .unwrap_or(len as isize); // If nothing is found, then place it at the beginning.

            let pos = (len as isize - pos_back) as usize;

            debug_assert_eq!(pos, sorted_events.iter()
                .find_position(|&e: &&Rc<SweepEvent<T, Ctr, P>>| e <= &event)
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
