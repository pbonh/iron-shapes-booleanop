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

//! Connect the resulting edges of the sweep line algorithm into polygons.

use iron_shapes::CoordinateType;
use super::sweep_event::{SweepEvent, PolygonType, EdgeType};
use super::Operation;
use std::rc::Rc;
use std::fmt::Debug;
use iron_shapes::polygon::Polygon;
use iron_shapes::point::Point;
use iron_shapes::simple_polygon::SimplePolygon;
use crate::PolygonSemantics;

#[derive(Debug, Clone, PartialEq)]
struct Event<T: CoordinateType> {
    /// Index of this event in the vector where it is stored.
    index: usize,
    /// Index of the other event of this pair.
    other_index: usize,
    /// The index of the segment just below.
    prev_index: Option<usize>,
    /// The endpoint of the edge which is represented by this event.
    p: Point<T>,
    /// Is this part of a hole? Used to distinguish between holes and hulls.
    is_hole: bool,
    /// Is this an upper boundary of the contour? Used to distinguish between holes and hulls.
    is_upper_boundary: bool,
    /// Tells if this is the left or right event of the segment.
    is_left_event: bool,
    polygon_type: PolygonType,
    contour_id: usize,
}

//impl<T> Eq for Event<T>
//    where T: CoordinateType {}
//
//impl<T> PartialOrd for Event<T>
//    where T: CoordinateType {
//    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
//        Some(self.cmp(other))
//    }
//}
//
//impl<T> Ord for Event<T>
//    where T: CoordinateType {
//    fn cmp(&self, other: &Self) -> Ordering {
//        let point_ordering = self.p.partial_cmp(&other.p).unwrap();
//        let left_right_ordering = self.is_left_event.cmp(&other.is_left_event);
//
//        // Ordering by point, break ties by sorting left before right events.
//        point_ordering.then(left_right_ordering)
//    }
//}


/// Check if the event contributes to the result.
fn contributes_to_result<T>(event: &SweepEvent<T>,
                            operation: Operation,
                            polygon_semantics: PolygonSemantics) -> bool
    where T: CoordinateType,
{
    debug_assert!(event.is_left_event());

    event.is_outer_boundary(polygon_semantics)
        &&
        match operation {
            Operation::Intersection => !event.is_outside_other(polygon_semantics),
            Operation::Union => event.is_outside_other(polygon_semantics),
            Operation::Difference => match event.polygon_type {
                PolygonType::Subject => event.is_outside_other(polygon_semantics),
                PolygonType::Clipping => !event.is_outside_other(polygon_semantics)
            }
            Operation::Xor => true,
        }

    // TODO: check correctness.
    // event.is_outer_boundary(polygon_semantics)
    //     && match event.get_edge_type() {
    //     EdgeType::Normal => match operation {
    //         Operation::Intersection => !event.is_outside_other(polygon_semantics),
    //         Operation::Union => event.is_outside_other(polygon_semantics),
    //         Operation::Difference => match event.polygon_type {
    //             PolygonType::Subject => event.is_outside_other(polygon_semantics),
    //             PolygonType::Clipping => !event.is_outside_other(polygon_semantics)
    //         }
    //         Operation::Xor => true,
    //     },
    //     // If the edge is a result of a overlapping intersection:
    //     // Same bounds (upper, upper) or (lower, lower):
    //     EdgeType::SameTransition => match operation {
    //         Operation::Intersection => true,
    //         Operation::Union => true,
    //         _ => false
    //     },
    //     // Opposite bounds.
    //     EdgeType::DifferentTransition => match operation {
    //         Operation::Difference => true,
    //         _ => false
    //     },
    //     EdgeType::NonContributing => false,
    // }
}

/// Take all the events that contribute to the result.
/// This depends on the boolean operation to be performed.
/// Also adjusts the `prev` pointers for hole attribution.
fn filter_events<T>(sorted_events: &[Rc<SweepEvent<T>>],
                    operation: Operation,
                    polygon_semantics: PolygonSemantics) -> Vec<Rc<SweepEvent<T>>>
    where T: CoordinateType + Debug,
{
    // Flags that tell whether the event contributes to the result or not.
    let mut contributes = vec![false; sorted_events.len()];

    for (i, event) in sorted_events.iter().enumerate() {
        event.set_pos(i);

        let contributes_to_result = if event.is_left_event() {
            contributes_to_result(event, operation, polygon_semantics)
        } else {
            event.get_other_event().map(|other|
                contributes_to_result(other.as_ref(), operation, polygon_semantics))
                .unwrap_or(false)
        };

        contributes[i] = contributes_to_result;

        // Update the prev field for hole attribution.
        if let Some(prev) = event.get_prev().upgrade() {
            if !contributes[prev.get_pos()] {
                // The previous event is not contributing to the result, so take the previous
                // of the previous.
                event.set_prev(prev.get_prev());
                debug_assert!({
                    // If the `prev` is set now, it must be a contributing edge.
                    if let Some(prevprev) = event.get_prev().upgrade() {
                        contributes[prevprev.get_pos()]
                    } else {
                        true
                    }
                });
            }
        }
    }

    // Filter relevant events.
    let result_events: Vec<_> = sorted_events.iter()
        .zip(contributes)
        .filter(|(_e, contributes)| *contributes)
        .map(|(e, _)| e)
        .cloned()
        .collect();
    result_events
}

/// Remove duplicate edges which would form empty polygons.
fn xor_cancel_double_edges<T>(sorted_events: Vec<Rc<SweepEvent<T>>>) -> Vec<Rc<SweepEvent<T>>>
    where T: CoordinateType + Debug {
    // Flags that tell whether the event contributes to the result or not.
    let mut contributes = vec![false; sorted_events.len()];

    // Store positions.
    for (i, event) in sorted_events.iter().enumerate() {
        event.set_pos(i);
    }

    let mut prev_edge = None;
    let mut prev_idx = 0;
    for (i, event) in sorted_events.iter().enumerate() {
        let other_idx = event.get_other_event().unwrap().get_pos();

        if event.is_left_event() {
            let edge = event.get_edge();

            if Some(edge) == prev_edge {
                // Duplicate edges cancel eachother.
                prev_edge = None;
                contributes[i] = false;
                contributes[other_idx] = false; // Cancel the right event.

                // Also cancel the first edge.
                contributes[prev_idx] = false;
                let prev_other_idx = sorted_events[prev_idx].get_other_event().unwrap().get_pos();
                contributes[prev_other_idx] = false; // Cancel the right event.
            } else {
                prev_edge = Some(edge);
                prev_idx = i;
                contributes[i] = true;
                contributes[other_idx] = true;
            };
        }
    }

    // Update the prev field for hole attribution.
    for event in sorted_events.iter() {
        if let Some(prev) = event.get_prev().upgrade() {
            if !contributes[prev.get_pos()] {
                // The previous event is not contributing to the result, so take the previous
                // of the previous.
                event.set_prev(prev.get_prev());
                debug_assert!({
                    // If the `prev` is set now, it must be a contributing edge.
                    if let Some(prevprev) = event.get_prev().upgrade() {
                        contributes[prevprev.get_pos()]
                    } else {
                        true
                    }
                });
            }
        }
    }

    // Filter relevant events.
    sorted_events.into_iter()
        .zip(contributes)
        .filter(|(_e, contributes)| *contributes)
        .map(|(e, _)| e)
        .collect()
}

/// Sort the events and insert indices.
/// Input events must already be filtered such that they only contain relevant events.
fn order_events<T>(events: &mut Vec<Rc<SweepEvent<T>>>) -> Vec<Event<T>>
    where
        T: CoordinateType,
{

    // Sort the events.
    // The events are probably almost sorted.
    let mut sorted = false;
    while !sorted {
        sorted = true;
        for i in 1..events.len() {
            if events[i - 1] < events[i] {
                events.swap(i - 1, i);
                sorted = false;
            }
        }
    }

    // Check if events are sorted.
    debug_assert!(events.windows(2).all(|e| e[0] >= e[1]),
                  "Must be sorted.");

    // And check if events are sorted by coordinates too.
    debug_assert!(events.windows(2).all(|e| e[0].p <= e[1].p),
                  "Must be sorted by coordinates.");

    // Sorted by coordinates implies that end-point and start-point of two connected edges are close together in the list.
    // Further, the start-point of the second edge will come after the end-point of the first edge.

    // Tell the events what index they have.
    for (pos, event) in events.iter().enumerate() {
        event.set_pos(pos)
    }

    // Swap positions of all event pairs.
    // This way we know for each event index of the other event.
    for event in events.iter() {
        if !event.is_left_event() {
            if let Some(other) = event.get_other_event() {
                let tmp = event.get_pos();
                event.set_pos(other.get_pos());
                other.set_pos(tmp);
            }
        }
    }

    // Convert the events into a simpler data structure.
    let result = events.iter().enumerate()
        .map(|(index, event)| {
            Event {
                index,
                other_index: event.get_pos(),
                prev_index: event.get_prev().upgrade()
                    .map(|p| p.get_pos()),
                p: event.p,
                is_left_event: event.is_left_event(),
                is_hole: false,
                is_upper_boundary: false, // TODO: Is this used?
                polygon_type: event.polygon_type,
                contour_id: usize::MAX,
            }
        }
        ).collect();

    result
}

/// Given an index of an event get the index of another event with the same coordinates that is not yet
/// marked as used.
fn next_index<T: CoordinateType>(events: &[Event<T>],
                                 start_index: usize,
                                 used: &[bool]) -> Option<usize> {
    debug_assert!(start_index < events.len());
    debug_assert!(events.len() == used.len());

    let event = &events[start_index];


    // Find the next event by linear search in both directions.
    let point = event.p;

    // Search to the right.
    let next_to_the_right = events[start_index + 1..]
        .iter()
        .take_while(|e| e.p == point)
        .find(|e| !used[e.index])
        .map(|e|
            // Return the index of this event.
            e.index
        );

    if next_to_the_right.is_some() {
        next_to_the_right
    } else {
        // Search to the left.
        let next_to_the_left = events[0..start_index]
            .iter().rev()
            .take_while(|e| e.p == point)
            .find(|e| !used[e.index])
            .map(|e|
                // Return the index of this event.
                e.index
            );
        next_to_the_left
    }
}

/// Given the processed and sorted events, connect the edges to polygons.
///
/// This uses the property events at the same point lie next to each other in the list
/// of sorted events. This way it is easy to follow the contour: 1) Start at some left event,
/// 2) go to its right event, 3) from there find a event with the same location.
pub fn connect_edges<T>(sorted_events: &[Rc<SweepEvent<T>>],
                        operation: Operation,
                        polygon_semantics: PolygonSemantics) -> Vec<Polygon<T>>
    where
        T: CoordinateType + Debug,
{

    let mut relevant_events = filter_events(sorted_events, operation, polygon_semantics);

    if operation == Operation::Xor || polygon_semantics == PolygonSemantics::XOR {
        relevant_events = xor_cancel_double_edges(relevant_events);
    }

    let mut events = order_events(&mut relevant_events);

    debug_assert!(events.len() % 2 == 0, "Expect an even number of events.");

    // Store found contours.
    let mut polygons: Vec<Polygon<T>> = Vec::new();

    // Remember which events have been processed.
    let mut processed: Vec<bool> = vec![false; events.len()];

    for i in 0..events.len() {
        // Find the next unprocessed event from the left.

        if processed[i] {
            continue;
        }

        // Sanity check: There must be an even number of unprocessed events,
        // because events always come in pairs.
        debug_assert!(
            processed
                .iter()
                .filter(|&&b| !b)
                .count() % 2 == 0,
            "Expect to have an even number of non-processed events."
        );

        // Sanity check: There must be as many right events as left events among the unprocessed events.
        debug_assert!(
            (0..events.len())
                .into_iter()
                .filter_map(|i| if processed[i] { None } else {
                    if events[i].is_left_event {
                        Some(1)
                    } else {
                        Some(-1)
                    }
                })
                .fold(0, |a, b| a + b) == 0,
            "Expect to have the same amount of left and right events."
        );

        // Buffer for polygon.
        let mut contour = Vec::new();

        let mut pointer = i;
        let initial_event = &events[i];

        // Find contour index if this is a hole.

        let is_hull = initial_event.prev_index
            .map(|prev| {
                let prev_event = &events[prev];
                if prev_event.is_upper_boundary {
                    !prev_event.is_hole
                } else {
                    // A hole inside a hole is a hull.
                    prev_event.is_hole
                }
            })
            .unwrap_or(true);
        let is_hole = !is_hull;

        let polygon_id = if is_hull {
            polygons.len()
        } else {
            initial_event.prev_index
                .map(|prev| events[prev].contour_id)
                // If there is no previous segment, this is a contour and not a hole.
                // Create a new polygon id.
                .unwrap_or(polygons.len())
        };

        let initial_point = initial_event.p;
        debug_assert!(initial_event.is_left_event, "Initial event is expected to be a left event.");

        // Follow the lines until the contour is closed.
        loop {
            let other_pointer = {
                // Propagate fields to the right event.
                let e = &mut events[pointer];
                e.contour_id = polygon_id;
                e.is_hole = is_hole;
                e.other_index
            };
            {
                let other = &mut events[other_pointer];
                other.contour_id = polygon_id;
                other.is_hole = is_hole;
            }

            if events[pointer].p.x > events[other_pointer].p.x {
                // This is an upper boundary of the contour.
                events[pointer].is_upper_boundary = true;
                events[other_pointer].is_upper_boundary = true;
            }

            let event = &events[pointer];
            let other_event = &events[other_pointer];

            contour.push(event.p);

            processed[pointer] = true;
            processed[other_pointer] = true;

            debug_assert!(event.is_left_event ^ other_event.is_left_event,
                          "Need to get exactly one left event and one right event.");

            if other_event.p == initial_point {
                // Contour is closed.
                break;
            }

            // Get the start of an adjacent edge.
            if let Some(next_index) = next_index(&events, other_pointer, &processed) {
                pointer = next_index;
            } else {
                break;
            }
        }


        if polygon_id < polygons.len() {
            // Add hole to existing polygon.
            let hole = SimplePolygon::new(contour).normalized_orientation::<T>();
            polygons[polygon_id].interiors.push(hole);
        } else {
            let p = Polygon::new(contour);
            polygons.push(p);
        }
    }

    polygons
}
