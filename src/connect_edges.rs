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

#[derive(Debug, Clone, PartialEq)]
struct Event<T: CoordinateType> {
    index: usize,
    other_index: usize,
    p: Point<T>,
    is_hull: bool,
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
fn contributes_to_result<T>(event: &SweepEvent<T>, operation: Operation) -> bool
    where T: CoordinateType,
{
    debug_assert!(event.is_left_event());
    // TODO: check correctness.
    match event.get_edge_type() {
        EdgeType::Normal => match operation {
            Operation::Intersection => !event.is_outside_other(),
            Operation::Union => event.is_outside_other(),
            Operation::Difference => match event.polygon_type {
                PolygonType::Subject => event.is_outside_other(),
                PolygonType::Clipping => !event.is_outside_other()
            }
            Operation::Xor => true,
        },
        // If the edge is a result of a overlapping intersection:
        // Same bounds (upper, upper) or (lower, lower):
        EdgeType::SameTransition => match operation {
            Operation::Intersection => true,
            Operation::Union => true,
            _ => false
        },
        // Opposite bounds.
        EdgeType::DifferentTransition => match operation {
            Operation::Difference => true,
            _ => false
        },
        EdgeType::NonContributing => false,
    }
}

/// Take all the events that contribute to the result.
/// This depends on the boolean operation to be performed.
fn filter_events<T>(sorted_events: &[Rc<SweepEvent<T>>], operation: Operation) -> Vec<Rc<SweepEvent<T>>>
    where
        T: CoordinateType + Debug,
{
    // Filter relevant events.
    let result_events: Vec<_> = sorted_events.iter()
        .filter(|event|
            if event.is_left_event() {
                contributes_to_result(event, operation)
            } else {
                event.get_other_event().map(|o|
                    contributes_to_result(o.as_ref(), operation))
                    .unwrap_or(false)
            }
        )
        .cloned()
        .collect();
    result_events
}

/// Sort the events and insert indices.
/// Input events must already be filtered such that they only contain relevant events.
fn order_events<T>(events: &mut Vec<Rc<SweepEvent<T>>>) -> Vec<Event<T>>
    where
        T: CoordinateType,
{

    // Sort the events.
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
                p: event.p,
                is_left_event: event.is_left_event(),
                is_hull: event.is_hull,
                polygon_type: event.polygon_type,
                contour_id: event.contour_id,
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
pub fn connect_edges<T>(sorted_events: &[Rc<SweepEvent<T>>], operation: Operation) -> Vec<Polygon<T>>
    where
        T: CoordinateType + Debug,
{
    let mut relevant_events = filter_events(sorted_events, operation);

    let events = order_events(&mut relevant_events);

    debug_assert!(events.len() % 2 == 0, "Expect an even number of events.");

    let mut polygons: Vec<Polygon<T>> = Vec::new();
    let mut processed: Vec<bool> = vec![false; events.len()];

    for i in 0..events.len() {
        if processed[i] {
            continue;
        }

        // Sanity check.
        debug_assert!(
            processed
                .iter()
                .filter(|&&b| !b)
                .count() % 2 == 0,
            "Expect to have an even number of non-processed events." // Because events always come in couples.
        );

        // Sanity check.
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
        let initial_point = initial_event.p;
        debug_assert!(initial_event.is_left_event, "Initial event is expected to be a left event.");

        // Follow the lines until the contour is closed.
        loop {
            let event = &events[pointer];
            let other_pointer = event.other_index;
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
        polygons.push(Polygon::new(contour));
//
//        let index = polygons.iter().enumerate()
//            .find(|(_pos, poly)|
//                contour.iter()
//                    .any(|&p| poly.exterior.contains_point(p))
//            ).map(|(pos, _)| pos);
//        if let Some(index) = index {
//            polygons[index].interiors.push(SimplePolygon::new(contour))
//        } else {
//            polygons.push(Polygon::new(contour));
//        }

//        // Decide whether the found contour is a hull or a hole.
//        if !initial_event.is_hull {
//            if polygons.is_empty() {
//                polygons.push(Polygon::new(contour));
//            } else {
//                // TODO
//            }
//        } else if operation == Operation::Difference
//            && initial_event.polygon_type == PolygonType::Clipping {
//            debug_assert!(polygons.len() > 1, "Expect to already have a hull polygon.");
//            // This is a hole.
//            // TODO
//        } else {
//            // This is an exterior contour of a polygon.
//            polygons.push(Polygon::new(contour));
//        }


//        if !events[i].is_hull {
//            if result.is_empty() {
//                result.push(Polygon::new(contour));
//            } else {
//                result
//                    .last_mut()
//                    .expect("Result must not be empty at this point")
//                    .interiors
//                    .push(SimplePolygon::new(contour));
//            }
//        } else if operation == Operation::Difference
//            && events[i].polygon_type == PolygonType::Clipping
//            && result.len() > 1 {
//            result
//                .last_mut()
//                .expect("Result must not be empty at this point")
//                .interiors
//                .push(SimplePolygon::new(contour));
//        } else {
//            result.push(Polygon::new(contour));
//        }
    }

    polygons
}
