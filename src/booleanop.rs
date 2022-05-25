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
use iron_shapes::multi_polygon::MultiPolygon;

use iron_shapes::CoordinateType;
use std::fmt::Debug;
use std::ops::{RangeFrom};

use crate::Operation;

use super::sweep_line::sweep_event::*;
use super::sweep_line::intersection::subdivide_segments;

use crate::connect_edges::connect_edges;

use crate::PolygonSemantics;


/// Insert the edges of the polygons into the event queue.
fn fill_queue<'a, T, S, C, Ctr>(
    subject: S,
    clipping: C,
) -> BinaryHeap<Rc<SweepEvent<T, Ctr, PolygonType>>>
    where S: Iterator<Item=&'a Polygon<T>>,
          C: Iterator<Item=&'a Polygon<T>>,
          T: CoordinateType + 'a,
          Ctr: Default {
    let mut event_queue = BinaryHeap::new();
    let mut event_id_generator = (1..).into_iter();

    /// Add edges of a polygon to the event queue.
    fn process_polygon<T: CoordinateType, Ctr: Default>(event_queue: &mut BinaryHeap<Rc<SweepEvent<T, Ctr, PolygonType>>>,
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

                let event_a = SweepEvent::new_rc_with_property(
                    edge_id,
                    edge.start,
                    edge.end,
                    event_a_is_left,
                    Weak::new(),
                    is_upper_boundary,
                    Some(polygon_type),
                );
                let event_b = SweepEvent::new_rc_with_property(
                    edge_id,
                    edge.end,
                    edge.start,
                    !event_a_is_left,
                    Rc::downgrade(&event_a),
                    is_upper_boundary,
                    Some(polygon_type),
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

#[derive(Copy, Clone, Default)]
pub struct DualCounter {
    clipping_count: i32,
    subject_count: i32,
}

/// Compute flags and fields for a segment based on its predecessor in the scan line (if there is one).
fn update_counter<T>(event: &Rc<SweepEvent<T, DualCounter, PolygonType>>,
                     maybe_prev: Option<&Rc<SweepEvent<T, DualCounter, PolygonType>>>)
    where T: CoordinateType,
{

    debug_assert!(event.is_left_event());
    let polygon_type = event.property.unwrap(); // Property must be set for all left events.

    // Update counter.
    {
        let mut updated_ctr = maybe_prev.map(|prev| {
            debug_assert_eq!(event.is_left_event(), prev.is_left_event());
            prev.with_counter(|ctr| *ctr)
        })
            .unwrap_or(Default::default());

        match polygon_type {
            PolygonType::Subject => updated_ctr.subject_count += event.edge_weight(),
            PolygonType::Clipping => updated_ctr.clipping_count += event.edge_weight(),
        }

        event.with_counter_mut(|ctr| {
            *ctr = updated_ctr;
        });
    }
}


// Check if the event is contained in the result.
fn contributes_to_result_binary_booleanop<T: CoordinateType>(event: &SweepEvent<T, DualCounter, PolygonType>, polygon_semantics: PolygonSemantics, operation: Operation) -> bool {

    debug_assert!(event.is_left_event());
    let polygon_type = event.property.unwrap(); // Property must be set for all left events.

    let (own_counter, other_counter) = {
        let counter = event.with_counter(|ctr| *ctr);

        match polygon_type {
            PolygonType::Subject => (counter.subject_count, counter.clipping_count),
            PolygonType::Clipping => (counter.clipping_count, counter.subject_count),
        }
    };

    let is_upper_boundary = match polygon_semantics {
        PolygonSemantics::Union => own_counter == 0,
        PolygonSemantics::XOR => own_counter % 2 == 0
    };

    let is_lower_boundary = {
        let w = event.edge_weight();
        w == 1 && match polygon_semantics {
            PolygonSemantics::Union => own_counter == 1,
            PolygonSemantics::XOR => own_counter % 2 == 1
        }
    };

    let is_outer_boundary = match polygon_semantics {
        PolygonSemantics::Union => is_upper_boundary || is_lower_boundary,
        PolygonSemantics::XOR => true,
    };

    let is_outside_other = match polygon_semantics {
        PolygonSemantics::Union => other_counter == 0,
        PolygonSemantics::XOR => other_counter % 2 == 0
    };

    is_outer_boundary
        &&
        match operation {
            Operation::Intersection => !is_outside_other,
            Operation::Union => is_outside_other,
            Operation::Difference => match polygon_type {
                PolygonType::Subject => is_outside_other,
                PolygonType::Clipping => !is_outside_other
            }
            Operation::Xor => true,
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
    let mut event_queue: BinaryHeap<Rc<SweepEvent<_, DualCounter, PolygonType>>> = fill_queue(
        subject.into_iter(),
        clipping.into_iter(),
    );

    // Compute the edge intersections, the result is a set of sorted non-intersecting edges stored
    // as events.
    let sorted_events = subdivide_segments(
        edge_intersection,
        &mut event_queue,
        |event, prev| update_counter(event, prev),
    );

    // Connect the edges into polygons.
    let r = connect_edges(
        &sorted_events,
        operation,
        polygon_semantics,
        |event| contributes_to_result_binary_booleanop(event, polygon_semantics, operation),
    );

    MultiPolygon::from_polygons(r)
}
