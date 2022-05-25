// SPDX-FileCopyrightText: 2022 Thomas Kramer
//
// SPDX-License-Identifier: AGPL-3.0-or-later

use std::collections::BinaryHeap;
use std::rc::{Rc, Weak};
use std::ops::RangeFrom;

use iron_shapes::prelude::{CoordinateType, SimplePolygon, Polygon};

use crate::sweep_line::sweep_event::SweepEvent;

/// Insert the edges of the polygons into the event queue.
pub fn fill_queue<'a, T, PolygonIter, Ctr, Property>(polygons: PolygonIter) -> BinaryHeap<Rc<SweepEvent<T, Ctr, Property>>>
    where PolygonIter: Iterator<Item=(&'a Polygon<T>, Property)>,
          T: CoordinateType + 'a,
          Ctr: Default,
          Property: Clone, {
    let mut event_queue = BinaryHeap::new();
    let mut event_id_generator = (1..).into_iter();

    /// Add edges of a polygon to the event queue.
    fn process_polygon<T, Ctr, P>(event_queue: &mut BinaryHeap<Rc<SweepEvent<T, Ctr, P>>>,
                                  poly: &SimplePolygon<T>,
                                  property: P,
                                  event_id_generator: &mut RangeFrom<usize>, )
        where T: CoordinateType,
              Ctr: Default,
              P: Clone {
        for edge in poly.edges() {
            // Skip degenerate edges.
            if !edge.is_degenerate() {
                let edge_id = event_id_generator.next().unwrap();
                let event_a_is_left = edge.start < edge.end;

                // Upper boundary edges are directed from right to left.
                let is_upper_boundary = edge.end < edge.start;

                let (property_a, property_b) = if event_a_is_left {
                    (Some(property.clone()), None)
                } else {
                    (None, Some(property.clone()))
                };

                let event_a = SweepEvent::new_rc_with_property(
                    edge_id,
                    edge.start,
                    edge.end,
                    event_a_is_left,
                    Weak::new(),
                    is_upper_boundary,
                    property_a,
                );

                let event_b = SweepEvent::new_rc_with_property(
                    edge_id,
                    edge.end,
                    edge.start,
                    !event_a_is_left,
                    Rc::downgrade(&event_a),
                    is_upper_boundary,
                    property_b,
                );

                event_a.set_other_event(&event_b);

                event_queue.push(event_a);
                event_queue.push(event_b);
            }
        }
    }

    // Process all polygons
    for (polygon, property) in polygons {
        process_polygon(&mut event_queue, &polygon.exterior, property.clone(), &mut event_id_generator);
        for i in &polygon.interiors {
            // Holes
            process_polygon(&mut event_queue, i, property.clone(), &mut event_id_generator);
        }
    }

    event_queue
}