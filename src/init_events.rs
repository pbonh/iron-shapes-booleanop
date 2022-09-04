// SPDX-FileCopyrightText: 2022 Thomas Kramer
//
// SPDX-License-Identifier: AGPL-3.0-or-later

use iron_shapes::edge::EdgeEndpoints;
use std::collections::BinaryHeap;
use std::rc::{Rc, Weak};

use iron_shapes::prelude::{CoordinateType};

use crate::sweep_line::sweep_event::SweepEvent;

/// Insert the edges of the polygons into the event queue.
pub fn fill_queue<Coord, Ctr, Edge, Property>(edges: impl Iterator<Item=(Edge, Property)>) -> BinaryHeap<Rc<SweepEvent<Coord, Ctr, Property>>>
    where Coord: CoordinateType,
          Edge: EdgeEndpoints<Coord>,
          Ctr: Default,
          Property: Clone, {
    let mut event_queue = BinaryHeap::new();
    let mut event_id_generator = (1..).into_iter();

    // Add edges of a polygon to the event queue.

    for (edge, property) in edges {
        // Skip degenerate edges.
        let is_not_degenerate = edge.start() != edge.end();
        if is_not_degenerate {
            let edge_id = event_id_generator.next().unwrap();
            let event_a_is_left = edge.start() < edge.end();

            // Upper boundary edges are directed from right to left.
            let is_upper_boundary = edge.end() < edge.start();

            let (property_a, property_b) = if event_a_is_left {
                (Some(property), None)
            } else {
                (None, Some(property))
            };

            let event_a = SweepEvent::new_rc_with_property(
                edge_id,
                edge.start(),
                edge.end(),
                event_a_is_left,
                Weak::new(),
                is_upper_boundary,
                property_a,
            );

            let event_b = SweepEvent::new_rc_with_property(
                edge_id,
                edge.end(),
                edge.start(),
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


    event_queue
}