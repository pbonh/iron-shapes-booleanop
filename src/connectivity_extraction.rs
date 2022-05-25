// SPDX-FileCopyrightText: 2022 Thomas Kramer
//
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Extract the connectivity graph of polygons.

use iron_shapes::prelude::*;

use std::fmt::Debug;

use crate::PolygonSemantics;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::rc::Rc;
use std::hash::Hash;
use crate::sweep_line::sweep_event::SweepEvent;
use crate::sweep_line::intersection::subdivide_segments;

/// Extract a connectivity graph of a set of polygons.
pub fn extract_connectivity<'a, I, T, Polygons, ID>(
    edge_intersection: I,
    polygons: Polygons,
    polygon_semantics: PolygonSemantics,
) -> HashMap<ID, HashSet<ID>>
    where I: Fn(&Edge<T>, &Edge<T>) -> EdgeIntersection<T, T, Edge<T>>,
          T: CoordinateType + Debug + 'a,
          Polygons: IntoIterator<Item=(ID, &'a Polygon<T>)>,
          ID: Clone + Hash + Eq
{

    // Prepare the event queue.
    let mut event_queue: BinaryHeap<Rc<SweepEvent<_, Counter<ID>, ID>>> = crate::init_events::fill_queue(
        polygons.into_iter()
            .map(|(id, polygon)| (polygon, id))
    );

    // Compute the edge intersections, the result is a set of sorted non-intersecting edges stored
    // as events.
    let sorted_events = subdivide_segments(
        edge_intersection,
        &mut event_queue,
        |event, prev| update_counter(event, prev),
    );

    // Extract connectivity graph.
    {
        let mut graph: HashMap<ID, HashSet<ID>> = Default::default();

        let left_events = sorted_events.into_iter()
            .filter(|e| e.is_left_event());

        let is_inside = |count: i32| -> bool {
            match polygon_semantics {
                PolygonSemantics::Union => count != 0,
                PolygonSemantics::XOR => count % 2 != 0
            }
        };

        // Create graph edges.
        for current_event in left_events {
            debug_assert!(current_event.is_left_event());
            let shape_id = current_event.property.as_ref().unwrap(); // Left event must have the property.

            current_event.with_counter(|ctr| {

                // Look at edge counts and find shapes enclosing the current event.
                let connected_ids = ctr.counters.iter()
                    .filter(|(id, _count)| id != &shape_id) // Look only at other polygons.
                    .filter(|(_id, count)| is_inside(**count))
                    .map(|(id, _)| id);

                // Create graph edges.
                for connected_id in connected_ids {
                    graph.entry(shape_id.clone())
                        .or_insert(Default::default())
                        .insert(connected_id.clone());
                    // Insert reverse edge.
                    graph.entry(connected_id.clone())
                        .or_insert(Default::default())
                        .insert(shape_id.clone());
                }
            });
        }

        graph
    }
}

#[derive(Clone)]
struct Counter<K> {
    counters: HashMap<K, i32>
}

impl<K> Default for Counter<K> {
    fn default() -> Self {
        Self {
            counters: Default::default()
        }
    }
}


fn update_counter<T, ID>(event: &Rc<SweepEvent<T, Counter<ID>, ID>>,
                         maybe_prev: Option<&Rc<SweepEvent<T, Counter<ID>, ID>>>)
    where T: CoordinateType,
          ID: Clone + Hash + Eq
{
    debug_assert!(event.is_left_event());


    // Update counter.
    event.with_counter_mut(|ctr| {
        let edge_id = event.property.as_ref().unwrap(); // Property must be set for all left events.

        // Clone counters of previous edge.
        let prev_counter = maybe_prev.as_ref().map(|prev| {
            debug_assert!(prev.is_left_event());
            prev.with_counter(|ctr| ctr.clone())
        })
            .unwrap_or(Default::default());

        *ctr = prev_counter;

        // Increment counter.
        {
            let edge_weight = event.edge_weight();
            let current_counter = ctr.counters.entry(edge_id.clone())
                .or_insert(0);
            *current_counter += edge_weight;

            // Remove counter if it is zero.
            if *current_counter == 0 {
                ctr.counters.remove(edge_id);
            };
        }
    });
}