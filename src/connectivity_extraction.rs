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

        // Take left events only.
        let left_events = sorted_events.into_iter()
            .filter(|e| e.is_left_event());

        // Check if an edge count signals the inside or the outside of a polygon.
        let is_inside = |count: i32| -> bool {
            match polygon_semantics {
                PolygonSemantics::Union => count != 0,
                PolygonSemantics::XOR => count % 2 != 0
            }
        };


        // Create iterator of graph edges.
        // Edges may appear multiple times.
        let multi_graph_edges = left_events
            .flat_map(|current_event| {
                debug_assert!(current_event.is_left_event());
                let shape_id = current_event.property.as_ref().unwrap().clone(); // Left event must have the property.

                // Take counter from event to own it.
                // This destroys the structure of the sweep events.
                let counter = current_event.with_counter_mut(|ctr| {
                    std::mem::replace(ctr, Default::default())
                });

                // Look at edge counts and find shapes enclosing the current event.
                let shape_id_clone = shape_id.clone();
                let connected_ids = counter.counters.into_iter()
                    .filter(move |(id, _count)| id != &shape_id_clone) // Look only at other polygons.
                    .filter(|(_id, count)| is_inside(*count))
                    .map(|(id, _)| id);

                connected_ids
                    .map(move |id| (shape_id.clone(), id))
            });

        // Insert graph edges.
        for (id_a, id_b) in multi_graph_edges {
            graph.entry(id_a.clone())
                .or_insert(Default::default())
                .insert(id_b.clone());
            // Insert reverse edge.
            graph.entry(id_b)
                .or_insert(Default::default())
                .insert(id_a);
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