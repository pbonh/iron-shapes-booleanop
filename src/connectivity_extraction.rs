// SPDX-FileCopyrightText: 2022 Thomas Kramer
//
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Extract the connectivity graph of polygons.

use iron_shapes::prelude::*;

use std::fmt::Debug;

use crate::sweep_line::intersection::subdivide_segments;
use crate::sweep_line::sweep_event::SweepEvent;
use crate::PolygonSemantics;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::hash::Hash;
use std::rc::Rc;

/// Extract a connectivity of a set of polygons.
/// The polygons are supplied in the form of an interator over the polygon edges. If the supplied edges are inconsistent, e.g. don't form closed polygon shapes,
/// then the result is undefined.
///
/// Each edge must be labelled with the identifier of the polygon it belongs to.
/// Connectivity is encoded as an iterator over multi-graph edges.
/// Each consists of the IDs of the two touching polygons and a location where the both polygons touch.
/// Edges can appear many times.
///
/// # Examples
/// ```
/// use iron_shapes::prelude::*;
/// use iron_shapes::traits::Translate;
/// use iron_shapes_booleanop::connectivity_extraction::*;
/// use iron_shapes_booleanop::*;
/// use std::collections::HashSet;
///
/// let p0 = Polygon::from(vec![(0f64, 0.), (2., 0.), (2., 2.), (0., 2.)]);
/// let p1 = p0.translate((1., 1.).into());
/// let p2 = p0.translate((0., 100.).into()); // Far away, not touching anything.
///
/// let polygons = vec![p0, p1, p2];
///
/// // Convert the polygons into edges labelled with an identifier of their polygon.
/// let polygon_edges = polygons
///     .iter()
///     .enumerate()
///     .flat_map(|(id, polygon)| polygon.all_edges_iter().map(move |e| (e, id)));
///
/// let connectivity_graph = extract_connectivity_graph(
///     edge_intersection_float,
///     polygon_edges,
///     PolygonSemantics::Union,
/// );
///
/// // p0 and p1 are connected.
/// assert_eq!(connectivity_graph.get(&0), Some(&HashSet::from([1])));
/// assert_eq!(connectivity_graph.get(&1), Some(&HashSet::from([0])));
///
/// // p2 is not connected to any other polygon.
/// assert_eq!(connectivity_graph.get(&2), None);
/// ```
pub fn extract_connectivity<'a, I, T, ID>(
    edge_intersection: I,
    polygon_edges: impl Iterator<Item = (Edge<T>, ID)>,
    polygon_semantics: PolygonSemantics,
) -> impl Iterator<Item = (ID, ID, Point<T>)>
where
    I: Fn(&Edge<T>, &Edge<T>) -> EdgeIntersection<T, T, Edge<T>>,
    T: CoordinateType + Debug + 'a,
    ID: Clone + Hash + Eq,
{
    // Prepare the event queue.
    let mut event_queue: BinaryHeap<Rc<SweepEvent<_, Counter<ID>, ID>>> =
        crate::init_events::fill_queue(polygon_edges);

    // Compute the edge intersections, the result is a set of sorted non-intersecting edges stored
    // as events.
    let sorted_events = subdivide_segments(edge_intersection, &mut event_queue, |event, prev| {
        update_counter(event, prev)
    });

    // Extract connectivity graph.
    {
        // Take left events only.
        let left_events = sorted_events.into_iter().filter(|e| e.is_left_event());

        // Check if an edge count signals the inside or the outside of a polygon.
        let is_inside = move |count: i32| -> bool {
            match polygon_semantics {
                PolygonSemantics::Union => count != 0,
                PolygonSemantics::XOR => count % 2 != 0,
            }
        };

        // Create iterator of graph edges.
        // Edges may appear multiple times. Each edge comes with a point which indicates the location of the connectivity.
        let multi_graph_edges = left_events.flat_map(move |current_event| {
            // Get connectivity of this event.

            debug_assert!(current_event.is_left_event());
            let shape_id = current_event.property.as_ref().unwrap().clone(); // Left event must have the property.
            let point = current_event.p; // Location of connectivity.

            // Take counter from event to own it.
            // This destroys the structure of the sweep events.
            let counter =
                current_event.with_counter_mut(|ctr| std::mem::replace(ctr, Default::default()));

            // Look at edge counts and find shapes enclosing the current event.
            let shape_id_clone = shape_id.clone();
            let connected_ids = counter
                .counters
                .into_iter()
                .filter(move |(id, _count)| id != &shape_id_clone) // Look only at other polygons.
                .filter(move |(_id, count)| is_inside(*count))
                .map(|(id, _)| id);

            connected_ids.map(move |id| (shape_id.clone(), id, point))
        });

        multi_graph_edges
    }
}

/// Extract a connectivity graph of a set of polygons.
pub fn extract_connectivity_graph<'a, I, T, ID>(
    edge_intersection: I,
    polygon_edges: impl Iterator<Item = (Edge<T>, ID)>,
    polygon_semantics: PolygonSemantics,
) -> HashMap<ID, HashSet<ID>>
where
    I: Fn(&Edge<T>, &Edge<T>) -> EdgeIntersection<T, T, Edge<T>>,
    T: CoordinateType + Debug + 'a,
    ID: Clone + Hash + Eq,
{
    let multi_graph_edges =
        extract_connectivity(edge_intersection, polygon_edges, polygon_semantics);

    let mut graph: HashMap<ID, HashSet<ID>> = Default::default();
    // Insert graph edges.
    for (id_a, id_b, _point) in multi_graph_edges {
        graph
            .entry(id_a.clone())
            .or_insert(Default::default())
            .insert(id_b.clone());
        // Insert reverse edge.
        graph.entry(id_b).or_insert(Default::default()).insert(id_a);
    }

    graph
}

#[derive(Clone)]
struct Counter<K> {
    counters: HashMap<K, i32>,
}

impl<K> Default for Counter<K> {
    fn default() -> Self {
        Self {
            counters: Default::default(),
        }
    }
}

fn update_counter<T, ID>(
    event: &Rc<SweepEvent<T, Counter<ID>, ID>>,
    maybe_prev: Option<&Rc<SweepEvent<T, Counter<ID>, ID>>>,
) where
    T: CoordinateType,
    ID: Clone + Hash + Eq,
{
    debug_assert!(event.is_left_event());

    // Update counter.
    event.with_counter_mut(|ctr| {
        let edge_id = event.property.as_ref().unwrap(); // Property must be set for all left events.

        // Clone counters of previous edge.
        let prev_counter = maybe_prev
            .as_ref()
            .map(|prev| {
                debug_assert!(prev.is_left_event());
                prev.with_counter(|ctr| ctr.clone())
            })
            .unwrap_or(Default::default());

        *ctr = prev_counter;

        // Increment counter.
        {
            let edge_weight = event.edge_weight();
            let current_counter = ctr.counters.entry(edge_id.clone()).or_insert(0);
            *current_counter += edge_weight;

            // Remove counter if it is zero.
            if *current_counter == 0 {
                ctr.counters.remove(edge_id);
            };
        }
    });
}
