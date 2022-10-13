// SPDX-FileCopyrightText: 2022 Thomas Kramer
//
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Tests for connectivity extraction.

extern crate iron_shapes_booleanop;

#[cfg(test)]
mod booleanop {
    extern crate rand;

    use iron_shapes::prelude::*;
    use iron_shapes::traits::Translate;
    use iron_shapes_booleanop::connectivity_extraction::*;
    use iron_shapes_booleanop::*;
    use std::collections::HashSet;

    #[test]
    fn test_simple_connectivity_extraction() {
        let p0 = Polygon::from(vec![(0f64, 0.), (2., 0.), (2., 2.), (0., 2.)]);
        let p1 = p0.translate((1., 1.).into());
        let p2 = p0.translate((0., 100.).into()); // Far away, not touching anything.

        let polygons = vec![p0, p1, p2];

        // Convert the polygons into edges labelled with an identifier of their polygon.
        let polygon_edges = polygons
            .iter()
            .enumerate()
            .flat_map(|(id, polygon)| polygon.all_edges_iter().map(move |e| (e, id)));

        let connectivity_graph = extract_connectivity_graph(
            edge_intersection_float,
            polygon_edges,
            PolygonSemantics::Union,
        );

        // p0 and p1 are connected.
        assert_eq!(connectivity_graph.get(&0), Some(&HashSet::from([1])));
        assert_eq!(connectivity_graph.get(&1), Some(&HashSet::from([0])));

        // p2 is not connected to any other polygon.
        assert_eq!(connectivity_graph.get(&2), None);
    }

    #[test]
    fn test_connectivity_of_touching_polygons() {
        let p0 = Polygon::from(vec![(0f64, 0.), (2., 0.), (2., 2.), (0., 2.)]);
        let p1 = p0.translate((0., 2.).into());

        let polygons = vec![p0, p1];
        let polygon_edges = polygons
            .iter()
            .enumerate()
            .flat_map(|(id, polygon)| polygon.all_edges_iter().map(move |e| (e, id)));

        let connectivity_graph = extract_connectivity_graph(
            edge_intersection_float,
            polygon_edges,
            PolygonSemantics::Union,
        );

        assert_eq!(connectivity_graph.get(&0), Some(&HashSet::from([1])));
        assert_eq!(connectivity_graph.get(&1), Some(&HashSet::from([0])));
    }

    #[test]
    fn test_connectivity_of_corner_kissing_polygons() {
        let p0 = Polygon::from(vec![(0f64, 0.), (2., 0.), (2., 2.), (0., 2.)]);
        let p1 = p0.translate((2., 2.).into());

        let polygons = vec![p0, p1];
        let polygon_edges = polygons
            .iter()
            .enumerate()
            .flat_map(|(id, polygon)| polygon.all_edges_iter().map(move |e| (e, id)));

        let connectivity_graph = extract_connectivity_graph(
            edge_intersection_float,
            polygon_edges,
            PolygonSemantics::Union,
        );

        assert_eq!(connectivity_graph.get(&0), None);
        assert_eq!(connectivity_graph.get(&1), None);
    }
}
