// Copyright (c) 2018-2021 Thomas Kramer.
// SPDX-FileCopyrightText: 2022 Thomas Kramer
//
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Tests for boolean operations.

#[cfg(test)]
mod test {
    extern crate rand;

    use iron_shapes_booleanop::*;
    use iron_shapes::prelude::*;
    use iron_shapes::traits::{Translate, Scale};
    use num_rational::Rational64;
    use self::rand::distributions::{Uniform, Distribution};
    use self::rand::rngs::StdRng;
    use self::rand::SeedableRng;

    #[test]
    fn test_boolean_op_simple() {
        // Union of two well formed rectilinear polygons.

        let p1 = Polygon::from(vec![(0., 0.), (2., 0.), (2., 2.), (0., 2.)]);
        let p2 = p1.translate((1., 1.).into());
        // let p1 = Polygon::from(vec![(1., 0.), (2., 1.), (1., 2.), (0., 1.)]);
        // let p2 = p1.translate((1., 0.).into());

        let expected_union = Polygon::from(vec![(0., 0.), (2., 0.), (2., 1.), (3., 1.),
                                                (3., 3.), (1., 3.), (1., 2.), (0., 2.)]);

        let i = p1.union(&p2);

        assert_eq!(i.len(), 1);
        assert_eq!(i.polygons[0], expected_union);
    }

    #[test]
    fn test_simple_union_integer() {
        // Polygon union in integer coordinates.
        let p1 = Polygon::from(vec![(0, 0), (2, 0), (2, 2), (0, 2)]);
        let p2 = p1.translate((1, 1).into());

        let expected_union = Polygon::from(vec![(0, 0), (2, 0), (2, 1), (3, 1),
                                                (3, 3), (1, 3), (1, 2), (0, 2)]);

        let i = boolean_op(
            edge_intersection_integer, &[p1], &[p2],
            Operation::Union,
            PolygonSemantics::XOR);

        assert_eq!(i.len(), 1);
        assert_eq!(i.polygons[0], expected_union);
    }

    #[test]
    fn test_degenerate_polygons() {
        // Polygons with zero area.

        let p1 = Polygon::from(vec![(0., 0.), (1., 1.)]);
        let p2 = Polygon::from(vec![(1., 0.), (0., 1.)]);

        let i = p1.intersection(&p2);

        assert_eq!(i.len(), 0);
    }

    #[test]
    fn test_boolean_op_same_polygon() {
        // Union of two identical polygons.
        let p1 = Polygon::from(vec![(0., 0.), (2., 0.), (2., 2.)]);

        let i = boolean_op(
            edge_intersection_float, vec![&p1], vec![&p1], Operation::Union,
            PolygonSemantics::XOR);

        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);
    }


    #[test]
    fn test_boolean_op_duplicate_union_semantics() {

        // Use the same polygon multiple times as a subject or clipping polygon.
        // Union semantics should reduce multiple overlapping polygons into one polygon.

        let p1 = Polygon::from(vec![(0., 0.), (2., 0.), (2., 2.)]);

        let i = boolean_op(
            edge_intersection_float, vec![], vec![&p1, &p1],
            Operation::Union,
            PolygonSemantics::Union);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);

        let i = boolean_op(
            edge_intersection_float, vec![&p1, &p1], vec![],
            Operation::Union,
            PolygonSemantics::Union);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);

        let i = boolean_op(
            edge_intersection_float, vec![&p1], vec![&p1],
            Operation::Union,
            PolygonSemantics::Union);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);

        let i = boolean_op(
            edge_intersection_float, vec![&p1, &p1], vec![&p1],
            Operation::Union,
            PolygonSemantics::Union);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);

        let i = boolean_op(
            edge_intersection_float, vec![&p1], vec![&p1, &p1],
            Operation::Union,
            PolygonSemantics::Union);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);

        let i = boolean_op(
            edge_intersection_float, vec![&p1, &p1], vec![&p1, &p1],
            Operation::Union,
            PolygonSemantics::Union);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);

        let i = boolean_op(
            edge_intersection_float, vec![&p1, &p1, &p1, &p1, &p1], vec![&p1, &p1, &p1],
            Operation::Union,
            PolygonSemantics::Union);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);
    }

    #[test]
    fn test_xor_same_polygon() {
        // XOR of multiple identical polygons.
        let p1 = Polygon::from(vec![(0., 0.), (2., 0.), (2., 2.)]);

        let i = boolean_op(
            edge_intersection_float, vec![&p1], vec![],
            Operation::Xor,
            PolygonSemantics::XOR);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);

        let i = boolean_op(
            edge_intersection_float, vec![], vec![&p1],
            Operation::Xor,
            PolygonSemantics::XOR);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);

        let i = boolean_op(
            edge_intersection_float, vec![&p1], vec![&p1],
            Operation::Xor,
            PolygonSemantics::XOR);
        assert_eq!(i.len(), 0);

        let i = boolean_op(
            edge_intersection_float, vec![&p1, &p1], vec![],
            Operation::Xor,
            PolygonSemantics::XOR);
        assert_eq!(i.len(), 0);

        let i = boolean_op(
            edge_intersection_float, vec![], vec![&p1, &p1],
            Operation::Xor,
            PolygonSemantics::XOR);
        assert_eq!(i.len(), 0);

        let i = boolean_op(
            edge_intersection_float, vec![&p1, &p1], vec![&p1],
            Operation::Xor,
            PolygonSemantics::XOR);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);

        let i = boolean_op(
            edge_intersection_float, vec![&p1], vec![&p1, &p1],
            Operation::Xor,
            PolygonSemantics::XOR);
        assert_eq!(i.len(), 1);
        assert_eq!(&i.polygons[0], &p1);
    }


    #[test]
    fn test_holes() {

        // Test the hole attribution (assignment of holes to the correct polygon).

        let little_square = Polygon::from(vec![(0., 0.), (1., 0.), (1., 1.), (0., 1.)]);
        let big_square = little_square.scale(4.);
        let little_square_outside = little_square.translate((1., 5.).into());
        let little_square_inside = little_square.translate((2., 1.).into());

        let i = boolean_op(
            edge_intersection_float,
            &[big_square],
            &[little_square_inside, little_square_outside],
            Operation::Xor,
            PolygonSemantics::XOR,
        );

        assert!(i.polygons.iter().any(|p| p.interiors.len() == 1));
        assert!(i.polygons.iter().any(|p| p.interiors.len() == 0));

        assert!(i.contains_point((0.1, 0.1).into()));
        assert!(!i.contains_point((2.1, 1.1).into()));
        assert!(i.contains_point((1.1, 5.1).into()));
        assert!(!i.contains_point((100., 100.).into()));
    }


    #[test]
    fn test_separate_polygons_or_hole() {
        let little_square = Polygon::from(vec![(0., 0.), (1., 0.), (1., 1.), (0., 1.)]);
        let square1 = little_square.scale(2.);
        let square2 = square1.translate((1., 1.).into());

        let result = boolean_op(
            edge_intersection_float,
            vec![&square1],
            vec![&square2],
            Operation::Xor,
            PolygonSemantics::XOR,
        );

        let probe_points = [(0., 0.), (0.5, 0.5), (1., 1.), (1.5, 1.5), (2.5, 2.5)];

        // Create grid of probe points.
//        let n = 32;
//        let scale = 3.0 * n as f64;
//        let grid = (0..n).into_iter()
//            .flat_map(|x|
//                (0..n).into_iter()
//                    .map(move |y| (x,y))
//            ).map(|(x,y)| (x as f64 * scale, y as f64 * scale));

        for p in &probe_points {
            let p = p.into();
            let in1 = square1.contains_point(p);
            let in2 = square2.contains_point(p);

            let expected = in1 ^ in2; // Xor

            assert_eq!(result.contains_point(p), expected);
        }
    }


    #[test]
    fn test_rational_collinear_edge() {
        // The edges on the x-axis are collinear and overlap.

        let rp = |a: i64, b: i64| Point::new(Rational64::from(a), Rational64::from(b));
        let r = |a: i64, b: i64| Rational64::new(a, b);

        let a = Polygon::from(vec![rp(0, 0), rp(5, 0), rp(3, 1)]);
        let b = Polygon::from(vec![rp(1, 0), rp(5, 0), rp(3, 2)]);

        let result = boolean_op(
            edge_intersection_rational,
            vec![&a],
            vec![&b],
            Operation::Intersection,
            PolygonSemantics::XOR,
        );

        let expected = Polygon::from(vec![rp(1, 0), rp(5, 0),
                                          rp(3, 1), (r(3, 2), r(1, 2)).into()]);

        assert_eq!(result.polygons[0], expected);
    }

    #[test]
    fn test_rational_collinear_edge_2() {
        // The edges on the x-axis are collinear and overlap.

        let rp = |a: i64, b: i64| Point::new(Rational64::from(a), Rational64::from(b));
//        let rp = |a: i64, b: i64| Point::new(a, b);
        let r = |a: i64, b: i64| Rational64::new(a, b);

        let scale = r(1, 1);
        let a = Polygon::from(vec![rp(3, 2), rp(7, 6), rp(2, 5)]).scale(scale);
        let b = Polygon::from(vec![rp(1, 0), rp(7, 6), rp(3, 3)]).scale(scale);

        let result = boolean_op(
            edge_intersection_rational,
//            edge_intersection_integer,
            vec![&a],
            vec![&b],
            Operation::Intersection,
            PolygonSemantics::XOR,
        );

        dbg!(&result);

//        let expected = Polygon::from(vec![rp(1, 0), rp(5, 0),
//                                          rp(3, 1), (r(3,2), r(1,2)).into()]);

        assert_eq!(result.polygons.len(), 1);
        assert_eq!(result.polygons[0].len(), 4);
    }

    #[test]
    fn test_integer_collinear_edge() {
        // The edges on the x-axis are collinear and overlap.

        let p = |a: i64, b: i64| Point::new(a, b);

        let a = Polygon::from(vec![p(0, 0), p(10, 0), p(6, 2)]);
        let b = Polygon::from(vec![p(2, 0), p(10, 0), p(6, 4)]);

        let result = boolean_op(
            edge_intersection_integer,
            vec![&a],
            vec![&b],
            Operation::Intersection,
            PolygonSemantics::XOR,
        );

        let expected = Polygon::from(vec![p(2, 0), p(10, 0),
                                          p(6, 2), p(3, 1).into()]);

        assert_eq!(result.polygons.len(), 1);
        assert_eq!(result.polygons[0], expected);
    }

    #[test]
    fn test_multipolygon_intersection() {
        // Intersection of a vertical stripe with two horizontal stripes.
        // This used to trigger a bug: Polygons where falsely recognized as holes.

        let p = |a: i64, b: i64| Point::new(a, b);

        let horizontal_stripe = Polygon::from(vec![p(0, 0), p(10, 0), p(10, 1), p(0, 1)]);
        let vertical_stripe = Polygon::from(vec![p(0, -1), p(1, -1), p(1, 11), p(0, 11)]);
        let horizontal_stripe2 = horizontal_stripe.translate((0, 4).into());

        let result = boolean_op(
            edge_intersection_integer,
            vec![&horizontal_stripe, &horizontal_stripe2],
            vec![&vertical_stripe],
            Operation::Intersection,
            PolygonSemantics::Union,
        );

        // assert_eq!(result.len(), 2);

        for p in result.polygons {
            dbg!(&p);
            assert_eq!(p.interiors.len(), 0)
        }
    }

    #[test]
    fn test_x_overlapping_rectangles() {
        // Intersection of two rectangles which have their right boundaries aligned.

        let a = Rect::new((0, 0), (10, 10)).to_polygon();
        let b = Rect::new((5, 0), (10, 5)).to_polygon();

        let subject = vec![&a, &b];


        let result = boolean_op(
            edge_intersection_integer,
            subject,
            vec![],
            Operation::Union,
            PolygonSemantics::Union,
        );
        dbg!(&result);

        assert_eq!(result.len(), 1);

        for p in result.polygons {
            dbg!(&p);
            assert_eq!(p.exterior.len(), 6);
            assert_eq!(p.interiors.len(), 0);
        }
    }

    #[test]
    fn test_vertical_degenerate_polygon() {
        // The edges on the x-axis are collinear and overlap.

        let p = |a: i64, b: i64| Point::new(Rational64::from(a), Rational64::from(b));

        let a = Polygon::from(vec![p(0, 0), p(0, 2)]);
        let b = Polygon::from(vec![p(0, 1), p(1, 1), p(1, 2)]);

        let result = boolean_op(
            edge_intersection_rational,
            vec![&a],
            vec![&b],
            Operation::Intersection,
            PolygonSemantics::XOR,
        );

        dbg!(&result);

        assert_eq!(result.polygons.len(), 0);
    }

    #[test]
    fn test_rational_collinear_edge3() {
        // The edges on the x-axis are collinear and overlap.

        let p = |a: i64, b: i64| Point::new(Rational64::from(a), Rational64::from(b));

        // Some random translation
        let t = Vector::new(Rational64::new(0, 1), Rational64::new(0, 1));

        let a = Polygon::from(vec![p(0, 0), p(11, 11), p(4, 8)]).translate(t);
        let b = Polygon::from(vec![p(4, 4), p(11, 11), p(1, 10)]).translate(t);

        let result = boolean_op(
//            edge_intersection_integer,
edge_intersection_rational,
vec![&a],
vec![&b],
Operation::Intersection,
PolygonSemantics::XOR,
        );

        let expected = Polygon::from(vec![p(4, 4), p(11, 11),
                                          p(4, 8), p(3, 6).into()]).translate(t);

        assert_eq!(result.polygons[0], expected);
    }

    #[test]
    fn test_polygon_with_vertical_zero_width_part() {

        let a = Polygon {
            exterior: SimplePolygon {
                points: vec![
                    Point::new(1., 0.),
                    Point::new(2., 2.),
                    Point::new(0., 3.),
                    Point::new(0., 1.),
                ],
            },
            interiors: vec![],
        };

        let b = Polygon {
            exterior: SimplePolygon {
                points: vec![
                    Point::new(0., 2.),
                    Point::new(2., 0.),
                    Point::new(0., 1.),
                    Point::new(0., 3.), // Zero-width spike.
                ],
            },
            interiors: vec![],
        };

        let intersection =  boolean_op(
            edge_intersection_float,
            vec![&a],
            vec![&b],
            Operation::Intersection,
            PolygonSemantics::XOR,
        );

        dbg!(&intersection);

        let p = Point::new(1.06, 0.83);
        assert!(a.contains_point(p));
        assert!(b.contains_point(p));
        assert!(intersection.contains_point(p));
    }

    /// Test boolean operations on random polygons in rational coordinates.
    #[test]
    fn test_random_rational() {
        let seed = 3u8;
        let seed1 = [seed + 0; 32];
        let seed2 = [seed + 1; 32];

        assert_eq!(Rational64::new(1, 2), Rational64::new(2, 4));

        let max_coordinate = 4;
        let between = Uniform::from(0..max_coordinate);
        let mut rng = StdRng::from_seed(seed1);

        let mut rand_polygon = |n_points: usize| -> Polygon<Rational64> {
            let points: Vec<_> = (0..n_points).into_iter()
                .map(|_| (Rational64::new(between.sample(&mut rng), 1),
                          Rational64::new(between.sample(&mut rng), 1)))
                .collect();

            Polygon::new(&points)
        };

        for _i in 0..100 {
            let a = rand_polygon(5);
            let b = rand_polygon(4);

            let results: Vec<_> = vec![Operation::Intersection,
                                       Operation::Union,
                                       Operation::Difference,
                                       Operation::Xor]
                .into_iter()
                .map(|operation| boolean_op(
                    edge_intersection_rational,
                    vec![&a],
                    vec![&b],
                    operation,
                    PolygonSemantics::XOR,
                )).collect();


            // Create grid of probe points.
            let mut rng = StdRng::from_seed(seed2);
            let num_probe_points = 10;
            let denominator = 100;
            let between = Uniform::from(0..max_coordinate * denominator);

            // Create random probe points.
            let probe_points = (0..num_probe_points).into_iter()
                .map(|_| (Rational64::new(between.sample(&mut rng), denominator),
                          Rational64::new(between.sample(&mut rng), denominator)));

            let mut num_checks = 0usize;
            for p in probe_points {
                let p = p.into();

                let on_border_1 = a.exterior.edges().iter()
                    .any(|e| e.contains_point(p).inclusive_bounds());
                let on_border_2 = b.exterior.edges().iter()
                    .any(|e| e.contains_point(p).inclusive_bounds());

                if on_border_1 || on_border_2 {
                    // Ignore cases where the probe point lies on the border of the polygon because,
                    // there `contains_point` is known to be inconsistent with `boolean_op`.
                    continue;
                }

                let in1 = a.contains_point(p);
                let in2 = b.contains_point(p);

                let expected = [
                    in1 & in2,
                    in1 | in2,
                    in1 & !in2,
                    in1 ^ in2,
                ];

                let result_contains_p: Vec<_> = results.iter().map(|r|
                    r.contains_point(p)
                )
                    .collect();

                let success = result_contains_p.iter()
                    .zip(expected.iter())
                    .all(|(actual, expected)| actual == expected);

                if !success {
                    dbg!(&a);
                    dbg!(&b);
                    dbg!(&results);
                    dbg!(&p);
                    dbg!(&expected);
                    dbg!(&result_contains_p);
                }

                assert!(success);
                num_checks += 1;
            }

            assert!(num_checks >= num_probe_points / 2);
        }
    }


    // TOOD
   // #[test]
   // fn test_almost_degenerate() {
   //     let p1 = Polygon::from(vec![(0., 0.), (1., 1.), (1., 1.0001)]);
   //     let p2 = Polygon::from(vec![(1., 0.), (0., 1.), (0., 1.0001)]);
   //
   //     let i = boolean_op(
   //         edge_intersection_float,
   //         vec![&p1],
   //         vec![&p2],
   //         Operation::Intersection,
   //         PolygonSemantics::XOR
   //     );
   //     println!("{:?}", i);
   //     assert_eq!(i.len(), 1);
   //     assert_eq!(i.polygons[0].len(), 4);
   // }


//
//    #[test]
//    fn test_random1() {
//        let seed = 2u8;
//        let seed1 = [seed + 0; 32];
//        let seed2 = [seed + 1; 32];
//
//        let between = Uniform::from(0..8);
//        let mut rng = ChaChaRng::from_seed(seed1);
//
//        let mut rand_polygon = |n_points: usize| -> Polygon<f64> {
//            let points: Vec<(f64, f64)> = (0..n_points).into_iter()
//                .map(|_| (between.sample(&mut rng) as f64, between.sample(&mut rng) as f64))
//                .collect();
//
//            Polygon::new(&points)
//        };
//
//        for i in 0..4000 {
//            println!("{:?}", i);
//
//            let a = rand_polygon(3);
//            let b = rand_polygon(3);
//
//            println!("a {:?}", a);
//            println!("b {:?}", b);
//
//            let result = boolean_op(
//                edge_intersection_float,
//                &[&a],
//                &[&b],
//                Operation::Intersection,
//            );
//
//            println!("result {:?}", result);
//
//            let a_in_b = a.exterior.iter().all(|&p| b.contains_point_non_oriented(p));
//            let b_in_a = b.exterior.iter().all(|&p| a.contains_point_non_oriented(p));
//
//            // Skip cases where there will be a hole. Not supported yet.
//            if a_in_b || b_in_a {
//                continue;
//            }
//
//            // Create grid of probe points.
//            let mut rng = ChaChaRng::from_seed(seed2);
//            let num_probe_points = 100;
//            let probe_points = (0..num_probe_points).into_iter()
//                .map(|_| (between.sample(&mut rng) as f64, between.sample(&mut rng) as f64));
//
//            let mut num_checks = 0usize;
//            for p in probe_points {
//                let p = p.into();
//
//                let on_border_1 = a.exterior.edges().iter()
//                    .any(|e| e.contains_point(p).inclusive_bounds());
//                let on_border_2 = b.exterior.edges().iter()
//                    .any(|e| e.contains_point(p).inclusive_bounds());
//
//                if on_border_1 || on_border_2 {
//                    // Ignore cases where the probe point lies on the border of the polygon because,
//                    // there `contains_point` is known to be inconsistent with `boolean_op`.
//                    continue;
//                }
//
//                let in1 = a.contains_point_non_oriented(p);
//                let in2 = b.contains_point_non_oriented(p);
//
//                let expected = in1 & in2; // Xor
//
//                let result_contains_p = result.contains_point_non_oriented(p);
//                if result_contains_p != expected {
//                    println!("a {:?}", a);
//                    println!("b {:?}", b);
//                    println!("result {:?}", result);
//                }
//
//                assert_eq!(result_contains_p, expected);
//                num_checks += 1;
//            }
//
//            assert!(num_checks >= num_probe_points / 2);
//        }
//    }
}