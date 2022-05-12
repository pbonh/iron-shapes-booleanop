// Copyright (c) 2018-2021 Thomas Kramer.
// SPDX-FileCopyrightText: 2022 Thomas Kramer
//
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Drop-in replacement for the splay set used for the scanline.
//! Very easy implementation but inefficient for large problems.

#![allow(unused)]

use std::cmp::Ordering;

/// Naive and inefficient implementation of the data structure used for the scan line.
pub struct NaiveScanLine<K, C>
    where C: Fn(&K, &K) -> Ordering,
{
    comparator: C,
    content: Vec<K>,
}

impl<T, C> NaiveScanLine<T, C>
    where
        C: Fn(&T, &T) -> Ordering,
{
    pub fn new(comparator: C) -> NaiveScanLine<T, C> {
        NaiveScanLine {
            comparator,
            content: Vec::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.content.len()
    }
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    pub fn clear(&mut self) {
        self.content.clear()
    }

    pub fn contains(&self, t: &T) -> bool {
        self.find(t).is_some()
    }

    pub fn find(&self, t: &T) -> Option<&T> {
        self.find_index(t).map(|(index, t)| t)
    }

    pub fn next(&self, t: &T) -> Option<&T> {
        self.content.iter()
            .filter(|e| (&self.comparator)(e, t) == Ordering::Greater)
            .min_by(|a, b| (&self.comparator)(a, b))
    }

    pub fn prev(&self, t: &T) -> Option<&T> {
        self.content.iter()
            .filter(|e| (&self.comparator)(e, t) == Ordering::Less)
            .max_by(|a, b| (&self.comparator)(a, b))
    }

    fn find_index(&self, t: &T) -> Option<(usize, &T)> {
        self.content.iter()
            .enumerate()
            .find(|(index, e)| (&self.comparator)(e, t) == Ordering::Equal)
    }

    pub fn insert(&mut self, t: T) -> bool {
        if let Some((index, _)) = self.find_index(&t) {
            self.content[index] = t;
            true
        } else {
            self.content.push(t);
            true
        }
    }

    pub fn remove(&mut self, t: &T) -> bool {
        if let Some((index, _)) = self.find_index(t) {
            self.content.remove(index);
            true
        } else {
            false
        }
    }

    pub fn min(&self) -> Option<&T> {
        self.content.iter().min_by(|a, b| (&self.comparator)(a, b))
    }

    pub fn max(&self) -> Option<&T> {
        self.content.iter().max_by(|a, b| (&self.comparator)(a, b))
    }
}

#[cfg(test)]
mod test_naive_scanline {
    extern crate rand;

    use super::*;
    use std::cmp::Ordering;
    use std::i32;

    fn int_comparator(a: &i32, b: &i32) -> Ordering {
        a.cmp(b)
    }

    #[test]
    fn insert_simple() {
        let mut t = NaiveScanLine::new(int_comparator);
        t.insert(1);
        t.insert(3);
        t.insert(2);

        assert_eq!(t.min(), Some(&1));
        assert_eq!(t.max(), Some(&3));

        assert_eq!(t.next(&1), Some(&2));
        assert_eq!(t.next(&2), Some(&3));
        assert_eq!(t.next(&3), None);

        assert_eq!(t.prev(&1), None);
        assert_eq!(t.prev(&2), Some(&1));
        assert_eq!(t.prev(&3), Some(&2));
    }
}