// Copyright (c) 2018-2021 Thomas Kramer.
// SPDX-FileCopyrightText: 2022 Thomas Kramer
//
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Scanline based on a splay set.

#![allow(unused)]

use libreda_splay::SplaySet;
use std::cmp::Ordering;
use std::borrow::Borrow;

/// Scanline based on a splay set.
pub struct SplayScanLine<T, C>
    where C: Fn(&T, &T) -> Ordering {
    content: SplaySet<T, C>,
}

impl<T, C> SplayScanLine<T, C>
    where C: Fn(&T, &T) -> Ordering
{
    pub fn new(comparator: C) -> SplayScanLine<T, C> {
        SplayScanLine {
            content: SplaySet::new(comparator)
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
        self.content.contains(t)
    }

    pub fn next(&self, t: &T) -> Option<&T> {
        self.content.next(t)
    }

    pub fn prev(&self, t: &T) -> Option<&T> {
        self.content.prev(t)
    }


    pub fn insert(&mut self, t: T) -> bool {
        self.content.insert(t)
    }

    pub fn remove(&mut self, t: &T) -> bool {
        self.content.remove(t)
    }

    pub fn min(&self) -> Option<&T> {
        self.content.min()
    }

    pub fn max(&self) -> Option<&T> {
        self.content.max()
    }
}

#[cfg(test)]
mod test_splay_scanline {
    extern crate rand;

    use super::*;
    use std::cmp::Ordering;
    use std::i32;

    fn int_comparator(a: &i32, b: &i32) -> Ordering {
        a.cmp(b)
    }

    #[test]
    fn insert_simple() {
        let mut t = SplayScanLine::new(int_comparator);
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