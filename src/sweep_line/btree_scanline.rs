// SPDX-FileCopyrightText: 2022 Thomas Kramer
//
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Drop-in replacement for the splay set used for the scanline.
//! Based on the BTreeSet of the standard library.
//! This might be less performant than the implementation based on a splay set, but
//! it is here for comparisons.

#![allow(unused)]

use std::cmp::Ordering;
use std::collections::BTreeSet;
use std::ops::Bound;
use std::borrow::Borrow;

/// Data structure for the scan-line based on a BTreeSet.
pub struct BTreeScanLine<K>
{
    pub(crate) content: BTreeSet<K>,
}

impl<T> BTreeScanLine<T>
    where T: Ord
{
    pub fn new() -> BTreeScanLine<T> {
        BTreeScanLine {
            content: Default::default(),
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

    pub fn contains<Q: ?Sized>(&self, value: &Q) -> bool
        where
            T: Borrow<Q> + Ord,
            Q: Ord,
    {
        self.content.contains(value)
    }

    pub fn next<Q: ?Sized>(&self, t: &Q) -> Option<&T>
        where T: Borrow<Q> + Ord,
              Q: Ord {
        self.content.range((Bound::Excluded(t), Bound::Unbounded))
            .next()
    }

    pub fn prev<Q: ?Sized>(&self, t: &Q) -> Option<&T>
        where T: Borrow<Q> + Ord,
              Q: Ord {
        self.content.range((Bound::Unbounded, Bound::Excluded(t)))
            .rev()
            .next()
    }


    pub fn insert(&mut self, t: T) -> bool {
        self.content.insert(t)
    }

    pub fn remove(&mut self, t: &T) -> bool {
        self.content.remove(t)
    }

    pub fn min(&self) -> Option<&T> {
        self.content.iter().next()
    }

    pub fn max(&self) -> Option<&T> {
        self.content.iter().rev().next()
    }
}

#[cfg(test)]
mod test_btree_scanline {
    use super::*;
    use std::i32;

    #[test]
    fn insert_simple() {
        let mut t = BTreeScanLine::new();
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