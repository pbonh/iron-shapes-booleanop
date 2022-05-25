// SPDX-FileCopyrightText: 2022 Thomas Kramer
//
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Implement the general sweep line algorithm used for algorithms like Boolean operations and connectivity extraction.

pub mod sweep_event;

pub mod intersection;

mod compare_segments;
mod possible_intersection;
mod naive_scanline;
mod btree_scanline;
mod splay_scanline;