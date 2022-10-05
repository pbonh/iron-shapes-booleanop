// SPDX-FileCopyrightText: 2022 Thomas Kramer
//
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Implement the general sweep line algorithm used for algorithms like Boolean operations and connectivity extraction.

pub mod sweep_event;

pub mod intersection;

mod btree_scanline;
mod compare_segments;
mod naive_scanline;
mod possible_intersection;
mod splay_scanline;
