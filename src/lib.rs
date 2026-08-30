// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

#![allow(clippy::type_complexity)]
#![no_std]

extern crate alloc;

pub mod flat;
pub mod funnel;
mod layer;
pub mod math;
pub mod mlfa;

pub use layer::LayerId;
