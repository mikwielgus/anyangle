// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//! An implementation of the [simple stupid funnel
//! algorithm](https://digestingduck.blogspot.com/2010/03/simple-stupid-funnel-algorithm.html).

use core::ops::{ControlFlow, Mul, Sub};

use alloc::vec::Vec;
use numlike::alg::PoringKit;
use numlike::types::Vector2;

pub struct Funnel<K> {
    pub portals: Vec<[Vector2<K>; 2]>,
    pub apex: Vector2<K>,
    pub left: Vector2<K>,
    pub right: Vector2<K>,
    pub path: Vec<Vector2<K>>,
    pub curr_portal_index: usize,
    pub left_index: usize,
    pub right_index: usize,
}

impl<K: PoringKit> Funnel<K> {
    pub fn new(from: [K; 2], portals: Vec<[Vector2<K>; 2]>) -> Self {
        let from = Vector2::new(from[0].clone(), from[1].clone());

        Self {
            portals,
            apex: from.clone(),
            left: from.clone(),
            right: from.clone(),
            path: Vec::from([from]),
            curr_portal_index: 0,
            left_index: 0,
            right_index: 0,
        }
    }

    pub fn step(&mut self) -> ControlFlow<(), Option<Vector2<K>>> {
        if self.curr_portal_index >= self.portals.len() {
            return ControlFlow::Break(());
        }

        let portal = self.portals[self.curr_portal_index].clone();
        let new_left = portal[0].clone();
        let new_right = portal[1].clone();

        if Vector2::tpdp(self.apex.clone(), self.right.clone(), new_right.clone()) <= K::zero() {
            if self.apex == self.right
                || Vector2::tpdp(self.apex.clone(), self.left.clone(), new_right.clone())
                    > K::zero()
            {
                self.right = new_right;
                self.right_index = self.curr_portal_index;

                ControlFlow::Continue(None)
            } else {
                self.path.push(self.left.clone());

                self.apex = self.left.clone();
                self.right = self.apex.clone();

                let new_apex_index = self.left_index;
                self.right_index = new_apex_index;

                self.curr_portal_index = new_apex_index + 1;

                ControlFlow::Continue(Some(self.apex.clone()))
            }
        } else if Vector2::tpdp(self.apex.clone(), self.left.clone(), new_left.clone()) >= K::zero()
        {
            if self.apex == self.left
                || Vector2::tpdp(self.apex.clone(), self.right.clone(), new_left.clone())
                    < K::zero()
            {
                self.left = new_left;
                self.left_index = self.curr_portal_index;

                ControlFlow::Continue(None)
            } else {
                self.path.push(self.right.clone());

                self.apex = self.right.clone();
                self.left = self.apex.clone();

                let new_apex_index = self.right_index;
                self.left_index = new_apex_index;

                self.curr_portal_index = new_apex_index + 1;

                ControlFlow::Continue(Some(self.apex.clone()))
            }
        } else {
            self.curr_portal_index += 1;

            ControlFlow::Continue(None)
        }
    }

    pub fn run_to(&mut self, to: Vector2<K>) -> Vec<Vector2<K>> {
        while self.step().is_continue() {}

        self.path.push(to);
        core::mem::take(&mut self.path)
    }
}
