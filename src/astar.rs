// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::{cmp::Ordering, ops::Add};

use alloc::{
    collections::{BTreeMap, BinaryHeap, btree_map::Entry},
    vec,
    vec::Vec,
};

#[derive(Clone)]
struct GraphAstarFrontierNode<N, S> {
    pub f_score: S,
    pub g_score: S,
    pub graph_node: N,
}

impl<N: PartialEq, S: PartialEq> PartialEq for GraphAstarFrontierNode<N, S> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score && self.graph_node == other.graph_node
    }
}

impl<N: Eq, S: Eq> Eq for GraphAstarFrontierNode<N, S> {}

impl<N: Ord, S: Ord> PartialOrd for GraphAstarFrontierNode<N, S> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<N: Ord, S: Ord> Ord for GraphAstarFrontierNode<N, S> {
    #[inline]
    fn cmp(&self, other: &Self) -> Ordering {
        // Intentionally flipped to have min-heap, not max-heap.
        other
            .f_score
            .cmp(&self.f_score)
            .then_with(|| self.graph_node.cmp(&other.graph_node))
    }
}

pub struct Astar<N, S> {
    frontier: BinaryHeap<GraphAstarFrontierNode<N, S>>,
    g_scores: BTreeMap<N, S>,
    came_from: BTreeMap<N, N>,
    curr_node: GraphAstarFrontierNode<N, S>,
}

impl<N: Clone + Ord, S: Add<S, Output = S> + Copy + Default + Ord> Astar<N, S> {
    pub fn curr_node(&self) -> &GraphAstarFrontierNode<N, S> {
        &self.curr_node
    }

    pub fn curr_graph_node(&self) -> &N {
        &self.curr_node.graph_node
    }

    pub fn new(start: N) -> Self {
        Self {
            frontier: BinaryHeap::new(),
            g_scores: BTreeMap::from([(start.clone(), S::default())]),
            came_from: BTreeMap::new(),
            curr_node: GraphAstarFrontierNode {
                f_score: S::default(),
                g_score: S::default(),
                graph_node: start,
            },
        }
    }

    pub fn expand(&mut self, new_nodes: &[(S, S, N)]) -> Option<N> {
        for new_node in new_nodes {
            self.push(new_node.clone());
        }

        self.pop()
    }

    fn push(&mut self, new_node: (S, S, N)) {
        let curr_g_score = *self.g_scores.get(self.curr_graph_node()).unwrap();
        let (edge_g_cost, h_heuristic, new_graph_node) = new_node;
        let tentative_g = curr_g_score + edge_g_cost;

        match self.g_scores.entry(new_graph_node.clone()) {
            Entry::Occupied(mut entry) => {
                if tentative_g >= *entry.get() {
                    return;
                }

                entry.insert(tentative_g);
            }
            Entry::Vacant(entry) => {
                entry.insert(tentative_g);
            }
        }

        self.came_from
            .insert(new_graph_node.clone(), self.curr_graph_node().clone());

        self.frontier.push(GraphAstarFrontierNode {
            f_score: tentative_g + h_heuristic,
            g_score: tentative_g,
            graph_node: new_graph_node,
        });
    }

    fn pop(&mut self) -> Option<N> {
        // Loop to skip all nodes with better f but still worse g.
        while let Some(node) = self.frontier.pop() {
            let best_g = *self.g_scores.get(&node.graph_node).unwrap();

            if node.g_score <= best_g {
                return Some(self.curr_graph_node().clone());
            }

            self.curr_node = node;
        }

        None
    }

    pub fn reconstruct_path(&self, end: &N) -> Vec<N> {
        let mut path = vec![end.clone()];
        let mut current = end;

        while let Some(previous) = self.came_from.get(current) {
            path.push(previous.clone());
            current = previous;
        }

        path.reverse();

        path
    }
}
