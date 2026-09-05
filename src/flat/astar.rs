// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use alloc::{
    collections::{BTreeMap, BTreeSet, BinaryHeap, btree_map},
    vec,
    vec::Vec,
};
use core::{cmp::Ordering, fmt, iter};

use approx::AbsDiffEq;
use num_traits::Zero;

use super::{LayerIds, MultiLayerNavmesh, Topo2DComplex};
use crate::LayerId;

/// A node of a path from source to sink, including layer information
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Node<T> {
    pub fixed: T,
    pub layer: LayerId,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Endpoint<T> {
    pub fixed: BTreeSet<T>,
    pub layers: LayerIds,
}

/// The data for each step from source to sink with explicit layer transitions
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FunnelEntry<V> {
    Point(Node<V>),
    LayerTransition(LayerId, LayerId),
}

struct BestPaths<S, T>(BTreeMap<Node<T>, Entry<S, T>>);

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum BestPathKey<F, V> {
    Face(F),
    Vertex(V),
}

impl<S: Ord + fmt::Debug, T: Ord + fmt::Debug> BestPaths<S, T> {
    fn reconstruct_path<'a>(&'a self, current: &'a Node<T>) -> Vec<&'a Node<T>> {
        let mut current = current;
        let mut ret = vec![current];
        while let Some(Entry { key, .. }) = self.0.get(current) {
            current = key;
            ret.push(key);
        }
        ret.reverse();
        ret
    }
}

/// An annotated node or other key, for ranking in a binary heap.
#[derive(Clone, Debug)]
struct Entry<S, T> {
    // end score = g_score + guessed_final_lower_score
    score: S,
    key: Node<T>,
}

impl<S: Ord, T: Ord> Ord for Entry<S, T> {
    fn cmp(&self, other: &Self) -> Ordering {
        // flip ordering on score
        S::cmp(&other.score, &self.score).then_with(|| self.key.cmp(&other.key))
    }
}

impl<S: Ord, T: Eq> Eq for Entry<S, T> {}

impl<S: Ord, T: Ord> PartialOrd for Entry<S, T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<S: Ord, T: PartialEq> PartialEq for Entry<S, T> {
    fn eq(&self, other: &Self) -> bool {
        S::cmp(&self.score, &other.score) == Ordering::Equal && self.key == other.key
    }
    #[allow(clippy::partialeq_ne_impl)]
    fn ne(&self, other: &Self) -> bool {
        S::cmp(&self.score, &other.score) != Ordering::Equal || self.key != other.key
    }
}

/// The constant environment during an A* search (i.e. what never changes during an [`astar`] invocation).
struct Environment<'a, T: Topo2DComplex, Score, Pnf> {
    mesh: &'a T,
    point_norm: Pnf,
    final_end: Endpoint<T::VertexId>,
    epsilon: <T::Scalar as AbsDiffEq>::Epsilon,
    layer_transition_penality: Score,
}

impl<T, Score, Pnf> Environment<'_, T, Score, Pnf>
where
    T: MultiLayerNavmesh,
    <<T as Topo2DComplex>::Scalar as AbsDiffEq>::Epsilon: Clone,
    Score: Clone + Ord + Zero + fmt::Debug,
    Pnf: Fn(&[T::Scalar; 2]) -> Score,
{
    fn funnel(
        &self,
        best_paths: &BestPaths<Score, BestPathKey<T::FaceId, T::VertexId>>,
        // if intermed_end is set, then `best_paths` is only traversed up to that,
        // and `end` is assumed to be directly linked to that.
        intermed_end: Option<&Node<T::FaceId>>,
        // the final triangle
        end: &Node<T::FaceId>,
    ) -> (Vec<FunnelEntry<T::VertexId>>, Vec<Node<T::FaceId>>) {
        let end = Node {
            fixed: BestPathKey::Face(end.fixed),
            layer: end.layer,
        };
        let intermed_end = intermed_end.map(|intermed_end| Node {
            fixed: BestPathKey::Face(intermed_end.fixed),
            layer: intermed_end.layer,
        });
        let mut best_path = match &intermed_end {
            None => best_paths.reconstruct_path(&end),
            Some(intermed_end) => {
                let mut best_path = best_paths.reconstruct_path(intermed_end);
                if best_path.iter().find(|&&i| i == &end).is_some() {
                    return (Vec::new(), Vec::new());
                }
                best_path.push(&end);
                best_path
            }
        };

        // reconstruct funnel
        if best_path.is_empty() {
            // unreachable
            return (Vec::new(), Vec::new());
        }
        let start = match best_path.remove(0) {
            Node {
                fixed: BestPathKey::Vertex(fixed),
                layer,
            } => Node {
                fixed: *fixed,
                layer: *layer,
            },
            _ => panic!("Start of every path is a vertex"),
        };
        let mut funnel = crate::funnel::SimpleFunnel::<T::Scalar, Node<T::VertexId>>::new((
            self.mesh.vertex_position(start.fixed),
            start,
        ));
        let mut ret = Vec::<FunnelEntry<T::VertexId>>::new();

        if start.layer != best_path.first().unwrap().layer {
            // illegal move
            return (Vec::new(), Vec::new());
        }

        ret.push(FunnelEntry::Point(funnel.apex.1));

        let best_path = best_path
            .into_iter()
            .map(|Node { fixed, layer }| match fixed {
                BestPathKey::Face(fixed) => Node {
                    fixed: *fixed,
                    layer: *layer,
                },
                _ => panic!("Continuation of every path is a face"),
            })
            .collect::<Vec<_>>();

        for i in best_path.windows(2) {
            let (i, j) = (i[0], i[1]);
            match (i.layer == j.layer, i.fixed == j.fixed) {
                (true, true) | (false, false) => {
                    // invalid move
                    return (Vec::new(), Vec::new());
                }
                (false, true) => {
                    // layer transition
                    ret.push(FunnelEntry::LayerTransition(i.layer, j.layer));
                }
                (true, false) => {
                    // face transition
                    let Some(portal) = self.mesh.portal_between(i.fixed, j.fixed) else {
                        return (Vec::new(), Vec::new());
                    };
                    funnel.push(portal.map(|fixed| {
                        (
                            self.mesh.vertex_position(fixed),
                            Node {
                                fixed,
                                layer: j.layer,
                            },
                        )
                    }));
                }
            }
        }

        ret.extend(
            funnel
                .with_epsilon(self.epsilon.clone())
                .map(|(_, node)| FunnelEntry::Point(node)),
        );

        let final_measure_point = &funnel.apex;
        let mut choices: Vec<_> = self
            .final_end
            .fixed
            .iter()
            .map(|&final_end_fixed| {
                let final_end_pos = (
                    self.mesh.vertex_position(final_end_fixed),
                    Node {
                        fixed: final_end_fixed,
                        layer: funnel.apex.1.layer,
                    },
                );
                let mut final_funnel = funnel.clone();
                final_funnel.push([final_end_pos.clone(), final_end_pos.clone()]);
                let final_stretch = final_funnel
                    .with_epsilon(self.epsilon.clone())
                    .collect::<Vec<_>>();
                let final_stretch_positions = iter::once(final_measure_point.0.clone())
                    .chain(final_stretch.iter().map(|(pos, _)| pos.clone()))
                    .chain(iter::once(final_end_pos.0))
                    .collect::<Vec<_>>();
                let length = final_stretch_positions
                    .array_windows::<2>()
                    .fold(Zero::zero(), |length: Score, [from, to]| {
                        length + self.point_distance(from, to)
                    });
                (length, final_stretch, final_end_pos.1)
            })
            .collect();
        choices.sort_by_key(|(k, _, _)| k.clone());
        let &(_, ref final_stretch, mut final_end) = choices.first().unwrap();
        for (_, new_apex) in final_stretch {
            final_end.layer = new_apex.layer;
            ret.push(FunnelEntry::Point(*new_apex));
        }
        if !self.final_end.layers.is_on_layer(final_end.layer) {
            let mut current_layer = final_end.layer;
            let (closest, next) = match self
                .final_end
                .layers
                .closest_contained_layer_to(current_layer)
            {
                [None, None] => {
                    // all moves are illegal
                    return (Vec::new(), Vec::new());
                }
                [Some(closest), _] => (
                    closest,
                    LayerId::checked_sub_one as fn(LayerId) -> Option<LayerId>,
                ),
                [_, Some(closest)] => (
                    closest,
                    LayerId::checked_add_one as fn(LayerId) -> Option<LayerId>,
                ),
            };
            while current_layer != closest {
                let prev_layer = current_layer;
                // UNWRAP SAFETY: closest is reachable from current_layer via next
                // because the same function is used in `closest_contained_layer_to`.
                current_layer = next(prev_layer).unwrap();
                ret.push(FunnelEntry::LayerTransition(prev_layer, current_layer));
            }
            final_end.layer = current_layer;
        }
        ret.push(FunnelEntry::Point(final_end));
        (ret, best_path)
    }

    fn point_distance(&self, a: &[T::Scalar; 2], b: &[T::Scalar; 2]) -> Score {
        (self.point_norm)(&crate::math::delta(a, b))
    }

    fn calculate_score(
        &self,
        best_paths: &BestPaths<Score, BestPathKey<T::FaceId, T::VertexId>>,
        // if intermed_end is set, then `best_paths` is only traversed up to that,
        // and `end` is assumed to be directly linked to that.
        intermed_end: Option<&Node<T::FaceId>>,
        // the final triangle
        end: &Node<T::FaceId>,
    ) -> Option<(Score, Vec<FunnelEntry<T::VertexId>>, Vec<Node<T::FaceId>>)> {
        let (funneled, best_path) = self.funnel(best_paths, intermed_end, end);

        if funneled.is_empty() {
            return None;
        }

        let mut score = Score::zero();

        let funneled_apices: Vec<_> = funneled
            .iter()
            .filter_map(|i| match i {
                FunnelEntry::LayerTransition(_, _) => {
                    score = score.clone() + self.layer_transition_penality.clone();
                    None
                }
                FunnelEntry::Point(new_apex) => Some(new_apex),
            })
            .collect();

        for i in funneled_apices.windows(2) {
            score = score
                + self.point_distance(
                    &self.mesh.vertex_position(i[0].fixed),
                    &self.mesh.vertex_position(i[1].fixed),
                );
        }

        Some((score, funneled, best_path))
    }
}

pub struct Astar<'a, T: Topo2DComplex, Score, Pnf> {
    best_paths: BestPaths<Score, BestPathKey<T::FaceId, T::VertexId>>,
    heap: BinaryHeap<Entry<Score, T::FaceId>>,
    env: Environment<'a, T, Score, Pnf>,
}

#[derive(Clone, Debug)]
pub enum Output<T: Topo2DComplex, Score> {
    /// A successful A* result
    Result(Vec<FunnelEntry<T::VertexId>>),

    IntermediateStep(Node<T::FaceId>, Vec<(Node<T::FaceId>, Score)>),
}

impl<T, Score, Pnf> Iterator for Astar<'_, T, Score, Pnf>
where
    T: MultiLayerNavmesh,
    <T::Scalar as AbsDiffEq>::Epsilon: Clone,
    Score: Clone + Ord + Zero + fmt::Debug,
    Pnf: Fn(&[T::Scalar; 2]) -> Score,
{
    type Item = Output<T, Score>;

    fn next(&mut self) -> Option<Output<T, Score>> {
        // main search loop
        let cur = self.heap.pop()?;
        if self.env.final_end.layers.is_on_layer(cur.key.layer)
            && self
                .env
                .mesh
                .face_adjacent_vertices(cur.key.fixed)
                .find(|fixed| self.env.final_end.fixed.contains(fixed))
                .is_some()
        {
            let ret = self.env.funnel(&self.best_paths, None, &cur.key);
            return Some(Output::Result(ret.0));
        }

        // TODO(fogti): catch cases where this iteration has a worse score than the best path to this node

        let face_transition = self
            .env
            .mesh
            .face_adjacent_faces(cur.key.fixed)
            // filter untraversable faces
            .filter(|&inner_face| {
                self.env
                    .mesh
                    .face_layers(inner_face)
                    .is_on_layer(cur.key.layer)
            })
            .map(|fixed| Node {
                fixed,
                layer: cur.key.layer,
            });

        let layer_transition = self
            .env
            .mesh
            .face_layers(cur.key.fixed)
            .adjacent_layers_to(cur.key.layer)
            .into_iter()
            .map(|layer| Node {
                fixed: cur.key.fixed,
                layer,
            });

        let next_nodes = face_transition
            .chain(layer_transition)
            // weigh candidates
            .filter_map(|next_node| {
                self.env
                    .calculate_score(&self.best_paths, Some(&cur.key), &next_node)
                    .map(|(score, _, _)| (next_node, score))
            })
            .collect::<Vec<_>>();

        for (next_node, score) in &next_nodes {
            let next_node = *next_node;
            let bp_node = self.best_paths.0.entry(Node {
                fixed: BestPathKey::Face(next_node.fixed),
                layer: next_node.layer,
            });
            // filter cases of worse newer scores
            if let btree_map::Entry::Occupied(occ) = &bp_node
                && let Entry {
                    score: old_score, ..
                } = occ.get()
                && score >= old_score
            {
                continue;
            }
            let new_bp_data = Entry {
                key: Node {
                    fixed: BestPathKey::Face(cur.key.fixed),
                    layer: cur.key.layer,
                },
                score: score.clone(),
            };
            match bp_node {
                btree_map::Entry::Occupied(mut occ) => {
                    occ.insert(new_bp_data);
                }
                btree_map::Entry::Vacant(vac) => {
                    vac.insert(new_bp_data);
                }
            }
            self.heap.push(Entry {
                key: next_node,
                score: score.clone(),
            });
        }

        Some(Output::IntermediateStep(cur.key, next_nodes))
    }
}

pub fn astar<'mesh, T, Score, Pnf>(
    mesh: &'mesh T,
    point_norm: Pnf,
    start: Endpoint<T::VertexId>,
    end: Endpoint<T::VertexId>,
    epsilon: <T::Scalar as AbsDiffEq>::Epsilon,
    layer_transition_penality: Score,
) -> Astar<'mesh, T, Score, Pnf>
where
    T: MultiLayerNavmesh,
    <T::Scalar as AbsDiffEq>::Epsilon: Clone,
    Score: Clone + Ord + Zero + fmt::Debug,
    Pnf: Fn(&[T::Scalar; 2]) -> Score,
{
    let mut best_paths = BestPaths::<_, BestPathKey<_, _>>(Default::default());
    let mut heap = BinaryHeap::<Entry<Score, T::FaceId>>::new();
    let env = Environment {
        mesh,
        point_norm,
        final_end: end,
        epsilon,
        layer_transition_penality,
    };

    // initialize heap
    for &fixed_vertex in &start.fixed {
        let bp_entry = |layer: LayerId| Entry {
            score: Score::zero(),
            key: Node {
                fixed: BestPathKey::Vertex(fixed_vertex),
                layer,
            },
        };
        for inner_face in mesh
            .vertex_adjacent_faces(fixed_vertex)
            .collect::<BTreeSet<_>>()
        {
            // no need to adjust best_paths, that stores only paths via faces,
            // not vertex-face paths.

            // filter untraversable triangles
            for layer in &((&mesh.face_layers(inner_face)) & (&start.layers)) {
                let node = Node {
                    fixed: inner_face,
                    layer,
                };
                best_paths.0.insert(
                    Node {
                        fixed: BestPathKey::Face(inner_face),
                        layer,
                    },
                    bp_entry(layer),
                );
                let Some((score, _, _)) = env.calculate_score(&best_paths, None, &node) else {
                    continue;
                };

                heap.push(Entry { score, key: node });
            }
        }
    }

    Astar {
        best_paths,
        heap,
        env,
    }
}
