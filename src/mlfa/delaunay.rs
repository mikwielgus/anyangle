// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use alloc::{vec, vec::Vec};
use core::{
    iter::{ExactSizeIterator, repeat_n},
    ops::{Add, Div},
};

use i_triangle::{
    float::triangulatable::Triangulatable,
    i_overlay::{
        core::{integer::OverlayInt, overlay::ShapeType, relate::PredicateOverlay},
        float::relate::FloatPredicateOverlay,
        i_float::{
            float::{number::FloatNumber, point::FloatPoint},
            int::{number::int::IntNumber, point::IntPoint},
        },
        i_shape::int::shape::IntShapes,
    },
    int::triangulatable::IntTriangulatable,
};
use num_traits::One;
use rstar::{AABB, RTree, RTreeNum, RTreeObject};

use super::{Navmesh, Remesh};
use crate::LayerId;

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct DelaunayTriangleId {
    layer: LayerId,
    index: usize,
}

impl DelaunayTriangleId {
    pub fn new(layer: LayerId, index: usize) -> Self {
        Self { layer, index }
    }

    pub fn layer(&self) -> LayerId {
        self.layer
    }

    pub fn index(&self) -> usize {
        self.index
    }
}

#[derive(Clone, Debug, Default)]
pub struct DelaunayTriangulation<K> {
    vertices: Vec<[[K; 2]; 3]>,
    adjacents: Vec<[DelaunayTriangleId; 3]>,
}

#[derive(Clone, Debug)]
pub struct DelaunayTriangle<K> {
    pub trid: DelaunayTriangleId,
    pub vertices: [[K; 2]; 3],
    pub adjacents: [DelaunayTriangleId; 3],
}

impl<K: RTreeNum> RTreeObject for DelaunayTriangle<K> {
    type Envelope = AABB<[K; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_points(self.vertices.iter())
    }
}

struct DelaunayRTreeCache<'a, K: RTreeNum> {
    inner: &'a mut DelaunayNavmesh<K>,
    layers: Vec<Option<RTree<DelaunayTriangle<K>>>>,
}

#[derive(Clone, Copy, Debug)]
enum StitchDirection {
    Downward,
    Upward,
}

impl<'a, K: RTreeNum> DelaunayRTreeCache<'a, K> {
    fn new(inner: &'a mut DelaunayNavmesh<K>) -> Self {
        let layers = vec![None; inner.layers.len()];
        Self { inner, layers }
    }

    fn update_int_stitches(&mut self, layer: LayerId, direction: StitchDirection)
    where
        K: IntNumber + OverlayInt,
    {
        if usize::from(layer) == 0 && self.inner.layers.len() < 2 {
            return;
        }
        let layer_u: usize = layer.into();
        let (il_below, il_cur) = self.inner.layers.split_at_mut(layer_u);
        let (il_cur, il_above) = il_cur.split_at_mut(1);
        let il_cur = &mut il_cur[0];
        let (stitches, cur_triang, oth_triang, oth_cache, oth_layer) = match direction {
            StitchDirection::Downward => (
                &mut il_cur.downward_stitches,
                &il_cur.triangulation,
                &il_below.last().unwrap().triangulation,
                &mut self.layers[layer_u - 1],
                layer_u - 1,
            ),
            StitchDirection::Upward => (
                &mut il_cur.upward_stitches,
                &il_cur.triangulation,
                &il_above.first().unwrap().triangulation,
                &mut self.layers[layer_u + 1],
                layer_u + 1,
            ),
        };
        let oth_rtree = oth_cache.get_or_insert_with(|| oth_triang.rtree(LayerId::from(oth_layer)));
        *stitches = cur_triang
            .vertices
            .iter()
            .map(|cur_vertices| {
                let envelope = AABB::from_points(cur_vertices);
                let contour = cur_vertices.map(|i| IntPoint { x: i[0], y: i[1] });
                oth_rtree
                    .locate_in_envelope_intersecting(envelope)
                    .filter(|oth_triangle| {
                        let mut po = PredicateOverlay::new(6);
                        po.add_contour(&contour, ShapeType::Subject);
                        po.add_contour(
                            &oth_triangle.vertices.map(|i| IntPoint { x: i[0], y: i[1] }),
                            ShapeType::Clip,
                        );
                        po.intersects()
                    })
                    .map(|i| i.trid)
                    .collect::<Vec<_>>()
            })
            .collect();
    }

    fn update_float_stitches(&mut self, layer: LayerId, direction: StitchDirection)
    where
        K: FloatNumber,
    {
        if usize::from(layer) == 0 && self.inner.layers.len() < 2 {
            return;
        }
        let layer_u: usize = layer.into();
        let (il_below, il_cur) = self.inner.layers.split_at_mut(layer_u);
        let (il_cur, il_above) = il_cur.split_at_mut(1);
        let il_cur = &mut il_cur[0];
        let (stitches, cur_triang, oth_triang, oth_cache, oth_layer) = match direction {
            StitchDirection::Downward => (
                &mut il_cur.downward_stitches,
                &il_cur.triangulation,
                &il_below.last().unwrap().triangulation,
                &mut self.layers[layer_u - 1],
                layer_u - 1,
            ),
            StitchDirection::Upward => (
                &mut il_cur.upward_stitches,
                &il_cur.triangulation,
                &il_above.first().unwrap().triangulation,
                &mut self.layers[layer_u + 1],
                layer_u + 1,
            ),
        };
        let oth_rtree = oth_cache.get_or_insert_with(|| oth_triang.rtree(LayerId::from(oth_layer)));
        *stitches = cur_triang
            .vertices
            .iter()
            .map(|cur_vertices| {
                let envelope = AABB::from_points(cur_vertices);
                let contour = cur_vertices.map(|i| FloatPoint { x: i[0], y: i[1] });
                oth_rtree
                    .locate_in_envelope_intersecting(envelope)
                    .filter(|oth_triangle| {
                        FloatPredicateOverlay::with_subj_and_clip(
                            &contour,
                            &oth_triangle
                                .vertices
                                .map(|i| FloatPoint { x: i[0], y: i[1] }),
                        )
                        .intersects()
                    })
                    .map(|i| i.trid)
                    .collect::<Vec<_>>()
            })
            .collect();
    }
}

impl<K> DelaunayTriangulation<K> {
    pub fn vertices(&self) -> &[[[K; 2]; 3]] {
        &self.vertices
    }

    pub fn adjacents(&self) -> &[[DelaunayTriangleId; 3]] {
        &self.adjacents
    }

    pub fn rtree(&self, layer: LayerId) -> RTree<DelaunayTriangle<K>>
    where
        K: RTreeNum,
    {
        RTree::bulk_load(
            self.vertices
                .iter()
                .zip(self.adjacents.iter())
                .enumerate()
                .map(|(index, (vertices, adjacents))| DelaunayTriangle {
                    trid: DelaunayTriangleId { layer, index },
                    vertices: *vertices,
                    adjacents: *adjacents,
                })
                .collect(),
        )
    }
}

#[derive(Clone, Debug, Default)]
pub struct DelaunayNavmeshLayer<K> {
    triangulation: DelaunayTriangulation<K>,
    upward_stitches: Vec<Vec<DelaunayTriangleId>>,
    downward_stitches: Vec<Vec<DelaunayTriangleId>>,
}

impl<K> DelaunayNavmeshLayer<K> {
    pub fn triangulation(&self) -> &DelaunayTriangulation<K> {
        &self.triangulation
    }
}

#[derive(Clone, Debug, Default)]
pub struct DelaunayNavmesh<K> {
    layers: Vec<DelaunayNavmeshLayer<K>>,
}

impl<K> DelaunayNavmesh<K> {
    pub fn layers(&self) -> &[DelaunayNavmeshLayer<K>] {
        &self.layers
    }
}

impl<K: Add<K, Output = K> + Clone + One + Div<K, Output = K>> Navmesh<K> for DelaunayNavmesh<K> {
    type NavnodeId = DelaunayTriangleId;

    fn adjacents(&self, node: DelaunayTriangleId) -> Vec<DelaunayTriangleId> {
        self.layers[usize::from(node.layer())]
            .triangulation
            .adjacents[node.index()]
        .to_vec()
    }

    fn upward_stitches(&self, node: DelaunayTriangleId) -> Vec<DelaunayTriangleId> {
        self.layers[usize::from(node.layer())].upward_stitches[node.index()].clone()
    }

    fn downward_stitches(&self, node: DelaunayTriangleId) -> Vec<DelaunayTriangleId> {
        self.layers[usize::from(node.layer())].downward_stitches[node.index()].clone()
    }

    fn position(&self, node: DelaunayTriangleId) -> [K; 2] {
        let v = &self.layers[usize::from(node.layer())]
            .triangulation
            .vertices[node.index()];
        let three = K::one() + K::one() + K::one();

        [
            (v[0][0].clone() + v[1][0].clone() + v[2][0].clone()) / three.clone(),
            (v[0][1].clone() + v[1][1].clone() + v[2][1].clone()) / three,
        ]
    }
}

fn reserve_navmesh_layer<K: Copy>(layers: &mut Vec<DelaunayNavmeshLayer<K>>, layer_index: LayerId) {
    layers.extend(repeat_n(
        DelaunayNavmeshLayer {
            triangulation: DelaunayTriangulation {
                vertices: Vec::new(),
                adjacents: Vec::new(),
            },
            upward_stitches: Vec::new(),
            downward_stitches: Vec::new(),
        },
        (usize::from(layer_index) + 1).saturating_sub(layers.len()),
    ));
}

fn store_delaunay_in_navmesh_layer<K: Copy>(
    layer: &mut DelaunayNavmeshLayer<K>,
    layer_index: LayerId,
    boundary_id: DelaunayTriangleId,
    corner_positions: &[[K; 2]],
    triangle_indices: &[usize],
    triangle_neighbors: Vec<[usize; 3]>,
) {
    layer.triangulation = DelaunayTriangulation {
        vertices: triangle_indices
            .as_chunks::<3>()
            .0
            .iter()
            .map(|triangle| triangle.map(|i| corner_positions[i]))
            .collect(),
        adjacents: triangle_neighbors
            .into_iter()
            .map(|n| {
                n.map(|neighbor_index| {
                    if neighbor_index == usize::MAX {
                        boundary_id
                    } else {
                        DelaunayTriangleId::new(layer_index, neighbor_index)
                    }
                })
            })
            .collect(),
    };
}

macro_rules! impl_remesh_float {
    ($k:ty) => {
        impl Remesh<$k> for DelaunayNavmesh<$k> {
            fn remesh(
                &mut self,
                shapes_per_layer: impl ExactSizeIterator<Item = Vec<Vec<Vec<[$k; 2]>>>>,
            ) {
                let boundary_id = DelaunayTriangleId::new(LayerId::MAX, usize::MAX);
                reserve_navmesh_layer(&mut self.layers, LayerId::from(shapes_per_layer.len() - 1));

                for (layer_index, (shapes, layer)) in
                    shapes_per_layer.zip(self.layers.iter_mut()).enumerate()
                {
                    let triangulation = shapes.as_slice().triangulate().into_delaunay();
                    let points = triangulation.points();
                    let indices = triangulation.triangle_indices::<usize>();
                    let neighbors = triangulation.triangle_neighbors();

                    store_delaunay_in_navmesh_layer(
                        layer,
                        layer_index.into(),
                        boundary_id,
                        points.as_slice(),
                        &indices,
                        neighbors,
                    );
                }

                let layer_count = self.layers.len();
                if let Some(last_layer) = layer_count.checked_sub(1) {
                    let mut cache = DelaunayRTreeCache::new(self);
                    for i in 0..layer_count {
                        let li = LayerId::from(i);
                        if i != 0 {
                            cache.update_float_stitches(li, StitchDirection::Downward);
                        }
                        if i != last_layer {
                            cache.update_float_stitches(li, StitchDirection::Upward);
                        }
                    }
                }
            }
            fn remesh_at(&mut self, layer_index: LayerId, shapes: Vec<Vec<Vec<[$k; 2]>>>) {
                let boundary_id = DelaunayTriangleId::new(LayerId::MAX, usize::MAX);
                reserve_navmesh_layer(&mut self.layers, layer_index);

                let triangulation = shapes.as_slice().triangulate().into_delaunay();
                let points = triangulation.points();
                let indices = triangulation.triangle_indices::<usize>();
                let neighbors = triangulation.triangle_neighbors();

                store_delaunay_in_navmesh_layer(
                    &mut self.layers[usize::from(layer_index)],
                    layer_index,
                    boundary_id,
                    points.as_slice(),
                    &indices,
                    neighbors,
                );

                let layer_count = self.layers.len();
                let mut cache = DelaunayRTreeCache::new(self);
                if let Some(prev_index) = usize::from(layer_index).checked_sub(1) {
                    cache.update_float_stitches(layer_index, StitchDirection::Downward);
                    cache.update_float_stitches(LayerId::from(prev_index), StitchDirection::Upward);
                }
                if let Some(next_index) = usize::from(layer_index).checked_add(1)
                    && next_index != layer_count
                {
                    cache.update_float_stitches(layer_index, StitchDirection::Upward);
                    cache.update_float_stitches(
                        LayerId::from(next_index),
                        StitchDirection::Downward,
                    );
                }
            }
        }
    };
}

macro_rules! impl_remesh_int {
    ($k:ty) => {
        impl Remesh<$k> for DelaunayNavmesh<$k> {
            fn remesh(
                &mut self,
                shapes_per_layer: impl ExactSizeIterator<Item = Vec<Vec<Vec<[$k; 2]>>>>,
            ) {
                let boundary_id = DelaunayTriangleId::new(LayerId::MAX, usize::MAX);
                reserve_navmesh_layer(&mut self.layers, LayerId::from(shapes_per_layer.len() - 1));

                for (layer_index, (shapes, layer)) in
                    shapes_per_layer.zip(self.layers.iter_mut()).enumerate()
                {
                    let int_shapes: IntShapes<_> = shapes
                        .iter()
                        .map(|shape| {
                            shape
                                .iter()
                                .map(|contour| {
                                    contour
                                        .iter()
                                        .map(|p| IntPoint::new(p[0] as i32, p[1] as i32))
                                        .collect()
                                })
                                .collect()
                        })
                        .collect();

                    let triangulation = int_shapes.triangulate().into_delaunay();
                    let points = triangulation.points();
                    let indices = triangulation.triangle_indices::<usize>();
                    let neighbors = triangulation.triangle_neighbors();

                    let corner_positions: Vec<[$k; 2]> =
                        points.iter().map(|p| [p.x as $k, p.y as $k]).collect();

                    store_delaunay_in_navmesh_layer(
                        layer,
                        layer_index.into(),
                        boundary_id,
                        corner_positions.as_slice(),
                        &indices,
                        neighbors,
                    );
                }

                let layer_count = self.layers.len();
                if let Some(last_layer) = layer_count.checked_sub(1) {
                    let mut cache = DelaunayRTreeCache::new(self);
                    for i in 0..layer_count {
                        let li = LayerId::from(i);
                        if i != 0 {
                            cache.update_int_stitches(li, StitchDirection::Downward);
                        }
                        if i != last_layer {
                            cache.update_int_stitches(li, StitchDirection::Upward);
                        }
                    }
                }
            }
            fn remesh_at(&mut self, layer_index: LayerId, shapes: Vec<Vec<Vec<[$k; 2]>>>) {
                let boundary_id = DelaunayTriangleId::new(LayerId::MAX, usize::MAX);
                reserve_navmesh_layer(&mut self.layers, layer_index);

                let int_shapes: IntShapes<_> = shapes
                    .iter()
                    .map(|shape| {
                        shape
                            .iter()
                            .map(|contour| {
                                contour
                                    .iter()
                                    .map(|p| IntPoint::new(p[0] as i32, p[1] as i32))
                                    .collect()
                            })
                            .collect()
                    })
                    .collect();

                let triangulation = int_shapes.triangulate().into_delaunay();
                let points = triangulation.points();
                let indices = triangulation.triangle_indices::<usize>();
                let neighbors = triangulation.triangle_neighbors();

                let corner_positions: Vec<[$k; 2]> =
                    points.iter().map(|p| [p.x as $k, p.y as $k]).collect();

                store_delaunay_in_navmesh_layer(
                    &mut self.layers[usize::from(layer_index)],
                    layer_index,
                    boundary_id,
                    corner_positions.as_slice(),
                    &indices,
                    neighbors,
                );

                let layer_count = self.layers.len();
                let mut cache = DelaunayRTreeCache::new(self);
                if let Some(prev_index) = usize::from(layer_index).checked_sub(1) {
                    cache.update_int_stitches(layer_index, StitchDirection::Downward);
                    cache.update_int_stitches(LayerId::from(prev_index), StitchDirection::Upward);
                }
                if let Some(next_index) = usize::from(layer_index).checked_add(1)
                    && next_index != layer_count
                {
                    cache.update_int_stitches(layer_index, StitchDirection::Upward);
                    cache.update_int_stitches(LayerId::from(next_index), StitchDirection::Downward);
                }
            }
        }
    };
}

impl_remesh_float!(f32);
impl_remesh_float!(f64);
impl_remesh_int!(i16);
impl_remesh_int!(i32);
