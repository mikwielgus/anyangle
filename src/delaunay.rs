// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use std::ops::{Add, Div};

use i_triangle::{
    float::triangulatable::Triangulatable,
    i_overlay::{i_float::int::point::IntPoint, i_shape::int::shape::IntShapes},
    int::triangulatable::IntTriangulatable,
};
use num_traits::One;

use crate::navmesh::{Navmesh, Remesh};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct DelaunayTriangleId {
    layer: usize,
    index: usize,
}

impl DelaunayTriangleId {
    pub fn new(layer: usize, index: usize) -> Self {
        Self { layer, index }
    }

    pub fn layer(&self) -> usize {
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

impl<K> DelaunayTriangulation<K> {
    pub fn vertices(&self) -> &[[[K; 2]; 3]] {
        &self.vertices
    }

    pub fn adjacents(&self) -> &[[DelaunayTriangleId; 3]] {
        &self.adjacents
    }
}

#[derive(Clone, Debug, Default)]
pub struct DelaunayNavmeshLayer<K> {
    triangulation: DelaunayTriangulation<K>,
    upward_stitches: Vec<Vec<([K; 2], DelaunayTriangleId)>>,
    downward_stitches: Vec<Vec<([K; 2], DelaunayTriangleId)>>,
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
        self.layers[node.layer()].triangulation.adjacents[node.index()]
            .clone()
            .to_vec()
    }

    fn upward_stitches(&self, node: DelaunayTriangleId) -> Vec<([K; 2], DelaunayTriangleId)> {
        self.layers[node.layer()].upward_stitches[node.index()].clone()
    }

    fn downward_stitches(&self, node: DelaunayTriangleId) -> Vec<([K; 2], DelaunayTriangleId)> {
        self.layers[node.layer()].downward_stitches[node.index()].clone()
    }

    fn position(&self, node: DelaunayTriangleId) -> [K; 2] {
        let v = &self.layers[node.layer()].triangulation.vertices[node.index()];
        let three = K::one() + K::one() + K::one();

        [
            (v[0][0].clone() + v[1][0].clone() + v[2][0].clone()) / three.clone(),
            (v[0][1].clone() + v[1][1].clone() + v[2][1].clone()) / three,
        ]
    }
}

fn store_delaunay_in_navmesh_layer<K: Copy>(
    layers: &mut Vec<DelaunayNavmeshLayer<K>>,
    layer_index: usize,
    boundary_id: DelaunayTriangleId,
    corner_positions: &[[K; 2]],
    triangle_indices: &[usize],
    triangle_neighbors: Vec<[usize; 3]>,
) {
    let mut vertices = Vec::with_capacity(triangle_indices.len() / 3);
    for triangle in triangle_indices.chunks_exact(3) {
        vertices.push([
            corner_positions[triangle[0]],
            corner_positions[triangle[1]],
            corner_positions[triangle[2]],
        ]);
    }

    let adjacents: Vec<[DelaunayTriangleId; 3]> = triangle_neighbors
        .into_iter()
        .map(|n| {
            std::array::from_fn(|i| {
                let neighbor_index = n[i];
                if neighbor_index == usize::MAX {
                    boundary_id
                } else {
                    DelaunayTriangleId::new(layer_index, neighbor_index)
                }
            })
        })
        .collect();

    while layers.len() <= layer_index {
        layers.push(DelaunayNavmeshLayer {
            triangulation: DelaunayTriangulation {
                vertices: Vec::new(),
                adjacents: Vec::new(),
            },
            upward_stitches: Vec::new(),
            downward_stitches: Vec::new(),
        });
    }

    layers[layer_index].triangulation = DelaunayTriangulation {
        vertices,
        adjacents,
    };
}

macro_rules! impl_remesh_float {
    ($k:ty) => {
        impl Remesh<$k> for DelaunayNavmesh<$k> {
            fn remesh(&mut self, shapes_per_layer: &[Vec<Vec<Vec<[$k; 2]>>>]) {
                let boundary_id = DelaunayTriangleId::new(usize::MAX, usize::MAX);

                for (layer_index, shapes) in shapes_per_layer.iter().enumerate() {
                    let triangulation = shapes.as_slice().triangulate().into_delaunay();
                    let points = triangulation.points();
                    let indices = triangulation.triangle_indices::<usize>();
                    let neighbors = triangulation.triangle_neighbors();

                    store_delaunay_in_navmesh_layer(
                        &mut self.layers,
                        layer_index,
                        boundary_id,
                        points.as_slice(),
                        &indices,
                        neighbors,
                    );
                }
            }
        }
    };
}

macro_rules! impl_remesh_int {
    ($k:ty) => {
        impl Remesh<$k> for DelaunayNavmesh<$k> {
            fn remesh(&mut self, shapes_per_layer: &[Vec<Vec<Vec<[$k; 2]>>>]) {
                let boundary_id = DelaunayTriangleId::new(usize::MAX, usize::MAX);

                for (layer_index, shapes) in shapes_per_layer.iter().enumerate() {
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
                        &mut self.layers,
                        layer_index,
                        boundary_id,
                        corner_positions.as_slice(),
                        &indices,
                        neighbors,
                    );
                }
            }
        }
    };
}

impl_remesh_float!(f32);
impl_remesh_float!(f64);
impl_remesh_int!(i8);
impl_remesh_int!(i16);
impl_remesh_int!(i32);
