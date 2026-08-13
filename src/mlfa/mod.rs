// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

mod delaunay;
mod navmesher;

pub use delaunay::{
    DelaunayNavmesh, DelaunayNavmeshLayer, DelaunayTriangleId, DelaunayTriangulation,
};
pub use navmesher::Navmesher;
pub use polygon_unionfind::{Laminate, PolygonWithData, Rings};

pub trait Navmesh<K> {
    type NavnodeId;

    fn adjacents(&self, node: Self::NavnodeId) -> Vec<Self::NavnodeId>;
    fn upward_stitches(&self, node: Self::NavnodeId) -> Vec<([K; 2], Self::NavnodeId)>;
    fn downward_stitches(&self, node: Self::NavnodeId) -> Vec<([K; 2], Self::NavnodeId)>;
    fn position(&self, node: Self::NavnodeId) -> [K; 2];
}

pub trait Remesh<K>: Navmesh<K> {
    fn remesh(
        &mut self,
        shapes_per_layer: impl core::iter::ExactSizeIterator<Item = Vec<Vec<Vec<[K; 2]>>>>,
    );
    fn remesh_at(&mut self, layer_index: usize, shapes: Vec<Vec<Vec<[K; 2]>>>);
}
