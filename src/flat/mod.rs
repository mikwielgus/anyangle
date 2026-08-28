// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//! Annotated (polygonal) 2D complex/complices such that the intersection of boundaries between
//! two faces is [connected](https://en.wikipedia.org/wiki/Connected_space).

use approx::AbsDiffEq;
use num_traits::Num;

mod layer;
pub use layer::{GetLayerIds, Iter as LayerIdsIter, LayerIds};

mod refine;
pub use refine::{Face, FrozenFace, FrozenFaceNeighbour, FrozenTesselation, Tesselation};

pub trait Topo2DComplex {
    type VertexId: Sized + Copy + Eq + Ord;
    type FaceId: Sized + Copy + Eq + Ord;
    type Scalar: Sized + Clone + AbsDiffEq + Num + PartialOrd;

    fn vertex_position(&self, vertex: Self::VertexId) -> [Self::Scalar; 2];

    fn vertex_adjacent_faces(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = Self::FaceId> + '_;
    fn face_adjacent_vertices(
        &self,
        face: Self::FaceId,
    ) -> impl Iterator<Item = Self::VertexId> + '_;
    fn face_adjacent_faces(&self, face: Self::FaceId) -> impl Iterator<Item = Self::FaceId> + '_;

    /// The returned value should be (if any) of the form `[left vertex, right vertex]`, i.e. ordered clockwise.
    fn portal_between(
        &self,
        face_from: Self::FaceId,
        face_to: Self::FaceId,
    ) -> Option<[Self::VertexId; 2]>;
}

pub trait MultiLayerNavmesh: Topo2DComplex {
    fn face_layers(&self, face: <Self as Topo2DComplex>::FaceId) -> LayerIds;
}
