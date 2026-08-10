// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//! flipped version
//! `Triangulation * (Triangle -> LayerIds)` instead of `(LayerId' -> InterlaminaData) * (LayerId -> Triangulation)`

use i_triangle::{
    i_overlay::{
        core::{integer::OverlayInt, overlay::ShapeType, relate::PredicateOverlay},
        i_float::int::number::int::IntNumber,
        i_shape::int::IntPoint,
    },
    int::triangulatable::IntTriangulatable as _,
};
use rstar::AABB;
use std::borrow::Cow;

use crate::{GetLayers, LayerId, LayerIds, math};

pub struct TriangulationNavmesh<S /* scalar */> {
    triangles: Vec<Triangle>,
    points: Vec<[S; 2]>,
}

pub struct Triangle {
    pub vertices: [usize; 3],
    pub neighbors: [usize; 3],
    pub layers: LayerIds,
}

pub trait Obstacle: GetLayers {
    type Scalar: Clone;
    fn to_polygon(&self) -> Cow<'_, [[Self::Scalar; 2]]>;
}

impl<S> TriangulationNavmesh<S> {
    pub fn triangles(&self) -> &[Triangle] {
        &self.triangles[..]
    }

    pub fn points(&self) -> &[[S; 2]] {
        &self.points[..]
    }

    pub fn from_int_obstacle_rtree<T, Params>(
        all_layers: core::ops::Range<usize>,
        ext_boundary: &[[S; 2]],
        rtree: &rstar::RTree<T, Params>,
    ) -> Self
    where
        S: Clone
            + PartialOrd
            + core::iter::Sum
            + IntNumber
            + OverlayInt
            + num_traits::Num
            + rstar::RTreeNum,
        T: rstar::RTreeObject<Envelope = AABB<[S; 2]>> + Obstacle<Scalar = S>,
        Params: rstar::RTreeParams,
    {
        let shapes: Vec<_> = rtree
            .iter()
            .map(|obstacle| tr_to_int_points(&obstacle.to_polygon()))
            .chain(core::iter::once({
                // this is necessary to make the triangulation large enough,
                // but it doesn't block any layers.
                tr_to_int_points(ext_boundary)
            }))
            .map(|contour| vec![/* exterior */ contour])
            .collect();

        let itria = shapes.triangulate();

        let layers_used = {
            let mut layers_used = LayerIds::default();
            for i in all_layers {
                layers_used.insert(LayerId(i));
            }
            layers_used
        };

        let triangles: Vec<_> = itria
            .triangle_indices()
            .as_chunks::<3>()
            .0
            .iter()
            .zip(itria.triangle_neighbors())
            .map(|(vertices, mut neighbors)| {
                let mut vertices = *vertices;
                // the vertices are always either clockwise or counter-clockwise
                // swap their order if they're clockwise.
                let contour: [IntPoint<_>; 3] = vertices.map(|i| itria.points()[i]);
                let points = contour.map(|i| [i.x, i.y]);
                if matches!(
                    math::poly_convex_hull_rotation_sense(&points.map(|i| (i, ())), 0,),
                    math::RotationSense::Clockwise
                ) {
                    vertices.reverse();
                    neighbors.reverse();
                }
                // the layers for a given triangle are all the layers at which
                // *no* obstacle is present.
                let layers = rtree
                    .locate_in_envelope_intersecting(AABB::from_points(&points))
                    .filter(|obstacle| {
                        let obs_contour = tr_to_int_points(&obstacle.to_polygon());
                        let mut po = PredicateOverlay::new(2);
                        po.add_contour(&obs_contour[..], ShapeType::Subject);
                        po.add_contour(&contour[..], ShapeType::Clip);
                        po.interiors_intersect()
                    })
                    .fold(layers_used.clone(), |mut acc, cur| {
                        acc.0.difference_with(&cur.layers().0);
                        acc
                    });
                Triangle {
                    vertices,
                    neighbors,
                    layers,
                }
            })
            .collect();

        Self {
            triangles,
            points: itria.points().iter().map(|i| [i.x, i.y]).collect(),
        }
    }
}

fn tr_to_int_points<T: IntNumber>(x: &[[T; 2]]) -> Vec<IntPoint<T>> {
    x.iter()
        .map(|&[x, y]| IntPoint { x, y })
        .collect::<Vec<_>>()
}

impl<S> super::Topo2DComplex for TriangulationNavmesh<S>
where
    S: Clone + Default + num_traits::Signed + PartialOrd,
{
    type VertexId = usize;
    type FaceId = usize;
    type Scalar = S;

    #[inline]
    fn vertex_position(&self, vertex: Self::VertexId) -> [S; 2] {
        self.points[vertex].clone()
    }

    /// NOTE: this is very inefficient,
    /// but it might not be too bad given that this is only called once per A* invocation.
    fn vertex_adjacent_faces(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = Self::FaceId> + '_ {
        self.triangles
            .iter()
            .enumerate()
            .filter(move |(_, triangle)| triangle.vertices.contains(&vertex))
            .map(|(n, _)| n)
    }

    #[inline]
    fn face_adjacent_vertices(
        &self,
        face: Self::FaceId,
    ) -> impl Iterator<Item = Self::VertexId> + '_ {
        self.triangles[face].vertices.iter().copied()
    }

    #[inline]
    fn face_adjacent_faces(&self, face: Self::FaceId) -> impl Iterator<Item = Self::FaceId> + '_ {
        self.triangles[face]
            .neighbors
            .iter()
            .copied()
            .filter(|&i| i != usize::MAX)
    }

    fn portal_between(
        &self,
        face_from: Self::FaceId,
        face_to: Self::FaceId,
    ) -> Option<[Self::VertexId; 2]> {
        let face_from = &self.triangles.get(face_from)?.vertices;
        let face_to = &self.triangles.get(face_to)?.vertices;

        let not_in_to = face_from
            .iter()
            .copied()
            .enumerate()
            .find(|(_, i)| !face_to.contains(i))?;

        // NOTE: the interface expects Some([left, right]),
        // so we have to return them in clockwise ordering.
        Some(match not_in_to.0 {
            0 => [face_from[2], face_from[1]],
            1 => [face_from[0], face_from[2]],
            2 => [face_from[1], face_from[0]],
            3.. => unreachable!(),
        })
    }
}

impl<S> super::MultiLayerNavmesh for TriangulationNavmesh<S>
where
    S: Clone + Default + num_traits::Signed + PartialOrd,
{
    #[inline]
    fn face_layers(&self, face: Self::FaceId) -> &LayerIds {
        &self.triangles[face].layers
    }
}
