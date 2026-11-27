// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use alloc::vec::Vec;

use i_triangle::advanced::delaunay::IntDelaunay;
use i_triangle::i_overlay::core::integer::OverlayInt;
use i_triangle::i_overlay::i_float::int::number::int::IntNumber;
use i_triangle::i_overlay::i_float::int::point::IntPoint;
use i_triangle::i_overlay::i_shape::int::shape::IntShapes;
use i_triangle::int::triangulatable::IntTriangulatable;
use nonany::NonMaxUsize;
use numlike::elem::Zero;
use numlike::types::Vector2;

use crate::mesh::{FaceAdjacentsConstN, FaceContourConstN, Faced, FacesSharedVerticesConstN};

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct DelaunayTriangleId(NonMaxUsize);

impl DelaunayTriangleId {
    pub fn new(index: usize) -> Self {
        Self(NonMaxUsize::new(index).expect("index is MAX, which is forbidden"))
    }

    pub fn index(&self) -> usize {
        self.0.get()
    }
}

#[derive(Clone, Debug, Default)]
pub struct DelaunayTriangulation<K> {
    triangles: Vec<[Vector2<K>; 3]>,
}

impl<K> DelaunayTriangulation<K> {
    pub fn triangles(&self) -> &[[Vector2<K>; 3]] {
        &self.triangles
    }
}

impl<K> DelaunayTriangulation<K>
where
    K: Clone + IntNumber + OverlayInt,
{
    pub fn new(polygons_with_holes: Vec<Vec<Vec<Vector2<K>>>>) -> Self {
        let i_triangle_polygons_with_holes: IntShapes<K> =
            Self::polygons_with_holes_to_i_triangle_shapes(polygons_with_holes);

        let i_triangle_delaunay = i_triangle_polygons_with_holes.triangulate().into_delaunay();
        let triangles = Self::i_triangle_delaunay_to_triangles(i_triangle_delaunay);

        Self { triangles }
    }

    fn polygons_with_holes_to_i_triangle_shapes(
        polygons_with_holes: Vec<Vec<Vec<Vector2<K>>>>,
    ) -> IntShapes<K> {
        polygons_with_holes
            .into_iter()
            .map(|shape| {
                shape
                    .into_iter()
                    .map(|contour| {
                        contour
                            .into_iter()
                            .map(|p| IntPoint::new(p.x(), p.y()))
                            .collect()
                    })
                    .collect()
            })
            .collect()
    }

    fn i_triangle_delaunay_to_triangles(
        i_triangle_delaunay: IntDelaunay<K>,
    ) -> Vec<[Vector2<K>; 3]> {
        let points = i_triangle_delaunay.points();

        i_triangle_delaunay
            .triangles
            .iter()
            .map(|triangle| {
                let v = &triangle.vertices;

                [
                    Vector2::new(points[v[0].index].x, points[v[0].index].y),
                    Vector2::new(points[v[1].index].x, points[v[1].index].y),
                    Vector2::new(points[v[2].index].x, points[v[2].index].y),
                ]
            })
            .collect()
    }
}

impl<K> Faced for DelaunayTriangulation<K> {
    type FaceId = DelaunayTriangleId;
}

impl<K> FaceContourConstN<K, 3> for DelaunayTriangulation<K> {
    fn face_contour(&self, face: &DelaunayTriangleId) -> &[Vector2<K>; 3] {
        &self.triangles[face.index()]
    }
}

#[derive(Clone, Debug, Default)]
pub struct DelaunayMesh<K> {
    triangulation: DelaunayTriangulation<K>,
    adjacents: Vec<[Option<DelaunayTriangleId>; 3]>,
}

impl<K> DelaunayMesh<K> {
    pub fn vertices(&self) -> &[[Vector2<K>; 3]] {
        self.triangulation.triangles()
    }

    pub fn adjacents(&self) -> &[[Option<DelaunayTriangleId>; 3]] {
        &self.adjacents
    }
}

impl<K> DelaunayMesh<K>
where
    K: Clone + IntNumber + OverlayInt,
{
    pub fn new(polygons_with_holes: Vec<Vec<Vec<Vector2<K>>>>) -> Self {
        let i_triangle_polygons_with_holes =
            DelaunayTriangulation::polygons_with_holes_to_i_triangle_shapes(polygons_with_holes);
        let i_triangle_delaunay = i_triangle_polygons_with_holes.triangulate().into_delaunay();
        let adjacents = Self::i_triangle_delaunay_adjacents(&i_triangle_delaunay);

        Self {
            triangulation: DelaunayTriangulation {
                triangles: DelaunayTriangulation::i_triangle_delaunay_to_triangles(
                    i_triangle_delaunay,
                ),
            },
            adjacents,
        }
    }

    fn i_triangle_delaunay_adjacents(
        i_triangle_delaunay: &IntDelaunay<K>,
    ) -> Vec<[Option<DelaunayTriangleId>; 3]> {
        i_triangle_delaunay
            .triangle_neighbors()
            .into_iter()
            .map(|neighbors| {
                neighbors.map(|index| {
                    if index == usize::MAX {
                        None
                    } else {
                        Some(DelaunayTriangleId::new(index))
                    }
                })
            })
            .collect()
    }
}

impl<K> Faced for DelaunayMesh<K> {
    type FaceId = DelaunayTriangleId;
}

impl<K> FaceContourConstN<K, 3> for DelaunayMesh<K> {
    fn face_contour(&self, face: &DelaunayTriangleId) -> &[Vector2<K>; 3] {
        self.triangulation.face_contour(face)
    }
}

impl<K> FaceAdjacentsConstN<3> for DelaunayMesh<K> {
    fn face_adjacents(&self, face: &DelaunayTriangleId) -> &[Option<DelaunayTriangleId>; 3] {
        &self.adjacents[face.index()]
    }
}

impl<K: Clone + PartialEq + Zero> FacesSharedVerticesConstN<K, 2> for DelaunayMesh<K> {
    fn faces_shared_vertices(
        &self,
        face1: &DelaunayTriangleId,
        face2: &DelaunayTriangleId,
    ) -> Option<[Vector2<K>; 2]> {
        let contour1 = self.face_contour(face1);
        let contour2 = self.face_contour(face2);

        for i in 0..3 {
            let a = contour1[i].clone();
            let b = contour1[(i + 1) % 3].clone();

            if contour2.contains(&a) && contour2.contains(&b) {
                return Some([a, b]);
            }
        }

        None
    }
}

#[derive(Clone, Debug, Default)]
pub struct DelaunayMultimesh<K> {
    layers: Vec<DelaunayMesh<K>>,
}

impl<K> DelaunayMultimesh<K> {
    pub fn layers(&self) -> &[DelaunayMesh<K>] {
        &self.layers
    }
}
