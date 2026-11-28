// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use alloc::vec::Vec;
use core::iter::{ExactSizeIterator, IntoIterator, once};

use i_triangle::i_overlay::core::integer::OverlayInt;
use i_triangle::i_overlay::i_float::int::number::int::IntNumber;
use numlike::types::Vector2;
use polygon_unionfind::{
    Contours, Difference, Inflate, Inflated, Intersection, Laminate, Negated, Paralleled,
    PolygonSet, PolygonUnionFind, PolygonWithData, Scalar, Union,
};

use crate::delaunay::{DelaunayLayerId, DelaunayMesh, MultilayerDelaunayMesh};

pub struct Navmesher<K: Scalar, D> {
    laminate: Laminate<K, PolygonWithData<K, D>>,
    navmeshes: Vec<MultilayerDelaunayMesh<K>>,
}

impl<K: Scalar, D> Navmesher<K, D> {
    pub fn navmeshes(&self) -> &[MultilayerDelaunayMesh<K>] {
        &self.navmeshes
    }

    pub fn laminate(&self) -> &Laminate<K, PolygonWithData<K, D>> {
        &self.laminate
    }
}

impl<K, D> Navmesher<K, D>
where
    K: Scalar + Default + Clone + Ord + IntNumber + OverlayInt,
    D: Clone + Default,
    PolygonWithData<K, D>: Clone
        + Contours<K>
        + Inflate<K>
        + Union<PolygonWithData<K, D>>
        + Difference<PolygonWithData<K, D>>
        + Intersection<PolygonWithData<K, D>>,
{
    pub fn new<PI>(
        boundary: impl IntoIterator<Item = [K; 2]>,
        num_layers: usize,
        parallel_inflations: PI,
        rail_offsets: impl IntoIterator<Item = K>,
    ) -> Self
    where
        PI: IntoIterator<Item = K>,
        PI::IntoIter: ExactSizeIterator,
    {
        let parallel_inflations = parallel_inflations.into_iter();
        let row_count = parallel_inflations.len() + 1;

        let boundary = PolygonWithData {
            exterior: boundary.into_iter().collect(),
            interiors: Vec::new(),
            data: D::default(),
        };

        let laminate = Laminate::<K, PolygonWithData<K, D>>::new(
            boundary,
            num_layers,
            parallel_inflations,
            rail_offsets,
        );

        let navmeshes: Vec<_> = (0..row_count)
            .map(|row| {
                MultilayerDelaunayMesh::from_layers(
                    shapes_per_layer(&laminate, row, 0)
                        .map(|shapes| DelaunayMesh::new(shapes_to_polygons_with_holes(shapes)))
                        .collect(),
                )
            })
            .collect();

        Self {
            laminate,
            navmeshes,
        }
    }

    pub fn insert_polygon(&mut self, layer: crate::LayerId, polygon: PolygonWithData<K, D>) {
        self.laminate.add_into_lamina(usize::from(layer), polygon);
        let lamina = &self.laminate.laminas()[usize::from(layer)];

        for (row, navmesh) in self.navmeshes.iter_mut().enumerate() {
            *navmesh.layer_mut(DelaunayLayerId::new(usize::from(layer))) = DelaunayMesh::new(
                shapes_to_polygons_with_holes(shapes_for_lamina(lamina, row, 0)),
            );
        }
    }
}

fn shapes_to_polygons_with_holes<K: Clone>(
    shapes: Vec<Vec<Vec<[K; 2]>>>,
) -> Vec<Vec<Vec<Vector2<K>>>> {
    shapes
        .into_iter()
        .map(|shape| {
            shape
                .into_iter()
                .map(|contour| {
                    contour
                        .into_iter()
                        .map(|p| Vector2::new(p[0].clone(), p[1].clone()))
                        .collect()
                })
                .collect()
        })
        .collect()
}

fn shapes_per_layer<K: Scalar, D>(
    laminate: &Laminate<K, PolygonWithData<K, D>>,
    row: usize,
    subrow: usize,
) -> impl ExactSizeIterator<Item = Vec<Vec<Vec<[K; 2]>>>> + '_ {
    laminate
        .laminas()
        .iter()
        .map(move |lamina| shapes_for_lamina(lamina, row, subrow))
}

fn shapes_for_lamina<K: Scalar, D>(
    lamina: &Paralleled<
        Paralleled<
            Negated<
                PolygonSet<K, PolygonWithData<K, D>>,
                Inflated<PolygonUnionFind<K, PolygonWithData<K, D>>, K>,
            >,
        >,
    >,
    row: usize,
    subrow: usize,
) -> Vec<Vec<Vec<[K; 2]>>> {
    lamina
        .row(row)
        .row(subrow)
        .minuend()
        .polygons()
        .values()
        .map(|polygon| {
            once(polygon.exterior().to_vec())
                .chain(polygon.interiors().map(|interior| interior.to_vec()))
                .collect()
        })
        .collect()
}
