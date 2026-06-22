// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::iter::{ExactSizeIterator, IntoIterator, once};

use polygon_unionfind::{
    Difference, Inflate, Inflated, Intersection, Laminate, Negated, Paralleled, PolygonSet,
    PolygonUnionFind, PolygonWithData, Rings, Union,
};
use rstar::RTreeNum;

use crate::navmesh::Remesh;

pub struct Navmesher<K: RTreeNum, N, D> {
    laminate: Laminate<K, PolygonWithData<K, D>>,
    navmeshes: Vec<N>,
}

impl<K: RTreeNum, N, D> Navmesher<K, N, D> {
    pub fn navmeshes(&self) -> &[N] {
        &self.navmeshes
    }

    pub fn laminate(&self) -> &Laminate<K, PolygonWithData<K, D>> {
        &self.laminate
    }
}

impl<K, N, D> Navmesher<K, N, D>
where
    K: RTreeNum + Default + Clone + Ord,
    N: Default + Remesh<K>,
    D: Clone + Default,
    PolygonWithData<K, D>: Clone
        + Rings<K>
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
                let mut navmesh = N::default();
                navmesh.remesh(shapes_per_layer(&laminate, row, 0));
                navmesh
            })
            .collect();

        Self {
            laminate,
            navmeshes,
        }
    }

    pub fn insert_polygon(&mut self, layer: usize, polygon: PolygonWithData<K, D>) {
        self.laminate.add_into_lamina(layer, polygon);
        let lamina = &self.laminate.laminas()[layer];

        for (row, navmesh) in self.navmeshes.iter_mut().enumerate() {
            navmesh.remesh_at(layer, shapes_for_lamina(lamina, row, 0));
        }
    }
}

fn shapes_per_layer<K: RTreeNum, D>(
    laminate: &Laminate<K, PolygonWithData<K, D>>,
    row: usize,
    subrow: usize,
) -> impl Iterator<Item = Vec<Vec<Vec<[K; 2]>>>> + ExactSizeIterator + '_ {
    laminate
        .laminas()
        .iter()
        .map(move |lamina| shapes_for_lamina(lamina, row, subrow))
}

fn shapes_for_lamina<K: RTreeNum, D>(
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
