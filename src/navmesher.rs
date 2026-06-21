// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::iter::{ExactSizeIterator, IntoIterator};

use polygon_unionfind::{
    Difference, Inflate, Intersection, Laminate, PolygonWithData, Rings, Union,
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

        let mut navmeshes = vec![];

        for row in 0..row_count {
            let mut navmesh = N::default();
            navmesh.remesh(&Self::shapes_per_layer(&laminate, row, 0));
            navmeshes.push(navmesh);
        }

        Self {
            laminate,
            navmeshes,
        }
    }

    pub fn insert_polygon(&mut self, layer: usize, polygon: PolygonWithData<K, D>) {
        self.laminate.add_into_lamina(layer, polygon);

        for (row, navmesh) in self.navmeshes.iter_mut().enumerate() {
            navmesh.remesh(&Self::shapes_per_layer(&self.laminate, row, 0));
        }
    }

    fn shapes_per_layer(
        laminate: &Laminate<K, PolygonWithData<K, D>>,
        row: usize,
        subrow: usize,
    ) -> Vec<Vec<Vec<Vec<[K; 2]>>>> {
        let mut shapes_per_layer = Vec::new();

        for lamina in laminate.laminas().iter() {
            let polygon_set = lamina.row(row).row(subrow).minuend();
            let mut shape = vec![];

            for polygon in polygon_set.polygons().values() {
                let mut contours = vec![polygon.exterior().to_vec()];

                for interior in polygon.interiors() {
                    contours.push(interior.to_vec());
                }

                shape.push(contours);
            }

            shapes_per_layer.push(shape);
        }

        shapes_per_layer
    }
}
