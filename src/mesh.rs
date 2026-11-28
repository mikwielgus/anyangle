// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use numlike::types::Vector2;

pub trait Faced {
    type FaceId;
}

pub trait FaceContourConstN<K, const N: usize>: Faced {
    fn face_contour(&self, face: &Self::FaceId) -> &[Vector2<K>; N];
}

pub trait FaceAdjacentsConstN<const N: usize>: Faced {
    fn face_adjacents(&self, face: &Self::FaceId) -> &[Option<Self::FaceId>; N];
}

pub trait FacesSharedVerticesConstN<K, const N: usize>: Faced {
    fn faces_shared_vertices(
        &self,
        face1: &Self::FaceId,
        face2: &Self::FaceId,
    ) -> Option<[Vector2<K>; N]>;
}

pub trait Layered {
    type LayerId;
}
