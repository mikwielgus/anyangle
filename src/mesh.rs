// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use numlike::types::Vector2;

pub trait Faces {
    type FaceId;
}

pub trait FaceAdjacentsConstN<const N: usize>: Faces {
    fn face_adjacents(&self, face: &Self::FaceId) -> [Self::FaceId; N];
}

pub trait FaceContourConstN<K, const N: usize>: Faces {
    fn face_contour(&self, face: &Self::FaceId) -> [Vector2<K>; N];
}

pub trait SharedEdge<K>: Faces {
    fn shared_edge(&self, face1: &Self::FaceId, face2: &Self::FaceId) -> [Vector2<K>; 2];
}
