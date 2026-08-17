// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0
//! Refinement methods for batches of obstacles

use core::slice;
use i_triangle::i_overlay::{
    core::{
        extract::BooleanExtractionBuffer, fill_rule::FillRule, integer::OverlayInt,
        overlay::Overlay, overlay_rule::OverlayRule,
    },
    i_float::int::number::int::IntNumber,
    i_shape::int::IntPoint,
};
use i_triangle::int::triangulatable::IntTriangulatable as _;
use rstar::{AABB, RTree, RTreeNum, RTreeObject, RTreeParams};

use super::LayerIds;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Obstacle<Scalar: IntNumber> {
    shape: Vec<Vec<IntPoint<Scalar>>>,
    layers: LayerIds,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ConvexObstacle<Scalar: IntNumber> {
    contour: Vec<IntPoint<Scalar>>,
    layers: LayerIds,
}

impl<Scalar: IntNumber + RTreeNum> RTreeObject for Obstacle<Scalar> {
    type Envelope = AABB<[Scalar; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_points(
            self.shape
                .iter()
                .flat_map(|i| i.iter())
                .map(|i| [i.x, i.y])
                .collect::<Vec<_>>()
                .iter(),
        )
    }
}

impl<Scalar: IntNumber + RTreeNum> RTreeObject for ConvexObstacle<Scalar> {
    type Envelope = AABB<[Scalar; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_points(
            self.contour
                .iter()
                .map(|i| [i.x, i.y])
                .collect::<Vec<_>>()
                .iter(),
        )
    }
}

/// Makes obstacles convex
struct Collector<Scalar: IntNumber + RTreeNum, Params: RTreeParams>(
    RTree<ConvexObstacle<Scalar>, Params>,
);

impl<Scalar, Params> Extend<Obstacle<Scalar>> for Collector<Scalar, Params>
where
    Scalar: IntNumber + RTreeNum + OverlayInt,
    Params: RTreeParams,
{
    fn extend<T: IntoIterator<Item = Obstacle<Scalar>>>(&mut self, iter: T) {
        for i in iter.into_iter().flat_map(|Obstacle { shape, layers }| {
            shape
                .triangulate()
                .into_delaunay()
                .to_convex_polygons()
                .into_iter()
                .map(move |contour| ConvexObstacle {
                    contour,
                    layers: layers.clone(),
                })
        }) {
            self.0.insert(i);
        }
    }
}

/// Turn an [`RTree`] of [`Obstacle`]s into one where none of those obstacles overlap,
/// and such that all of them are convex (such that they're compatible to [`crate::funnel::SimpleFunnel`].
pub fn make_non_overlapping_and_convex<
    Scalar: IntNumber + RTreeNum + OverlayInt,
    Params: RTreeParams,
>(
    objects: RTree<Obstacle<Scalar>, Params>,
) -> RTree<ConvexObstacle<Scalar>, Params> {
    let mut ret = Collector::<Scalar, Params>(RTree::new_with_params());
    let mut bx_buffer = BooleanExtractionBuffer::default();

    // Make obstacles non-overlapping
    for i in objects {
        let i_layers = i.layers.clone();
        let i_envelope = i.envelope();
        let mut i = vec![i.shape.clone()];
        let mut tmp = Vec::new();
        for inters in ret.0.drain_in_envelope_intersecting(i_envelope) {
            let mut overlay =
                Overlay::with_shapes(&i, slice::from_ref(&vec![inters.contour.clone()]));
            let local_graph = overlay
                .build_graph_view(FillRule::EvenOdd)
                .expect("unable to build graph view");
            let inters2 = local_graph.extract_shapes(OverlayRule::Intersect, &mut bx_buffer);
            if inters2.is_empty() {
                tmp.push(Obstacle {
                    shape: vec![inters.contour],
                    layers: inters.layers,
                });
                continue;
            }
            // we have an intersection, process it
            let inters2_layers: LayerIds = (&i_layers) | (&inters.layers);
            tmp.extend(inters2.into_iter().map(|shape| Obstacle {
                shape,
                layers: inters2_layers.clone(),
            }));
            // update involved obstacles
            i = local_graph.extract_shapes(OverlayRule::Difference, &mut bx_buffer);
            tmp.extend(
                local_graph
                    .extract_shapes(OverlayRule::InverseDifference, &mut bx_buffer)
                    .into_iter()
                    .map(|shape| Obstacle {
                        shape,
                        layers: inters.layers.clone(),
                    }),
            );
        }
        ret.extend(tmp);
        ret.extend(i.into_iter().map(|shape| Obstacle {
            shape,
            layers: i_layers.clone(),
        }));
    }
    ret.0
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::LayerId;
    use i_triangle::i_overlay::i_shape::int_path;

    #[test]
    fn two_rectangles() {
        let mut rtree = RTree::new();
        rtree.insert(Obstacle {
            shape: vec![int_path![[0i32, 0], [2, 0], [2, 2], [0, 2]]],
            layers: {
                let mut layers = LayerIds::default();
                layers.insert(LayerId::from(0));
                layers
            },
        });
        rtree.insert(Obstacle {
            shape: vec![int_path![[1i32, 1], [3, 1], [3, 3], [1, 3]]],
            layers: {
                let mut layers = LayerIds::default();
                layers.insert(LayerId::from(1));
                layers
            },
        });

        let refined = make_non_overlapping_and_convex(rtree);
        assert!(refined.contains(&ConvexObstacle {
            contour: int_path![[2i32, 2], [1, 2], [1, 1], [2, 1]],
            layers: {
                let mut layers = LayerIds::default();
                layers.insert(LayerId::from(0));
                layers.insert(LayerId::from(1));
                layers
            },
        }));
    }
}
