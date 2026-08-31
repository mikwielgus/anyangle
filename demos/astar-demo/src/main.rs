// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use anyangle::{
    flat::{
        FrozenTesselation, GetLayerIds, LayerIds, Tesselation,
        astar::{Endpoint, FunnelEntry, Node, astar},
    },
    math::diagonal_taxicab::DiagonalTaxicabNorm,
};
use core::ops::ControlFlow;
use macroquad::prelude::*;
use rstar::{AABB, RTree, RTreeObject};

type Scalar = i32;
const LAYER_WEIGHT: f32 = 1.0;

#[derive(Clone, Copy, Debug)]
struct Viewport {
    // `scaling` is stored in logarithmic scale
    scaling: f32,
    offset: [f32; 2],
}

impl Viewport {
    fn translate(&self, pt: &[Scalar; 2]) -> Vec2 {
        Vec2 {
            x: (pt[0] as f32) * self.scaling.exp() + self.offset[0],
            y: (pt[1] as f32) * self.scaling.exp() + self.offset[1],
        }
    }

    #[allow(dead_code)]
    fn translate_inv(&self, pt: &Vec2) -> [Scalar; 2] {
        [
            ((pt.x - self.offset[0]) / self.scaling.exp()) as Scalar,
            ((pt.y - self.offset[1]) / self.scaling.exp()) as Scalar,
        ]
    }

    fn scroll_at(&mut self, pt: &(f32, f32), mut delta: f32) {
        self.scaling += delta;
        let clamped = self.scaling.clamp(-20., 20.);
        if (self.scaling - clamped).abs() > f32::EPSILON * 16.0 {
            delta += self.scaling - clamped;
            self.scaling = clamped;
        }

        // fix point `pt`, affine combination of `pt` and `self.offset`
        let delta_exp = delta.exp();
        self.offset = [
            pt.0 * (1. - delta_exp) + self.offset[0] * delta_exp,
            pt.1 * (1. - delta_exp) + self.offset[1] * delta_exp,
        ];
    }
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
struct Demo {
    ext_boundary: Vec<[Scalar; 2]>,
    obstacles: Vec<Obstacle>,
    endpoints: [Obstacle; 2],
    norm: Norm,
    layer_transition_penality: Scalar,
}

#[derive(
    Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Deserialize, serde::Serialize,
)]
#[serde(rename_all = "kebab-case")]
enum Norm {
    Euclidean,
    DiagonalTaxicab,
}

impl Norm {
    fn fun(&self) -> fn(&[Scalar; 2]) -> DiagonalTaxicabNorm<Scalar> {
        match self {
            Norm::Euclidean => |pt| DiagonalTaxicabNorm {
                // TODO: make this exact
                along_axis: (pt[0] * pt[0] + pt[1] * pt[1]).isqrt(),
                diagonal: 0,
            },
            Norm::DiagonalTaxicab => {
                |pt| DiagonalTaxicabNorm::new(*pt).expect("values should always be in range")
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, serde::Deserialize, serde::Serialize)]
struct Obstacle {
    // TODO: networks and margins/inflations
    /// Invariant: `!exterior.is_empty()`.
    pub exterior: Vec<[Scalar; 2]>,
    /// Invariant: `!layers.is_empty()`.
    pub layers: LayerIds,
    //pub marked: bool,
}

impl GetLayerIds for Obstacle {
    fn layers(&self) -> LayerIds {
        self.layers.clone()
    }
}

impl RTreeObject for Obstacle {
    type Envelope = AABB<[Scalar; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_points(&self.exterior)
    }
}

async fn draw_navmesh(navmesh: &FrozenTesselation<Scalar, LayerIds>, viewport: &Viewport) {
    const LAYER_ALPHA_FACTOR: f32 = LAYER_WEIGHT * 2. / core::f32::consts::PI;
    for face in navmesh.faces() {
        let color = Color::new(
            0.,
            1.,
            1.,
            (face.data.0.count() as f32).atan() * LAYER_ALPHA_FACTOR,
        );
        for v in face
            .contour
            .iter()
            .chain(face.contour.first())
            .map(|&i| viewport.translate(&navmesh.vertices()[i as usize]))
            .collect::<Vec<_>>()
            .windows(2)
        {
            draw_line(v[0].x, v[0].y, v[1].x, v[1].y, 1., color);
        }
    }
}

#[macroquad::main("anyangle A* demo")]
async fn main() {
    let mut viewport = Viewport {
        scaling: 1.,
        offset: [0., 0.],
    };

    let demo = std::fs::read(
        std::env::args()
            .nth(1)
            .expect("Expected one command line argument (demo filename)"),
    )
    .expect("Unable to read demo file");
    let demo: Demo = toml::from_slice(&demo[..]).expect("Unable to parse demo file");

    let all_layers: LayerIds = demo
        .endpoints
        .iter()
        .chain(demo.obstacles.iter())
        .map(|i| &i.layers)
        .collect();
    let rtree = RTree::bulk_load(demo.obstacles);

    let mut navmesh = Tesselation::<_, _>::default();

    navmesh.allocate_shapes(vec![vec![
        demo.ext_boundary.iter().copied().map(Into::into).collect(),
    ]]);

    for obstacle in &rtree {
        let contour: Vec<_> = obstacle.exterior.iter().copied().map(Into::into).collect();
        navmesh.allocate_shapes(vec![vec![contour.clone()]]);
        let _ = navmesh.update_data(&contour, |_, layers| {
            *layers |= &obstacle.layers;
            ControlFlow::<()>::Continue(())
        });
    }

    {
        let root_envelope = navmesh.envelope();
        // make amount of faces minimal
        navmesh.optimize_envelope(root_envelope);
        // invert the layers for astar
        let _ = navmesh.update_data(
            &[
                root_envelope.lower(),
                [root_envelope.lower()[0], root_envelope.upper()[1]],
                root_envelope.upper(),
                [root_envelope.upper()[0], root_envelope.lower()[1]],
            ]
            .map(Into::into),
            |_, layers| {
                *layers = &*layers ^ &all_layers;
                ControlFlow::<()>::Continue(())
            },
        );
    }
    navmesh.rebalance();

    println!("navmesh: {:#?}", navmesh);

    let navmesh = navmesh.freeze();

    // find out where the endpoints are
    let endpoints = demo.endpoints.map(|i| Endpoint {
        fixed: i
            .exterior
            .iter()
            .map(|need_vertex| {
                navmesh
                    .vertices()
                    .iter()
                    .enumerate()
                    .find(|&(_, vertex)| need_vertex == vertex)
                    .unwrap()
                    .0 as u32
            })
            .collect(),
        layers: i.layers,
    });

    let mut astar_data = astar(
        &navmesh,
        demo.norm.fun(),
        endpoints[0].clone(),
        endpoints[1].clone(),
        0,
        DiagonalTaxicabNorm {
            along_axis: demo.layer_transition_penality,
            diagonal: 0,
        },
    );

    let astar_result = loop {
        let Some(tmp) = astar_data.next() else {
            break Vec::new();
        };
        match tmp {
            anyangle::flat::astar::Output::Result(res) => break res,
            // TODO: visualize intermediates
            _ => {}
        }
    };

    println!("astar result:");
    for i in &astar_result {
        print!("  - ");
        use anyangle::flat::{
            Topo2DComplex,
            astar::{FunnelEntry, Node},
        };
        match i {
            FunnelEntry::Point(Node { fixed, layer }) => {
                println!(
                    "point {:?} on layer {layer:?}",
                    navmesh.vertex_position(*fixed)
                );
            }
            FunnelEntry::LayerTransition(from_layer, to_layer) => {
                println!("layer transition from {from_layer:?} to {to_layer:?}");
            }
        }
    }
    println!();

    loop {
        // handle input

        // - wheel
        {
            let (_, wheel) = mouse_wheel();
            if wheel.abs() >= f32::EPSILON {
                viewport.scroll_at(&mouse_position(), wheel);
            }
        }

        // draw stuff

        clear_background(BLACK);

        draw_navmesh(&navmesh, &viewport).await;

        let mut last_point: Option<Node<u32>> = None;
        let mut encountered_layer_transition = false;
        for i in &astar_result {
            match i {
                FunnelEntry::LayerTransition(_, _) => {
                    encountered_layer_transition = true;
                }
                FunnelEntry::Point(pt) => {
                    if let Some(last_pt) = last_point {
                        let points = [last_pt.fixed, pt.fixed]
                            .map(|fixed| viewport.translate(&navmesh.vertices()[fixed as usize]));
                        draw_line(
                            points[0][0],
                            points[0][1],
                            points[1][0],
                            points[1][1],
                            1.0,
                            if encountered_layer_transition {
                                MAGENTA
                            } else {
                                RED
                            },
                        );
                        encountered_layer_transition &= last_pt.fixed == pt.fixed;
                    } else {
                        encountered_layer_transition = false;
                    }
                    last_point = Some(*pt);
                }
            }
        }

        next_frame().await
    }
}
