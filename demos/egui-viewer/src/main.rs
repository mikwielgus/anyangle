// SPDX-FileCopyrightText: 2026 anyangle contributors
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use anyangle::{
    LayerId,
    flat::{
        FrozenTesselation, LayerIds, Tesselation, Topo2DComplex,
        astar::{Endpoint, FunnelEntry, Output, astar},
    },
    math::diagonal_taxicab::DiagonalTaxicabNorm,
    mlfa::{Contours, DelaunayNavmesh, Navmesher, PolygonWithData},
};
use core::ops::ControlFlow;
use eframe::egui::{self, Color32, Pos2, Sense, Stroke, Ui, Vec2};
use rstared::RTreeObject;
use std::{collections::BTreeSet, f32::consts::TAU};

const NUM_LAYERS: usize = 4;

fn layer_color(layer_index: usize) -> Color32 {
    let hue = (layer_index as f32 / NUM_LAYERS.max(1) as f32) * 360.0;
    let (r, g, b) = hsv_to_rgb(hue, 0.72, 1.0);

    Color32::from_rgb((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8)
}

fn interlamina_color(inter_index: usize, inter_count: usize) -> Color32 {
    let hue = (inter_index as f32 / inter_count.max(1) as f32) * 110.0 + 125.0;
    let (r, g, b) = hsv_to_rgb(hue, 0.55, 1.0);

    Color32::from_rgb((r * 255.0) as u8, (g * 255.0) as u8, (b * 255.0) as u8)
}

fn hsv_to_rgb(h: f32, s: f32, v: f32) -> (f32, f32, f32) {
    let c = v * s;
    let hh = (h / 60.0).rem_euclid(6.0);
    let x = c * (1.0 - (hh % 2.0 - 1.0).abs());
    let m = v - c;

    let (rp, gp, bp) = match hh as i32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };

    (rp + m, gp + m, bp + m)
}

fn with_alpha(color: Color32, alpha: u8) -> Color32 {
    Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), alpha)
}

fn cross_i64(o: [i32; 2], a: [i32; 2], b: [i32; 2]) -> i64 {
    let oa_x = (a[0] - o[0]) as i64;
    let oa_y = (a[1] - o[1]) as i64;
    let ob_x = (b[0] - o[0]) as i64;
    let ob_y = (b[1] - o[1]) as i64;

    oa_x * ob_y - oa_y * ob_x
}

fn convex_hull(mut points: Vec<[i32; 2]>) -> Vec<[i32; 2]> {
    points.sort_by(|p, q| p[0].cmp(&q[0]).then_with(|| p[1].cmp(&q[1])));
    points.dedup_by(|a, b| a[0] == b[0] && a[1] == b[1]);

    if points.len() <= 2 {
        return points;
    }

    let mut lower = Vec::new();

    for &p in &points {
        while lower.len() >= 2 && cross_i64(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0
        {
            lower.pop();
        }

        lower.push(p);
    }

    let mut upper = Vec::new();

    for &p in points.iter().rev() {
        while upper.len() >= 2 && cross_i64(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0
        {
            upper.pop();
        }

        upper.push(p);
    }

    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

struct XorShift64(u64);

impl XorShift64 {
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;

        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    fn next_f32(&mut self) -> f32 {
        (self.next_u64() as f32) / (u64::MAX as f32)
    }

    fn next_range_i32(&mut self, lo: i32, hi: i32) -> i32 {
        if hi <= lo {
            return lo;
        }

        lo + (self.next_u64() % (hi - lo + 1) as u64) as i32
    }
}

fn random_convex_polygon(
    rng: &mut XorShift64,
    center: [i32; 2],
    radius_max: i32,
    count: usize,
) -> PolygonWithData<i32, ()> {
    let mut pts = Vec::with_capacity(count);

    for _ in 0..count {
        let angle = rng.next_f32() * TAU;
        let rf = radius_max as f32 * (0.35 + 0.65 * rng.next_f32());
        let x = center[0] as f32 + rf * angle.cos();
        let y = center[1] as f32 + rf * angle.sin();

        pts.push([x.round() as i32, y.round() as i32]);
    }

    let mut hull = convex_hull(pts);

    if hull.len() < 3 {
        let r = radius_max.max(12);

        hull = vec![
            [center[0] - r, center[1] - r],
            [center[0] + r, center[1] - r],
            [center[0], center[1] + r],
        ];
    }

    PolygonWithData {
        exterior: hull,
        interiors: Vec::new(),
        data: (),
    }
}

struct ViewerApp {
    boundary: Vec<[i32; 2]>,
    holes: Vec<(usize, Vec<[i32; 2]>)>,
    navmesher: Navmesher<i32, DelaunayNavmesh<i32>, ()>,
    layer_visible: Vec<bool>,
    interlamina_visible: Vec<bool>,
    active_layer: usize,
    active_parallel_row: usize,
    zoom: f32,
    pan: Vec2,
    rng: XorShift64,
    origin: Option<[i32; 2]>,
    destination: Option<[i32; 2]>,
    path: Option<Vec<[i32; 2]>>,
    path_status: String,
    frozen_navmesh: Option<FrozenTesselation<i32, LayerIds>>,
    is_tesselation_visible: bool,
}

impl ViewerApp {
    fn new() -> Self {
        let boundary = vec![[0_i32, 0], [1200, 0], [1100, 900], [600, 1000], [80, 820]];
        let parallel_inflations = [20_i32];
        let rail_offsets = [5_i32];

        let navmesher = Navmesher::<i32, DelaunayNavmesh<i32>, ()>::new(
            boundary.iter().copied(),
            NUM_LAYERS,
            parallel_inflations,
            rail_offsets,
        );

        let n_layers = navmesher.navmeshes()[0].layers().len();
        let n_inter = n_layers.saturating_sub(1);

        Self {
            boundary,
            holes: Vec::new(),
            navmesher,
            layer_visible: vec![true; n_layers],
            interlamina_visible: vec![true; n_inter],
            active_layer: 0,
            active_parallel_row: 0,
            zoom: 0.7,
            pan: Vec2::new(40.0, 760.0),
            rng: XorShift64(0xC0FFEE_u64),
            origin: None,
            destination: None,
            path: None,
            path_status: String::new(),
            frozen_navmesh: None,
            is_tesselation_visible: true,
        }
    }

    fn all_layers() -> LayerIds {
        let mut layers = LayerIds::default();

        for i in 0..NUM_LAYERS {
            layers.insert(LayerId::from(i));
        }

        layers
    }

    fn clear_path(&mut self) {
        self.path = None;
        self.path_status.clear();
        self.frozen_navmesh = None;
    }

    fn find_path(&mut self) {
        let Some(origin) = self.origin else {
            self.path = None;
            self.path_status = "set an origin first".into();
            return;
        };
        let Some(destination) = self.destination else {
            self.path = None;
            self.path_status = "set a destination first".into();
            return;
        };

        let mut tess = Tesselation::<i32, LayerIds>::default();

        tess.allocate_shapes(vec![vec![
            self.boundary.iter().copied().map(Into::into).collect(),
        ]]);

        for (layer, exterior) in &self.holes {
            let contour: Vec<_> = exterior.iter().copied().map(Into::into).collect();

            tess.allocate_shapes(vec![vec![contour.clone()]]);

            let layer_id = LayerId::from(*layer);
            let _ = tess.update_data(&contour, |_, layers| {
                layers.insert(layer_id);
                ControlFlow::<()>::Continue(())
            });
        }

        let all_layers = Self::all_layers();
        let root_envelope = tess.envelope();
        let invert_contour = [
            root_envelope.lower(),
            [root_envelope.lower()[0], root_envelope.upper()[1]],
            root_envelope.upper(),
            [root_envelope.upper()[0], root_envelope.lower()[1]],
        ]
        .map(Into::into);

        tess.optimize_envelope(root_envelope);

        let _ = tess.update_data(&invert_contour, |_, layers| {
            *layers = &*layers ^ &all_layers;
            ControlFlow::<()>::Continue(())
        });

        tess.rebalance();

        self.frozen_navmesh = Some(tess.freeze());
        let frozen = self.frozen_navmesh.as_ref().unwrap();
        let snap = |need: [i32; 2]| -> Option<u32> {
            frozen
                .vertices()
                .iter()
                .enumerate()
                .min_by_key(|(_, vertex)| {
                    let dx = (vertex[0] - need[0]) as i64;
                    let dy = (vertex[1] - need[1]) as i64;

                    dx * dx + dy * dy
                })
                .map(|(index, _)| index as u32)
        };

        let Some(start_id) = snap(origin) else {
            self.path = None;
            self.path_status = "flat navmesh has no vertices".into();
            return;
        };
        let Some(end_id) = snap(destination) else {
            self.path = None;
            self.path_status = "flat navmesh has no vertices".into();
            return;
        };

        let start = Endpoint {
            fixed: BTreeSet::from([start_id]),
            layers: all_layers.clone(),
        };
        let end = Endpoint {
            fixed: BTreeSet::from([end_id]),
            layers: all_layers,
        };
        let norm = |pt: &[i32; 2]| {
            DiagonalTaxicabNorm::new(*pt).expect("diagonal taxicab values should be in range")
        };
        let mut search = astar(
            frozen,
            norm,
            start,
            end,
            0,
            DiagonalTaxicabNorm {
                along_axis: 10,
                diagonal: 0,
            },
        );

        for step in &mut search {
            if let Output::Result(entries) = step {
                let mut points = Vec::new();

                for entry in entries {
                    if let FunnelEntry::Point(node) = entry {
                        points.push(frozen.vertex_position(node.fixed));
                    }
                }

                if points.is_empty() {
                    self.path = None;
                    self.path_status = "astar returned an empty path".into();
                } else {
                    self.path = Some(points);
                    self.path_status = "path found".into();
                }

                return;
            }
        }

        self.path = None;
        self.path_status = "no path found".into();
    }

    fn collect_vertices(&self) -> Vec<[i32; 2]> {
        let mut vertices = Vec::new();
        let mesh = &self.navmesher.navmeshes()[self.active_parallel_row];

        for (layer_i, layer) in mesh.layers().iter().enumerate() {
            if !self.layer_visible[layer_i] {
                continue;
            }

            for tri in layer.triangulation().vertices() {
                vertices.extend_from_slice(tri);
            }
        }

        for (layer_i, lamina) in self.navmesher.laminate().laminas().iter().enumerate() {
            if !self.layer_visible[layer_i] {
                continue;
            }

            let polygon_set = lamina.row(self.active_parallel_row).row(0).minuend();

            for polygon in polygon_set.polygons().values() {
                vertices.extend_from_slice(polygon.exterior());

                for interior in polygon.interiors() {
                    vertices.extend_from_slice(interior);
                }
            }
        }

        vertices.sort_unstable();
        vertices.dedup();
        vertices
    }

    fn nearest_vertex(&self, screen: Pos2, max_screen_dist: f32) -> Option<[i32; 2]> {
        let max_dist_sq = max_screen_dist * max_screen_dist;
        let mut best: Option<([i32; 2], f32)> = None;

        for &vertex in &self.collect_vertices() {
            let p = self.world_to_screen(vertex);
            let dist_sq = (p.x - screen.x).powi(2) + (p.y - screen.y).powi(2);

            if dist_sq <= max_dist_sq
                && best
                    .as_ref()
                    .is_none_or(|(_, best_dist_sq)| dist_sq < *best_dist_sq)
            {
                best = Some((vertex, dist_sq));
            }
        }

        best.map(|(vertex, _)| vertex)
    }

    fn world_to_screen(&self, p: [i32; 2]) -> Pos2 {
        Pos2::new(
            self.pan.x + p[0] as f32 * self.zoom,
            self.pan.y - p[1] as f32 * self.zoom,
        )
    }

    fn screen_to_world(&self, screen: Pos2) -> [i32; 2] {
        let wx = (screen.x - self.pan.x) / self.zoom;
        let wy = (self.pan.y - screen.y) / self.zoom;

        [wx.round() as i32, wy.round() as i32]
    }

    fn draw_closed_polyline(&self, painter: &egui::Painter, points: &[[i32; 2]], stroke: Stroke) {
        if points.len() < 2 {
            return;
        }

        let screen: Vec<Pos2> = points.iter().map(|&p| self.world_to_screen(p)).collect();

        painter.add(egui::Shape::closed_line(screen, stroke));
    }

    fn paint_scene(&self, painter: &egui::Painter) {
        let mesh = &self.navmesher.navmeshes()[self.active_parallel_row];

        let n_inter = self.interlamina_visible.len();

        for (inter_i, &is_visible) in self.interlamina_visible.iter().enumerate() {
            if !is_visible {
                continue;
            }

            let accent = interlamina_color(inter_i, n_inter.max(1));
            let stroke = Stroke::new(4.25, with_alpha(accent, 235));
            let inter = &self.navmesher.laminate().interlaminas()[inter_i];

            for polygon in inter.main().polygons().values() {
                self.draw_closed_polyline(painter, polygon.exterior(), stroke);

                for interior in polygon.interiors() {
                    self.draw_closed_polyline(
                        painter,
                        interior,
                        Stroke::new(3.0, with_alpha(accent, 220)),
                    );
                }
            }
        }

        for (layer_i, layer) in mesh.layers().iter().enumerate().rev() {
            if !self.layer_visible[layer_i] {
                continue;
            }

            let fill = layer_color(layer_i);
            let fill_a = with_alpha(fill, if layer_i == self.active_layer { 50 } else { 30 });
            let stroke = Stroke::new(
                if layer_i == self.active_layer {
                    1.0
                } else {
                    0.6
                },
                with_alpha(fill, 140),
            );

            for verts in layer.triangulation().vertices() {
                let tri = [
                    self.world_to_screen(verts[0]),
                    self.world_to_screen(verts[1]),
                    self.world_to_screen(verts[2]),
                ];

                painter.add(egui::Shape::convex_polygon(tri.to_vec(), fill_a, stroke));
            }
        }

        if self.is_tesselation_visible
            && let Some(frozen) = &self.frozen_navmesh
        {
            let stroke = Stroke::new(1.25, Color32::from_gray(160));

            for (_face, face_vertices) in frozen.faces_coordinates() {
                let screen: Vec<Pos2> = face_vertices.map(|p| self.world_to_screen(*p)).collect();

                if screen.len() >= 2 {
                    painter.add(egui::Shape::closed_line(screen, stroke));
                }
            }
        }

        if let Some(origin) = self.origin {
            painter.circle_filled(
                self.world_to_screen(origin),
                7.0,
                Color32::from_rgb(80, 220, 120),
            );
            painter.circle_stroke(
                self.world_to_screen(origin),
                7.0,
                Stroke::new(2.0, Color32::WHITE),
            );
        }

        if let Some(destination) = self.destination {
            painter.circle_filled(
                self.world_to_screen(destination),
                7.0,
                Color32::from_rgb(240, 90, 90),
            );
            painter.circle_stroke(
                self.world_to_screen(destination),
                7.0,
                Stroke::new(2.0, Color32::WHITE),
            );
        }

        if let Some(path) = &self.path {
            let screen: Vec<Pos2> = path.iter().map(|&p| self.world_to_screen(p)).collect();

            if screen.len() >= 2 {
                painter.add(egui::Shape::line(
                    screen,
                    Stroke::new(3.0, Color32::from_rgb(255, 220, 60)),
                ));
            }
        } else if let (Some(origin), Some(destination)) = (self.origin, self.destination) {
            painter.line_segment(
                [
                    self.world_to_screen(origin),
                    self.world_to_screen(destination),
                ],
                Stroke::new(1.5, Color32::from_rgba_unmultiplied(255, 255, 255, 140)),
            );
        }
    }

    fn side_panel(&mut self, ui: &mut Ui) {
        let n_layers = self.layer_visible.len();
        let n_rows = self.navmesher.navmeshes().len();

        ui.heading("Navmesher");
        ui.label(format!("{NUM_LAYERS} layers"));
        ui.separator();

        ui.label("Lamina layers");

        for i in 0..n_layers {
            ui.horizontal(|ui| {
                ui.radio_value(&mut self.active_layer, i, "");
                ui.checkbox(
                    &mut self.layer_visible[i],
                    egui::RichText::new(format!("layer {i}")).color(layer_color(i)),
                );
            });
        }

        ui.separator();
        ui.label("Parallel row");

        for r in 0..n_rows {
            ui.radio_value(
                &mut self.active_parallel_row,
                r,
                format!("row {r} (core / peripheral)"),
            );
        }

        if !self.interlamina_visible.is_empty() {
            ui.separator();
            ui.label("Interlaminas");

            let n_inter = self.interlamina_visible.len();

            for i in 0..n_inter {
                ui.checkbox(
                    &mut self.interlamina_visible[i],
                    egui::RichText::new(format!("between {i} and {}", i + 1))
                        .color(interlamina_color(i, n_inter)),
                );
            }
        }

        ui.separator();
        ui.label("Left-click: origin vertex");
        ui.label("Right-click: destination vertex");
        ui.label("Middle-click: insert hole on all visible layers");
        ui.label("Middle-drag: pan");
        ui.label("Scroll: zoom");
        ui.label(format!("zoom: {:.2}", self.zoom));

        ui.separator();

        match self.origin {
            Some(p) => ui.label(format!("origin: [{}, {}]", p[0], p[1])),
            None => ui.label("origin: (none)"),
        };

        match self.destination {
            Some(p) => ui.label(format!("destination: [{}, {}]", p[0], p[1])),
            None => ui.label("destination: (none)"),
        };

        ui.separator();

        let can_find = self.origin.is_some() && self.destination.is_some();

        if ui
            .add_enabled(can_find, egui::Button::new("Find path"))
            .clicked()
        {
            self.find_path();
        }

        ui.add_enabled(
            self.frozen_navmesh.is_some(),
            egui::Checkbox::new(&mut self.is_tesselation_visible, "Show tesselation"),
        );

        if !self.path_status.is_empty() {
            ui.label(&self.path_status);
        }
    }
}

impl eframe::App for ViewerApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::left("controls")
            .resizable(true)
            .default_size(240.0)
            .show(ui, |ui| {
                self.side_panel(ui);
            });

        egui::CentralPanel::default().show(ui, |ui| {
            let (response, painter) =
                ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
            let rect = response.rect;

            painter.rect_filled(rect, 0.0, Color32::from_gray(12));

            if response.hovered() {
                let scroll = ui.input(|i| i.smooth_scroll_delta.y);

                if scroll != 0.0 {
                    let zoom_prev = self.zoom;
                    let pointer = response.hover_pos().unwrap_or(rect.center());

                    self.zoom *= 1.0 + scroll * 0.0015;
                    self.zoom = self.zoom.clamp(0.15, 12.0);

                    let ratio = self.zoom / zoom_prev;

                    self.pan.x = pointer.x - (pointer.x - self.pan.x) * ratio;
                    self.pan.y = pointer.y - (pointer.y - self.pan.y) * ratio;
                }
            }

            if response.dragged_by(egui::PointerButton::Middle) {
                self.pan += response.drag_delta();
            }

            if response.clicked_by(egui::PointerButton::Middle)
                && !response.dragged_by(egui::PointerButton::Middle)
                && let Some(pos) = response.interact_pointer_pos()
            {
                let center = self.screen_to_world(pos);
                let radius = self.rng.next_range_i32(28, 96);
                let poly = random_convex_polygon(&mut self.rng, center, radius, 9);
                let visible_layers: Vec<usize> = self
                    .layer_visible
                    .iter()
                    .enumerate()
                    .filter_map(|(layer, &is_visible)| is_visible.then_some(layer))
                    .collect();

                for &layer in &visible_layers {
                    self.holes.push((layer, poly.exterior.clone()));
                    self.navmesher.insert_polygon(layer.into(), poly.clone());
                }

                self.clear_path();
            }

            if response.clicked_by(egui::PointerButton::Primary)
                && let Some(pos) = response.interact_pointer_pos()
                && let Some(vertex) = self.nearest_vertex(pos, 14.0)
            {
                self.origin = Some(vertex);
                self.clear_path();
            }

            if response.clicked_by(egui::PointerButton::Secondary)
                && !response.dragged_by(egui::PointerButton::Secondary)
                && let Some(pos) = response.interact_pointer_pos()
                && let Some(vertex) = self.nearest_vertex(pos, 14.0)
            {
                self.destination = Some(vertex);
                self.clear_path();
            }

            self.paint_scene(&painter);
        });
    }
}

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 800.0])
            .with_title("anyangle Navmesher viewer"),
        ..Default::default()
    };

    eframe::run_native(
        "anyangle Navmesher viewer",
        options,
        Box::new(|_cc| Ok(Box::new(ViewerApp::new()))),
    )
}
