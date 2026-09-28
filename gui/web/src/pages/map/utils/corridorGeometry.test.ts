import {describe, expect, it} from "vitest";
import {
    buildCorridorBandPolygon,
    insertMidpoint,
    polylineLengthM,
    removeVertex,
    simplifyPolyline,
    smoothPolyline,
} from "./corridorGeometry.ts";

describe("corridorGeometry", () => {
    it("measures polyline length", () => {
        expect(polylineLengthM([{x: 0, y: 0}, {x: 3, y: 4}, {x: 3, y: 10}])).toBeCloseTo(11);
        expect(polylineLengthM([{x: 1, y: 1}])).toBe(0);
    });

    it("inserts a vertex at a segment midpoint", () => {
        const out = insertMidpoint([{x: 0, y: 0}, {x: 4, y: 0}], 0);
        expect(out).toEqual([{x: 0, y: 0}, {x: 2, y: 0}, {x: 4, y: 0}]);
        expect(insertMidpoint([{x: 0, y: 0}, {x: 4, y: 0}], 5)).toHaveLength(2);
    });

    it("never removes a vertex below two points", () => {
        const two = [{x: 0, y: 0}, {x: 1, y: 0}];
        expect(removeVertex(two, 0)).toEqual(two);
        expect(removeVertex([{x: 0, y: 0}, {x: 1, y: 0}, {x: 2, y: 0}], 1)).toHaveLength(2);
    });

    it("smoothing passes through the original vertices and keeps endpoints", () => {
        const pts = [{x: 0, y: 0}, {x: 4, y: 3}, {x: 8, y: 0}];
        const out = smoothPolyline(pts, 0.5);
        expect(out.length).toBeGreaterThan(pts.length);
        expect(out[0]).toEqual(pts[0]);
        const last = out[out.length - 1];
        expect(last.x).toBeCloseTo(8);
        expect(last.y).toBeCloseTo(0);
        // The middle control point is still on the curve.
        expect(out.some((p) => Math.abs(p.x - 4) < 1e-6 && Math.abs(p.y - 3) < 1e-6)).toBe(true);
    });

    it("smoothing leaves a two-point line alone and stays bounded", () => {
        const two = [{x: 0, y: 0}, {x: 10, y: 0}];
        expect(smoothPolyline(two)).toEqual(two);
        const long = Array.from({length: 40}, (_, i) => ({x: i * 5, y: (i % 2) * 2}));
        expect(smoothPolyline(long, 0.1).length).toBeLessThanOrEqual(200);
    });

    it("simplify drops collinear vertices but keeps endpoints and real corners", () => {
        const line = [{x: 0, y: 0}, {x: 1, y: 0}, {x: 2, y: 0}, {x: 3, y: 0}];
        expect(simplifyPolyline(line, 0.1)).toEqual([{x: 0, y: 0}, {x: 3, y: 0}]);
        const corner = [{x: 0, y: 0}, {x: 5, y: 0}, {x: 5, y: 5}];
        expect(simplifyPolyline(corner, 0.1)).toEqual(corner);
    });

    it("bands a straight segment as a widthM-wide rectangle, closed", () => {
        const bands = buildCorridorBandPolygon([{x: 0, y: 0}, {x: 10, y: 0}], 1.0);
        expect(bands).toHaveLength(1);
        const [ring] = bands;
        expect(ring[0]).toEqual(ring[ring.length - 1]); // closed
        expect(ring.every((p) => Math.abs(p.y) <= 0.5 + 1e-9)).toBe(true);
        expect(ring.some((p) => Math.abs(p.y - 0.5) < 1e-9)).toBe(true);
        expect(ring.some((p) => Math.abs(p.y + 0.5) < 1e-9)).toBe(true);
    });

    it("bands a bent polyline as ONE continuous, mitred ring — not one piece per segment", () => {
        const bands = buildCorridorBandPolygon([{x: 0, y: 0}, {x: 5, y: 0}, {x: 5, y: 5}], 0.4);
        expect(bands).toHaveLength(1);
        const [ring] = bands;
        // 3 input vertices -> 3 left-bank + 3 right-bank points, ring-closed.
        expect(ring).toHaveLength(3 + 3 + 1);
        // The outer corner (mitred) sits further than half-width from the
        // bend vertex (5, 0) — that's the whole point of mitring instead of
        // leaving a gap/overlap between two independent quads.
        const outerCorner = ring.find((p) => p.x > 5.05 && p.y < -0.05);
        expect(outerCorner).toBeDefined();
    });

    it("does not choke on a degenerate (repeated) point", () => {
        const bands = buildCorridorBandPolygon([{x: 0, y: 0}, {x: 0, y: 0}, {x: 5, y: 0}], 0.4);
        expect(bands).toHaveLength(1);
        expect(bands[0].every((p) => Number.isFinite(p.x) && Number.isFinite(p.y))).toBe(true);
    });

    it("clamps a zero or negative width to a thin sliver rather than collapsing", () => {
        const bands = buildCorridorBandPolygon([{x: 0, y: 0}, {x: 1, y: 0}], 0);
        expect(bands[0].some((p) => p.y !== 0)).toBe(true);
    });

    it("returns nothing for a single point (no line to band)", () => {
        expect(buildCorridorBandPolygon([{x: 0, y: 0}], 0.5)).toEqual([]);
    });
});
