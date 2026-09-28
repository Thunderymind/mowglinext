/// Pure polyline helpers for editing LiDAR-ignore lines (ROS map frame, metres).
/// A "curve" is stored as a dense polyline: the filter and map_server only ever
/// see straight segments, so smoothing just adds vertices along a spline.

export interface XY {
    x: number;
    y: number;
}

/// Above this many vertices the map stops rendering per-vertex handles (a
/// smoothed long line can get dense); use simplifyPolyline to get handles back.
export const MAX_EDITABLE_VERTICES = 80;

/// Hard cap so a smoothing pass can never produce an absurd polyline.
const MAX_SMOOTHED_VERTICES = 200;

const dist = (a: XY, b: XY): number => Math.hypot(b.x - a.x, b.y - a.y);

export const polylineLengthM = (points: XY[]): number => {
    let total = 0;
    for (let i = 0; i + 1 < points.length; i++) total += dist(points[i], points[i + 1]);
    return total;
};

/// Midpoint of segment i (between points[i] and points[i + 1]).
export const segmentMidpoint = (points: XY[], i: number): XY => ({
    x: (points[i].x + points[i + 1].x) / 2,
    y: (points[i].y + points[i + 1].y) / 2,
});

/// Insert a vertex at the midpoint of segment i.
export const insertMidpoint = (points: XY[], i: number): XY[] => {
    if (i < 0 || i + 1 >= points.length) return points;
    return [...points.slice(0, i + 1), segmentMidpoint(points, i), ...points.slice(i + 1)];
};

/// Remove vertex i; a line always keeps at least 2 vertices.
export const removeVertex = (points: XY[], i: number): XY[] => {
    if (points.length <= 2 || i < 0 || i >= points.length) return points;
    return points.filter((_, idx) => idx !== i);
};

/// Centripetal-free uniform Catmull-Rom spline THROUGH the given vertices,
/// sampled roughly every `stepM` metres. The original vertices are kept, so
/// the line still passes exactly through what the operator placed. Endpoints
/// are duplicated as phantom neighbours, so the curve starts and ends on them.
export const smoothPolyline = (points: XY[], stepM = 0.5): XY[] => {
    // Already at/over the cap: smoothing could only add vertices (and the
    // coarser-step fallback below would never terminate).
    if (points.length < 3 || points.length >= MAX_SMOOTHED_VERTICES) return points;
    const p = (i: number): XY => points[Math.min(Math.max(i, 0), points.length - 1)];
    const out: XY[] = [points[0]];
    for (let i = 0; i + 1 < points.length; i++) {
        const p0 = p(i - 1), p1 = p(i), p2 = p(i + 1), p3 = p(i + 2);
        const samples = Math.max(1, Math.ceil(dist(p1, p2) / stepM));
        for (let s = 1; s <= samples; s++) {
            const t = s / samples;
            const t2 = t * t, t3 = t2 * t;
            const cr = (a: number, b: number, c: number, d: number) =>
                0.5 * ((2 * b) + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t2 + (-a + 3 * b - 3 * c + d) * t3);
            out.push({
                x: cr(p0.x, p1.x, p2.x, p3.x),
                y: cr(p0.y, p1.y, p2.y, p3.y),
            });
        }
    }
    if (out.length > MAX_SMOOTHED_VERTICES) {
        // Too dense: fall back to a coarser step rather than return a huge line.
        return smoothPolyline(points, stepM * 2);
    }
    return out;
};

const perpendicularDistance = (pt: XY, a: XY, b: XY): number => {
    const dx = b.x - a.x, dy = b.y - a.y;
    const len2 = dx * dx + dy * dy;
    if (len2 < 1e-12) return dist(pt, a);
    const t = Math.min(1, Math.max(0, ((pt.x - a.x) * dx + (pt.y - a.y) * dy) / len2));
    return dist(pt, {x: a.x + t * dx, y: a.y + t * dy});
};

/// Douglas-Peucker: drop vertices closer than `toleranceM` to the simplified
/// line. Endpoints are always kept.
export const simplifyPolyline = (points: XY[], toleranceM = 0.15): XY[] => {
    if (points.length < 3) return points;
    const keep = new Array<boolean>(points.length).fill(false);
    keep[0] = keep[points.length - 1] = true;
    const stack: [number, number][] = [[0, points.length - 1]];
    while (stack.length > 0) {
        const [lo, hi] = stack.pop()!;
        let maxD = 0, idx = -1;
        for (let i = lo + 1; i < hi; i++) {
            const d = perpendicularDistance(points[i], points[lo], points[hi]);
            if (d > maxD) {
                maxD = d;
                idx = i;
            }
        }
        if (idx !== -1 && maxD > toleranceM) {
            keep[idx] = true;
            stack.push([lo, idx], [idx, hi]);
        }
    }
    return points.filter((_, i) => keep[i]);
};

/// A mitred offset (see buildCorridorBandPolygon) can shoot arbitrarily far
/// out at a very sharp bend (the classic "stroke spike" problem) — cap it at
/// this multiple of the half-width, same idea as an SVG/canvas miterLimit.
const MITER_LIMIT = 4;

/// Left-hand perpendicular unit normal of a direction vector.
const leftNormal = (dx: number, dy: number): XY => {
    const len = Math.hypot(dx, dy);
    return len < 1e-9 ? {x: 0, y: 0} : {x: -dy / len, y: dx / len};
};

/// Build the visible "ignore band" for a corridor: ONE continuous polygon
/// (not a quad per segment — see git history for the earlier, blockier
/// version), offset `widthM / 2` to each side of the polyline with a mitred
/// join at every interior vertex so the band flows smoothly through a bend
/// instead of showing per-segment seams. This is a visual aid only, not a
/// proper geometric buffer (no self-intersection repair for a very tight,
/// sharply doubling-back line) — costmap_scan_filter_node's own point-to-
/// segment test remains the ground truth for what's actually suppressed.
/// Returns a single closed ring wrapped in an array (kept as XY[][] so
/// existing callers — one Feature per ring — don't need to change), or []
/// for a degenerate (<2-point) input.
export const buildCorridorBandPolygon = (points: XY[], widthM: number): XY[][] => {
    if (points.length < 2) return [];
    const halfW = Math.max(0.01, widthM) / 2;
    const n = points.length;
    const left: XY[] = [];
    const right: XY[] = [];
    for (let i = 0; i < n; i++) {
        const prev = points[Math.max(i - 1, 0)];
        const cur = points[i];
        const next = points[Math.min(i + 1, n - 1)];
        const nIn = leftNormal(cur.x - prev.x, cur.y - prev.y);
        const nOut = leftNormal(next.x - cur.x, next.y - cur.y);
        // Endpoints (or a zero-length neighbour segment) only have one real
        // normal — the other came back {0,0} and drops out of the sum.
        let bx = nIn.x + nOut.x, by = nIn.y + nOut.y;
        const bLen = Math.hypot(bx, by);
        if (bLen < 1e-9) {
            // Both neighbours degenerate (an isolated duplicate point) —
            // nothing sensible to offset; place both banks on the vertex.
            left.push(cur);
            right.push(cur);
            continue;
        }
        bx /= bLen;
        by /= bLen;
        // cos(half the turn angle) = dot(bisector, either normal) — the
        // miter scale factor (1/cos) blows up as the turn approaches 180°,
        // so clamp it (MITER_LIMIT) rather than let a hairpin spike out.
        const cosHalfAngle = Math.max(bx * nIn.x + by * nIn.y, 1 / MITER_LIMIT);
        const miter = halfW / cosHalfAngle;
        left.push({x: cur.x + bx * miter, y: cur.y + by * miter});
        right.push({x: cur.x - bx * miter, y: cur.y - by * miter});
    }
    const ring = [...left, ...right.reverse()];
    ring.push(ring[0]);
    return [ring];
};
