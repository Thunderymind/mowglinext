import type {Map as MapType} from "../../../types/ros.ts";

const MIN_POLYGON_POINTS = 3;

export type MapBackupParseResult =
    | {ok: true; map: MapType; hasDock: boolean}
    | {ok: false; reason: string};

const isRecord = (value: unknown): value is Record<string, unknown> =>
    typeof value === "object" && value !== null && !Array.isArray(value);

const isFiniteNumber = (value: unknown): value is number =>
    typeof value === "number" && Number.isFinite(value);

function polygonProblem(polygon: unknown, label: string): string | null {
    if (!isRecord(polygon) || !Array.isArray(polygon.points)) {
        return `${label} has no points list`;
    }
    if (polygon.points.length < MIN_POLYGON_POINTS) {
        return `${label} has fewer than ${MIN_POLYGON_POINTS} points`;
    }
    const isValidPoint = (p: unknown) => isRecord(p) && isFiniteNumber(p.x) && isFiniteNumber(p.y);
    return polygon.points.every(isValidPoint) ? null : `${label} has a non-numeric coordinate`;
}

function areaListProblem(list: unknown, label: string): string | null {
    if (list === undefined) {
        return null;
    }
    if (!Array.isArray(list)) {
        return `${label} is not a list`;
    }
    for (const [index, area] of list.entries()) {
        const areaLabel = `${label}[${index}]`;
        if (!isRecord(area)) {
            return `${areaLabel} is not an object`;
        }
        const obstacles = area.obstacles ?? [];
        if (!Array.isArray(obstacles)) {
            return `${areaLabel}.obstacles is not a list`;
        }
        const problem = polygonProblem(area.area, areaLabel)
            ?? obstacles.map((o, i) => polygonProblem(o, `${areaLabel}.obstacles[${i}]`)).find((p) => p !== null);
        if (problem) {
            return problem;
        }
    }
    return null;
}

/**
 * Validate the text of a map.json backup as a COMPLETE restore candidate
 * before any editor state is touched (#704). A backup with no area at all is
 * refused: restoring it and pressing Save would replace the real map with
 * nothing. `hasDock` is false when the dock fields are absent or not all
 * finite numbers — a missing dock is NOT a dock at (0, 0, 0).
 */
export function parseMapBackup(text: string): MapBackupParseResult {
    let parsed: unknown;
    try {
        parsed = JSON.parse(text);
    } catch (e) {
        return {ok: false, reason: `not valid JSON (${e instanceof Error ? e.message : String(e)})`};
    }
    if (!isRecord(parsed)) {
        return {ok: false, reason: "not a map object"};
    }
    const problem = areaListProblem(parsed.working_area, "working_area")
        ?? areaListProblem(parsed.navigation_areas, "navigation_areas");
    if (problem) {
        return {ok: false, reason: problem};
    }
    const areaCount = [parsed.working_area, parsed.navigation_areas]
        .reduce<number>((count, list) => count + (Array.isArray(list) ? list.length : 0), 0);
    if (areaCount === 0) {
        return {ok: false, reason: "the backup contains no area"};
    }
    const hasDock = [parsed.dock_x, parsed.dock_y, parsed.dock_heading].every(isFiniteNumber);
    return {ok: true, map: parsed as MapType, hasDock};
}
