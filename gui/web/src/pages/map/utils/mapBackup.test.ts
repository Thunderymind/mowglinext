import {describe, expect, it} from "vitest";
import {parseMapBackup} from "./mapBackup.ts";

const square = {points: [{x: 0, y: 0, z: 0}, {x: 1, y: 0, z: 0}, {x: 1, y: 1, z: 0}, {x: 0, y: 1, z: 0}]};

const validMap = (extra: Record<string, unknown> = {}) => ({
    working_area: [{name: "Ängen", area: square, obstacles: []}],
    navigation_areas: [],
    ...extra,
});

describe("parseMapBackup", () => {
    it("accepts a backup with at least one area and reports its dock", () => {
        const result = parseMapBackup(JSON.stringify(validMap({dock_x: 1.5, dock_y: -2, dock_heading: 0.3})));

        expect(result.ok).toBe(true);
        if (!result.ok) return;
        expect(result.hasDock).toBe(true);
        expect(result.map.working_area?.[0].name).toBe("Ängen");
    });

    it("keeps a legitimate dock at the origin distinct from a missing dock", () => {
        const atOrigin = parseMapBackup(JSON.stringify(validMap({dock_x: 0, dock_y: 0, dock_heading: 0})));
        const missing = parseMapBackup(JSON.stringify(validMap()));

        expect(atOrigin.ok && atOrigin.hasDock).toBe(true);
        expect(missing.ok && !missing.hasDock).toBe(true);
    });

    it("treats a partial or non-finite dock as missing", () => {
        const partial = parseMapBackup(JSON.stringify(validMap({dock_x: 1})));
        const notANumber = parseMapBackup(JSON.stringify(validMap({dock_x: "1", dock_y: 2, dock_heading: 0})));

        expect(partial.ok && !partial.hasDock).toBe(true);
        expect(notANumber.ok && !notANumber.hasDock).toBe(true);
    });

    it.each([
        ["not JSON", "{"],
        ["null", "null"],
        ["an array", "[]"],
        ["an empty object", "{}"],
        ["empty area lists", JSON.stringify({working_area: [], navigation_areas: []})],
        ["a non-array area list", JSON.stringify({working_area: {}})],
        ["an area without a polygon", JSON.stringify({working_area: [{name: "a"}]})],
        ["a polygon with under 3 points", JSON.stringify({working_area: [{area: {points: [{x: 0, y: 0}]}}]})],
        ["a non-finite coordinate", JSON.stringify({working_area: [{area: {points: [{x: 0, y: 0}, {x: "1", y: 0}, {x: 1, y: 1}]}}]})],
        ["a malformed obstacle", JSON.stringify({working_area: [{area: square, obstacles: [{points: "x"}]}]})],
    ])("rejects %s", (_label, text) => {
        expect(parseMapBackup(text).ok).toBe(false);
    });
});
