import {describe, expect, it} from "vitest";
import {
    applyFirmwareModelDefaults,
    firmwareDefaultsForModel,
    manualOverridesFromProvenance,
} from "./firmwareModelDefaults.ts";

describe("firmware model defaults", () => {
    it("maps the canonical YardForce 500 permutation", () => {
        expect(firmwareDefaultsForModel("YardForce500")).toEqual({
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
        });
    });

    it("maps the canonical YardForce 500B permutation", () => {
        expect(firmwareDefaultsForModel("YardForce500B")).toEqual({
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
        });
    });

    it("maps Biltema RM1000 to the F401 board, 900 ECO panel, and native target", () => {
        expect(firmwareDefaultsForModel("BiltemaRM1000")).toEqual({
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_900_ECO",
            firmwareTarget: "BiltemaRM1000",
        });
        expect(firmwareDefaultsForModel("BiltemaRM1000")?.firmwareTarget)
            .not.toBe("Yardforce500B");
    });

    it.each(["CUSTOM", "LUV1000RI", "unknown", undefined])(
        "does not guess for unsupported model %s",
        (model) => {
            expect(firmwareDefaultsForModel(model)).toBeUndefined();
            expect(applyFirmwareModelDefaults(model, {
                boardType: "BOARD_YARDFORCE500",
                panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
            })).toEqual({boardType: "", panelType: "", firmwareTarget: ""});
        },
    );

    it("keeps manual board and panel overrides across model changes", () => {
        const manual = {
            boardType: "BOARD_LUV1000RI",
            panelType: "PANEL_TYPE_YARDFORCE_900_ECO",
        };
        expect(applyFirmwareModelDefaults("YardForce500B", manual, {
            boardType: true,
            panelType: true,
        })).toEqual({...manual, firmwareTarget: ""});
    });

    it("preserves manual board, panel, and target overrides", () => {
        const manual = {
            boardType: "BOARD_LUV1000RI",
            panelType: "PANEL_TYPE_YARDFORCE_900_ECO",
            firmwareTarget: "LUV1000RI",
        };
        expect(applyFirmwareModelDefaults("BiltemaRM1000", manual, {
            boardType: true,
            panelType: true,
            firmwareTarget: true,
        })).toEqual(manual);
    });

    it("updates auto-managed fields when changing to and away from RM1000", () => {
        const rm1000 = applyFirmwareModelDefaults("BiltemaRM1000", {
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
            firmwareTarget: "",
        });
        expect(rm1000).toEqual({
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_900_ECO",
            firmwareTarget: "BiltemaRM1000",
        });
        expect(applyFirmwareModelDefaults("YardForce500B", rm1000)).toEqual({
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
            firmwareTarget: "",
        });
    });

    it("updates the model-following field when only the other field is overridden", () => {
        expect(applyFirmwareModelDefaults("YardForce500B", {
            boardType: "BOARD_LUV1000RI",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
        }, {boardType: true})).toEqual({
            boardType: "BOARD_LUV1000RI",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
            firmwareTarget: "",
        });
    });

    it("updates only automatic fields and preserves unrelated firmware settings", () => {
        const current = {
            boardType: "BOARD_YARDFORCE500",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
            repository: "https://example.test/mowgli",
            maxMps: 0.7,
        };
        expect(applyFirmwareModelDefaults("YardForce500B", current)).toEqual({
            ...current,
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
            firmwareTarget: "",
        });
    });

    it("treats missing provenance as a legacy manual selection", () => {
        expect(manualOverridesFromProvenance({
            boardType: "BOARD_YARDFORCE500",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
        })).toEqual({boardType: true, panelType: true, firmwareTarget: false});
    });

    it("keeps only explicitly automatic fields following model changes", () => {
        expect(manualOverridesFromProvenance({
            boardType: "BOARD_YARDFORCE500",
            panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
            boardTypeOrigin: "manual",
            panelTypeOrigin: "auto",
            firmwareSelectionModel: "YardForce500",
        })).toEqual({boardType: true, panelType: false, firmwareTarget: false});
    });

    it("recognizes an auto-managed target independently from board and panel", () => {
        expect(manualOverridesFromProvenance({
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
            firmwareTarget: "BiltemaRM1000",
            boardTypeOrigin: "auto",
            panelTypeOrigin: "auto",
            firmwareTargetOrigin: "auto",
        })).toEqual({boardType: false, panelType: false, firmwareTarget: false});
    });

    it("migrates targetless legacy fields to RM1000 defaults while retaining modern manual overrides", () => {
        const legacy = {
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
        };
        const legacyOverrides = manualOverridesFromProvenance(legacy, "BiltemaRM1000");
        expect(legacyOverrides).toEqual({boardType: false, panelType: false, firmwareTarget: false});
        expect(applyFirmwareModelDefaults("BiltemaRM1000", legacy, legacyOverrides)).toEqual({
            boardType: "BOARD_YARDFORCE500B",
            panelType: "PANEL_TYPE_YARDFORCE_900_ECO",
            firmwareTarget: "BiltemaRM1000",
        });

        expect(manualOverridesFromProvenance({
            boardType: "BOARD_LUV1000RI",
            panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
            boardTypeOrigin: "manual",
            panelTypeOrigin: "auto",
        }, "BiltemaRM1000")).toEqual({boardType: true, panelType: false, firmwareTarget: false});
    });
});
