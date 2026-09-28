/**
 * Firmware targets that are safe to infer from the selected mower model.
 *
 * These are only the fields that can be safely inferred from the published
 * firmware permutations in firmware/scripts/package_release.py. Models
 * without an unambiguous published mapping are intentionally absent: the
 * firmware flashing UI must not guess for them.
 */
export type FirmwareSelection = {
    boardType?: string;
    panelType?: string;
    /** An exact PlatformIO/release-manifest target when the board is ambiguous. */
    firmwareTarget?: string;
    /** Persisted provenance for each independently editable field. */
    boardTypeOrigin?: FirmwareFieldOrigin;
    panelTypeOrigin?: FirmwareFieldOrigin;
    firmwareTargetOrigin?: FirmwareFieldOrigin;
    /** Mower model whose automatic defaults were last applied. */
    firmwareSelectionModel?: string;
};

export type FirmwareFieldOrigin = "auto" | "manual" | "legacy";

export type FirmwareModelDefaults = Readonly<Pick<FirmwareSelection, "boardType" | "panelType" | "firmwareTarget">>;

export const FIRMWARE_MODEL_DEFAULTS: Readonly<Record<string, FirmwareModelDefaults>> = {
    YardForce500: {
        // YardForce500 is shared by the Vermut and Mowgli controller variants;
        // the mechanical mower model does not identify the board. The panel
        // is common to both classic variants and is safe to infer.
        panelType: "PANEL_TYPE_YARDFORCE_500_CLASSIC",
    },
    YardForce500B: {
        boardType: "BOARD_YARDFORCE500B",
        panelType: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
    },
    BiltemaRM1000: {
        boardType: "BOARD_YARDFORCE500B",
        panelType: "PANEL_TYPE_YARDFORCE_900_ECO",
        firmwareTarget: "BiltemaRM1000",
    },
};

export const firmwareDefaultsForModel = (
    mowerModel: unknown,
): FirmwareModelDefaults | undefined => {
    if (typeof mowerModel !== "string") return undefined;
    return FIRMWARE_MODEL_DEFAULTS[mowerModel];
};

/**
 * Apply only inferred board/panel fields. Other firmware settings are returned
 * untouched, and a field marked as manually overridden is left untouched even
 * when the mower model changes.
 */
export const applyFirmwareModelDefaults = <T extends FirmwareSelection>(
    mowerModel: unknown,
    selection: T,
    manualOverrides: Partial<Record<keyof FirmwareSelection, boolean>> = {},
): T => {
    const defaults = firmwareDefaultsForModel(mowerModel);
    return {
        ...selection,
        // An empty string is deliberate: it overrides Formily's static
        // defaults and leaves an unsupported model visibly unselected.
        ...(manualOverrides.boardType ? {} : {boardType: defaults?.boardType ?? ""}),
        ...(manualOverrides.panelType ? {} : {panelType: defaults?.panelType ?? ""}),
        ...(manualOverrides.firmwareTarget ? {} : {firmwareTarget: defaults?.firmwareTarget ?? ""}),
    } as T;
};

/**
 * Convert persisted field provenance to the conservative override policy used
 * by the form. Missing/unknown provenance is treated as legacy, so a saved
 * value is never silently replaced with a guessed firmware target.
 */
export const manualOverridesFromProvenance = (
    selection: FirmwareSelection,
): Partial<Record<"boardType" | "panelType" | "firmwareTarget", boolean>> => ({
    boardType: selection.boardTypeOrigin !== "auto",
    panelType: selection.panelTypeOrigin !== "auto",
    // The target field is new. A legacy config cannot contain a user's
    // explicit target override, so infer it from the model while keeping the
    // existing legacy board and panel values untouched.
    firmwareTarget: selection.firmwareTargetOrigin !== undefined
        ? selection.firmwareTargetOrigin !== "auto"
        : selection.firmwareTarget !== undefined,
});
