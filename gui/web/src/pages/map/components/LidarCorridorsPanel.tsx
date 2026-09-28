import {CheckOutlined, CloseOutlined, InfoCircleOutlined} from "@ant-design/icons";
import {Button, InputNumber, Tooltip} from "antd";
import {useTranslation} from "react-i18next";
import {useThemeMode} from "../../../theme/ThemeContext.tsx";
import type {LidarIgnoreCorridor} from "../../../types/ros.ts";

/// Line colour on the map and the accent of each row (kept in sync with the
/// map layer in MapPage).
export const CORRIDOR_COLOR = '#eb2f96';

interface LidarCorridorsPanelProps {
    corridors: LidarIgnoreCorridor[];
    busy: boolean;
    /// False outside the map edit mode: the list is then read-only, and the
    /// draw button is hidden entirely (there is nothing to draw into).
    editable: boolean;
    /// True while the operator is clicking points onto the map.
    drawing: boolean;
    drawPointCount: number;
    /// Drawing is started from the map editor toolbar's "+ Add" menu now
    /// (works on mobile too), not from a button in this panel — there is no
    /// onStartDraw prop here any more.
    onFinishDraw: () => void;
    onCancelDraw: () => void;
    /// Width in METRES (the panel edits centimetres).
    onChangeWidth: (index: number, widthM: number) => void;
    /// Select a row's line on the map (simple_select on its DrawControl
    /// feature) — the only way to delete a line now: select it here or on
    /// the map, then use the map editor toolbar's trash button. There is no
    /// per-row delete any more (9+ lines made every row noisy, and the
    /// toolbar trash already deletes whatever's selected, line or area).
    onSelect: (index: number) => void;
    /// Index of the line currently selected on the map, or null.
    selectedIndex: number | null;
    /// Round the selected line through its points / thin it out again.
    onSmooth: () => void;
    onSimplify: () => void;
}

/// Operator-drawn LiDAR-ignore lines. Inside a line's width the LiDAR returns
/// are dropped for BOTH the costmap (FTC/Nav2 avoidance) and collision_monitor,
/// so the robot follows the recorded boundary next to e.g. a hedge instead of
/// being pushed off it. Everywhere else the LiDAR keeps working normally.
export const LidarCorridorsPanel = ({
    corridors, busy, editable, drawing, drawPointCount, onFinishDraw, onCancelDraw, onChangeWidth,
    onSelect, selectedIndex, onSmooth, onSimplify,
}: LidarCorridorsPanelProps) => {
    const {colors} = useThemeMode();
    const {t} = useTranslation();

    return (
        <div style={{display: 'flex', flexDirection: 'column', minWidth: 0, minHeight: 0, flex: '1 1 auto'}}>
            <div style={{
                padding: '8px 12px',
                fontSize: 12,
                fontWeight: 600,
                color: colors.muted,
                textTransform: 'uppercase',
                letterSpacing: '0.05em',
                borderBottom: `1px solid ${colors.borderSubtle}`,
                display: 'flex',
                alignItems: 'center',
                gap: 6,
            }}>
                <span style={{flex: 1, minWidth: 0}}>{t('mapLidarCorridors.header', {count: corridors.length})}</span>
                <Tooltip
                    title={
                        <div style={{display: 'flex', flexDirection: 'column', gap: 6}}>
                            <div>{t('mapLidarCorridors.hint')}</div>
                            <div style={{color: colors.warning}}>{t('mapLidarCorridors.safetyWarning')}</div>
                            {!editable && <div>{t('mapLidarCorridors.lockedHint')}</div>}
                        </div>
                    }
                    overlayStyle={{maxWidth: 320}}
                >
                    <InfoCircleOutlined style={{color: colors.muted, fontSize: 13, cursor: 'help'}}/>
                </Tooltip>
            </div>
            <div style={{overflowY: 'auto', flex: 1, minHeight: 0}} className="scrollbar-thin">
                {corridors.map((corridor, index) => (
                    <div key={corridor.id ?? index}
                        onClick={() => onSelect(index)}
                        style={{
                            display: 'flex',
                            alignItems: 'center',
                            gap: 6,
                            padding: '6px 12px',
                            borderLeft: `3px solid ${CORRIDOR_COLOR}`,
                            // Very light tint of the same pink used for the line/band on
                            // the map (CORRIDOR_COLOR), not the generic bgElevated highlight
                            // — that one was nearly invisible against this panel's dark
                            // background and gave no clue which line was selected.
                            background: selectedIndex === index ? `${CORRIDOR_COLOR}22` : 'transparent',
                            cursor: 'pointer',
                        }}>
                        <div style={{flex: 1, fontSize: 12, color: colors.text, minWidth: 0}}>
                            {corridor.name || t('mapLidarCorridors.unnamed', {id: corridor.id ?? index + 1})}
                        </div>
                        <InputNumber
                            size="small"
                            style={{width: 84}}
                            min={5}
                            max={100}
                            step={5}
                            precision={0}
                            addonAfter="cm"
                            disabled={busy || !editable}
                            value={Math.round((corridor.width_m ?? 0.2) * 100)}
                            aria-label={t('mapLidarCorridors.widthLabel')}
                            title={t('mapLidarCorridors.widthTooltip')}
                            onClick={(e) => e.stopPropagation()}
                            onChange={(value) => {
                                if (typeof value === 'number') onChangeWidth(index, value / 100);
                            }}
                        />
                    </div>
                ))}
            </div>
            <div style={{padding: '6px 12px 10px', display: 'flex', gap: 6, flexWrap: 'wrap'}}>
                {drawing ? (
                    <>
                        <div style={{width: '100%', fontSize: 11, color: colors.muted}}>
                            {t('mapLidarCorridors.drawing', {count: drawPointCount})}
                        </div>
                        <Button size="small" type="primary" icon={<CheckOutlined aria-hidden="true"/>}
                            disabled={drawPointCount < 2 || busy} onClick={onFinishDraw}>
                            {t('mapLidarCorridors.finish')}
                        </Button>
                        <Button size="small" icon={<CloseOutlined aria-hidden="true"/>} onClick={onCancelDraw}>
                            {t('mapLidarCorridors.cancel')}
                        </Button>
                    </>
                ) : (
                    <>
                        {selectedIndex !== null && editable && (
                            <>
                                <Button size="small" disabled={busy} onClick={onSmooth}
                                    title={t('mapLidarCorridors.smoothTooltip')}>
                                    {t('mapLidarCorridors.smooth')}
                                </Button>
                                <Button size="small" disabled={busy} onClick={onSimplify}
                                    title={t('mapLidarCorridors.simplifyTooltip')}>
                                    {t('mapLidarCorridors.simplify')}
                                </Button>
                            </>
                        )}
                        {editable && (
                            <div style={{width: '100%', fontSize: 11, color: colors.muted}}>
                                {t('mapLidarCorridors.editHint')}
                            </div>
                        )}
                    </>
                )}
            </div>
        </div>
    );
};
