import {act, renderHook} from '@testing-library/react';
import {beforeEach, describe, expect, it, vi} from 'vitest';
import {useMapEditing, type ShrinkMemory, type UseMapEditingOptions} from './useMapEditing';
import {MowingAreaFeature, ObstacleFeature, type MowingFeature} from '../../../types/map.ts';

interface DeleteDialog {
    title: string;
    content: string;
    onOk: () => void;
}
const {confirm} = vi.hoisted(() => ({confirm: vi.fn<(dialog: DeleteDialog) => void>()}));
vi.mock('antd', () => ({App: {useApp: () => ({modal: {confirm}})}}));
vi.mock('react-i18next', () => ({useTranslation: () => ({t: (key: string) => key})}));

describe('map delete confirmation', () => {
    beforeEach(() => confirm.mockClear());

    function setup(mode: string, points: number, selected = ['area-0']) {
        const draw = {
            getMode: () => mode,
            getSelectedPoints: () => ({features: Array.from({length: points})}),
            getSelectedIds: () => selected,
            trash: vi.fn(),
        };
        const options = {
            features: {}, setFeatures: vi.fn(), editMap: true, mowingAreas: [],
            drawRef: {current: draw}, notification: {}, mapInstanceRef: {current: null},
        } as unknown as UseMapEditingOptions;
        const hook = renderHook(() => useMapEditing(options));
        act(() => hook.result.current.handleTrash());
        return draw;
    }

    it.each([1, 2])('describes deleting %i selected vertices and waits for confirmation', points => {
        const draw = setup('direct_select', points);
        const dialog = confirm.mock.calls[0][0];
        expect(dialog.title).toBe('mapEditing.deletePointsConfirmTitle');
        expect(dialog.content).toBe('mapEditing.deletePointsConfirmBody');
        expect(draw.trash).not.toHaveBeenCalled();
        act(() => dialog.onOk());
        expect(draw.trash).toHaveBeenCalledOnce();
    });

    it('keeps the area warning for a complete feature selection', () => {
        setup('simple_select', 0);
        expect(confirm.mock.calls[0][0].content).toBe('mapEditing.deleteAreaConfirmBody');
    });

    it('does not offer deletion with no selected feature', () => {
        const draw = setup('simple_select', 0, []);
        expect(confirm).not.toHaveBeenCalled();
        expect(draw.trash).not.toHaveBeenCalled();
    });

    it('describes direct-select deletion even before the selected-point cache updates', () => {
        // The custom midpoint handler selects the new vertex internally before
        // updating Draw's public getSelectedPoints cache.
        const draw = setup('direct_select', 0);
        const dialog = confirm.mock.calls[0][0];
        expect(dialog.title).toBe('mapEditing.deletePointsConfirmTitle');
        act(() => dialog.onOk());
        expect(draw.trash).toHaveBeenCalledOnce();
    });
});

function square(x0: number, y0: number, x1: number, y1: number) {
    return {
        type: 'Polygon' as const,
        coordinates: [[[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]],
    };
}

function areaFeature(id: string, geom: ReturnType<typeof square>) {
    const f = new MowingAreaFeature(id, 1);
    f.setGeometry(geom);
    return f;
}

// In-memory stand-in for the persisted shrink memory MapPage provides.
function makeShrinkMemory(): ShrinkMemory {
    const rows: {original: unknown; shrunk: unknown}[] = [];
    return {
        find: (ring) => rows.find((r) => JSON.stringify(r.shrunk) === JSON.stringify(ring))?.original as never,
        remember: (original, shrunk) => { rows.push({original, shrunk}); },
        forget: (shrunk) => {
            const i = rows.findIndex((r) => JSON.stringify(r.shrunk) === JSON.stringify(shrunk));
            if (i >= 0) rows.splice(i, 1);
        },
    };
}

describe('recorded-outline shrink survives an obstacle round trip', () => {
    it('restores the original outline when the shrunk obstacle becomes an area again', () => {
        const parent = areaFeature('area-0-area-0', square(0, 0, 10, 10));
        const recorded = areaFeature('area-1-area-0', square(4, 4, 6, 6));
        const shrunk = square(4.3, 4.3, 5.7, 5.7);

        const memory = makeShrinkMemory();
        let current: Record<string, MowingFeature> = {[parent.id]: parent, [recorded.id]: recorded};
        const setFeatures = vi.fn((next: Record<string, MowingFeature>) => { current = next; });
        const options = () => ({
            features: current, setFeatures, editMap: true, mowingAreas: [],
            drawRef: {current: null}, notification: {error: vi.fn(), info: vi.fn(), success: vi.fn()},
            mapInstanceRef: {current: null}, shrinkMemory: memory,
        } as unknown as UseMapEditingOptions);
        const hook = renderHook(() => useMapEditing(options()));

        const convert = (from: string, to: string, corrected?: ReturnType<typeof square>) => {
            act(() => hook.result.current.setCurMowingAreaFeature({
                id: recorded.id, index: 0, name: '', mowing_order: 1, orig_mowing_order: 1,
                feature_type: to, orig_feature_type: from, shrink_recorded: true,
            }));
            act(() => hook.result.current.updateMowingArea(corrected));
            hook.rerender();
        };

        convert('workarea', 'obstacle', shrunk);
        expect(current[recorded.id]).toBeInstanceOf(ObstacleFeature);
        expect((current[recorded.id] as ObstacleFeature).geometry.coordinates).toEqual(shrunk.coordinates);

        convert('obstacle', 'workarea');
        expect(current[recorded.id]).toBeInstanceOf(MowingAreaFeature);
        expect((current[recorded.id] as MowingAreaFeature).geometry.coordinates)
            .toEqual(square(4, 4, 6, 6).coordinates);
    });

    it('keeps an obstacle outline the operator edited after the shrink', () => {
        const parent = areaFeature('area-0-area-0', square(0, 0, 10, 10));
        const recorded = areaFeature('area-1-area-0', square(4, 4, 6, 6));
        const memory = makeShrinkMemory();
        let current: Record<string, MowingFeature> = {[parent.id]: parent, [recorded.id]: recorded};
        const setFeatures = vi.fn((next: Record<string, MowingFeature>) => { current = next; });
        const options = () => ({
            features: current, setFeatures, editMap: true, mowingAreas: [],
            drawRef: {current: null}, notification: {error: vi.fn(), info: vi.fn(), success: vi.fn()},
            mapInstanceRef: {current: null}, shrinkMemory: memory,
        } as unknown as UseMapEditingOptions);
        const hook = renderHook(() => useMapEditing(options()));
        const convert = (from: string, to: string, corrected?: ReturnType<typeof square>) => {
            act(() => hook.result.current.setCurMowingAreaFeature({
                id: recorded.id, index: 0, name: '', mowing_order: 1, orig_mowing_order: 1,
                feature_type: to, orig_feature_type: from, shrink_recorded: true,
            }));
            act(() => hook.result.current.updateMowingArea(corrected));
            hook.rerender();
        };

        convert('workarea', 'obstacle', square(4.3, 4.3, 5.7, 5.7));
        const edited = square(4.2, 4.2, 5.8, 5.8);
        (current[recorded.id] as ObstacleFeature).setGeometry(edited);
        convert('obstacle', 'workarea');

        expect((current[recorded.id] as MowingAreaFeature).geometry.coordinates).toEqual(edited.coordinates);
    });
});
