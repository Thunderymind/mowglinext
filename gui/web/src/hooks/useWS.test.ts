import {act, renderHook} from "@testing-library/react";
import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";

const socketMock = vi.hoisted(() => ({
    urls: [] as Array<string | null>,
    options: undefined as Record<string, () => void> | undefined,
    sendJsonMessage: vi.fn(),
}));

vi.mock("react-use-websocket", () => ({
    default: (url: string | null, options: Record<string, () => void>) => {
        socketMock.urls.push(url);
        socketMock.options = options;
        return {
            sendJsonMessage: socketMock.sendJsonMessage,
            readyState: url === null ? 3 : 1,
        };
    },
}));

import {useWS} from "./useWS.ts";

describe("useWS joy heartbeat", () => {
    beforeEach(() => {
        vi.useFakeTimers();
        socketMock.urls = [];
        socketMock.options = undefined;
        socketMock.sendJsonMessage.mockReset();
    });

    afterEach(() => {
        vi.useRealTimers();
    });

    it("recycles a half-open joy socket after heartbeat timeout", () => {
        const {result} = renderHook(() => useWS(vi.fn(), vi.fn(), vi.fn()));
        act(() => result.current.start("/api/mowglinext/publish/joy"));
        const joyUrl = socketMock.urls[socketMock.urls.length - 1]!;
        expect(joyUrl).toMatch(/\/api\/mowglinext\/publish\/joy$/);

        act(() => socketMock.options?.onOpen());
        act(() => vi.advanceTimersByTime(4001));
        expect(socketMock.urls[socketMock.urls.length - 1]).toBeNull();
        expect(result.current.heartbeatStale).toBe(true);

        act(() => vi.advanceTimersByTime(50));
        expect(socketMock.urls[socketMock.urls.length - 1]).toBe(joyUrl);
    });
});
