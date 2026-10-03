import {afterEach, beforeEach, describe, expect, it, vi} from "vitest";
import {MULTIPLEX_STALE_TIMEOUT_MS, MultiplexedSocket} from "./multiplexedSocket.ts";

class FakeWebSocket {
    static instances: FakeWebSocket[] = [];

    binaryType = "";
    onopen: (() => void) | null = null;
    onmessage: ((event: MessageEvent) => void) | null = null;
    onerror: (() => void) | null = null;
    onclose: (() => void) | null = null;
    send = vi.fn();
    close = vi.fn();

    constructor(public readonly url: string) {
        FakeWebSocket.instances.push(this);
    }

    open(): void {
        this.onopen?.();
    }
}

describe("MultiplexedSocket stale detection", () => {
    beforeEach(() => {
        vi.useFakeTimers();
        vi.setSystemTime(0);
        FakeWebSocket.instances = [];
        vi.stubGlobal("WebSocket", FakeWebSocket);
    });

    afterEach(() => {
        vi.unstubAllGlobals();
        vi.useRealTimers();
    });

    it("abandons a half-open socket and schedules a reconnect for live topics", () => {
        const socket = new MultiplexedSocket("ws://robot/multiplex");
        socket.subscribe("pose", vi.fn());
        const first = FakeWebSocket.instances[0];
        first.open();

        vi.advanceTimersByTime(MULTIPLEX_STALE_TIMEOUT_MS);

        expect(first.close).toHaveBeenCalledOnce();
        expect(socket.getStatus()).toBe("closed");

        vi.advanceTimersByTime(1_000);
        expect(FakeWebSocket.instances).toHaveLength(2);
        expect(socket.getStatus()).toBe("connecting");
    });

    it("does not churn a socket that only carries latched state", () => {
        const socket = new MultiplexedSocket("ws://robot/multiplex");
        socket.subscribe("path", vi.fn());
        const first = FakeWebSocket.instances[0];
        first.open();

        vi.advanceTimersByTime(MULTIPLEX_STALE_TIMEOUT_MS * 2);

        expect(first.close).not.toHaveBeenCalled();
        expect(socket.getStatus()).toBe("open");
    });
});
