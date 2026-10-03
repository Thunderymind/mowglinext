import reactUseWebSocketModule from "react-use-websocket";
import {useEffect, useRef, useState} from "react";
import {wsBase} from "../utils/apiHost";
import {
    getMultiplexedSocket,
    isMultiplexableSubscribeUri,
    topicFromSubscribeUri,
    type MultiplexStatus,
} from "./multiplexedSocket.ts";

// Vite 8 CJS interop may wrap the default export differently at runtime
const useWebSocket = (reactUseWebSocketModule as unknown as { default: typeof reactUseWebSocketModule }).default ?? reactUseWebSocketModule;

/**
 * useMultiplexStatus — live connection status of the shared multiplex
 * WebSocket. Lets the app shell render a truthful offline indicator instead
 * of assuming the stream is up. "closed" covers both idle (no subscribers)
 * and a dropped connection awaiting reconnect.
 */
export const useMultiplexStatus = (): MultiplexStatus => {
    const [status, setStatus] = useState<MultiplexStatus>(() => getMultiplexedSocket().getStatus());
    useEffect(() => getMultiplexedSocket().onStatusChange(setStatus), []);
    return status;
};

/**
 * useWS — start/stop a stream of payloads from the GUI backend.
 *
 * Subscribe URIs (`/api/mowglinext/subscribe/<topic>`) are multiplexed
 * over a single shared WebSocket via {@link getMultiplexedSocket}.
 * Other URIs (e.g. `/api/mowglinext/publish/joy`) keep their dedicated
 * connection because they need bidirectional traffic that the
 * multiplex protocol does not handle yet.
 *
 * The signature is unchanged from before #177 so every existing hook
 * keeps working without modification.
 */
export const useWS = <T>(
    onError: (e: Error) => void,
    onInfo: (msg: string) => void,
    onData: (data: T, first?: boolean) => void,
) => {
    const [latencyMs, setLatencyMs] = useState<number | null>(null);
    const [heartbeatStale, setHeartbeatStale] = useState(false);
    const heartbeatTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);
    const lastHeartbeatRef = useRef(0);
    const sendJsonRef = useRef<(message: unknown) => void>(() => {});
    const recyclePublishSocketRef = useRef<() => void>(() => {});

    const stopPublishHeartbeat = () => {
        if (heartbeatTimerRef.current !== null) {
            clearInterval(heartbeatTimerRef.current);
            heartbeatTimerRef.current = null;
        }
    };

    const startPublishHeartbeat = () => {
        stopPublishHeartbeat();
        if (!pubUriRef.current?.endsWith("/api/mowglinext/publish/joy")) return;
        lastHeartbeatRef.current = Date.now();
        setHeartbeatStale(false);
        const sendHeartbeat = () => {
            const now = Date.now();
            if (now - lastHeartbeatRef.current > 3000) {
                setHeartbeatStale(true);
                stopPublishHeartbeat();
                recyclePublishSocketRef.current();
                return;
            }
            sendJsonRef.current({op: "ping", sent_at_ms: now});
        };
        sendHeartbeat();
        heartbeatTimerRef.current = setInterval(sendHeartbeat, 1000);
    };

    // Refs to always invoke the latest callbacks, avoiding stale closures.
    const onDataRef = useRef(onData);
    onDataRef.current = onData;
    const onErrorRef = useRef(onError);
    onErrorRef.current = onError;
    const onInfoRef = useRef(onInfo);
    onInfoRef.current = onInfo;

    // Active multiplex unsubscribe + status-listener unregister (set when
    // start() targets a subscribe URI).
    const muxUnsubscribeRef = useRef<(() => void) | null>(null);
    const muxStatusUnsubRef = useRef<(() => void) | null>(null);

    // Publish-side socket (only used for non-subscribe URIs). The state
    // drives useWebSocket below; the ref mirrors it so teardown() never
    // closes over a stale value.
    const [pubUri, setPubUri] = useState<string | null>(null);
    const pubUriRef = useRef<string | null>(null);
    const pubFirstRef = useRef(true);
    const pubDecodeWarnedRef = useRef(false);
    const ws = useWebSocket(pubUri, {
        // This is an operator-control channel, not a shared read stream. A
        // dedicated socket lets the application heartbeat tear it down and
        // create a fresh connection when browser-level ping/pong looks alive
        // but application frames have stopped flowing.
        share: false,
        shouldReconnect: () => true,
        reconnectAttempts: Infinity,
        reconnectInterval: (attempt: number) => Math.min(1000 * Math.pow(2, attempt), 30000),
        onOpen: () => {
            startPublishHeartbeat();
            onInfoRef.current("Stream connected");
        },
        onError: () => {
            onErrorRef.current(new Error("Stream error"));
        },
        onClose: () => {
            stopPublishHeartbeat();
            setHeartbeatStale(true);
            onErrorRef.current(new Error("Stream closed"));
        },
        onMessage: (e: MessageEvent) => {
            let decoded: string;
            try {
                decoded = atob(e.data);
            } catch (err) {
                if (!pubDecodeWarnedRef.current) {
                    pubDecodeWarnedRef.current = true;
                    console.warn("useWS: dropping non-base64 frame", {uri: pubUriRef.current}, err);
                }
                return;
            }

            try {
                const heartbeat = JSON.parse(decoded) as {op?: string; sent_at_ms?: number};
                if (heartbeat.op === "pong" && typeof heartbeat.sent_at_ms === "number") {
                    const now = Date.now();
                    lastHeartbeatRef.current = now;
                    setLatencyMs(Math.max(0, now - heartbeat.sent_at_ms));
                    setHeartbeatStale(false);
                    return;
                }
            } catch {
                // Normal subscribe/publish payload; deliver it unchanged.
            }
            const isFirst = pubFirstRef.current;
            if (isFirst) pubFirstRef.current = false;
            onDataRef.current(decoded as T, isFirst);
        },
    });
    sendJsonRef.current = ws.sendJsonMessage;
    recyclePublishSocketRef.current = () => {
        const uri = pubUriRef.current;
        if (uri === null) return;
        setPubUri(null);
        window.setTimeout(() => {
            if (pubUriRef.current === uri) setPubUri(uri);
        }, 50);
    };

    const teardown = () => {
        stopPublishHeartbeat();
        if (muxUnsubscribeRef.current) {
            muxUnsubscribeRef.current();
            muxUnsubscribeRef.current = null;
        }
        if (muxStatusUnsubRef.current) {
            muxStatusUnsubRef.current();
            muxStatusUnsubRef.current = null;
        }
        if (pubUriRef.current !== null) {
            pubUriRef.current = null;
            setPubUri(null);
            pubFirstRef.current = false;
        }
        setLatencyMs(null);
        setHeartbeatStale(false);
    };

    const start = (uri: string) => {
        teardown();

        if (isMultiplexableSubscribeUri(uri)) {
            const topic = topicFromSubscribeUri(uri);
            let firstReported = false;
            muxUnsubscribeRef.current = getMultiplexedSocket().subscribe(
                topic,
                (data, isFirst) => {
                    if (isFirst && !firstReported) {
                        firstReported = true;
                        onInfoRef.current("Stream connected");
                    }
                    onDataRef.current(data as T, isFirst);
                },
            );
            // Surface shared-socket drops/reconnects through the caller's
            // handlers so they are no longer dead code on the multiplex path.
            let sawClose = false;
            muxStatusUnsubRef.current = getMultiplexedSocket().onStatusChange((status) => {
                if (status === "closed") {
                    sawClose = true;
                    onErrorRef.current(new Error("Stream closed"));
                } else if (status === "open" && sawClose) {
                    sawClose = false;
                    onInfoRef.current("Stream connected");
                }
            });
            return;
        }

        // Publish path: open a dedicated socket as before.
        pubFirstRef.current = true;
        pubDecodeWarnedRef.current = false;
        pubUriRef.current = `${wsBase()}${uri}`;
        setPubUri(pubUriRef.current);
    };

    const stop = () => {
        teardown();
    };

    return {
        start,
        stop,
        sendJsonMessage: ws.sendJsonMessage,
        latencyMs,
        heartbeatStale,
        readyState: ws.readyState,
    };
};
