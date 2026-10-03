#!/usr/bin/env python3
"""Send a bounded test command directly to the robot-local cmd_vel relay."""

import argparse
import asyncio
import json
import sys
import time

import websockets


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--linear", type=float, default=0.08)
    parser.add_argument("--angular", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--rate", type=float, default=10.0)
    args = parser.parse_args()
    if not 0.0 < args.duration <= 30.0:
        parser.error("duration must be in (0, 30] seconds")
    if not 5.0 <= args.rate <= 20.0:
        parser.error("rate must be in [5, 20] Hz")
    if abs(args.linear) > 0.20:
        parser.error("absolute linear speed must be <= 0.20 m/s")
    if abs(args.angular) > 1.0:
        parser.error("absolute angular speed must be <= 1.0 rad/s")
    return args


def frame(linear: float, angular: float) -> str:
    return json.dumps(
        {
            "twist": {
                "linear": {"x": linear, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": angular},
            }
        },
        separators=(",", ":"),
    )


async def run(args: argparse.Namespace) -> None:
    period = 1.0 / args.rate
    sent = 0
    max_gap = 0.0
    gaps_over_150_ms = 0
    started = time.monotonic()
    previous_send = started
    print(
        json.dumps(
            {
                "event": "start",
                "monotonic": started,
                "linear": args.linear,
                "angular": args.angular,
                "duration": args.duration,
                "rate": args.rate,
            }
        ),
        flush=True,
    )

    async with websockets.connect("ws://127.0.0.1:8766", ping_interval=None) as socket:
        next_send = started
        try:
            while True:
                now = time.monotonic()
                if now - started >= args.duration:
                    break
                gap = now - previous_send
                if sent:
                    max_gap = max(max_gap, gap)
                    if gap > 0.150:
                        gaps_over_150_ms += 1
                        print(
                            json.dumps({"event": "send_gap", "elapsed": now - started, "gap": gap}),
                            flush=True,
                        )
                await socket.send(frame(args.linear, args.angular))
                sent += 1
                previous_send = now
                next_send += period
                await asyncio.sleep(max(0.0, next_send - time.monotonic()))
        finally:
            zero = frame(0.0, 0.0)
            for _ in range(20):
                await socket.send(zero)
                await asyncio.sleep(0.05)

    print(
        json.dumps(
            {
                "event": "summary",
                "sent": sent,
                "max_gap_ms": round(max_gap * 1000.0, 3),
                "gaps_over_150_ms": gaps_over_150_ms,
            }
        ),
        flush=True,
    )


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(run(args))
    except Exception as exc:
        print(json.dumps({"event": "error", "error": str(exc)}), file=sys.stderr, flush=True)
        raise


if __name__ == "__main__":
    main()
