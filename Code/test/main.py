from __future__ import annotations

import argparse
import collections
import logging
import math
import signal
import threading
import time

import serial

from .protocol import FrameParser, Measurement, RotationAssembler, valid_measurements
from .render import ScanRenderer
from .viewer import Viewer

LOG = logging.getLogger("ms200")
DEFAULT_START_COMMAND = bytes.fromhex("A5 F5 A2 C1 01 81 B3 31 F2")


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MS200 LiDAR browser viewer")
    parser.add_argument("--serial-port", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--web-host", default="0.0.0.0")
    parser.add_argument("--web-port", type=int, default=8000)
    parser.add_argument("--refresh-ms", type=int, default=100)
    parser.add_argument("--render-fps", type=float, default=10.0)
    parser.add_argument("--size", type=int, default=600, help="output image width/height")
    parser.add_argument("--range-mm", type=int, default=6000, help="radius shown around LiDAR")
    parser.add_argument("--min-distance-mm", type=int, default=30)
    parser.add_argument("--min-confidence", type=int, default=1)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--mode", choices=("rotation", "trail"), default="rotation")
    parser.add_argument("--trail-points", type=int, default=1200)
    parser.add_argument("--reconnect-seconds", type=float, default=2.0)
    parser.add_argument("--no-start-command", action="store_true")
    parser.add_argument("--demo", action="store_true", help="run without LiDAR hardware")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def open_serial(args: argparse.Namespace) -> serial.Serial:
    device = serial.Serial(args.serial_port, args.baud, timeout=0.1)
    device.reset_input_buffer()
    if not args.no_start_command:
        device.write(DEFAULT_START_COMMAND)
        device.flush()
    return device


def demo_points(phase: float, range_mm: int) -> list[Measurement]:
    result = []
    for degree in range(360):
        radians = math.radians(degree)
        distance = range_mm * (0.45 + 0.12 * math.sin(radians * 3 + phase))
        result.append(Measurement(float(degree), int(distance), 255))
    return result


def run() -> int:
    args = arguments()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    if args.render_fps <= 0 or args.range_mm <= 0 or not 1 <= args.jpeg_quality <= 100:
        raise SystemExit("render-fps/range-mm must be positive and jpeg-quality must be 1..100")

    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda _signum, _frame: stop.set())

    viewer = Viewer(args.web_host, args.web_port, args.refresh_ms)
    renderer = ScanRenderer(args.size, args.range_mm, args.jpeg_quality)
    viewer.start()
    LOG.info("Open http://<Raspberry-Pi-IP>:%d/ on the Windows PC", args.web_port)

    parser = FrameParser()
    rotations = RotationAssembler()
    trail: collections.deque[Measurement] = collections.deque(maxlen=args.trail_points)
    displayed: list[Measurement] = []
    packets = 0
    reconnects = 0
    last_render = 0.0

    try:
        if args.demo:
            viewer.update_status(state="streaming", message="demo mode")
            phase = 0.0
            while not stop.wait(1.0 / args.render_fps):
                points = demo_points(phase, args.range_mm)
                viewer.update_jpeg("MS200", renderer.jpeg(renderer.render(points)))
                viewer.update_status(points=len(points), last_packet_monotonic=time.monotonic())
                phase += 0.08
            return 0

        while not stop.is_set():
            device: serial.Serial | None = None
            try:
                viewer.update_status(state="connecting", message=f"opening {args.serial_port}")
                device = open_serial(args)
                viewer.update_status(state="streaming", message=args.serial_port, serial_reconnects=reconnects)
                LOG.info("MS200 connected on %s at %d baud", args.serial_port, args.baud)

                while not stop.is_set():
                    data = device.read(512)
                    if not data:
                        continue
                    for packet in parser.feed(data):
                        packets += 1
                        valid = valid_measurements(
                            packet.measurements,
                            args.min_distance_mm,
                            args.range_mm,
                            args.min_confidence,
                        )
                        if args.mode == "trail":
                            trail.extend(valid)
                            displayed = list(trail)
                        else:
                            completed = rotations.add(packet)
                            if completed is not None:
                                displayed = valid_measurements(
                                    completed,
                                    args.min_distance_mm,
                                    args.range_mm,
                                    args.min_confidence,
                                )

                        now = time.monotonic()
                        viewer.update_status(
                            packets=packets,
                            points=len(displayed),
                            crc_errors=parser.crc_errors,
                            speed_dps=packet.speed_dps,
                            last_packet_monotonic=now,
                        )
                        if displayed and now - last_render >= 1.0 / args.render_fps:
                            viewer.update_jpeg("MS200", renderer.jpeg(renderer.render(displayed)))
                            last_render = now
            except (serial.SerialException, OSError) as error:
                if stop.is_set():
                    break
                reconnects += 1
                message = f"{type(error).__name__}: {error}"
                LOG.warning("LiDAR disconnected: %s; retrying", message)
                viewer.update_status(state="error", message=message, serial_reconnects=reconnects)
                stop.wait(args.reconnect_seconds)
            finally:
                if device and device.is_open:
                    device.close()
    finally:
        viewer.update_status(state="stopping", message="shutdown")
        viewer.stop()
        LOG.info("Stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
