#!/usr/bin/env python3
"""Record short, labelled GIF previews of the running MuJoCo/RViz windows (X11)."""
import argparse
import ctypes
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import tempfile
import time

from PIL import Image, ImageDraw


def decode_xwd(data):
    header = struct.unpack(">25I", data[:100])
    if not (header[1] == 7 and header[7] == 0 and header[11] in (24, 32)
            and header[14:17] == (0xff0000, 0xff00, 0xff)):
        raise ValueError("Unsupported XWD visual format; use a 24/32-bit TrueColor X display")
    offset = header[0] + header[19]*12
    return Image.frombytes("RGB", (header[4], header[5]), data[offset:], "raw",
                           "BGRX" if header[11] == 32 else "BGR", header[12], 1)


def activate(window):
    class Data(ctypes.Union):
        _fields_ = [("l", ctypes.c_long*5), ("b", ctypes.c_char*20)]
    class Client(ctypes.Structure):
        _fields_ = [("type", ctypes.c_int), ("serial", ctypes.c_ulong), ("send_event", ctypes.c_int),
                    ("display", ctypes.c_void_p), ("window", ctypes.c_ulong), ("message_type", ctypes.c_ulong),
                    ("format", ctypes.c_int), ("data", Data)]
    class Event(ctypes.Union):
        _fields_ = [("client", Client), ("pad", ctypes.c_long*24)]
    x = ctypes.CDLL("libX11.so.6")
    x.XOpenDisplay.restype = ctypes.c_void_p
    x.XDefaultRootWindow.argtypes = [ctypes.c_void_p]; x.XDefaultRootWindow.restype = ctypes.c_ulong
    x.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]; x.XInternAtom.restype = ctypes.c_ulong
    x.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long, ctypes.c_void_p]
    x.XFlush.argtypes = [ctypes.c_void_p]; x.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = x.XOpenDisplay(None)
    if not display:
        raise RuntimeError("Cannot open X display; check DISPLAY and XAUTHORITY")
    event = Event()
    event.client.type, event.client.send_event, event.client.display = 33, 1, display
    event.client.window = int(window, 16)
    event.client.message_type = x.XInternAtom(display, b"_NET_ACTIVE_WINDOW", 0)
    event.client.format, event.client.data.l[0] = 32, 2
    x.XSendEvent(display, x.XDefaultRootWindow(display), 0, (1 << 19) | (1 << 20), ctypes.byref(event))
    x.XFlush(display); x.XCloseDisplay(display)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).parents[1]/"docs/media")
    parser.add_argument("--delay", type=float, default=25.)
    parser.add_argument("--seconds", type=float, default=8.)
    parser.add_argument("--fps", type=int, default=4)
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--run", default="", help="Artifact directory identifying the source run")
    parser.add_argument("--people", type=int, default=8)
    parser.add_argument("--window", choices=['both', 'mujoco', 'rviz'], default='both')
    args = parser.parse_args()
    if not (0 < args.seconds <= 30 and 1 <= args.fps <= 10 and 320 <= args.width <= 1600):
        parser.error("Use 0–30 seconds, 1–10 fps and a width of 320–1600")
    os.environ.setdefault("DISPLAY", ":0")
    os.environ.setdefault("XAUTHORITY", "/tmp/.docker.xauth")
    time.sleep(max(0., args.delay))
    tree = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True)
    windows = {}
    for line in tree.splitlines():
        name = "onboard-crowd" if "MuJoCo :" in line else "onboard-decisions" if "g1_onboard_waypoints.rviz" in line else None
        match = re.search(r"0x[0-9a-f]+", line)
        if name and match:
            windows.setdefault(name, match.group())
    wanted = ['onboard-crowd', 'onboard-decisions'] if args.window == 'both' else [
        'onboard-crowd' if args.window == 'mujoco' else 'onboard-decisions']
    if any(name not in windows for name in wanted):
        raise RuntimeError("Requested MuJoCo/RViz window must be running")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    metadata = {"recorded_at": datetime.now(timezone.utc).isoformat(), "run": args.run,
                "kind": "unscored live visualization", "fps": args.fps, "clips": []}
    with tempfile.TemporaryDirectory(prefix="geoff-preview-") as temp:
        for name in wanted:
            activate(windows[name])
            time.sleep(1.)
            frames = []
            for frame in range(round(args.seconds*args.fps)):
                started = time.monotonic()
                capture = Path(temp)/"frame.xwd"
                subprocess.run(["xwd", "-silent", "-id", windows[name], "-out", str(capture)], check=True)
                image = decode_xwd(capture.read_bytes())
                if name == "onboard-crowd":
                    image = image.crop((int(image.width*.315), 0, image.width, image.height))
                image = image.resize((args.width, round(image.height*args.width/image.width)), getattr(Image, "Resampling", Image).LANCZOS)
                canvas = Image.new("RGB", (image.width, image.height+30), "#102033")
                canvas.paste(image, (0, 30))
                label = f"MUJOCO: {args.people} independent walkers" if name == "onboard-crowd" else "RVIZ: onboard sensing, predictions and decisions"
                ImageDraw.Draw(canvas).text((10, 9), label+" | unscored preview", fill="white")
                frames.append(canvas)
                time.sleep(max(0., 1./args.fps-(time.monotonic()-started)))
            palette = frames[0].quantize(colors=128)
            frames = [frame.quantize(palette=palette, dither=getattr(Image, "Dither", Image).NONE) for frame in frames]
            output = args.output_dir/(name+".gif")
            frames[0].save(output, save_all=True, append_images=frames[1:], duration=round(1000/args.fps), loop=0, optimize=True, disposal=2)
            metadata["clips"].append({"file": output.name, "frames": len(frames), "bytes": output.stat().st_size})
            print(output)
    (args.output_dir/"recordings.json").write_text(json.dumps(metadata, indent=2)+"\n")


if __name__ == "__main__":
    main()
