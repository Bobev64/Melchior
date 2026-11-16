#!/usr/bin/env python3
"""Animate roll/pitch telemetry as a 3D body.

Install: python -m pip install matplotlib numpy pillow
Run: python Sim/visualize_attitude.py Sim/telemetry_TIMESTAMP.csv
Export: python Sim/visualize_attitude.py capture.csv --save attitude.gif --fps 30

Convention: right-handed axes, roll about X, pitch about Y; yaw is zero.
Body-to-world rotation is Ry(pitch) @ Rx(roll). X is the body's nose.
Space or the button pauses playback; drag the slider to seek.
Drag the 3D view to orbit the camera. Playback stops at the end.
"""

import argparse
import csv
import math
import sys
import time

from pathlib import Path
from plot_telemetry import load_telemetry


def rotation_matrix(roll_deg, pitch_deg):
    """Return Ry(pitch) @ Rx(roll), using right-handed degree angles."""
    roll, pitch = math.radians(roll_deg), math.radians(pitch_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    return ((cp, sp * sr, sp * cr), (0, cr, -sr), (-sp, cp * sr, cp * cr))


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_file", type=Path)
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed multiplier (default: 1)")
    parser.add_argument("--save", type=Path, help="Export a GIF using Pillow instead of opening a window")
    parser.add_argument("--fps", type=int, default=30, help="GIF frames per second (1-100; default: 30)")
    args = parser.parse_args()
    if args.save and args.save.suffix.lower() != ".gif":
        parser.error("--save must name a .gif file")
    if not 1 <= args.fps <= 100:
        parser.error("--fps must be between 1 and 100")
    if not math.isfinite(args.speed) or args.speed <= 0:
        parser.error("--speed must be finite and positive")
    try:
        times, rolls, pitches = load_telemetry(args.csv_file)
        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError("Timestamps must strictly increase; split captures at resets")
    except (OSError, ValueError, csv.Error) as exc:
        print(f"Cannot read telemetry: {exc}", file=sys.stderr)
        return 1

    try:
        import numpy as np
        import matplotlib
        if args.save:
            matplotlib.use("Agg")
            from PIL import Image  # Verify Pillow is installed before rendering.
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation, PillowWriter
        from matplotlib.widgets import Button, Slider
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except ImportError:
        print("Install dependencies: python -m pip install matplotlib numpy pillow", file=sys.stderr)
        return 1

    # Unwrap before interpolation to avoid a full rotation at +/-180 degrees.
    elapsed = np.asarray(times) - times[0]
    roll_curve = np.unwrap(np.radians(rolls))
    pitch_curve = np.unwrap(np.radians(pitches))
    duration = float(elapsed[-1])
    vertices = np.array([
        [-1, -.55, -.12], [1, -.55, -.12], [1, .55, -.12], [-1, .55, -.12],
        [-1, -.55, .12], [1, -.55, .12], [1, .55, .12], [-1, .55, .12],
    ])
    faces = np.array([[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
                      [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]])
    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(bottom=.22, top=.90)
    ax.set(xlim=(-1.5, 1.5), ylim=(-1.5, 1.5), zlim=(-1.5, 1.5),
           xlabel="World X", ylabel="World Y", zlabel="World Z",
           title=f"{args.csv_file.name}\nRoll about X • Pitch about Y • Yaw = 0")
    ax.set_box_aspect((1, 1, 1))
    body = Poly3DCollection(vertices[faces], edgecolors="black", alpha=.85,
                            facecolors=["gray", "skyblue", "steelblue", "red", "steelblue", "gray"])
    ax.add_collection3d(body)
    body_axes = [ax.plot([], [], [], color=color, linewidth=3, label=label)[0]
                 for color, label in zip(("red", "green", "blue"),
                                        ("Body X / nose", "Body Y", "Body Z"))]
    ax.legend(loc="upper left", fontsize=8)
    status = fig.text(.5, .15, "", ha="center")
    slider = Slider(fig.add_axes([.18, .09, .65, .03]), "Time (s)",
                    0, duration if duration > 0 else 1, valinit=0)
    button = Button(fig.add_axes([.40, .02, .20, .045]), "Pause")
    state = {"playing": duration > 0, "position": 0.0, "clock": time.perf_counter()}

    def render(position):
        roll = float(np.degrees(np.interp(position, elapsed, roll_curve)))
        pitch = float(np.degrees(np.interp(position, elapsed, pitch_curve)))
        matrix = np.asarray(rotation_matrix(roll, pitch))
        rotated = vertices @ matrix.T
        body.set_verts(rotated[faces])
        for index, line in enumerate(body_axes):
            end = matrix[:, index] * 1.35
            line.set_data_3d([0, end[0]], [0, end[1]], [0, end[2]])
        status.set_text(f"Elapsed: {position:.2f} s    Roll: {roll:.2f}°    Pitch: {pitch:.2f}°")
        fig.canvas.draw_idle()

    def seek(value):
        state["position"] = min(float(value), duration)
        state["clock"] = time.perf_counter()
        render(state["position"])

    def toggle(event=None):
        if duration == 0:
            return
        if not state["playing"] and state["position"] >= duration:
            slider.set_val(0)
        state["playing"] = not state["playing"]
        state["clock"] = time.perf_counter()
        button.label.set_text("Pause" if state["playing"] else "Play")
        fig.canvas.draw_idle()

    def tick(_):
        now = time.perf_counter()
        delta = now - state["clock"]
        state["clock"] = now
        if state["playing"]:
            position = min(duration, state["position"] + delta * args.speed)
            slider.set_val(position)
            if position >= duration:
                state["playing"] = False
                button.label.set_text("Replay")

    slider.on_changed(seek)
    button.on_clicked(toggle)
    fig.canvas.mpl_connect("key_press_event", lambda event: toggle() if event.key == " " else None)
    if duration == 0:
        slider.set_active(False)
        button.label.set_text("Single sample")
    render(0)
    if args.save:
        # Export deterministic sample times, independent of rendering speed.
        slider.ax.set_visible(False)
        button.ax.set_visible(False)
        frame_count = max(1, math.ceil(duration / args.speed * args.fps) + 1)
        animation = FuncAnimation(
            fig, lambda frame: render(min(frame * args.speed / args.fps, duration)),
            frames=frame_count, interval=1000 / args.fps, cache_frame_data=False,
        )
        try:
            args.save.parent.mkdir(parents=True, exist_ok=True)
            print(f"Rendering {frame_count} frames to {args.save}...")
            animation.save(str(args.save), writer=PillowWriter(fps=args.fps), dpi=100)
            print(f"Saved GIF to {args.save}")
        except (OSError, ValueError) as exc:
            print(f"Cannot save GIF: {exc}", file=sys.stderr)
            return 1
        finally:
            plt.close(fig)
        return 0
    # Keep a reference alive while the GUI runs.
    animation = FuncAnimation(fig, tick, interval=33, cache_frame_data=False)
    plt.show()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
