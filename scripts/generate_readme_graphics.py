#!/usr/bin/env python3
"""Generate the six lightweight, accessible animated README illustrations."""
import argparse
from html import escape
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PANELS = {
    "banner.svg": ("Geoff-Sim", "Sensor-only G1 navigation and an eight-person visual crowd", [
        ("8 independent walkers", "16 walk poses + relaxed idle", "Distance-driven gait; no robot yielding"),
        ("Onboard perception", "LiDAR + IMU + joint encoders", "Online SLAM; observed people only"),
        ("30-minute sessions", "MuJoCo goal + RViz decisions", "Experimental: contacts still occur")]),
    "architecture.svg": ("Onboard runtime", "Navigation receives observations; simulator truth is evaluation-only", [
        ("Sense / estimate", "Local LiDAR, IMU, encoders", "odom -> base_footprint; SLAM -> map"),
        ("Track / plan", "People tracks + CV forecasts", "Nav2 events up to 5 Hz; local 45 ms budget"),
        ("Execute / evaluate", "Supervisor -> policy -> DDS", "Truth goes to scorer/logger, not planner")]),
    "command-safety.svg": ("Command checks", "Layered command filtering is not a collision-free guarantee", [
        ("Navigation intent", "/cmd_vel -> social planner", "Swept body checks; selected trajectory"),
        ("Final command", "/cmd_vel_safe -> walking policy", "Freshness, limits, slew and response"),
        ("Actuation feedback", "Joint targets -> rt/lowcmd", "LowState + estimated odometry return")]),
    "install-pipeline.svg": ("Reproducible source setup", "Pinned sources and reviewed patches precede every clean build", [
        ("Checkout", "Clone recursively; configure .env", "Read access to parent and source forks"),
        ("Provision / build", "Apply source patches; rosdep", "Generate crowd assets; build ROS + MuJoCo"),
        ("Verify / launch", "Checksum-pinned official policy", "run_onboard_navigation_demo.sh")]),
    "social-navigation.svg": ("Dynamic social waypoints", "Goals and people are interpreted in the observed map", [
        ("Observe the crowd", "Green: tracked / amber: lost", "Bounded display persistence; real timestamps"),
        ("Choose motion", "Green path; blue/red alternatives", "Cyan: final safety output; visible reasons"),
        ("Advance goals", "Prepare a connected free-space target", "Switch at <= 0.50 m; wait if none is valid")]),
    "verification-tiers.svg": ("Verification and evidence", "Keep unit contracts, visual sessions and scored acceptance separate", [
        ("Fast / unit checks", "Syntax, assets, source patches", "Python + C++ regressions; CI artifacts"),
        ("Visual integration", "Eight walkers, sensors and HUD", "Unscored; contacts logged; episode restarts"),
        ("Scored acceptance", "Goal completion + collision gates", "Current eight-person avoidance still fails")]),
}


def graphic(title, subtitle, panels):
    parts = [f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1200 330" role="img" aria-labelledby="title desc">
  <title id="title">{escape(title)}</title>
  <desc id="desc">{escape(subtitle)}. Schematic illustration, not recorded benchmark footage.</desc>
  <style>
    text {{font-family:Arial,Helvetica,sans-serif}}
    .flow {{stroke-dasharray:8 9;animation:flow 1.4s linear infinite}}
    .pulse {{animation:pulse 2.4s ease-in-out infinite}}
    @keyframes flow {{to {{stroke-dashoffset:-34}}}}
    @keyframes pulse {{0%,100% {{opacity:.45}} 50% {{opacity:1}}}}
    @media (prefers-reduced-motion:reduce) {{.flow,.pulse {{animation:none}}}}
  </style>
  <rect width="1200" height="330" rx="18" fill="#0b1727"/>
  <text x="34" y="53" fill="#edf6ff" font-size="32" font-weight="bold">{escape(title)}</text>
  <text x="35" y="85" fill="#a8c1d6" font-size="17">{escape(subtitle)}</text>
''']
    colors = ("#43d2ec", "#7de39a", "#d3a5ff")
    for i, (heading, first, second) in enumerate(panels):
        x = 35+i*395
        parts.append(f'''  <rect x="{x}" y="119" width="340" height="144" rx="12" fill="#14273b" stroke="{colors[i]}"/>
  <circle class="pulse" cx="{x+22}" cy="146" r="5" fill="{colors[i]}"/>
  <text x="{x+38}" y="153" fill="{colors[i]}" font-size="21" font-weight="bold">{escape(heading)}</text>
  <text x="{x+18}" y="192" fill="#edf6ff" font-size="16">{escape(first)}</text>
  <text x="{x+18}" y="229" fill="#acc2d6" font-size="14">{escape(second)}</text>
''')
        if i < 2:
            parts.append(f'  <path class="flow" d="M{x+346} 192h40" stroke="#43d2ec" stroke-width="3"/><path d="M{x+379} 187l8 5-8 5" fill="none" stroke="#43d2ec" stroke-width="2"/>\n')
    parts.append('  <text x="35" y="302" fill="#8ca8c0" font-size="13">Simulation research prototype | Illustrative data flow | See DEMO_RESULTS.md for measured outcomes</text>\n</svg>\n')
    return "".join(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    for name, content in PANELS.items():
        path = ROOT/"docs/media"/name
        expected = graphic(*content)
        if args.check:
            if not path.exists() or path.read_text() != expected:
                raise SystemExit(f"Outdated graphic: {path}")
        else:
            path.write_text(expected)
    print("README graphics are current")


if __name__ == "__main__":
    main()
