#!/usr/bin/env python3
"""Render the retained reactor campaign overview from its portable JSON."""

import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "docs/results/dougie_reactor_campaign_20261114.json"
OUTPUT = ROOT / "docs/media/dougie/campaign-overview.svg"


def render():
    raw = SOURCE.read_bytes()
    data = json.loads(raw)
    rows = []
    for index, (key, label, color) in enumerate((
        ("blind", "Radiation-blind", "#e69739"),
        ("aware", "Radiation-aware", "#28a99c"),
    )):
        result = data["outcomes"][key]
        y = 107 + index * 108
        coverage = result["explored_percent"]
        rows.append(f'''<text x="30" y="{y}" class="label">{label}</text>
<rect x="235" y="{y - 17}" width="270" height="22" rx="4" fill="#25364c"/>
<rect x="235" y="{y - 17}" width="{coverage * 2.7:.2f}" height="22" rx="4" fill="{color}"/>
<text x="515" y="{y}" class="value">{coverage:.2f}%</text>
<text x="620" y="{y}" class="value">{result['total_dose_usv']:.3f} µSv</text>
<text x="760" y="{y}" class="value">{result['robots_recovered']} / {result['robots_lost']}</text>''')
    digest = hashlib.sha256(raw).hexdigest()
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="900" height="320" viewBox="0 0 900 320" role="img" aria-labelledby="title desc">
<title id="title">Illustrative six-sortie reactor campaign</title>
<desc id="desc">Blind: 90.30 percent observed, 10.949 microsieverts, one robot recovered and five lost. Aware: 51.93 percent observed, 1.207 microsieverts, six recovered and none lost. Exploration is unequal and neither survey was completed.</desc>
<metadata>Source: docs/results/dougie_reactor_campaign_20261114.json; SHA-256: {digest}</metadata>
<rect width="900" height="320" rx="12" fill="#101c2d"/>
<style>text{{font-family:system-ui,sans-serif;fill:#e8eef6}} .title{{font-size:22px;font-weight:700}} .heading{{font-size:13px;fill:#aabbd0}} .label{{font-size:16px;font-weight:600}} .value{{font-size:16px;font-weight:600}} .note{{font-size:13px;fill:#b9c8d9}}</style>
<text x="30" y="35" class="title">One reactor campaign · six sorties per team</text>
<text x="235" y="70" class="heading">REFERENCE FLOOR OBSERVED</text>
<text x="620" y="70" class="heading">TOTAL DOSE</text>
<text x="760" y="70" class="heading">RECOVERED / LOST</text>
{''.join(rows)}
<text x="30" y="275" class="note">Unequal exploration: endpoint dose is not an equal-work comparison.</text>
<text x="30" y="296" class="note">Both campaigns reached the sortie limit; neither completed a frontier survey.</text>
</svg>
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="check the committed SVG is current")
    args = parser.parse_args()
    figure = render()
    if args.check:
        if not OUTPUT.exists() or OUTPUT.read_text(encoding="utf-8") != figure:
            parser.error("campaign overview differs from the retained source")
    else:
        OUTPUT.write_text(figure, encoding="utf-8")
    print(f"{'Checked' if args.check else 'Wrote'} {OUTPUT.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
