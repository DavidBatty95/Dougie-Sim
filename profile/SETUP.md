# Publish the GitHub profile

This folder is a ready-to-copy profile package for **DavidBatty95**.

## Installation

1. Create a **public** GitHub repository named **`DavidBatty95`** under the
   `DavidBatty95` account. GitHub uses this special repository's root README on
   your profile Overview page. If it already exists, edit that repository instead.
2. Copy `profile/README.md` to the new repository as **`README.md`**.
3. Copy the complete `profile/assets/` directory to **`assets/`** beside it.
4. Commit those files to that repository's default branch, then visit
   <https://github.com/DavidBatty95>.

The resulting layout must be:

```text
DavidBatty95/
├── README.md
└── assets/
    ├── robotics-banner.svg
    ├── vec-systems.svg
    └── research-map.svg
```

Uploading only the README, or leaving it inside a `profile/` subfolder of the
destination repository, will not produce the intended profile.

## Design and animation

- Original, self-contained SVG graphics; no external fonts, scripts, trackers,
  animation services or generated statistics cards.
- Geoff-Sim palette: midnight `#0b1727`, panel blue `#14273b`, cyan `#43d2ec`,
  mint `#7de39a`, violet `#d3a5ff` and off-white `#edf6ff`.
- CSS animation lives inside the SVG images, which are embedded with ordinary
  GitHub-compatible image tags. No README-level CSS or JavaScript is needed.
- Each image has a complete static composition, an accessible title/description,
  README alt text and a `prefers-reduced-motion` rule. Animation support depends
  on the viewer; clients that suppress animation retain the static artwork.
- The graphics are conceptual illustrations, not live telemetry or measured results.
- The hero includes a humanoid, a walking Go2-style quadruped and a hovering
  drone. These are stylised drawings, not manufacturer CAD models.

Open the SVGs directly in a browser to preview the animations. Preview the README
on GitHub after uploading to confirm its final layout and rendering.

## Content and links

Research descriptions and the email address were taken from the public PhD
repository README. The Pepper repository identifies the Virtual Engineering
Centre and its University of Liverpool affiliation. Geoff-Sim's description and
development focus follow the local project README.

Geoff-Sim is grouped under current VEC work based on the requested profile scope.
Confirm that grouping before publishing. The public GitHub repository listing
available during preparation did not include Geoff-Sim: ensure it is public so
profile visitors can follow its links. Its documentation links assume `main` is
the published default branch; adjust those URLs if needed.

The web-development section links directly to the live Scouts site at
<https://aardvarkesuwebsite.onrender.com>, as requested.

Published works are awaiting the author's titles, DOI links or Scholar/ORCID
profile. When supplied, add a **Selected publications** section after PhD research,
with each paper's title, authors, venue, year and DOI/publisher link, and add it to
the top navigation. Publication citations have deliberately not been guessed.

Update the **Development focus** paragraph as the work progresses. It is curated
profile content, not an automatically refreshed activity feed. You can add a
thesis or publications link to the PhD section when you have a public URL.
