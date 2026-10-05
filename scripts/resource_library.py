"""Visual resource cards and a topic index derived from verified paper records."""
from html import escape
import json
from pathlib import Path
from textwrap import wrap
from urllib.parse import urlsplit

from readme_navigation import LINKS, menu

TOPICS = (
    ('start', 'Start here & demos', 'Install, explore the worlds and understand the viewers.', 'START_HERE.md', '#43d2ec', 'demos'),
    ('navigation', 'Perception & navigation', 'Sensors, SLAM, people tracking, planning and execution.', 'NAVIGATION.md', '#7de39a', 'roadmap'),
    ('manipulation', 'Locomotion & manipulation', 'Policies, arm control, grasping and payload transport.', 'MANIPULATION.md', '#d3a5ff', 'develop'),
    ('experiments', 'Experiments & evidence', 'Datasets, dashboards, calibration and reproducible trials.', 'EXPERIMENTS.md', '#f4c978', 'results'),
    ('research', 'Research & papers', 'The literature review and all 50 works, grouped by topic.', 'RESEARCH.md', '#d3a5ff', 'research'),
    ('simulation', 'Tutorials, repos & libraries', 'ROS 2 guides, online tutorials, APIs and useful repositories.', 'SIMULATION.md', '#43d2ec', 'folders'),
    ('development', 'Development & contribution', 'Builds, tests, CI, patches and code ownership.', 'DEVELOPMENT.md', '#7de39a', 'develop'),
    ('history', 'Progress & historical archive', 'Milestones, forward plans and earlier research notes.', 'HISTORY.md', '#f4c978', 'progress'),
)

PAPER_GROUPS = (
    ('Surveys, evaluation principles and social behavior',
     'Start here to frame research questions, human-aware behavior and evaluation criteria.',
     ('survey2024', 'survey2013', 'survey2023', 'guidelines', 'proxemics2026')),
    ('Classical avoidance and control foundations',
     'Motion constraints, crowd interactions and safety-oriented control foundations.',
     ('dwa', 'vo', 'orca', 'teb', 'socialforce', 'freezing', 'biped2016', 'cbf', 'safecontrol')),
    ('Learned crowd and dynamic navigation',
     'Learned interaction models and policies for navigating dynamic environments.',
     ('cadrl', 'sarl', 'rgl', 'dsrnn', 'safehumanoid', 'flux', 'omni')),
    ('Perception, localization and motion prediction',
     'Human observations, odometry and forecasts that navigation decisions depend on.',
     ('jrdb', 'drow', 'drspaam', 'fastlio', 'orbslam', 'prediction', 'sociallstm', 'socialgan', 'trajectron')),
    ('Legged locomotion, whole-body control and transfer',
     'Locomotion learning, dynamics adaptation and whole-body skills; embodiments differ.',
     ('rma', 'rudin', 'rough', 'parkour', 'wheeledleg', 'expressive', 'deepmimic', 'domain', 'asap')),
    ('Benchmarks, datasets and embodied environments',
     'Resources for designing comparisons; inclusion does not mean Geoff-Sim integrates them.',
     ('socnavbench', 'socialgym', 'scand', 'omninav', 'metaurban', 'habitat3', 'dped')),
    ('Core software and simulation foundations',
     'Primary papers behind the simulation and ROS navigation ecosystem.',
     ('mujoco', 'ros2', 'nav2', 'slam')),
)


def topic_card(index, title, description, accent, icon):
    lines = wrap(description, width=49)
    if len(lines) > 2:
        raise ValueError(f'Resource card text is too long: {title}')
    body = ''.join(f'<text x="24" y="{87+i*20}" class="body">{escape(line)}</text>'
                   for i, line in enumerate(lines))
    return f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 520 170" width="520" height="170" role="img" aria-labelledby="title desc">
  <title id="title">{escape(title)}</title>
  <desc id="desc">{escape(description)} Open this topic collection using the surrounding link.</desc>
  <style>text{{font-family:Arial,Helvetica,sans-serif}}.body{{fill:#b5c9dc;font-size:15px}}
    .flow{{stroke-dasharray:20 460;animation:flow 10s linear infinite}}@keyframes flow{{to{{stroke-dashoffset:-480}}}}
    @media(prefers-reduced-motion:reduce){{.flow{{animation:none}}}}</style>
  <rect x="1" y="1" width="518" height="168" rx="14" fill="#0b1727" stroke="#29445b"/>
  <text x="24" y="27" fill="{accent}" font-size="11" letter-spacing="2">COLLECTION {index:02d}</text>
  <text x="24" y="57" fill="#edf6ff" font-size="23" font-weight="bold">{escape(title)}</text>
  {body}
  <circle cx="463" cy="46" r="27" fill="#14273b" stroke="#29445b"/>
  <path d="{icon}" transform="translate(446 29) scale(1.4)" fill="none" stroke="{accent}" stroke-width="1.4" stroke-linecap="round" stroke-linejoin="round"/>
  <path d="M24 123H496" stroke="#29445b"/><path class="flow" d="M24 123H496" stroke="{accent}" opacity=".7"/>
  <text x="24" y="148" fill="{accent}" font-size="11" font-weight="bold" letter-spacing="1">OPEN COLLECTION</text>
  <path d="M468 143h24m-6-6 6 6-6 6" fill="none" stroke="{accent}" stroke-width="1.6"/>
</svg>
'''


def graphics():
    icons = {slug: icon for slug, _, _, icon in LINKS}
    result = {f'docs/media/resources/{slug}.svg': topic_card(index, title, description, accent, icons[icon])
              for index, (slug, title, description, _, accent, icon) in enumerate(TOPICS, 1)}
    result['docs/media/resources/library.svg'] = '''<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="190" viewBox="0 0 1200 190" role="img" aria-labelledby="title desc">
  <title id="title">The Geoff-Sim resource library</title>
  <desc id="desc">Browse eight topic collections of project guides, source code, evidence, research papers, official tools and historical notes.</desc>
  <defs><linearGradient id="bg"><stop stop-color="#0b1727"/><stop offset="1" stop-color="#172b42"/></linearGradient></defs>
  <style>text{font-family:Arial,Helvetica,sans-serif}.pulse{animation:pulse 4s ease-in-out infinite}@keyframes pulse{50%{opacity:.5}}@media(prefers-reduced-motion:reduce){.pulse{animation:none}}</style>
  <rect x="1" y="1" width="1198" height="188" rx="18" fill="url(#bg)" stroke="#29445b"/>
  <text x="34" y="34" fill="#43d2ec" font-size="12" letter-spacing="2">EXPLORE BY TOPIC</text>
  <text x="34" y="81" fill="#edf6ff" font-size="35" font-weight="bold">The Geoff-Sim resource library</text>
  <text x="35" y="116" fill="#b5c9dc" font-size="18">Guides · Code · Results · Research · Official tools · Project history</text>
  <text x="35" y="155" fill="#7de39a" font-size="14">Eight curated collections. One place to find the next useful link.</text>
  <g fill="none" stroke="#38566f" stroke-width="2"><path d="M960 52h150v91H960zM980 33h130v110M1000 16h110v127"/><path d="M975 77h114m-114 18h92m-92 18h104" stroke="#43d2ec" opacity=".7"/></g>
  <circle class="pulse" cx="1110" cy="143" r="8" fill="#d3a5ff"/>
</svg>
'''
    return result


def paper_catalogue(root):
    """Use recorded metadata, preserving its inspection scope and every source."""
    root = Path(root)
    sources = json.loads((root/'docs/literature_review/verified_sources.json').read_text())
    records = {source['key']: source for source in sources}
    appraisal_path = root/'docs/literature_review/paper_appraisals.json'
    appraisals = {}
    if appraisal_path.exists():
        items = json.loads(appraisal_path.read_text())
        appraisals = {item['key']: item for item in items}
        if len(appraisals) != len(items) or set(appraisals) != set(records):
            raise ValueError('Expanded appraisals must match the complete verified bibliography')
    keys = [key for _, _, group in PAPER_GROUPS for key in group]
    if len(records) != len(sources) or len(keys) != len(set(keys)) or set(keys) != set(records):
        raise ValueError('Research topic groups must cover each verified source exactly once')
    path = root/'docs/resources/RESEARCH.md'
    parts = ['# Research and paper archive\n\n', menu(path, root), '''

[← Resource library](README.md) · [Research gaps and intended contributions](../literature_review/README.md#research-gaps-and-intended-contributions)
· [Experiments and evidence](EXPERIMENTS.md) · [Forward plan](../PROJECT_PLAN.md)

**Start with the intended contributions:** the
[gap-first Research page](../literature_review/README.md#research-gaps-and-intended-contributions)
sets out calibrated observation/execution coupling, causal onboard diagnosis,
safety–liveness coordination, body/payload risk and contextual human validation.
It links the latest single-PDF review and the evidence needed to support novelty.

[![Research and paper collection](../media/resources/research.svg)](#complete-paper-index)

## Review package and reading paths

| Resource | Use it for |
| --- | --- |
| [Literature-review guide](../literature_review/README.md) | Scope, main argument, build commands and update procedure |
| [Review PDF](../literature_review/literature_review.pdf) | One consolidated expanded review: fundamentals, thematic synthesis, detailed appraisals, novelty roadmap and linked bibliography |
| [Editable LaTeX](../literature_review/literature_review.tex) | Maintaining the review document |
| [Reading audit](../literature_review/READING_AUDIT.md) | What was inspected for each paper, beyond identity verification |
| [Expanded full-text audit](../literature_review/FULLTEXT_AUDIT.md) | Current selected full-text coverage, source copies and explicitly limited appraisals |
| [Search log](../literature_review/SEARCH_LOG.md) | Search scope, cutoff, screening and unresolved follow-up |
| [Verified metadata](../literature_review/verified_sources.json) / [BibTeX](../literature_review/references.bib) | Source identities, recorded retrieval evidence and citations |
| [Grove-G1 review](../GROVE_G1_REVIEW.md) | A separate implementation review informing integration experiments |
| [Historical research drafts](HISTORY.md#earlier-research-notes) | Earlier proposals and their relationship to the current plan |

**Suggested paths:** begin with surveys for orientation; compare classical and
learned navigation for baselines; use perception/prediction for observation
assumptions; use locomotion/transfer for policy response; finish with benchmarks
and evaluation principles when designing an experiment.

## Complete paper index

This index groups all **50 works** in the checked-in bibliography. Titles, years
and URLs come from the verified metadata snapshot; inspection labels use the
expanded structured appraisals when available. The topic grouping is an editorial
navigation aid. The literature-discovery cutoff remains **22 September 2026**;
the **5 October expansion** adds systematic extraction and selected primary
full-text analysis. A verified identity or appraisal is not an experimental replication.

Use browser Find to search a title or citation key. DOI links resolve to publisher
records; arXiv links retain the versions recorded by the review. Access to full
texts may depend on the publisher. See the reading audit for inspection depth.

''']
    for title, description, group in PAPER_GROUPS:
        parts.append(f'### {title}\n\n{description}\n\n| Year | Paper | Citation key | Recorded inspection |\n| --- | --- | --- | --- |\n')
        for key in group:
            record = records[key]
            url = record['url']
            if urlsplit(url).scheme != 'https':
                raise ValueError(f'Expected an HTTPS bibliographic identity for {key}')
            title = record['title'].replace('|', '\\|').replace('[', '\\[').replace(']', '\\]').replace('\n', ' ')
            if key in appraisals:
                reading = ('Selected full text (expanded review)' if appraisals[key]['inspection'] == 'selected-full-text'
                           else 'Record-limited; original full text unavailable')
            else:
                reading = record['reading_level'].replace('|', '\\|')
            parts.append(f"| {record['year']} | [{title}]({url}) | `{key}` | {reading} |\n")
        parts.append('\n[Back to the paper index](#complete-paper-index) · [Resource library](README.md)\n\n')
    parts.append('''## Maintaining the index

This page is generated by `scripts/resource_library.py` through the shared
graphics/documentation generator. Update the reviewed bibliography using its
[verification procedure](../literature_review/README.md#verification-and-updating),
then update the topic key groups if papers were added or removed. Regenerate with
`python3 scripts/generate_readme_graphics.py`; `--check` detects stale output and
rejects missing, duplicate or unclassified paper keys.
''')
    return ''.join(parts)
