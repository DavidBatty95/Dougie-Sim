# Literature review: humanoid social navigation

<!-- geoff-nav:start -->
<p align="center">
  <a href="../../README.md"><img src="../media/navigation/home.svg" width="132" height="44" alt="Home"></a>
  <a href="../../README.md#quick-start"><img src="../media/navigation/start.svg" width="132" height="44" alt="Quick start"></a>
  <a href="../DEMO_GUIDE.md"><img src="../media/navigation/demos.svg" width="132" height="44" alt="Demos"></a>
  <a href="../INDEX.md"><img src="../media/navigation/docs.svg" width="132" height="44" alt="Docs"></a>
  <a href="../REPOSITORY_GUIDE.md"><img src="../media/navigation/folders.svg" width="132" height="44" alt="Folders"></a>
  <br>
  <a href="../PROJECT_STATUS.md"><img src="../media/navigation/progress.svg" width="132" height="44" alt="Progress"></a>
  <a href="../PROJECT_PLAN.md"><img src="../media/navigation/roadmap.svg" width="132" height="44" alt="Roadmap"></a>
  <a href="../RESULTS_REVIEW.md"><img src="../media/navigation/results.svg" width="132" height="44" alt="Results"></a>
  <a href="README.md"><img src="../media/navigation/research-current.svg" width="132" height="44" alt="Research (current page)"></a>
  <a href="../DEVELOPMENT.md"><img src="../media/navigation/develop.svg" width="132" height="44" alt="Develop"></a>
</p>
<!-- geoff-nav:end -->

**Read the PDF:** [`literature_review.pdf`](literature_review.pdf)

**Edit the document:** [`literature_review.tex`](literature_review.tex)

**Browse every reference by topic:** [50-paper archive](../resources/RESEARCH.md).
For code, experiment and upstream links, open the [resource library](../resources/README.md).

This review evaluates **50 verified research works**, with a search cutoff of
**22 September 2026**, and positions the current Geoff-Sim implementation against
classical avoidance, learned crowd navigation, perception/prediction, humanoid
control, recent sensor-driven G1 navigation, and evaluation research.

It includes:

- A critical state-of-the-art synthesis and detailed analysis of close recent competitors.
- Five ranked research gaps and an explicit distinction between existing
  implementation, documented development results, and proposed contributions.
- Falsifiable hypotheses, baseline choices, ablations, metrics, and statistical guidance.
- An annotated evidence map covering all 50 works and a claim audit.
- A machine-readable verified bibliography, reading audit, and search log.

The principal recommendation is **observation-age-aware, response-calibrated
navigation**, evaluated through controlled oracle-to-onboard experiments.
The review acknowledges the project's documented onboard collisions/timeouts
and the existence of prior humanoid-navigation systems.

## Reading paths

- **Project overview:** read the synthesis and ranked research gaps, then compare
  the proposed contributions with the [recorded implementation evidence](../RESULTS_REVIEW.md).
- **Experiment design:** use the hypotheses, baseline choices, ablations and
  statistical guidance alongside the [avoidance methodology](../AVOIDANCE_EXPERIMENTS.md).
- **Source review:** inspect [the reading audit](READING_AUDIT.md) and
  [search log](SEARCH_LOG.md) to understand selection and inspection depth, then
  follow the identities in `sources.json` and the retrieved verification records.
- **Maintenance:** run the offline check below before editing citations; refresh
  identities and rebuild when updating the reviewed snapshot.

## Build the PDF

From the repository root:

```bash
bash docs/literature_review/build.sh
```

The script works from any current directory when invoked by its correct path.
Compilation intermediates go into `docs/literature_review/build/`, which is
locally ignored by Git. The checked PDF is also published beside this README so
the GitHub link works without a local build. It checks bibliography consistency before
compilation and rejects unresolved references, missing glyphs, and overfull boxes.

Supported toolchains, in selection order:

1. **Tectonic** (tested with 0.17.0; automatically runs BibTeX and required passes).
2. **latexmk + LuaLaTeX**.
3. **LuaLaTeX + BibTeX**, with explicit repeated passes.

For Ubuntu/Debian, the TeX Live option can be installed with:

```bash
sudo apt-get install latexmk texlive-luatex texlive-latex-extra texlive-fonts-recommended
```

Alternatively install [Tectonic](https://tectonic-typesetting.github.io/), or
point the script at an existing binary:

```bash
TECTONIC=/path/to/tectonic bash docs/literature_review/build.sh
```

Tectonic's first build needs network access to fetch its TeX bundle; subsequent
builds reuse its cache. The build does **not** re-query paper databases. Python 3
is required; the verification script uses only the Python standard library.
The document uses Unicode-capable fonts, so plain pdfLaTeX is not a supported
fallback.

## Verification and updating

Check the delivered snapshot and all citation keys offline:

```bash
bash docs/literature_review/build.sh --check-only
```

Re-fetch every selected identity record, validate identifiers/titles, retain new
retrieval evidence, and regenerate BibTeX:

```bash
python3 docs/literature_review/verify_sources.py --refresh --export
```

The refresh contacts Crossref and arXiv sequentially with retry/backoff. It fails
on unavailable/mismatched records rather than marking them verified. Review the
resulting diff: records and unversioned arXiv pages can change. For adding papers,
edit `sources.json`, refresh, inspect the new content, update the reading audit
and synthesis, then rebuild. Metadata refresh alone does not update the research
assessment or establish that a new full text has been read.

Project-evidence hashes describe the inspection snapshot. Later controller
iterations and measured outcomes have their own provenance in
[the avoidance datasets](../AVOIDANCE_EXPERIMENTS.md).

## Evidence files

| File | Purpose |
| --- | --- |
| `sources.json` | Selected DOI/arXiv identities and expected titles |
| `verified_sources.json` | Retrieved titles/authors/years/venues, abstracts where available, URLs, timestamps, response hashes |
| `references.bib` | Generated BibTeX for the verified snapshot |
| `READING_AUDIT.md` | Per-paper distinction between identity verification and content inspection |
| `supplemental_reading.json` | Additional arXiv abstract records inspected for DOI-cited papers |
| `SEARCH_LOG.md` | Search families, screening decisions, cutoff, limitations, and unresolved follow-up |
| `project_evidence.json` | Hashes and revision context for inspected project files |

**Verification boundary:** all 50 works have matching primary bibliographic
records. Content inspection varies by paper; four close recent competitors
received selected full-text methods/evaluation analysis. This is a structured
critical scoping review, not an exhaustive systematic review or experimental
replication. The PDF and audit explicitly preserve this distinction.
