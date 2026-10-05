# Literature discovery and screening record

## Expansion record — 5 October 2026

The expanded review retains the original 50-work corpus and discovery cutoff.
It applies a common extraction instrument to every work, rather than claiming
an exhaustive new search or inventing retrospective database-hit counts.
All 50 works have a structured appraisal; 46 have selected full-text passages
inspected and four remain record-limited. No external method was replicated.

Retrieval used the existing arXiv/supplemental identities, DOI-matched OpenAlex
location records, author copies and institutional repositories. OpenAlex was a
locator, not the authority for experimental findings. The exact endpoints,
timestamps, access failures and hashes are recorded in
[fulltext_retrieval.json](fulltext_retrieval.json). Selected primary passages and
critical interpretations are identified in [FULLTEXT_AUDIT.md](FULLTEXT_AUDIT.md)
and [paper_appraisals.json](paper_appraisals.json).

Additional targeted locator checks included an arXiv title search for
[SocNavBench](https://arxiv.org/search/?query=SocNavBench&searchtype=title&abstracts=show&order=-announced_date_first&size=50)
(one matching work), the [Parkour arXiv record](https://arxiv.org/abs/2306.14874),
the [Caltech cooperation record](https://authors.library.caltech.edu/47456/),
and the authors' [TEB repository](https://github.com/rst-tu-dortmund/teb_local_planner).
These were full-text location checks within the corpus, not a new exhaustive
discovery sweep. The Caltech record was metadata-only; software descriptions
are labelled context rather than substitutes for original-paper experiments.

The four unavailable originals are MuJoCo (2012), the 2013 sparse trajectory
optimization paper, velocity obstacles (1998) and the 2013 dense-crowd cooperation
paper. The earlier HAL access limitation for the biped paper was resolved.
Primary PDFs stay in the ignored build cache; the published package contains
original appraisals and links, not redistributed publisher papers.

The systematic analysis is bounded by purposive initial selection, uneven copy
versions and a single AI-assisted extraction/synthesis workflow. There was no
preregistered protocol or independent dual coding. A submission must extend
nearest-prior-work screening for uncertainty-aware MPC, delayed/sampled-data
CBFs, learned reachable sets and uncertainty calibration before making a
first-of-kind claim. The PDF explicitly distinguishes project deficits,
under-tested interfaces in this corpus and proposed contributions.

## Original discovery record — 22 September 2026

Search date/cutoff: **2026-09-22**. Scope: Geoff-Sim's G1/MuJoCo/ROS 2
human-aware navigation, sensing, locomotion response, and experimental validity.

## Procedure

1. Read the current repository README, onboard status, route-planning status,
   benchmark experiment contract, and shared motion-envelope implementation.
2. Search Crossref by title/bibliographic phrase for foundations, surveys,
   relevant method families, perception, control, and benchmarks.
3. Search arXiv by topic and title for recent humanoid navigation and source
   abstracts missing from registry records.
4. Follow references encountered in close competitors, including the 2016 biped
   crowd paper, Omni-Perception, and wheeled-legged autonomous navigation.
5. Select exact DOI/arXiv records, reject unrelated results and duplicate versions,
   then independently fetch each selected primary identity record.
6. Inspect abstracts where accessible; analyze selected full-text methods and
   evaluation sections of ASAP, Safe and Comfortable Locomotion, FLUX, and
   OmniNavBench. Interpret those claims in the project's actual evidence context.

This is a **purposive scoping search**, not an exhaustive database export. No
PRISMA-style total-hit count or independent dual-screening process is claimed.
The reproducible retained set is `sources.json`; failed/ranked search candidates
are not treated as papers in the final bibliography.

## Query families and reproducible endpoints

Crossref discovery endpoint:

```text
https://api.crossref.org/works?query.title=<URL-encoded title>&rows=2
https://api.crossref.org/works?query.bibliographic=<URL-encoded phrase>&rows=2
```

The exact title strings for retained works are in `sources.json`. Direct identity
checks use `https://api.crossref.org/works/<DOI>` and
`https://arxiv.org/abs/<identifier>`; `verify_sources.py --refresh --export`
replays those checks, not the discovery search.

| Family | Representative discovery phrases |
| --- | --- |
| Social-navigation framing | `A survey on socially aware robot navigation Taxonomy and future challenges`; `Human-aware robot navigation A survey`; `Principles and Guidelines for Evaluating Social Robot Navigation Algorithms` |
| Classical avoidance | `The dynamic window approach to collision avoidance`; `Velocity obstacles for real-time navigation`; `Reciprocal n-body collision avoidance`; `Efficient trajectory optimization using a sparse model` |
| Interaction and learning | `Robot navigation in dense human crowds the case for cooperation`; exact CADRL, SARL, RGL, and DS-RNN titles |
| Prediction/perception | exact Social LSTM, Social GAN, Trajectron++, DROW, DR-SPAAM, JRDB, SCAND, FAST-LIO2, ORB-SLAM3 titles |
| Legged/humanoid control | `Learning to Walk in Minutes`; `RMA Rapid Motor Adaptation`; `Learning robust perceptive locomotion`; `Expressive Whole-Body Control`; `ASAP Aligning Simulation and Real-World Physics` |
| Direct overlap and recency | arXiv all-fields `social humanoid navigation`, sorted by newest announcement; references in the safe-humanoid paper |
| Benchmarking | exact SocNavBench, SocialGym, Habitat 3.0, MetaUrban titles; recent FLUX/DynBench, OmniNavBench, DPed-VLN results |

The topic-search URL used for the recent sweep was:

<https://arxiv.org/search/?query=social+humanoid+navigation&searchtype=all&abstracts=show&order=-announced_date_first&size=50>

Additional arXiv searches used `searchtype=title` with exact titles. OpenAlex's
works search located the HAL record for the older biped paper; the direct DOI
record remained the identity authority.

## Screening and corrections

- Broad search ranking was unreliable for some foundational titles. DWA, VO,
  social force, Social LSTM, and ORB-SLAM3 were resolved through direct DOI records.
- An approximate VO search phrase was replaced by the verified title
  *Motion Planning in Dynamic Environments Using Velocity Obstacles*.
- The evaluation-guidelines paper was the second result in one search. The
  similarly relevant but different “Overlapping Social Navigation Principles”
  paper was not substituted for it.
- FAST-LIO2 returned an SSRN copy and an IEEE article. Only the IEEE DOI was retained.
- A search for AMP returned a different paper, AIP. That mismatch was rejected;
  it was not cited as AMP.
- Searches for a humanoid-transformer title and a high-speed quadrupedal
  navigation phrase did not provide reliable matching records in this pass.
  They were not added speculatively.
- Two initial candidate arXiv identifiers resolved to unrelated physics papers.
  They were rejected before inclusion. The retained set was subsequently checked
  against expected titles, rather than trusting an identifier recalled from memory.
- The “Social robot navigation: a review and benchmarking…” record is dated
  December **2025**. Its internal citation key `survey2023` is merely a stable
  key from discovery; the bibliography correctly uses the verified year.
- Online rate limits interrupted some discovery queries. Direct record checks
  subsequently succeeded for all 50 retained works.

## Detailed content anchors

| Work/version | Material inspected | Main use |
| --- | --- | --- |
| ASAP, `2502.01143v3` | Sections II–IV, open-loop/closed-loop tables | Distinguish residual sim-to-real alignment from navigation-level response calibration |
| Safe humanoid locomotion, `2508.07611v1` | Sections III–V, Tables I–III, physical-test description | Direct G1 overlap; training-cost versus invariant-set guarantee; threshold and comfort evidence |
| FLUX, `2603.12806v1` | Sections IV–VII, Tables II/IV, deployment and metric definitions | Direct G1 overlap; risk-proxy/contact distinction, remote inference, comparable timing boundaries |
| OmniNavBench, `2605.09441v1` | Sections III–IV, task/metric tables, embodiment description | Existing physics-based, multi-embodiment, multi-task benchmark scope |

These are selected sections, not a claim that all supplementary materials,
codebases, datasets, or every appendix have been reviewed.

## Access and remaining coverage limits

- The HAL landing page for the 2016 biped paper returned an access challenge.
  Its existence/title/authors/year/venue are DOI-verified, but no detailed
  experimental claim was extracted from its full text.
- Some Crossref records contain no abstract. Additional arXiv abstracts were
  inspected where title matching found the corresponding work; see the audit.
- The September 2026 DPed-VLN item is a very recent abstract-level inclusion.
  It is not used to support claims about low-level humanoid control.
- arXiv versions do not independently establish peer-review status. The review
  does not guess a conference venue from a project page or model name.
- Future publication work should deepen coverage of robust/chance-constrained
  MPC, delayed sampled-data barrier functions, uncertainty calibration,
  foothold-aware crowd planning, and learned perception-based navigation.
  In particular, inspect the full biped and Omni-Perception papers and follow
  citations forward from the 2024–2026 surveys before making a first-of-kind claim.
- No external benchmark was rerun, and the project's development trials were
  not re-analyzed as a frozen statistical dataset.
