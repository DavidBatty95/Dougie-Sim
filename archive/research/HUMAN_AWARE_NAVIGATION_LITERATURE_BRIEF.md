# Human-Aware Humanoid Navigation Literature Brief

## Literature Review Prompt

Use the following prompt with an AI system that has access to scholarly search
engines, publisher pages, Crossref, OpenAlex, Semantic Scholar, Scopus, Web of
Science, IEEE Xplore, ACM Digital Library, SpringerLink, ScienceDirect, arXiv,
and Google Scholar where available.

```text
Conduct an in-depth, reproducible, and source-verified literature review for a
research paper on uncertainty-aware human-aware navigation for a Unitree G1
humanoid robot.

Research objective:
Evaluate a time-aligned, risk-bounded command safety filter that combines
calibrated pedestrian trajectory uncertainty with a measured humanoid
command-response envelope, including command latency, acceleration,
deceleration, stopping distance, and post-stop drift. The proposed method will
be evaluated in ROS 2 and MuJoCo, with oracle and perception-only pedestrian
tracking and limited controlled physical G1 validation if available.

Central research question:
Does accounting for both pedestrian prediction uncertainty and measured
humanoid response reduce collisions and critical encounters compared with
fixed-radius reactive stopping and deterministic predictive filtering, without
an unacceptable reduction in navigation success or efficiency?

Perform the review using the following requirements:

1. Search peer-reviewed journal and conference literature as well as clearly
   identified preprints. Prioritize IEEE Xplore, ACM Digital Library, Springer,
   Elsevier, Sage, Wiley, Crossref, OpenAlex, Semantic Scholar, and publisher
   pages. Search backward and forward citations for the most relevant works.
2. Cover foundational literature and recent work through the date on which the
   search is performed. State the exact final search date.
3. Search at minimum these themes:
   - social and human-aware robot navigation;
   - socially aware navigation for humanoid and legged robots;
   - proxemics, passing conventions, yielding, and legibility;
   - dynamic obstacle avoidance and velocity obstacles;
   - model predictive control and control barrier functions around people;
   - uncertainty-aware and chance-constrained motion planning;
   - pedestrian detection, tracking, trajectory prediction, and uncertainty
     calibration;
   - navigation under occlusion, latency, and stale perception;
   - dynamics-aware safety filters and command governors;
   - social-navigation benchmarks, metrics, datasets, and simulation platforms;
   - simulation-to-real validation for humanoid social navigation.
4. Verify every bibliographic record against at least one authoritative source,
   preferably a DOI resolver, publisher page, or official proceedings record.
   For preprints, verify the arXiv identifier and version. Do not invent or infer
   missing citation fields.
5. Open and inspect the paper itself wherever access permits. Distinguish claims
   verified from the full text from claims based only on an abstract or metadata.
6. For every included work, record title, authors, year, venue, publication
   type, DOI or stable URL, robot/platform, sensing assumptions, prediction
   model, uncertainty treatment, robot dynamics treatment, planning or safety
   method, experimental setting, baselines, metrics, number of trials or seeds,
   hardware validation, main result, limitations, and relevance to the proposed
   work.
7. Mark each source as peer reviewed, accepted/in press, thesis, technical
   report, standard, dataset, software artifact, or unreviewed preprint. Never
   present a preprint as peer reviewed unless publication is independently
   confirmed.
8. Do not use a secondary source to support a claim when the primary paper is
   available. Secondary surveys may organize the field but should not replace
   verification of central technical claims.
9. Exclude papers that only mention social navigation without describing a
   relevant algorithm, evaluation method, dataset, or empirical result. Record
   exclusion reasons for papers that initially appear highly relevant.
10. Explicitly identify the closest prior methods to the proposed contribution.
    Compare their mathematical assumptions, uncertainty representation, robot
    response model, safety claim, command-selection mechanism, and evaluation.
11. Determine whether combining calibrated pedestrian uncertainty and an
    empirically measured humanoid command-response envelope in a time-aligned
    command filter is actually novel. If not, identify the prior work and state
    how the proposed contribution must be narrowed or changed.
12. Separate evidence from interpretation. Label unsupported hypotheses and
    unresolved questions. Report contradictory findings rather than selecting
    only evidence favorable to the proposed method.
13. Do not fabricate quotations, page numbers, equations, sample sizes, DOI
    values, or experimental outcomes. Write "not confirmed" when a field cannot
    be verified.
14. Provide a reproducible search log containing database, exact query, filters,
    search date, result count, records screened, records included, and duplicate
    handling. Use a PRISMA-style flow summary where practical.
15. Produce the following outputs:
    A. A concise executive summary of the field and the strongest novelty risk.
    B. A taxonomy of methods and evaluation practices.
    C. A verified evidence table containing all included sources.
    D. A closest-work comparison table focused on the proposed method.
    E. A critical synthesis organized by research question, not one paragraph
       per paper.
    F. A list of established metrics, benchmark protocols, and statistical
       practices suitable for the planned experiments.
    G. Research gaps supported by the reviewed evidence.
    H. A proposed related-work section written in cautious academic language.
    I. A BibTeX file containing only verified entries.
    J. A verification appendix mapping every material literature claim to its
       supporting source and indicating full-text or abstract-only verification.

Citation rules for the resulting paper text:
- Use only sources present in the verified evidence table.
- Attach citations directly to the claims they support.
- Do not claim that no prior work exists; state instead that no matching work
  was identified within the documented search scope.
- Do not describe the proposed method as safe, certified, optimal, novel, or
  state of the art unless the review and experiments support that exact term.
- Keep social comfort claims separate from geometric collision avoidance unless
  human-subject evidence is available.

Start by returning the search protocol and candidate keyword matrix. Do not
write the final narrative until source verification and evidence extraction are
complete.
```

## Review Purpose

The review must establish:

1. What has already been done in human-aware and social robot navigation.
2. Which methods account for pedestrian prediction uncertainty.
3. Which methods account for delayed or imperfect robot command response.
4. Whether these factors have been studied together for humanoid navigation.
5. Which baselines and metrics are expected for a credible evaluation.
6. What claim this project can make without overstating novelty or safety.

The review should support the research plan in
`docs/HUMAN_AWARE_NAVIGATION_RESEARCH_PLAN.md`.

## Proposed Review Questions

Use these questions to structure searches and synthesis:

- **RQ1:** How do social-navigation systems model human motion and interaction?
- **RQ2:** How is uncertainty represented, calibrated, and used by planners or
  safety layers?
- **RQ3:** How do existing methods account for robot dynamics, command delay,
  braking distance, tracking error, or locomotion constraints?
- **RQ4:** What evidence supports safety, efficiency, comfort, legibility, and
  social compliance claims?
- **RQ5:** Which methods have been evaluated on humanoid or legged platforms?
- **RQ6:** Which benchmarks, scenarios, metrics, and statistical protocols are
  established or commonly used?
- **RQ7:** What is the closest prior work to a time-aligned safety filter that
  combines human uncertainty with measured humanoid response?
- **RQ8:** What unresolved gap can this project address within a three-month
  simulation-led study?

## Topic Taxonomy

Classify each included work under one or more categories.

### Social Navigation

- Proxemic cost maps.
- Social-force models.
- Passing and yielding conventions.
- Group and interaction-space models.
- Legibility and predictability.
- Learning from demonstrations.
- Reinforcement-learning approaches.
- Human preference or comfort studies.

### Dynamic Collision Avoidance

- Velocity obstacles and reciprocal velocity obstacles.
- Dynamic window and trajectory rollout methods.
- Model predictive control.
- Control barrier functions.
- Reachability and invariant-set methods.
- Command governors and safety shields.
- Reactive stopping and emergency supervision.

### Human Motion Prediction

- Constant-velocity and Kalman models.
- Gaussian-process prediction.
- Interaction-aware prediction.
- Multimodal learned prediction.
- Occupancy and reachable-set prediction.
- Occlusion-aware prediction.
- Uncertainty calibration and conformal methods.

### Robot Execution Models

- Holonomic and differential-drive assumptions.
- Nonholonomic dynamics.
- Actuation delay and control latency.
- Acceleration and braking constraints.
- Model uncertainty and tracking-error bounds.
- Humanoid gait and balance constraints.
- Legged-robot safe navigation.

### Evaluation

- Simulation-only studies.
- Physical-robot studies.
- Human-subject studies.
- Public datasets and benchmarks.
- Oracle versus perception-based evaluation.
- Safety, task, social, smoothness, and computational metrics.

## Search Sources

Record access dates and exact queries for every source searched.

### Primary Scholarly Sources

- IEEE Xplore.
- ACM Digital Library.
- SpringerLink.
- ScienceDirect.
- Sage Journals.
- Wiley Online Library.
- Scopus or Web of Science if institutionally available.
- Google Scholar for discovery and citation chaining.

### Metadata And Verification Sources

- Crossref.
- OpenAlex.
- Semantic Scholar.
- DBLP for computing conference metadata.
- DOI resolver and official publisher pages.
- Official conference proceedings.
- arXiv for clearly marked preprints and accepted-author manuscripts.

### Related Artifacts

- Papers With Code where relevant.
- GitHub repositories linked by the authors.
- Dataset and benchmark project pages.
- ROS and Nav2 documentation for implementation context, not scholarly novelty.

## Candidate Search Queries

Adapt syntax to each database. Preserve the exact executed form in the search
log.

### Core Social Navigation

```text
("social navigation" OR "socially aware navigation" OR "human-aware navigation")
AND (robot OR mobile robot OR humanoid)
```

```text
("human-aware motion planning" OR "socially compliant navigation")
AND (pedestrian OR human OR crowd)
```

### Uncertainty And Safety

```text
("social navigation" OR "human-aware navigation")
AND (uncertainty OR probabilistic OR "chance constrained" OR covariance)
```

```text
(pedestrian OR human)
AND ("trajectory prediction" OR "motion prediction")
AND (uncertainty OR calibration OR covariance OR conformal)
AND (robot navigation OR motion planning)
```

```text
("chance-constrained" OR probabilistic OR risk-aware)
AND (robot navigation OR motion planning)
AND (pedestrian OR dynamic obstacle)
```

```text
("control barrier function" OR "safety filter" OR "safety shield")
AND (pedestrian OR human OR dynamic obstacle)
AND (navigation OR mobile robot)
```

### Robot Response And Latency

```text
("social navigation" OR pedestrian avoidance)
AND (latency OR delay OR braking OR "stopping distance" OR dynamics)
AND robot
```

```text
("command governor" OR "reference governor" OR "safety filter")
AND (delay OR "tracking error" OR dynamics OR braking)
AND robot
```

```text
("trajectory tracking error" OR "response envelope" OR reachability)
AND (dynamic obstacle OR pedestrian)
AND robot navigation
```

### Humanoid And Legged Navigation

```text
(humanoid OR "legged robot" OR quadruped)
AND ("social navigation" OR "human-aware navigation" OR pedestrian avoidance)
```

```text
(humanoid OR "biped robot")
AND (dynamic obstacle avoidance OR pedestrian navigation)
AND (prediction OR uncertainty OR safety)
```

```text
(Unitree OR "Unitree G1")
AND (navigation OR obstacle avoidance OR human-aware OR social)
```

### Benchmarks And Metrics

```text
("social navigation benchmark" OR "human-aware navigation benchmark")
AND (metrics OR evaluation OR dataset OR simulation)
```

```text
(robot social navigation)
AND (collision OR clearance OR TTC OR proxemics OR comfort)
AND evaluation
```

## Search Process

### Stage 1: Broad Discovery

1. Run the core queries across at least three complementary databases.
2. Export full metadata where licensing permits.
3. Merge records by DOI, title, and author/year.
4. Screen titles and abstracts against the review questions.
5. Identify surveys, benchmark papers, and highly cited foundational works.

### Stage 2: Focused Discovery

1. Search terminology found in the first set of relevant papers.
2. Run separate searches for uncertainty, robot dynamics, humanoids, and metrics.
3. Search references of the closest works.
4. Search papers that cite the closest works.
5. Search author pages for follow-up versions or published versions of preprints.

### Stage 3: Verification

1. Resolve each DOI.
2. Confirm title, authors, venue, year, and publication status.
3. Inspect the full text when accessible.
4. Record the exact location supporting each extracted technical claim.
5. Mark abstract-only evidence clearly.
6. Check whether software and datasets are still available.

### Stage 4: Synthesis

1. Group evidence by research question and method family.
2. Compare assumptions before comparing numerical results.
3. Separate collision avoidance from social acceptability.
4. Identify evaluation weaknesses and common confounds.
5. Build the closest-work table before drafting novelty claims.

## Inclusion Criteria

Include work that satisfies at least one of these conditions:

- Proposes or evaluates human-aware or socially aware robot navigation.
- Uses pedestrian motion prediction in robot motion planning or safety control.
- Treats uncertainty in dynamic-obstacle navigation.
- Accounts for robot response, delay, braking, dynamics, or tracking error in a
  navigation safety method.
- Evaluates social navigation on humanoid or legged robots.
- Defines a relevant benchmark, dataset, metric, or evaluation protocol.
- Provides a survey that materially supports taxonomy or search expansion.

Prefer peer-reviewed work. Include preprints when highly relevant, but label
their status and search for a published version.

## Exclusion Criteria

Exclude or separately classify work that:

- Mentions social navigation only as motivation.
- Addresses static obstacle avoidance without relevant dynamic interaction.
- Predicts pedestrian trajectories but does not connect them to robot planning,
  unless needed to justify the chosen prediction or calibration method.
- Studies autonomous driving without transferable planning or uncertainty ideas.
- Provides no verifiable primary source.
- Duplicates a later published version of the same work.
- Makes only conceptual claims without an algorithm, benchmark, dataset, or
  empirical evaluation relevant to the review questions.

Record a concise exclusion reason for every full-text exclusion.

## Search Log Template

| Search ID | Database | Exact query | Filters | Search date | Results | Screened | Included | Notes |
| --- | --- | --- | --- | --- | ---: | ---: | ---: | --- |
| S01 |  |  |  |  |  |  |  |  |

## Evidence Extraction Template

Use one row per distinct published work. Keep a separate notes file if cells
become too large.

| Field | Extracted evidence |
| --- | --- |
| Citation key |  |
| Verified title |  |
| Authors |  |
| Year and venue |  |
| Publication status | Peer reviewed / preprint / other |
| DOI or stable URL |  |
| Verification source |  |
| Full text inspected | Yes / No |
| Research problem |  |
| Robot/platform |  |
| Environment | Simulation / hardware / both |
| Human sensing |  |
| Tracking method |  |
| Prediction model |  |
| Uncertainty representation |  |
| Calibration method |  |
| Robot dynamics model |  |
| Delay/braking treatment |  |
| Planning or safety method |  |
| Safety claim and assumptions |  |
| Social behavior modeled |  |
| Baselines |  |
| Scenarios and participants |  |
| Trials, seeds, and duration |  |
| Primary metrics |  |
| Statistical analysis |  |
| Main verified result |  |
| Limitations stated by authors |  |
| Additional limitations |  |
| Code/data availability |  |
| Relevance to this project |  |
| Supporting page/section |  |

## Closest-Work Comparison

For the approximately five to ten most similar methods, complete this table
before deciding the final paper title or contribution statement.

| Work | Time-aligned human prediction | Calibrated uncertainty | Robot response/delay | Explicit body geometry | Risk bound | Least-disruptive command filter | Humanoid evaluation | Perception-only evaluation | Difference from proposed work |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Proposed project | Yes | Planned | Planned | Planned | Planned | Planned | G1 | Planned | Reference row |

Answer these questions for each closest work:

1. Does it represent a human as a point, disk, distribution, or reachable set?
2. Is uncertainty learned, assumed, propagated, or empirically calibrated?
3. Is risk evaluated at matched future times or flattened into occupancy?
4. Does the robot rollout include acceleration, delay, braking, and tracking
   error?
5. Does the method modify a command, generate a trajectory, or only stop?
6. Is safety formal, probabilistic, empirical, or heuristic?
7. Are claims tested with oracle state, simulated sensing, or real perception?
8. Is the platform a holonomic robot, differential-drive robot, vehicle, legged
   robot, or humanoid?
9. What scenarios and baselines make its evidence persuasive or weak?
10. Which exact element remains different in this project?

## Metrics Review

Extract definitions rather than metric names alone. In particular, determine how
the literature defines and aggregates:

- Collision and contact.
- Near collision or near miss.
- Center distance and body clearance.
- Time to collision.
- Time and distance at closest approach.
- Personal, social, and public proxemic zones.
- Goal success, timeout, and deadlock.
- Path efficiency and success weighted by path length.
- Navigation delay caused by interaction.
- Yielding, passing side, front crossing, and interaction order.
- Velocity, acceleration, jerk, and stop duration.
- Human comfort, naturalness, predictability, and perceived safety.
- Intervention frequency and false or unnecessary interventions.
- Prediction error and uncertainty calibration.
- Compute latency, message age, and deadline misses.

For each metric, record units, thresholds, sampling assumptions, whether it is
per-person or per-trial, and whether it has been validated against human
judgment. Do not use comfort terminology for geometric clearance alone.

## Expected Literature Gaps To Test

These are hypotheses for the review, not established conclusions:

- Social-navigation methods may assume ideal instantaneous robot velocity.
- Uncertainty-aware methods may use mobile-base or vehicle dynamics rather than
  measured humanoid locomotion response.
- Humanoid navigation studies may focus on locomotion feasibility without
  calibrated pedestrian uncertainty.
- Social-navigation evaluations may rely on oracle pedestrian states or small
  numbers of hand-selected scenarios.
- Safety improvements may be achieved through excessive stopping without
  reporting task degradation.
- Prediction covariance may be propagated without demonstrating empirical
  coverage.

The review must confirm, reject, or narrow each hypothesis using verified
sources.

## Paper-Ready Synthesis Structure

Write the related-work section by technical issue rather than as a chronological
list of papers.

### 1. Human-Aware And Social Navigation

Explain the transition from geometric obstacle avoidance to proxemics,
interaction-aware behavior, passing, and yielding. Distinguish social convention
from collision safety.

### 2. Predictive Safety Around Pedestrians

Compare velocity-obstacle, MPC, barrier-function, reachability, and command-filter
approaches. Identify their assumptions about human motion and robot execution.

### 3. Prediction Under Uncertainty

Discuss stochastic, multimodal, reachable-set, and calibrated prediction. Focus
on how uncertainty changes planning decisions rather than prediction accuracy
alone.

### 4. Humanoid And Legged Social Navigation

Describe platform-specific constraints, including gait response, balance,
nonholonomic behavior, latency, and stopping performance. Determine how often
these are represented in the planning model.

### 5. Evaluation And Remaining Gap

Compare benchmarks, metrics, oracle/perception separation, statistical strength,
and hardware evidence. End with the narrow gap supported by the review.

## Claim Verification Checklist

Before adding a literature claim to the paper, confirm:

- The cited source exists and metadata is correct.
- The cited version has the stated publication status.
- The source actually supports the sentence.
- Algorithm details were verified from full text where possible.
- The result is not taken from a different experimental condition.
- Platform and sensing assumptions are stated accurately.
- A survey is not being cited in place of the primary contribution.
- The claim does not turn absence from the search into proof of nonexistence.
- The BibTeX entry resolves to the same work.
- Any quotation and page number were checked directly.

## Review Deliverables

Create the following files when conducting the review:

```text
literature_review/
  search_log.csv
  screening_log.csv
  included_studies.csv
  evidence_notes.md
  closest_work_comparison.csv
  metrics_catalog.csv
  related_work_draft.md
  references_verified.bib
  claim_source_map.csv
  prisma_summary.md
```

Do not place copyrighted paper PDFs in the repository unless their licenses
explicitly permit redistribution. Store stable links, identifiers, and personal
access notes instead.

## Completion Criteria

The briefing phase is complete when:

- Searches cover all review questions across multiple databases.
- Search and screening decisions are reproducible.
- Central citations are verified against primary sources.
- The closest-work table contains enough detail to assess novelty.
- Proposed baselines and metrics are justified by literature.
- Contradictory and negative evidence is represented.
- The related-work draft contains no unverified citation or unsupported novelty
  claim.
- The final contribution statement has been revised to match the evidence.
# Archived research material

> Historical literature/research drafting material. Not maintained operational documentation.
> See `docs/INDEX.md` for the current project documentation.
