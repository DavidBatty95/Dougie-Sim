# Exposure-policy pilot results

Each row compares awareness with the same-revision blind ablation. Revisions and seed sets differ; before/after totals are not paired causal estimates.

| Pilot | Policy | Terminal / requested | Censored | Frontier complete blind / aware | ≥50% eligible pairs | Blind mean dose | Aware mean dose | Paired aware−blind 95% mean interval |
|---|---|---:|---:|---|---:|---:|---:|---|
| facility_policy_v3_validation | standard | 4/4 | 4 | 0 / 0 | 4 | 0.115888 | 0.275998 | [0.008937, 0.409004] |
| facility_policy_v4_validation | responsive | 4/4 | 4 | 0 / 0 | 4 | 0.138199 | 0.492387 | [0.085736, 0.621960] |
| facility_pathwise_validation | INVALID pilot: Experimental route-certificate handoff could override paths cleared or replaced by recovery. Raw observations are retained, but this pilot is not an accepted comparison. | — | — | — | — | — | — | — |
| facility_pathwise_safe_validation | pathwise | 4/4 | 4 | 0 / 0 | 4 | 0.138199 | 0.236822 | [-0.037986, 0.367309] |
| reactor_pathwise_safe_validation | pathwise | 2/2 | 2 | 0 / 0 | 2 | 0.152986 | 0.152986 | [0.000000, 0.000000] |
| facility_pathwise_evidence_validation | pathwise | 4/4 | 4 | 0 / 0 | 4 | 0.138199 | 0.147520 | [0.000000, 0.018642] |

Dose units are µSv. Negative paired differences favour awareness. Intervals are percentile bootstrap mean intervals over eligible paired seeds, not safety bounds. Actual milestone overshoots and all individual crossings are retained in the JSON.

All trials here are development/pilot experiments on declared fixed layouts. External wall guards are censoring, not successful frontier completion. Lower dose after less progress or earlier termination is not evidence of equal-work improvement. Missing higher milestones and failed/invalid revisions remain visible.
