# Multilevel facility

[Project home](../README.md) · [Campaigns](DOUGIE_CAMPAIGNS.md) ·
[Experimental scheduling and results](DOUGIE_CAMPAIGN_AGENDA.md)

The `facility` world is an experimental two-storey service building with rooms,
doors, equipment, a ground floor and an upper floor separated by **2.4 m**. Two
observed, shallow switchback ramps connect the levels via an intermediate landing.
An industrial stair fixture is also present; the current walking policy is **not
qualified for ordinary industrial stairs**. See [terrain qualification](DOUGIE_TERRAIN_QUALIFICATION.md).

```bash
# Stop an existing Mac dashboard before changing its world.
./dougie-mac compare-stop
DOUGIE_COMPARE_WORLD=facility ./demo

# Experimental campaign agenda, in a fresh session:
DOUGIE_COMPARE_WORLD=facility DOUGIE_COMPARE_CAMPAIGN_STRATEGY=agenda ./demo
```

The browser comparison uses ideal simulator pose by default. A layered map fits
observed support and bounded slopes from range returns; a graph edge requires
compatible support and clearance. Frontiers are derived from observed geometry,
not the simulator's hidden floor reference. The GP learns detector counts and
the evaluator independently scores the true simulated field and physical floor
observations. Current access and return behavior remain research prototypes,
not a sensor-only multilevel or hardware qualification.

An upper floor visible to the sensor is not automatically physically reached.
The evaluator reports separate height-band observation and an upper-landing foot
witness. Completion still requires the planner to declare zero frontiers on three
fresh scans; a return with unreachable or radiation-blocked frontiers is incomplete.
The [campaign agenda report](DOUGIE_CAMPAIGN_AGENDA.md) retains positive,
negative, interrupted and resource-failure outcomes, including the difference
between time-capped screens and uncapped runs.

The original dated facility development record, training notes and historical
figures are accessible in the [Go2 development archive](DOUGIE_HISTORY.md#go2-development-archive).
