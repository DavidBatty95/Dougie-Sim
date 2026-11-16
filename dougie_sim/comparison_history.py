"""Time-aligned session means, retaining incomplete and interrupted attempts."""
from bisect import bisect_left
import math
from statistics import mean, stdev

MODES = ('blind', 'aware')
METRICS = ('true_dose_usv', 'distance_m')


def usable_rows(rows):
    result = []
    for row in rows:
        if not all(isinstance(row.get(k), (int, float)) and math.isfinite(row[k])
                   and row[k] >= 0 for k in ('time_s', *METRICS)):
            continue
        if result and row['time_s'] < result[-1]['time_s']:
            continue
        if result and row['time_s'] == result[-1]['time_s']:
            result[-1] = row
        else:
            result.append(row)
    return result


def statistics(values):
    return dict(mean=mean(values) if values else None,
                sd=stdev(values) if len(values) > 1 else None, n=len(values))


def session_averages(pairs):
    """Only pairs with both observations at t contribute; never pad endpoints."""
    prepared = [{mode: usable_rows(pair[mode]) for mode in MODES} for pair in pairs]
    eligible = [p for p in prepared if all(p.values())]
    maximum = max((min(p[m][-1]['time_s'] for m in MODES) for p in eligible), default=0.)
    step = max(.5, maximum/400.)
    times = {i*step for i in range(int(maximum/step)+1)}
    # Retain short runs and the exact ends of the paired observation intervals.
    for pair in eligible:
        start = max(pair[m][0]['time_s'] for m in MODES)
        end = min(pair[m][-1]['time_s'] for m in MODES)
        if start <= end:
            times.update((start, end))
    indices = [{m: [r['time_s'] for r in p[m]] for m in MODES} for p in eligible]
    curves = []
    for stamp in sorted(times):
        samples = []
        for pair, clocks in zip(eligible, indices):
            values = {}
            for mode in MODES:
                rows = pair[mode]
                if not rows[0]['time_s'] <= stamp <= rows[-1]['time_s']:
                    break
                i = bisect_left(clocks[mode], stamp)
                if rows[i]['time_s'] == stamp:
                    values[mode] = {k: rows[i][k] for k in METRICS}
                else:
                    a, b = rows[i-1], rows[i]
                    weight = (stamp-a['time_s'])/(b['time_s']-a['time_s'])
                    values[mode] = {k: a[k]+weight*(b[k]-a[k]) for k in METRICS}
            if len(values) == 2:
                samples.append(values)
        if eligible:
            curves.append(dict(time_s=stamp, paired_n=len(samples), **{
                mode: {k: statistics([s[mode][k] for s in samples]) for k in METRICS} for mode in MODES}))
    return dict(ended_pairs=len(pairs), paired_observations=len(eligible), curves=curves,
        endpoints={m: {k: statistics([p[m][-1][k] for p in prepared if p[m]]) for k in METRICS} for m in MODES})
