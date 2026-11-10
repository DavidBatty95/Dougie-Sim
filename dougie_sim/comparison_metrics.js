/* Descriptive within-pair metrics. Never compare unequal endpoints as equal work. */
(function(root) {
  'use strict';
  function firstCrossing(rows, threshold) {
    return rows.find(row => Number.isFinite(row.explored_percent) && row.explored_percent >= threshold) || null;
  }
  function atTime(rows, time) {
    if (!rows.length || time < rows[0].time_s || time > rows.at(-1).time_s) return null;
    const right = rows.findIndex(row => row.time_s >= time);
    if (right === 0 || rows[right].time_s === time) return rows[right];
    const a = rows[right-1], b = rows[right], weight = (time-a.time_s)/(b.time_s-a.time_s);
    const result = {time_s: time};
    for (const key of ['true_dose_usv', 'explored_percent', 'distance_m']) result[key] = a[key]+weight*(b[key]-a[key]);
    return result;
  }
  function reduction(blind, aware) {
    return blind > 0 ? 100*(1-aware/blind) : null;
  }
  function atCoverage(pair, threshold) {
    const blind = firstCrossing(pair.blind.rows, threshold), aware = firstCrossing(pair.aware.rows, threshold);
    return blind && aware ? {blind, aware, threshold, reduction: reduction(blind.true_dose_usv, aware.true_dose_usv)} : null;
  }
  function atCommonTime(pair) {
    if (!pair.blind.rows.length || !pair.aware.rows.length) return null;
    const time = Math.min(pair.blind.rows.at(-1).time_s, pair.aware.rows.at(-1).time_s);
    const blind = atTime(pair.blind.rows, time), aware = atTime(pair.aware.rows, time);
    return blind && aware ? {time, blind, aware, reduction: reduction(blind.true_dose_usv, aware.true_dose_usv)} : null;
  }
  const api = {firstCrossing, atTime, reduction, atCoverage, atCommonTime};
  root.DougieMetrics = api;
  if (typeof module !== 'undefined') module.exports = api;
})(globalThis);
