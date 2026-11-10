const assert = require('node:assert/strict');
const metrics = require('../dougie_sim/comparison_metrics.js');
const row = (time_s, explored_percent, true_dose_usv) => ({time_s, explored_percent, true_dose_usv, distance_m:time_s});
const pair = {blind:{rows:[row(0,0,0),row(10,50,2),row(20,90,10)]},
              aware:{rows:[row(0,0,0),row(5,50,1),row(15,90,6)]}};
const matched = metrics.atCoverage(pair,90);
assert.equal(matched.reduction,40);
assert.equal(matched.blind.time_s,20);
assert.equal(matched.aware.time_s,15);
assert.equal(metrics.atCoverage(pair,100),null);
assert.equal(metrics.atCoverage({blind:{rows:[]},aware:pair.aware},50),null);
const common = metrics.atCommonTime(pair);
assert.equal(common.time,15);
assert.equal(common.blind.true_dose_usv,6);
assert.equal(common.aware.true_dose_usv,6);
assert.equal(common.reduction,0); // Unequal endpoints must not be substituted.
assert.equal(metrics.reduction(0,0),null);
assert.equal(metrics.reduction(2,3),-50); // Never force the aware result positive.
assert.equal(metrics.atTime(pair.aware.rows,16),null); // No extrapolation.
assert.equal(metrics.firstCrossing([row(0,0,0),row(1,76,1)],75).explored_percent,76);
console.log('Paired metric checks passed: threshold crossings, common-time interpolation, missing data and adverse outcomes.');
