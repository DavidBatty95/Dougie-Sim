"""Independent physical witnesses. Never expose this object to autonomy."""
import math
import numpy as np


def round_trip_failures(summary):
    """Evaluate an archived upstairs/downstairs/home outcome, failing closed."""
    reasons = []
    if summary.get('world') not in ('facility', 'mezzanine'):
        reasons.append('unsupported_round_trip_world')
    if summary.get('error') is not None:
        reasons.append('simulation_error')
    if summary.get('end_state') != 'returned' or summary.get('mission_completed') is not True:
        reasons.append('not_returned')
    if summary.get('clean_return') is not True:
        reasons.append('not_clean')

    def number(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)

    dose, budget = summary.get('true_dose_usv'), summary.get('budget_usv')
    if (not number(dose) or not number(budget) or not 0 <= dose <= budget or budget <= 0
            or summary.get('actual_budget_exceeded') is not False):
        reasons.append('dose_budget_failure')
    for key, limit in (('nonfoot_impulse_ns', .1), ('nonfoot_peak_force_n', 25.)):
        value = summary.get(key)
        if not number(value) or not 0 <= value <= limit:
            reasons.append(key+'_failure')
    simulation = summary.get('simulation', {})
    upright = simulation.get('upright')
    if not number(upright) or upright <= .8:
        reasons.append('not_upright')
    witness = summary.get('physical_witness', {})
    upper, lower = witness.get('upper_landing_witness_s'), witness.get('lower_return_witness_s')
    end = simulation.get('simulation_time_s')
    if (witness.get('vertical_round_trip_witness') is not True or
            not all(number(v) for v in (upper, lower, end)) or not 0 <= upper < lower <= end):
        reasons.append('missing_ordered_vertical_witness')
    return reasons


class MissionWitness:
    def __init__(self, sim, world):
        self.feet = [sim.model.body(leg+'_foot').id for leg in ('FR', 'FL', 'RR', 'RL')]
        self.upper = None
        self.upper_regions = []
        if world == 'mezzanine':
            index = sim.model.geom('mezzanine_slab').id
            centre = sim.data.geom_xpos[index].copy()
            half = sim.model.geom_size[index].copy()
            self.upper = centre, half
            self.upper_regions.append(self.upper)
        elif world == 'facility':
            for index in range(sim.model.ngeom):
                if sim.model.geom(index).name.startswith('upper_floor_'):
                    self.upper_regions.append((sim.data.geom_xpos[index].copy(), sim.model.geom_size[index].copy()))
            self.upper = self.upper_regions[0]
        self.upper_dwell = 0.
        self.lower_dwell = 0.
        self.upper_stamp = None
        self.lower_stamp = None
        self.motion = []

    def tick(self, sim):
        feet = sim.data.xpos[self.feet]
        dt = sim.model.opt.timestep
        if self.upper is not None:
            on_upper = any(np.all(np.abs(feet[:, :2]-centre[:2]) < half[:2]-.05) and
                np.all(np.abs(feet[:, 2]-.022-(centre[2]+half[2])) < .12)
                for centre, half in self.upper_regions)
            self.upper_dwell = self.upper_dwell+dt if on_upper else 0.
            if self.upper_stamp is None and self.upper_dwell >= .2:
                self.upper_stamp = float(sim.data.time)
            lower = self.upper_stamp is not None and np.all(np.abs(feet[:, 2]-.022) < .12)
            self.lower_dwell = self.lower_dwell+dt if lower else 0.
            if self.lower_stamp is None and self.lower_dwell >= .2:
                self.lower_stamp = float(sim.data.time)
        if sim.counter % 10 == 0:
            self.motion.append(np.r_[sim.data.time, sim.data.qpos, sim.last_command])

    def metrics(self):
        return dict(upper_landing_witness_s=self.upper_stamp, lower_return_witness_s=self.lower_stamp,
                    vertical_round_trip_witness=bool(self.upper_stamp is not None and self.lower_stamp is not None),
                    scope='evaluator-only foot poses, 0.2 s dwell; not exploration completeness')

    def save(self, path):
        np.savez_compressed(path, rows=np.asarray(self.motion),
                            columns=['time_s']+[f'qpos_{i}' for i in range(19)]+['vx', 'vy', 'yaw_rate'])
