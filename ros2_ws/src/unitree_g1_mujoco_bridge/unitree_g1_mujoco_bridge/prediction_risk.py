"""ROS-independent prediction preparation and time-aligned risk queries."""
import math
from dataclasses import dataclass


def _robot_position_at(start_x,start_y,start_yaw,vx,vy,wz,offset,dt):
    x,y,yaw = start_x,start_y,start_yaw
    remaining = max(0.,offset)
    step_size = max(.01,dt)
    while remaining>1e-9:
        step = min(step_size,remaining)
        x += (math.cos(yaw)*vx-math.sin(yaw)*vy)*step
        y += (math.sin(yaw)*vx+math.cos(yaw)*vy)*step
        yaw += wz*step
        remaining -= step
    return x,y


@dataclass(frozen=True)
class PreparedPredictions:
    raw: tuple
    dense: tuple
    integration_dt: float


def prepare_predictions(trajectories,integration_dt):
    """Validate/densify once per message; detach immutable data from its owner."""
    raw = tuple(tuple(tuple(point) for point in trajectory) for trajectory in trajectories)
    dense = []
    for trajectory in raw:
        valid_samples = []
        previous_offset = -math.inf
        for offset,x,y,cov_xx,cov_yy,confidence in trajectory:
            if not all(math.isfinite(value) for value in (offset,x,y,cov_xx,cov_yy,confidence)):
                continue
            if offset<=previous_offset:
                continue
            previous_offset = offset
            valid_samples.append((offset,x,y,cov_xx,cov_yy,confidence))
        dense_samples = []
        for index,right in enumerate(valid_samples):
            if index == 0:
                dense_samples.append(right)
                continue
            left = valid_samples[index-1]
            span = right[0]-left[0]
            steps = max(1,math.ceil(span/max(.01,integration_dt)))
            for step in range(1,steps+1):
                fraction = step/steps
                dense_samples.append(tuple(left[field]+fraction*(right[field]-left[field]) for field in range(6)))
        dense.append(tuple(dense_samples))
    return PreparedPredictions(raw,tuple(dense),integration_dt)


def time_aligned_prediction_risk(trajectories,message_age,robot_pose,command,horizon,
                                 integration_dt,collision_radius,uncertainty_scale,
                                 minimum_confidence,robot_trajectory=None,minimum_time=0.):
    """Return nearest same-time conflict as clearance, time, x, y.

    This preserves the existing sampled predicate. It is not a new swept-body
    or contact-force certificate.
    """
    if message_age<0.:
        return None
    if isinstance(trajectories,PreparedPredictions):
        prepared = trajectories if trajectories.integration_dt == integration_dt else prepare_predictions(trajectories.raw,integration_dt)
    else:
        prepared = prepare_predictions(trajectories,integration_dt)
    best = None
    for dense_samples in prepared.dense:
        for offset,x,y,cov_xx,cov_yy,confidence in dense_samples:
            time_from_now = offset-max(0.,message_age)
            if time_from_now<max(0.,minimum_time) or time_from_now>horizon or confidence<minimum_confidence:
                continue
            if robot_trajectory:
                index = min(len(robot_trajectory)-1,int(time_from_now/max(1e-6,robot_trajectory[-1][3])*(len(robot_trajectory)-1)))
                left = robot_trajectory[index]
                right = robot_trajectory[min(index+1,len(robot_trajectory)-1)]
                fraction = max(0.,min(1.,(time_from_now-left[3])/max(1e-6,right[3]-left[3])))
                robot_x = left[0]+fraction*(right[0]-left[0])
                robot_y = left[1]+fraction*(right[1]-left[1])
            else:
                robot_x,robot_y = _robot_position_at(*robot_pose[:3],*command[:3],time_from_now,integration_dt)
            uncertainty = uncertainty_scale*math.sqrt(max(0.,cov_xx,cov_yy))
            clearance = math.hypot(x-robot_x,y-robot_y)-uncertainty
            if best is None or clearance<best[0]:
                best = (clearance,time_from_now,x,y)
    if best is None or best[0]>collision_radius:
        return None
    return best
