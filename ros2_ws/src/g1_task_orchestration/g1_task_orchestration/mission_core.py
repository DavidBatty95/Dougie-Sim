from dataclasses import dataclass
import math


PHASE_ACQUIRE = "acquire_arm"
PHASE_PIN_BASE = "pin_base"
PHASE_UNPIN_BASE = "unpin_base"
PHASE_TUCK = "tuck_arms"
PHASE_NAVIGATE_PICKUP = "navigate_pickup"
PHASE_APPROACH_PICKUP = "approach_pickup"
PHASE_PICK = "pick"
PHASE_RETREAT_PICKUP = "retreat_pickup"
PHASE_CARRY = "carry"
PHASE_NAVIGATE_DELIVERY = "navigate_delivery"
PHASE_APPROACH_DELIVERY = "approach_delivery"
PHASE_PLACE = "place"
PHASE_RETREAT_DELIVERY = "retreat_delivery"
PHASE_RELEASE = "release_arm"


@dataclass
class MissionState:
    authority_active: bool = False
    object_may_be_held: bool = False
    base_pinned: bool = False

    @property
    def can_release_authority(self):
        return self.authority_active and not self.object_may_be_held


def validate_goal(goal):
    if not goal.object_id:
        return "object_id must not be empty"
    if not goal.surface_object_id:
        return "surface_object_id must not be empty"
    if goal.arm not in ("left", "right"):
        return "arm must be left or right"
    if not goal.pickup_staging_pose.header.frame_id:
        return "pickup staging pose frame must not be empty"
    if not goal.delivery_staging_pose.header.frame_id:
        return "delivery staging pose frame must not be empty"
    for name in ("navigation_timeout_s", "approach_timeout_s", "retreat_distance_m"):
        value = getattr(goal, name)
        if not math.isfinite(value) or value < 0.0:
            return f"{name} must be finite and non-negative"
    return ""
