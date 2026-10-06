"""Timing of a specific inference result reaching the joint-target publisher.

This is not an acknowledgement from the bridge, simulator or physical motors.
LowState has no acquisition stamp here; its age is explicitly receipt-based.
"""
import math


def result_is_stale(result,now,deadline):
    """Completion before a deadline does not permit arbitrarily late application."""
    age = now-result.submitted_at
    return not math.isfinite(age) or age<0. or age>deadline


def inference_timing(result,context,published_monotonic,published_ros):
    ordered = (all(math.isfinite(value) for value in
                   (result.submitted_at,result.completed_at,published_monotonic,published_ros)) and
               result.submitted_at <= result.completed_at <= published_monotonic)
    matched = context is not None and context.get('key') == (result.generation,result.sequence)
    output = dict(schema_version=1,role='policy_joint_target_publication_not_actuator_ack',
                  generation=result.generation,sequence=result.sequence,stamp=published_ros,
                  timing_valid=ordered,context_matched=matched,
                  submitted_monotonic_s=result.submitted_at,
                  completed_monotonic_s=result.completed_at,
                  published_monotonic_s=published_monotonic)
    if ordered:
        output.update(submit_to_complete_ms=1000.*(result.completed_at-result.submitted_at),
                      complete_to_publish_ms=1000.*(published_monotonic-result.completed_at),
                      submit_to_publish_ms=1000.*(published_monotonic-result.submitted_at))
    if matched:
        output['command_at_submit'] = context['command']
        age = published_ros-context['command_received_ros_s']
        output['command_receipt_age_at_target_publish_s'] = age if math.isfinite(age) and age>=0. else None
        lowstate_age = context['lowstate_receipt_age_at_submit_s']
        if ordered and math.isfinite(lowstate_age) and lowstate_age>=0.:
            output['lowstate_receipt_age_at_target_publish_s'] = lowstate_age+published_monotonic-result.submitted_at
    return output
