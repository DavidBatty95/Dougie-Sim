import threading
import time

from action_msgs.msg import GoalStatus
from g1_msgs.action import ApproachObject, Pick, Place, Retreat, SetArmPosture
from g1_msgs.action import TransportObject
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Empty, SetBool, Trigger

from .mission_core import (
    MissionState,
    PHASE_ACQUIRE,
    PHASE_APPROACH_DELIVERY,
    PHASE_APPROACH_PICKUP,
    PHASE_CARRY,
    PHASE_NAVIGATE_DELIVERY,
    PHASE_NAVIGATE_PICKUP,
    PHASE_PIN_BASE,
    PHASE_PICK,
    PHASE_PLACE,
    PHASE_RELEASE,
    PHASE_RETREAT_DELIVERY,
    PHASE_RETREAT_PICKUP,
    PHASE_TUCK,
    PHASE_UNPIN_BASE,
    validate_goal,
)


class MissionFailure(RuntimeError):
    def __init__(self, phase, message):
        super().__init__(message)
        self.phase = phase


class MissionCanceled(MissionFailure):
    pass


class ChildTerminationFailure(MissionFailure):
    pass


class TransportServer(Node):
    def __init__(self):
        super().__init__("g1_transport_server")
        self._group = ReentrantCallbackGroup()
        self._busy = False
        self._recovery_required = False
        self._busy_lock = threading.Lock()

        self._simulation_enabled = self.declare_parameter(
            "allow_simulation_manipulation", False
        ).value
        self._server_timeout = self.declare_parameter("server_timeout_s", 10.0).value
        self._cancel_timeout = self.declare_parameter("cancel_timeout_s", 5.0).value
        self._navigation_timeout = self.declare_parameter(
            "navigation_timeout_s", 300.0
        ).value
        self._approach_timeout = self.declare_parameter("approach_timeout_s", 60.0).value
        self._retreat_distance = self.declare_parameter("retreat_distance_m", 0.8).value
        self._retreat_timeout = self.declare_parameter("retreat_timeout_s", 30.0).value
        self._approach_attempts = self.declare_parameter("approach_attempts", 2).value
        self._pick_attempts = self.declare_parameter("pick_attempts", 2).value

        self._navigate = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=self._group
        )
        self._approach = ActionClient(
            self,
            ApproachObject,
            "/g1_base_approach/approach_object",
            callback_group=self._group,
        )
        self._retreat = ActionClient(
            self,
            Retreat,
            "/g1_base_approach/retreat",
            callback_group=self._group,
        )
        self._pick = ActionClient(
            self, Pick, "/g1_manipulation_server/pick", callback_group=self._group
        )
        self._place = ActionClient(
            self, Place, "/g1_manipulation_server/place", callback_group=self._group
        )
        self._posture = ActionClient(
            self,
            SetArmPosture,
            "/g1_manipulation_server/set_arm_posture",
            callback_group=self._group,
        )
        self._authority = self.create_client(
            SetBool,
            "/g1_arm_command_compositor/set_arm_authority",
            callback_group=self._group,
        )
        self._pin_base = self.create_client(
            Trigger, "/g1_sim_manipulation_io/pin_base", callback_group=self._group
        )
        self._unpin_base = self.create_client(
            Trigger, "/g1_sim_manipulation_io/unpin_base", callback_group=self._group
        )
        self._clear_global = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            callback_group=self._group,
        )
        self._clear_local = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            callback_group=self._group,
        )
        self._clear_octomap = self.create_client(
            Empty, "/clear_octomap", callback_group=self._group
        )
        self._clear_recovery = self.create_service(
            Trigger,
            "~/clear_recovery_latch",
            self._clear_recovery_latch,
            callback_group=self._group,
        )
        self._server = ActionServer(
            self,
            TransportObject,
            "~/transport_object",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._group,
        )

    def _goal(self, goal):
        if not self._simulation_enabled:
            self.get_logger().warning(
                "Rejecting transport goal: simulation manipulation is not enabled"
            )
            return GoalResponse.REJECT
        reason = validate_goal(goal)
        if reason:
            self.get_logger().warning(f"Rejecting transport goal: {reason}")
            return GoalResponse.REJECT
        with self._busy_lock:
            if self._busy or self._recovery_required:
                if self._recovery_required:
                    self.get_logger().error(
                        "Rejecting transport goal: operator recovery acknowledgment required"
                    )
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _clear_recovery_latch(self, _, response):
        with self._busy_lock:
            if self._busy:
                response.message = "cannot clear recovery latch during an active mission"
                return response
            self._recovery_required = False
        self.get_logger().warning("Operator acknowledged transport recovery state")
        response.success = True
        response.message = "transport recovery state acknowledged"
        return response

    @staticmethod
    def _value_or_default(value, default):
        return value if value > 0.0 else default

    def _feedback(self, handle, phase, message=""):
        feedback = TransportObject.Feedback()
        feedback.phase = phase
        feedback.message = message
        handle.publish_feedback(feedback)

    async def _sleep(self, duration):
        future = Future()
        timer = self.create_timer(
            duration,
            lambda: future.set_result(None) if not future.done() else None,
            callback_group=self._group,
        )
        try:
            await future
        finally:
            self.destroy_timer(timer)

    async def _call_action(self, handle, client, goal, phase, timeout, check_success=True):
        self._feedback(handle, phase)
        if not client.wait_for_server(timeout_sec=self._server_timeout):
            raise MissionFailure(phase, "action server unavailable")
        remote = await client.send_goal_async(goal)
        if not remote.accepted:
            raise MissionFailure(phase, "goal rejected")

        result_future = remote.get_result_async()
        deadline = time.monotonic() + timeout if timeout > 0.0 else None
        while not result_future.done():
            if handle.is_cancel_requested:
                await self._cancel_and_wait(remote, result_future, phase)
                raise MissionCanceled(phase, "mission cancelled")
            if deadline is not None and time.monotonic() >= deadline:
                await self._cancel_and_wait(remote, result_future, phase)
                raise MissionFailure(phase, "action timed out")
            await self._sleep(0.05)

        wrapped = await result_future
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            message = getattr(wrapped.result, "message", "") or "action did not succeed"
            raise MissionFailure(phase, message)
        if check_success and not getattr(wrapped.result, "success", False):
            message = getattr(wrapped.result, "message", "") or "action reported failure"
            raise MissionFailure(phase, message)
        return wrapped.result

    async def _cancel_and_wait(self, remote, result_future, phase):
        response = await remote.cancel_goal_async()
        if not response.goals_canceling:
            raise ChildTerminationFailure(phase, "child action rejected cancellation")
        deadline = time.monotonic() + self._cancel_timeout
        while not result_future.done():
            if time.monotonic() >= deadline:
                raise ChildTerminationFailure(
                    phase, "child action did not terminate after cancellation"
                )
            await self._sleep(0.05)

    async def _set_authority(self, enabled, phase):
        if not self._authority.wait_for_service(timeout_sec=self._server_timeout):
            raise MissionFailure(phase, "arm authority service unavailable")
        request = SetBool.Request()
        request.data = enabled
        response = await self._authority.call_async(request)
        if not response.success:
            raise MissionFailure(phase, response.message or "arm authority request failed")

    async def _set_base_pin(self, enabled, phase):
        client = self._pin_base if enabled else self._unpin_base
        if not client.wait_for_service(timeout_sec=self._server_timeout):
            raise MissionFailure(phase, "simulation base pin service unavailable")
        response = await client.call_async(Trigger.Request())
        if not response.success:
            raise MissionFailure(phase, response.message or "simulation base pin failed")

    async def _best_effort_clear(self):
        for client, request_type, name in (
            (self._clear_global, ClearEntireCostmap.Request, "global costmap"),
            (self._clear_local, ClearEntireCostmap.Request, "local costmap"),
            (self._clear_octomap, Empty.Request, "MoveIt octomap"),
        ):
            if not client.wait_for_service(timeout_sec=0.2):
                self.get_logger().warning(f"Skipping unavailable {name} clear service")
                continue
            try:
                await client.call_async(request_type())
            except Exception as error:  # Housekeeping must not fail the mission.
                self.get_logger().warning(f"Could not clear {name}: {error}")

    async def _retry_action(self, attempts, *args, **kwargs):
        failure = None
        for attempt in range(1, max(1, attempts) + 1):
            try:
                return await self._call_action(*args, **kwargs)
            except MissionCanceled:
                raise
            except MissionFailure as error:
                failure = error
                self.get_logger().warning(
                    f"{error.phase} attempt {attempt}/{max(1, attempts)} failed: {error}"
                )
        raise failure

    async def _execute(self, handle):
        state = MissionState()
        goal = handle.request
        navigation_timeout = self._value_or_default(
            goal.navigation_timeout_s, self._navigation_timeout
        )
        approach_timeout = self._value_or_default(
            goal.approach_timeout_s, self._approach_timeout
        )
        retreat_distance = self._value_or_default(
            goal.retreat_distance_m, self._retreat_distance
        )
        try:
            navigate = NavigateToPose.Goal()
            navigate.pose = goal.pickup_staging_pose
            await self._call_action(
                handle,
                self._navigate,
                navigate,
                PHASE_NAVIGATE_PICKUP,
                navigation_timeout,
                check_success=False,
            )

            approach = ApproachObject.Goal()
            approach.object_id = goal.object_id
            approach.arm = goal.arm
            approach.use_current_heading = True
            approach.timeout_s = approach_timeout
            await self._retry_action(
                self._approach_attempts,
                handle,
                self._approach,
                approach,
                PHASE_APPROACH_PICKUP,
                approach_timeout + 5.0,
            )

            await self._set_base_pin(True, PHASE_PIN_BASE)
            state.base_pinned = True
            await self._set_authority(True, PHASE_ACQUIRE)
            state.authority_active = True

            pick = Pick.Goal()
            pick.object_id = goal.object_id
            pick.arm = goal.arm
            # A rejected or canceled pick can leave physical attachment state uncertain.
            state.object_may_be_held = True
            await self._retry_action(
                self._pick_attempts,
                handle,
                self._pick,
                pick,
                PHASE_PICK,
                approach_timeout,
            )
            posture = SetArmPosture.Goal()
            posture.group = f"{goal.arm}_arm"
            posture.named_target = "policy_stand"
            await self._call_action(
                handle, self._posture, posture, PHASE_CARRY, approach_timeout
            )
            await self._set_base_pin(False, PHASE_UNPIN_BASE)
            state.base_pinned = False

            retreat = Retreat.Goal()
            retreat.distance_m = retreat_distance
            retreat.timeout_s = self._retreat_timeout
            await self._call_action(
                handle,
                self._retreat,
                retreat,
                PHASE_RETREAT_PICKUP,
                self._retreat_timeout + 5.0,
            )
            await self._best_effort_clear()

            navigate.pose = goal.delivery_staging_pose
            await self._call_action(
                handle,
                self._navigate,
                navigate,
                PHASE_NAVIGATE_DELIVERY,
                navigation_timeout,
                check_success=False,
            )

            approach.object_id = goal.surface_object_id
            await self._retry_action(
                self._approach_attempts,
                handle,
                self._approach,
                approach,
                PHASE_APPROACH_DELIVERY,
                approach_timeout + 5.0,
            )

            await self._set_base_pin(True, PHASE_PIN_BASE)
            state.base_pinned = True

            place = Place.Goal()
            place.surface_object_id = goal.surface_object_id
            place.arm = goal.arm
            await self._call_action(
                handle, self._place, place, PHASE_PLACE, approach_timeout
            )
            state.object_may_be_held = False

            posture.group = "both_arms"
            posture.named_target = "policy_stand"
            await self._call_action(
                handle, self._posture, posture, PHASE_TUCK, approach_timeout
            )
            await self._set_authority(False, PHASE_RELEASE)
            state.authority_active = False
            await self._set_base_pin(False, PHASE_UNPIN_BASE)
            state.base_pinned = False

            await self._call_action(
                handle,
                self._retreat,
                retreat,
                PHASE_RETREAT_DELIVERY,
                self._retreat_timeout + 5.0,
            )
            await self._best_effort_clear()

            result = TransportObject.Result()
            result.success = True
            result.message = f"transported {goal.object_id} to {goal.surface_object_id}"
            handle.succeed()
            return result
        except MissionFailure as error:
            self.get_logger().error(f"Transport failed in {error.phase}: {error}")
            if state.object_may_be_held or isinstance(error, ChildTerminationFailure):
                self._recovery_required = True
            if state.can_release_authority:
                try:
                    await self._set_authority(False, PHASE_RELEASE)
                    state.authority_active = False
                except MissionFailure as cleanup_error:
                    self.get_logger().error(f"Authority cleanup failed: {cleanup_error}")
                    self._recovery_required = True
            if state.base_pinned and not state.object_may_be_held:
                try:
                    await self._set_base_pin(False, PHASE_UNPIN_BASE)
                    state.base_pinned = False
                except MissionFailure as cleanup_error:
                    self.get_logger().error(f"Base pin cleanup failed: {cleanup_error}")
                    self._recovery_required = True
            result = TransportObject.Result()
            result.failed_phase = error.phase
            result.message = str(error)
            result.arm_authority_active = state.authority_active
            result.object_may_be_held = state.object_may_be_held
            if isinstance(error, MissionCanceled):
                handle.canceled()
            else:
                handle.abort()
            return result
        except Exception as error:
            self.get_logger().error(f"Unexpected transport failure: {error}")
            if state.object_may_be_held:
                self._recovery_required = True
            if state.can_release_authority:
                try:
                    await self._set_authority(False, PHASE_RELEASE)
                    state.authority_active = False
                except Exception as cleanup_error:
                    self.get_logger().error(f"Authority cleanup failed: {cleanup_error}")
                    self._recovery_required = True
            if state.base_pinned and not state.object_may_be_held:
                try:
                    await self._set_base_pin(False, PHASE_UNPIN_BASE)
                    state.base_pinned = False
                except Exception as cleanup_error:
                    self.get_logger().error(f"Base pin cleanup failed: {cleanup_error}")
                    self._recovery_required = True
            result = TransportObject.Result()
            result.failed_phase = "internal_error"
            result.message = str(error)
            result.arm_authority_active = state.authority_active
            result.object_may_be_held = state.object_may_be_held
            handle.abort()
            return result
        finally:
            with self._busy_lock:
                self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = TransportServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
