import math

import rclpy
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.duration import Duration
from rclpy.node import Node
from std_srvs.srv import Empty
from unitree_hg.msg import LowState


class G1SlamResetMonitor(Node):
    def __init__(self) -> None:
        super().__init__("g1_slam_reset_monitor")
        self.declare_parameter("fallen_tilt_threshold", 0.95)
        self.declare_parameter("upright_tilt_threshold", 0.45)
        self.declare_parameter("reset_cooldown", 4.0)
        self.declare_parameter("reset_mujoco_on_fall", False)
        self.declare_parameter("mujoco_process_names", ["unitree_mujoco"])
        self.declare_parameter("mapping_reset_delay", 1.5)
        self.declare_parameter("slam_reset_services", ["/slam_toolbox/reset", "/slam_toolbox/clear"])
        self.declare_parameter(
            "costmap_reset_services",
            ["/local_costmap/clear_entirely_local_costmap", "/global_costmap/clear_entirely_global_costmap"],
        )

        self.fallen = False
        self.last_reset_time = None
        self.pending_mapping_reset_time = None
        self.slam_reset_clients = [self.create_client(Empty, name) for name in self.get_parameter("slam_reset_services").value]
        self.costmap_reset_clients = [
            self.create_client(ClearEntireCostmap, name) for name in self.get_parameter("costmap_reset_services").value
        ]
        self.create_subscription(LowState, "/lowstate", self._lowstate_callback, 10)
        self.create_subscription(LowState, "rt/lowstate", self._lowstate_callback, 10)
        self.create_timer(0.2, self._timer_callback)
        self.get_logger().info("Fall recovery monitor ready; mapping will reset after falls")

    def _lowstate_callback(self, msg: LowState) -> None:
        roll = self._wrap_pi(float(msg.imu_state.rpy[0]))
        pitch = self._wrap_pi(float(msg.imu_state.rpy[1]))
        tilt = max(abs(roll), abs(pitch))
        if not self.fallen and tilt > float(self.get_parameter("fallen_tilt_threshold").value):
            self.fallen = True
            self.get_logger().warn(f"Robot fall detected (tilt={tilt:.2f}); requesting mapping reset")
            self._request_fall_recovery()
            return
        if self.fallen and tilt < float(self.get_parameter("upright_tilt_threshold").value):
            self.fallen = False
            self._schedule_mapping_reset()

    def _timer_callback(self) -> None:
        if self.pending_mapping_reset_time is None:
            return
        if self.get_clock().now() < self.pending_mapping_reset_time:
            return
        self.pending_mapping_reset_time = None
        self._request_mapping_reset()

    def _request_fall_recovery(self) -> None:
        now = self.get_clock().now()
        if self.last_reset_time is not None:
            cooldown = float(self.get_parameter("reset_cooldown").value)
            if (now - self.last_reset_time).nanoseconds * 1e-9 < cooldown:
                return
        self.last_reset_time = now
        if bool(self.get_parameter("reset_mujoco_on_fall").value):
            self._signal_mujoco_reset()
        self._schedule_mapping_reset()

    def _schedule_mapping_reset(self) -> None:
        delay = max(0.0, float(self.get_parameter("mapping_reset_delay").value))
        self.pending_mapping_reset_time = self.get_clock().now() + Duration(seconds=delay)

    def _signal_mujoco_reset(self) -> None:
        self.get_logger().warn(
            "MuJoCo reset by signal is disabled unless the simulator installs a SIGUSR1 handler; "
            "leaving simulator process running",
            throttle_duration_sec=2.0,
        )

    def _request_mapping_reset(self) -> None:
        sent = False
        for client in self.slam_reset_clients:
            if client.service_is_ready():
                client.call_async(Empty.Request())
                self.get_logger().warn(f"Requested mapping reset via {client.srv_name}")
                sent = True
        for client in self.costmap_reset_clients:
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())
                self.get_logger().warn(f"Requested costmap reset via {client.srv_name}")
                sent = True
        if not sent:
            self.get_logger().warn(
                "Fall recovery requested, but no configured mapping reset service is available yet",
                throttle_duration_sec=2.0,
            )

    @staticmethod
    def _wrap_pi(value: float) -> float:
        return math.atan2(math.sin(value), math.cos(value))


def main() -> None:
    rclpy.init()
    node = G1SlamResetMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
