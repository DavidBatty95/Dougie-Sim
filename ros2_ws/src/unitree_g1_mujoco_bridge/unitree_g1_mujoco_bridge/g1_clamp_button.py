import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

from .g1_constants import G1_29DOF_JOINTS


class G1ClampButton(Node):
    def __init__(self) -> None:
        super().__init__("g1_clamp_button")
        self.declare_parameter("rate_hz", 100.0)

        self.body_pub = self.create_publisher(Float64MultiArray, "/g1/body_position_commands", 10)
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.service = self.create_service(SetBool, "/g1/clamp_pose", self._set_clamp_cb)

        self.lock = threading.Lock()
        self.enabled = False
        self.latest_positions = {name: 0.0 for name in G1_29DOF_JOINTS}
        self.clamped_positions = [0.0] * len(G1_29DOF_JOINTS)

        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(period, self._publish_clamp_pose)
        self.get_logger().info("G1 clamp button ready: GUI button and /g1/clamp_pose service")

    def _joint_state_cb(self, msg: JointState) -> None:
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                if name in self.latest_positions:
                    self.latest_positions[name] = position

    def _set_clamp_cb(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            self.enable_clamp()
            response.message = "Robot clamp enabled at current body joint pose"
        else:
            self.disable_clamp()
            response.message = "Robot clamp disabled"
        response.success = True
        return response

    def enable_clamp(self) -> None:
        with self.lock:
            self.clamped_positions = [self.latest_positions[name] for name in G1_29DOF_JOINTS]
            self.enabled = True
        self.get_logger().info("Robot clamp enabled at current body joint pose")

    def disable_clamp(self) -> None:
        with self.lock:
            self.enabled = False
        self.get_logger().info("Robot clamp disabled")

    def is_enabled(self) -> bool:
        with self.lock:
            return self.enabled

    def _publish_clamp_pose(self) -> None:
        with self.lock:
            if not self.enabled:
                return
            positions = list(self.clamped_positions)

        msg = Float64MultiArray()
        msg.data = positions
        self.body_pub.publish(msg)


class ClampGui:
    def __init__(self, node: G1ClampButton) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("G1 Sensor Test Clamp")
        self.root.geometry("320x150")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.status = tk.StringVar(value="Clamp disabled")
        self.button_text = tk.StringVar(value="Clamp Robot")

        frame = ttk.Frame(self.root, padding=16)
        frame.pack(fill=tk.BOTH, expand=True)

        label = ttk.Label(frame, text="Hold robot body pose for sensor testing")
        label.pack(pady=(0, 12))

        self.button = ttk.Button(frame, textvariable=self.button_text, command=self.toggle)
        self.button.pack(fill=tk.X, pady=(0, 12))

        self.status_label = ttk.Label(frame, textvariable=self.status)
        self.status_label.pack()
        self.root.after(250, self.refresh)

    def toggle(self) -> None:
        if self.node.is_enabled():
            self.node.disable_clamp()
        else:
            self.node.enable_clamp()
        self.refresh()

    def refresh(self) -> None:
        if self.node.is_enabled():
            self.status.set("Clamp enabled: holding current pose")
            self.button_text.set("Release Robot")
        else:
            self.status.set("Clamp disabled")
            self.button_text.set("Clamp Robot")
        self.root.after(250, self.refresh)

    def run(self) -> None:
        self.root.mainloop()

    def close(self) -> None:
        self.root.quit()


def main() -> None:
    rclpy.init()
    node = G1ClampButton()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()

    try:
        ClampGui(node).run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
