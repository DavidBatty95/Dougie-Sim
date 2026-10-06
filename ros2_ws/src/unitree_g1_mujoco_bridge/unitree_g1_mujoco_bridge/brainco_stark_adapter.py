"""Optional normalized application interface for BrainCo's ROS 2 Stark driver.

Requires the vendor ros2_stark_msgs package and normalized 0..1000 SDK units.
The simulation uses the same /brainco/{left,right}/{cmd,state} application topics.
"""
import math
import time


def stark_command(positions,speeds,measured=None):
    if len(positions)!=6 or len(speeds)!=6:
        raise ValueError('six BrainCo channels required')
    if any(not math.isfinite(value) or not 0.<=value<=1. for value in (*positions,*speeds)):
        raise ValueError('BrainCo commands must be finite normalized values')
    result = list(positions)
    for index,speed in enumerate(speeds):
        if speed==0.:
            if measured is None or len(measured)!=6 or not math.isfinite(measured[index]) or not 0.<=measured[index]<=1.:
                raise ValueError('zero-speed hold requires fresh measured positions')
            result[index] = measured[index]
    return [round(value*1000) for value in result],[max(1,round(value*1000)) for value in speeds]


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from unitree_go.msg import MotorCmds,MotorStates,MotorState
    try:
        from ros2_stark_msgs.msg import SetMotorMulti,MotorStatus
    except ImportError as exc:
        raise RuntimeError('Install and source the vendor ros2_stark_msgs package for the physical-hand adapter') from exc

    class Adapter(Node):
        def __init__(self):
            super().__init__('brainco_stark_adapter')
            self.measured = {}
            self.current_scale = float(self.declare_parameter('current_scale_to_amp',0.).value)
            if not math.isfinite(self.current_scale) or self.current_scale<0.:
                raise ValueError('current_scale_to_amp must be finite and nonnegative')
            self.ids,self.commands,self.states = {},{},{}
            for side,default in (('left',1),('right',2)):
                self.ids[side] = self.declare_parameter(side+'_slave_id',default).value
                identifier = self.ids[side]
                self.commands[side] = self.create_publisher(SetMotorMulti,f'/set_motor_multi_{identifier}',10)
                self.states[side] = self.create_publisher(MotorStates,f'/brainco/{side}/state',10)
                self.create_subscription(MotorCmds,f'/brainco/{side}/cmd',lambda msg,side=side:self.command(msg,side),10)
                self.create_subscription(MotorStatus,f'/motor_status_{identifier}',lambda msg,side=side:self.state(msg,side),10)

        def command(self,msg,side):
            measured,stamp = self.measured.get(side,(None,0.))
            if time.monotonic()-stamp>.5:
                measured = None
            try:
                positions,speeds = stark_command([m.q for m in msg.cmds],[m.dq for m in msg.cmds],measured)
            except ValueError as exc:
                self.get_logger().warning(str(exc),throttle_duration_sec=2.)
                return
            command = SetMotorMulti()
            command.slave_id,command.mode = self.ids[side],6
            command.positions,command.speeds = positions,speeds
            self.commands[side].publish(command)

        def state(self,msg,side):
            if msg.slave_id!=self.ids[side] or any(len(getattr(msg,name))!=6 for name in ('positions','speeds','currents','states')):
                return
            positions = [float(value)/1000. for value in msg.positions]
            if any(not math.isfinite(value) or not 0.<=value<=1. for value in positions):
                return
            self.measured[side] = positions,time.monotonic()
            state = MotorStates()
            for position,speed,current,status in zip(positions,msg.speeds,msg.currents,msg.states):
                motor = MotorState()
                motor.q = position
                motor.dq = min(1.,abs(float(speed))/1000.)
                motor.mode = int(status)  # Preserve the vendor status byte.
                # Firmware unit modes vary; do not label normalized current as amperes.
                motor.tau_est = float(current)*self.current_scale if self.current_scale else float('nan')
                state.states.append(motor)
            self.states[side].publish(state)

    rclpy.init(args=args)
    node = Adapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
