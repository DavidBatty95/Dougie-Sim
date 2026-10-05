"""Sensor-driven demonstration: discover, dock, attach, carry and place.

The grasp actuator is the documented capture-radius weld. No base pin, object
pose setter, simulator layout or evaluation subscription is used by the mission.
"""
import argparse
import json
import math
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,PointStamped,Vector3Stamped
from nav_msgs.msg import Odometry,Path as RosPath
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool,String
from std_srvs.srv import SetBool,Trigger
from g1_msgs.srv import GraspObject
from unitree_social_navigation_msgs.msg import TrackedPersons
from vision_msgs.msg import Detection3DArray
from tf2_ros import Buffer,TransformListener,TransformException

from .arm_kinematics import ArmKinematics,rpy_matrix
from .transport_perception import TransportPerception
from .transport_actions import ManagedAction
from .transport_health import EvaluationHealth


def angle(value):
    return math.atan2(math.sin(value),math.cos(value))


class WorkspaceInterrupted(RuntimeError):
    """A terminated arm action may be retried after measured workspace recovery."""


class TransportMission(Node):
    def __init__(self,args,executor):
        super().__init__('g1_transport_demo')
        self.args,self.executor = args,executor
        self.ik_worker = ThreadPoolExecutor(max_workers=1)
        self.rest_arm_positions = None
        self.pose = None
        self.joints = {}
        self.rpy = (0.,0.,0.)
        self.observations = {}
        self.people = None
        self.kin = None
        self.active = False
        self.mode = 'navigate'
        self.attachment_state = 'empty'
        self.offset = np.zeros(3)
        self.grasp_R = None
        self.goal = None
        self.events = []
        self.phase = 'initializing'
        self.started = time.monotonic()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.goal_pub = self.create_publisher(PoseStamped,'/benchmark/current_goal',1)
        self.active_pub = self.create_publisher(Bool,'/benchmark/goal_active',1)
        self.mode_pub = self.create_publisher(String,'/navigation/goal_mode',1)
        self.carry_pub = self.create_publisher(Bool,'/navigation/carrying',1)
        self.payload_pub = self.create_publisher(PointStamped,'/navigation/payload_pose',1)
        self.path_pub = self.create_publisher(RosPath,'/plan',1)
        self.status_pub = self.create_publisher(String,'/transport/mission_state',10)
        self.nav = ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.arm = ActionClient(self,FollowJointTrajectory,'/arm_trajectory_controller/follow_joint_trajectory')
        self.authority = self.create_client(SetBool,'/g1_arm_command_compositor/set_arm_authority')
        self.grasp = self.create_client(GraspObject,'/g1_sim_manipulation_io/grasp_object')
        self.release = self.create_client(Trigger,'/g1_sim_manipulation_io/release_right')
        qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        durable = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String,'/robot_description',self._description,durable)
        self.create_subscription(String,'/transport/observations',self._observations,10)
        self.create_subscription(Odometry,'/odom',self._odom,qos)
        self.create_subscription(JointState,'/joint_states',lambda m:setattr(self,'joints',dict(zip(m.name,m.position))),qos)
        self.create_subscription(Vector3Stamped,'/onboard/measured_rpy',lambda m:setattr(self,'rpy',(m.vector.x,m.vector.y,m.vector.z)),qos)
        self.create_subscription(TrackedPersons,'/tracked_persons',lambda m:setattr(self,'people',m),qos)
        self.create_timer(.1,self._heartbeat)

    def seconds(self):
        return self.get_clock().now().nanoseconds*1e-9

    @property
    def held(self):
        return self.attachment_state == 'held'

    @property
    def object_may_be_held(self):
        return self.attachment_state != 'empty'

    def _description(self,msg):
        if self.kin is None:
            # Measured point on the native rubber hand, in wrist coordinates.
            self.kin = ArmKinematics(msg.data,tool_offset=(.12,.01,0.))

    def _observations(self,msg):
        data = json.loads(msg.data)
        if data.get('frame') == 'odom':
            self.observations = {item['id']:item for item in data['objects']}

    def _odom(self,msg):
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        self.pose = (p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.pose_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def _heartbeat(self):
        self.active_pub.publish(Bool(data=self.active))
        self.mode_pub.publish(String(data=self.mode))
        self.carry_pub.publish(Bool(data=self.object_may_be_held))
        self.status_pub.publish(String(data=json.dumps(dict(phase=self.phase,held=self.held,
            attachment_state=self.attachment_state,object_may_be_held=self.object_may_be_held,
            elapsed=time.monotonic()-self.started))))
        if self.active and self.mode in ('dock','scan') and self.pose is not None and self.goal is not None:
            path = RosPath()
            path.header.frame_id = 'odom'
            path.header.stamp = self.get_clock().now().to_msg()
            current = self.pose_message(self.pose,'odom')
            path.poses = [current,self.goal]
            self.path_pub.publish(path)
        if self.object_may_be_held and self.kin is not None:
            wrist,R,_,_ = self.kin.forward(self.joints)
            point = rpy_matrix((self.rpy[0],self.rpy[1],0.))@(wrist+R@self.offset)
            point[2] += self.joints.get('floating_base_joint',.793)
            msg = PointStamped()
            msg.header.frame_id = 'base_footprint'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x,msg.point.y,msg.point.z = map(float,point)
            self.payload_pub.publish(msg)

    def event(self,phase,**details):
        self.phase = phase
        self.events.append(dict(phase=phase,elapsed_s=time.monotonic()-self.started,**details))
        self.get_logger().info(f'{phase}: {details}')

    def spin(self,duration=.05):
        end = time.monotonic()+duration
        while rclpy.ok() and time.monotonic()<end:
            self.executor.spin_once(timeout_sec=min(.05,max(0.,end-time.monotonic())))
        if time.monotonic()-self.started > self.args.duration:
            raise RuntimeError('mission deadline exceeded')
        if self.active and self.pose is not None and self.seconds()-self.pose_stamp > 2.:
            raise RuntimeError('measured odometry stale during motion')

    def wait(self,predicate,timeout,message):
        end = time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<end:
            if predicate():
                return
            self.spin()
        raise RuntimeError(message)

    def future(self,future,timeout=15.):
        self.wait(future.done,timeout,'operation timed out')
        return future.result()

    def action(self,name,client,goal):
        return ManagedAction(name,client.send_goal_async(goal),self.future,
            lambda timeout:self.executor.spin_once(timeout_sec=timeout),rclpy.ok,
            lambda **details:self.event(self.phase,**details),before_cleanup=self.stop_intent)

    def stop_intent(self):
        self.active = False
        self._heartbeat()

    def service(self,client,request,*,attachment_after=None):
        self.wait(client.service_is_ready,15.,'manipulation service unavailable')
        if attachment_after is not None:
            # Once dispatched, timeout/rejection cannot prove actuator state.
            # Keep authority and conservative payload geometry on every failure.
            self.attachment_state = 'uncertain'
            self.event(self.phase,attachment_state=self.attachment_state)
            self._heartbeat()
        response = self.future(client.call_async(request))
        if not response.success:
            raise RuntimeError(response.message)
        if attachment_after is not None:
            self.attachment_state = attachment_after
            self.event(self.phase,attachment_state=self.attachment_state)
            self._heartbeat()
        return response

    @staticmethod
    def pose_message(pose,frame):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x,msg.pose.position.y = float(pose[0]),float(pose[1])
        msg.pose.orientation.z,msg.pose.orientation.w = math.sin(pose[2]/2),math.cos(pose[2]/2)
        return msg

    def set_goal(self,pose,mode):
        self.mode = mode
        self.goal = self.pose_message(pose,'odom')
        self.goal.header.stamp = self.get_clock().now().to_msg()
        if mode == 'navigate':
            transform = self.tf.lookup_transform('map','odom',rclpy.time.Time()).transform
            q = transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.goal = self.pose_message((transform.translation.x+math.cos(yaw)*pose[0]-math.sin(yaw)*pose[1],
                transform.translation.y+math.sin(yaw)*pose[0]+math.cos(yaw)*pose[1],pose[2]+yaw),'map')
            self.goal.header.stamp = self.get_clock().now().to_msg()
        self.active = True
        self.goal_pub.publish(self.goal)
        self._heartbeat()
        self.event(self.phase,goal_odom=list(pose),start_pose=self.pose,mode=mode)

    def navigate(self,pose,timeout=90.):
        self.wait(self.nav.server_is_ready,30.,'Nav2 unavailable')
        self.set_goal(pose,'navigate')
        request = NavigateToPose.Goal()
        request.pose = self.goal
        try:
            with self.action('navigation',self.nav,request) as action:
                def arrived():
                    close = self.pose is not None and math.hypot(self.pose[0]-pose[0],self.pose[1]-pose[1]) < .50
                    if action.result.done():
                        wrapped = action.result.result()
                        if wrapped.status != 4:
                            raise RuntimeError(f'navigation terminated with status {wrapped.status}')
                        if not close:
                            raise RuntimeError('navigation succeeded outside sensed staging tolerance')
                    return close
                self.wait(arrived,timeout,'navigation did not reach sensed staging pose')
        finally:
            self.active = False
            self._heartbeat()
        self.spin(1.)

    def dock(self,pose,timeout=75.):
        self.set_goal(pose,'dock')
        try:
            self.wait(lambda: self.pose is not None and math.hypot(self.pose[0]-pose[0],self.pose[1]-pose[1]) < .07 and
                      abs(angle(self.pose[2]-pose[2])) < .15,timeout,'precision docking failed')
        finally:
            self.active = False
            self._heartbeat()
        self.spin(1.5)

    def observe(self,identifier,max_age=5.):
        item = self.observations.get(identifier)
        return item if item is not None and 0. <= self.seconds()-item['observed_stamp'] <= max_age else None

    def placement_observed(self,identifier,pad,release_stamp):
        item = self.observe(identifier)
        return bool(item is not None and item['observed_stamp'] > release_stamp and
            np.linalg.norm(np.array(item['position'])[:2]-pad[:2]) < .13 and
            abs(item['position'][2]-pad[2]-.055) < .06)

    def rotate(self,yaw,timeout=45.):
        # Search requires a view direction, not centimetre-level base-position
        # recovery after every step of a loaded turn.
        self.set_goal((self.pose[0],self.pose[1],yaw),'scan')
        try:
            self.wait(lambda: abs(angle(self.pose[2]-yaw)) < .14,timeout,'search heading not reached')
        finally:
            self.active = False
            self._heartbeat()
        self.spin(1.)

    def discover(self,identifier):
        self.event('discover',object_id=identifier)
        for _ in range(9):
            if self.observe(identifier,20.) is not None:
                return self.observe(identifier,20.)
            self.rotate(angle(self.pose[2]+math.pi/4))
            self.spin(1.5)
        raise RuntimeError(f'RGB-D did not discover {identifier}')

    def workspace_clear(self):
        if self.people is None:
            return False
        stamp = self.people.header.stamp.sec+self.people.header.stamp.nanosec*1e-9
        if not 0. <= self.seconds()-stamp <= .8 or self.people.header.frame_id != 'odom':
            return False
        for person in self.people.persons:
            p,v = person.pose.pose.position,person.velocity.twist.linear
            for t in (0.,1.,2.):
                if math.hypot(p.x+v.x*t-self.pose[0],p.y+v.y*t-self.pose[1]) < 1.6:
                    return False
        return True

    def to_pelvis(self,position):
        relative = np.asarray(position)-np.array((self.pose[0],self.pose[1],self.joints.get('floating_base_joint',.793)))
        return rpy_matrix((self.rpy[0],self.rpy[1],self.pose[2])).T@relative

    def move_arm(self,target,orientation=None):
        desired = np.asarray(target,dtype=float)
        correction = np.zeros(3)
        recoveries = 0
        for attempt in range(3):
            while True:
                anchor = self.pose
                try:
                    measured = self._move_arm(desired+correction,orientation)
                    break
                except WorkspaceInterrupted:
                    if recoveries >= 2:
                        raise RuntimeError('arm workspace recovery limit exceeded; authority retained')
                    recoveries += 1
                    phase = self.phase
                    self.event('wait_for_clearance',resume_phase=phase,recovery=recoveries)
                    self.wait_workspace()
                    if (math.hypot(self.pose[0]-anchor[0],self.pose[1]-anchor[1]) > .04 or
                            abs(angle(self.pose[2]-anchor[2])) > .10):
                        raise RuntimeError('base moved during arm interruption; re-docking required')
                    self.event(phase,recovery=recoveries,resumed=True)
            error = desired-measured
            distance = float(np.linalg.norm(error))
            self.event(self.phase,hand_error_m=distance,servo_iteration=attempt)
            if distance <= .025:
                return
            correction += .8*error
            if np.linalg.norm(correction) > .08:
                break
        raise RuntimeError(f'measured hand missed target ({distance:.3f} m)')

    def wait_workspace(self,timeout=30.,dwell=.3):
        clear_since = None
        def ready():
            nonlocal clear_since
            if max(abs(self.rpy[0]),abs(self.rpy[1])) > .35:
                raise RuntimeError('body tilt exceeded manipulation limit while waiting')
            if not self.workspace_clear():
                clear_since = None
                return False
            now = time.monotonic()
            if clear_since is None:
                clear_since = now
            return now-clear_since >= dwell
        self.wait(ready,timeout,'manipulation workspace did not remain clear')

    def _move_arm(self,target,orientation=None,joint_solution=None):
        self.wait_workspace()
        if joint_solution is None:
            # Numerical IK must not prevent servicing measured-state callbacks.
            solution,error = self.future(self.ik_worker.submit(self.kin.solve,target,dict(self.joints),orientation))
        else:
            solution,error = joint_solution,0.
        self.event(self.phase,hand_target_pelvis=np.asarray(target).tolist(),ik_error_m=error)
        if error > .012:
            raise RuntimeError(f'hand target unreachable ({error:.3f} m)')
        # IK can outlast a people measurement. Never dispatch using the snapshot
        # that was fresh before the solve; revalidate after computation.
        if not self.workspace_clear():
            raise WorkspaceInterrupted('workspace changed during IK')
        names = self.kin.names
        initial = [float(self.joints.get(name,0.)) for name in names]
        positions = [solution[name] for name in names]
        duration = max(2.,max(abs(a-b) for a,b in zip(initial,positions))/.35)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        first = JointTrajectoryPoint()
        first.positions = initial
        final = JointTrajectoryPoint()
        final.positions = positions
        final.time_from_start.sec = int(duration)
        final.time_from_start.nanosec = int((duration-int(duration))*1e9)
        goal.trajectory.points = [first,final]
        self.event(self.phase,commanded_joints=dict(zip(names,positions)),duration_s=duration)
        self.wait(self.arm.server_is_ready,15.,'arm compositor unavailable')
        with self.action('arm',self.arm,goal) as action:
            end = time.monotonic()+duration+8.
            while not action.result.done():
                if max(abs(self.rpy[0]),abs(self.rpy[1])) > .35:
                    self.event(self.phase,interruption='body_tilt',body_rpy=list(self.rpy))
                    raise RuntimeError('body tilt exceeded manipulation limit')
                if not self.workspace_clear() or time.monotonic()>end:
                    stamp = None if self.people is None else self.people.header.stamp.sec+self.people.header.stamp.nanosec*1e-9
                    self.event(self.phase,interruption='workspace_or_timeout',people_age_s=None if stamp is None else self.seconds()-stamp,
                               people_frame=None if self.people is None else self.people.header.frame_id,
                               people_count=None if self.people is None else len(self.people.persons),arm_timeout=time.monotonic()>end)
                    if time.monotonic()>end:
                        raise RuntimeError('arm trajectory timed out; authority retained')
                    raise WorkspaceInterrupted('arm workspace interrupted; authority retained')
                self.spin()
            wrapped = action.result.result()
        self.event(self.phase,joint_target=dict(zip(names,positions)),
                   joint_measured={name:self.joints.get(name) for name in names},body_rpy=list(self.rpy))
        if wrapped.status != 4 or wrapped.result.error_code != 0:
            raise RuntimeError(f'arm execution failed: {wrapped.result.error_string}')
        self.spin(.6)
        measured = self.kin.forward(self.joints)[0]
        measured_error = float(np.linalg.norm(measured-target))
        self.event(self.phase,measured_hand_pelvis=measured.tolist(),hand_error_m=measured_error)
        return measured

    def run(self):
        self.wait(lambda: self.pose is not None and self.kin is not None and len(self.joints)>20,40.,'robot state unavailable')
        from dynamic_obstacle_benchmarks.source_contract import audit_onboard_graph
        self.wait(lambda: not audit_onboard_graph(self),30.,'onboard source audit failed')
        self.spin(3.)
        for pair in self.args.pairs:
            identifier,destination = pair['object_id'],pair['destination_id']
            item = self.discover(identifier)
            box = np.array(item['position'])
            yaw = math.atan2(box[1]-self.pose[1],box[0]-self.pose[0])
            forward,left = np.array((math.cos(yaw),math.sin(yaw))),np.array((-math.sin(yaw),math.cos(yaw)))
            self.event('navigate_pickup',object_id=identifier)
            stage = box[:2]-1.05*forward+.15*left
            self.navigate((*stage,yaw))
            self.event('dock_pickup')
            dock = box[:2]-.43*forward+.15*left
            self.dock((*dock,yaw))
            self.wait(lambda: self.observe(identifier,20.) is not None,8.,'no recent tabletop box observation')
            self.rest_arm_positions = {name:self.joints[name] for name in self.kin.names}
            self.service(self.authority,SetBool.Request(data=True))
            self.event('reach')
            box = np.array(self.observe(identifier,20.)['position'])
            # Raise through the validated carrying workspace first. Reaching
            # directly above a very close box folds the elbow/wrist to a much
            # less stable posture before the actual grasp approach.
            self.move_arm(np.array((.36,-.20,.37)),np.eye(3))
            # Reproject after reaching: the unpinned base may settle as the
            # arm's mass moves, so the old pelvis-relative target is stale.
            for correction in range(3):
                # The reaching hand can occlude a stationary tabletop box.
                # Retain its last sensed odom-frame pose through the reach;
                # attachment still requires the actual hand capture distance.
                self.wait(lambda: self.observe(identifier,20.) is not None,8.,'tabletop box observation expired')
                box = np.array(self.observe(identifier,20.)['position'])
                self.move_arm(self.to_pelvis(box)+np.array((-.055,0.,0.)),np.eye(3))
                self.wait(lambda: self.observe(identifier,20.) is not None,8.,'box observation expired at attachment')
                box = np.array(self.observe(identifier,20.)['position'])
                hand_distance = float(np.linalg.norm(self.kin.forward(self.joints)[0]-self.to_pelvis(box)))
                self.event('attachment_check',hand_to_box_m=hand_distance,correction=correction)
                if hand_distance <= .085:
                    break
            if hand_distance > .085:
                raise RuntimeError('measured hand outside magnetic attachment range')
            self.event('grasp')
            grasp_stamp = self.seconds()
            # Establish a conservative payload pose before a request can attach
            # the box, including when its acknowledgement is subsequently lost.
            wrist,R,_,_ = self.kin.forward(self.joints)
            self.offset = R.T@(self.to_pelvis(box)-wrist)
            self.grasp_R = R
            self.service(self.grasp,GraspObject.Request(arm='right',object_id=identifier),attachment_after='held')
            self.event('lift')
            self.move_arm(np.array((.36,-.20,.37)),R)
            self.wait(lambda: self.observe(identifier) is not None and
                      self.observe(identifier)['observed_stamp'] > grasp_stamp and
                      self.observe(identifier)['position'][2] > box[2]+.08,
                      8.,'camera did not verify the box lift')
            self.event('retreat_pickup')
            # Leave enough room for loaded-turn drift before searching; a
            # near-table scan can put Nav2's start inside its inflation region.
            self.dock((self.pose[0]-1.1*forward[0],self.pose[1]-1.1*forward[1],yaw))
            pad = self.discover(destination)
            point = np.array(pad['position'])
            bearing = math.atan2(point[1]-self.pose[1],point[0]-self.pose[0])
            yaw = min((pad['yaw']+k*math.pi/2 for k in range(4)),key=lambda value:abs(angle(value-bearing)))
            forward,left = np.array((math.cos(yaw),math.sin(yaw))),np.array((-math.sin(yaw),math.cos(yaw)))
            self.event('navigate_delivery')
            stage = point[:2]-1.05*forward+.15*left
            self.navigate((*stage,yaw))
            # At range, a horizontal pad spans very few image rows. Obtain a
            # closer face-direction observation before the final table approach.
            self.event('inspect_delivery')
            bearing = math.atan2(point[1]-self.pose[1],point[0]-self.pose[0])
            self.rotate(bearing)
            self.spin(1.)
            latest = self.observe(destination,2.)
            if latest is not None:
                point = np.array(latest['position'])
                yaw = min((latest['yaw']+k*math.pi/2 for k in range(4)),key=lambda value:abs(angle(value-bearing)))
                forward,left = np.array((math.cos(yaw),math.sin(yaw))),np.array((-math.sin(yaw),math.cos(yaw)))
                stage = point[:2]-1.0*forward+.15*left
                self.navigate((*stage,yaw))
            self.event('dock_delivery')
            latest = self.observe(destination,15.)
            if latest is not None:
                point = np.array(latest['position'])
            self.dock((*(point[:2]-.46*forward+.15*left),yaw))
            # The hand/payload can occlude the near pad edge. Use the recent
            # complete outline, not the biased centre of a clipped fragment.
            self.wait(lambda: self.observe(destination,45.) is not None,8.,'no recent complete destination outline')
            pad = np.array(self.observe(destination,45.)['position'])
            self.event('destination_observation',pad=pad.tolist(),age_s=self.seconds()-self.observe(destination,45.)['observed_stamp'])
            desired = self.to_pelvis(pad+np.array((0.,0.,.075)))
            hand_target = desired-self.grasp_R@self.offset
            for correction in range(2):
                _,error = self.future(self.ik_worker.submit(self.kin.solve,hand_target,dict(self.joints),self.grasp_R))
                if error <= .012:
                    break
                self.event('refine_delivery_dock',ik_error_m=error)
                self.dock((*(pad[:2]-(.42-.03*correction)*forward+.15*left),yaw))
                latest = self.observe(destination,45.)
                if latest is None:
                    raise RuntimeError('destination observation expired while refining reach')
                pad = np.array(latest['position'])
                hand_target = self.to_pelvis(pad+np.array((0.,0.,.075)))-self.grasp_R@self.offset
            self.event('lower')
            self.move_arm(hand_target,self.grasp_R)
            self.event('release')
            self.service(self.release,Trigger.Request(),attachment_after='empty')
            release_stamp = self.seconds()
            self.event('verify_place',object_id=identifier,destination_id=destination,release_ack_stamp=release_stamp)
            self.spin(2.)
            self.wait(lambda: self.placement_observed(identifier,pad,release_stamp),
                      8.,'camera did not verify post-release placement')
            self.event('retract')
            self.move_arm(np.array((.36,-.20,.37)),self.grasp_R)
            self.dock((self.pose[0]-.5*forward[0],self.pose[1]-.5*forward[1],yaw))
            self.event('restore_arm')
            rest = dict(self.joints,**self.rest_arm_positions)
            position,orientation,_,_ = self.kin.forward(rest)
            measured = self._move_arm(position,orientation,joint_solution=rest)
            if np.linalg.norm(measured-position) > .035:
                raise RuntimeError('arm did not return to its measured rest pose')
            self.service(self.authority,SetBool.Request(data=False))
            self.event('delivered',object_id=identifier,destination_id=destination)
        self.event('complete')


class TransportEvaluator(Node):
    """Independent truth-side check; its observations never drive the mission."""
    def __init__(self,pairs,people=0):
        super().__init__('transport_demo_evaluator')
        self.pairs = pairs
        self.people_requested = people
        self.aligned_samples = 0
        self.fallen = False
        self.robot_updated = -math.inf
        self.shapes = {}
        self.drops = set()
        self.objects,self.initial,self.history = {},{},{}
        self.contacts = 0
        self.minimum_clearance = math.inf
        required = ['robot',*[key for pair in pairs for key in (pair['object_id'],pair['destination_id'])]]
        if people:
            required.append('people')
        self.health = EvaluationHealth(required)
        self.create_subscription(Detection3DArray,'/evaluation/objects',self._objects,10)
        self.create_subscription(String,'/evaluation/aligned_state',self._people,10)
        self.create_subscription(Odometry,'/evaluation/odom',self._robot,10)
        self.create_subscription(String,'/transport/mission_state',self._mission,10)
        self.create_timer(.1,self.health.check)

    def _mission(self,msg):
        if json.loads(msg.data).get('phase','initializing') != 'initializing':
            self.health.start()

    def _robot(self,msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not self.health.observe('robot',stamp):
            return
        self.robot_updated = time.monotonic()
        q = msg.pose.pose.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
        self.fallen |= msg.pose.pose.position.z < .5 or max(abs(roll),abs(pitch)) > .6

    def _objects(self,msg):
        now = time.monotonic()
        for detection in msg.detections:
            identity = detection.results[0].hypothesis.class_id
            p = detection.bbox.center.position
            position = np.array((p.x,p.y,p.z))
            if not np.all(np.isfinite(position)):
                self.health.fail(f'{identity}: nonfinite pose')
                continue
            self.health.observe(identity)
            self.objects[identity] = position
            q = detection.bbox.center.orientation
            x,y,z,w = q.x,q.y,q.z,q.w
            R = np.array(((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
                          (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
                          (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))))
            size = detection.bbox.size
            self.shapes[identity] = (R,np.array((size.x,size.y,size.z)))
            if identity.startswith('box_') and p.z < .7:
                self.drops.add(identity)
            self.initial.setdefault(identity,position.copy())
            self.history.setdefault(identity,[]).append((now,position))

    def _people(self,msg):
        data = json.loads(msg.data)
        if (data.get('population_count') != self.people_requested or
                data.get('active_count') != self.people_requested or
                len(data['people']) != self.people_requested):
            self.health.fail('people: population mismatch')
            return
        if not self.health.observe('people',data['sim_time']):
            return
        self.aligned_samples += 1
        robot = data['robot']
        for person in data['people']:
            clearance = math.hypot(robot['x']-person['x'],robot['y']-person['y'])-.35-person['radius_m']
            self.minimum_clearance = min(self.minimum_clearance,clearance)
            if clearance < 0.:
                self.contacts += 1
            for identity,position in self.objects.items():
                if identity.startswith('box_') and position[2] > 1.08:
                    if math.hypot(position[0]-person['x'],position[1]-person['y']) < .095+person['radius_m']:
                        self.contacts += 1

    def report(self):
        deliveries = []
        now = time.monotonic()
        for pair in self.pairs:
            identifier,target = pair['object_id'],pair['destination_id']
            points = self.history.get(identifier,[])
            p,q = self.objects.get(identifier),self.objects.get(target)
            passed = False
            lift = travel = 0.
            if points and p is not None and q is not None:
                start = self.initial[identifier]
                lift = max(v[2]-start[2] for _,v in points)
                travel = max(float(np.linalg.norm(v[:2]-start[:2])) for _,v in points)
                recent = [v for stamp,v in points if now-stamp <= 1.5]
                recent_stamps = [stamp for stamp,_ in points if now-stamp <= 1.5]
                stable = (len(recent)>=10 and recent_stamps[-1]-recent_stamps[0] >= 1. and
                          max(float(np.linalg.norm(v-p)) for v in recent) < .02)
                R,size = self.shapes[identifier]
                pad_R,pad_size = self.shapes[target]
                corners = np.array([(x,y,z) for x in (-1.,1.) for y in (-1.,1.) for z in (-1.,1.)])*size*.5
                local = (corners@R.T+p-q)@pad_R
                inside = np.all(np.abs(local[:,:2]) <= pad_size[:2]*.5-.005)
                passed = bool(lift>.08 and travel>.8 and inside and abs(p[2]-q[2]-.055)<.035 and stable)
            deliveries.append(dict(**pair,verified=passed,lift_m=lift,travel_m=travel))
        health = self.health.report()
        healthy = health['valid'] and now-self.robot_updated < 1. and (self.people_requested == 0 or self.aligned_samples >= 10)
        required = {pair[key] for pair in self.pairs for key in ('object_id','destination_id')}
        healthy = healthy and all(self.history.get(key) and now-self.history[key][-1][0] < 1. for key in required)
        return dict(deliveries=deliveries,contact_samples=self.contacts,fallen=self.fallen,dropped_objects=sorted(self.drops),data_valid=healthy,
                    evaluation_health=health,
                    minimum_person_clearance_m=None if not math.isfinite(self.minimum_clearance) else self.minimum_clearance,
                    observed_object_ids=sorted(self.objects),
                    success=bool(deliveries) and all(d['verified'] for d in deliveries) and self.contacts==0 and not self.fallen and not self.drops and healthy)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pairs',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=420.)
    parser.add_argument('--people',type=int,default=0)
    args = parser.parse_args()
    args.pairs = json.loads(args.pairs.read_text())['pairs']
    rclpy.init()
    # Dedicated executors avoid high-rate IMU/image work starving the mission's
    # odometry and workspace callbacks in a shared thread-pool queue.
    executor = SingleThreadedExecutor()
    perception = TransportPerception()
    evaluator = TransportEvaluator(args.pairs,args.people)
    mission = TransportMission(args,executor)
    executor.add_node(mission)
    background = []
    for node in (perception,evaluator):
        worker = SingleThreadedExecutor()
        worker.add_node(node)
        thread = threading.Thread(target=worker.spin,daemon=True)
        thread.start()
        background.append((worker,thread))
    success,error = False,None
    try:
        mission.run()
        mission.spin(2.)
        success = True
    except (Exception,KeyboardInterrupt) as exc:
        error = str(exc) or 'interrupted'
        mission.get_logger().error(f'Mission failed in {mission.phase}: {error}')
    finally:
        mission.active = False
        mission._heartbeat()
        mission.ik_worker.shutdown(wait=True,cancel_futures=True)
        for worker,thread in background:
            worker.shutdown()
            thread.join()
        perception.save_snapshot(args.output.parent)
        result = evaluator.report()
        result.update(mission_complete=success,error=error,events=mission.events,
                      final_sensed_pose=mission.pose,pose_age_s=mission.seconds()-getattr(mission,'pose_stamp',0.),
                       grasp_model='capture_radius_weld',base_pinning=False,perception='registered_rgbd',
                       attachment_state=mission.attachment_state,object_may_be_held=mission.object_may_be_held)
        result['success'] = result['success'] and success
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({key:value for key,value in result.items() if key != 'events'},indent=2))
        executor.shutdown()
        for node in (mission,evaluator,perception):
            node.destroy_node()
        rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
