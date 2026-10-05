"""Known-colour box/pad detections from registered RGB-D and measured robot state."""
import json
import math
import struct
import zlib
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,JointState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException

from .onboard_lidar_odometry import lidar_extrinsics
from .arm_kinematics import rpy_matrix


def components(mask):
    remaining = set(map(tuple,np.argwhere(mask)))
    while remaining:
        first = remaining.pop()
        queue,group = [first],[first]
        while queue:
            y,x = queue.pop()
            for dy,dx in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
                point = (y+dy,x+dx)
                if point in remaining:
                    remaining.remove(point)
                    queue.append(point)
                    group.append(point)
        if len(group) >= 6:
            yield np.array(group)


def colour_detections(rgb,depth,K,origin,R,table_height=.95,diagnostics=None):
    """Return geometric candidates in a level robot frame; no layout lookup."""
    result = []
    for channel,colour in enumerate(('red','green','blue')):
        dominant = rgb[:,:,channel].astype(float)
        others = np.max(np.delete(rgb,channel,axis=2),axis=2)
        mask = (dominant > 65) & (dominant > 1.65*others) & (depth > .15) & (depth < 8.)
        for pixels in components(mask):
            v,u = pixels[:,0],pixels[:,1]
            z = depth[v,u]
            points = np.column_stack(((u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z))@R.T+origin
            low,high = np.quantile(points,.02,axis=0),np.quantile(points,.98,axis=0)
            # On a filled destination, separate the raised box from the outline.
            subsets = [points]
            if high[2]-low[2] > .05 and max(high[:2]-low[:2]) > .18:
                subsets = [points[points[:,2] < low[2]+.02],points[points[:,2] > low[2]+.025]]
            for subset in subsets:
                if len(subset) < 6:
                    continue
                lo,hi = np.min(subset,axis=0),np.max(subset,axis=0)
                extent = hi-lo
                centre = .5*(lo+hi)
                if diagnostics is not None:
                    diagnostics.append(dict(colour=colour,low=lo.tolist(),high=hi.tolist(),pixels=len(subset)))
                if .045 < max(extent[:2]) < .19 and extent[2] < .18:
                    kind = 'box'
                    centre[2] = hi[2]-.055
                    if centre[2] < table_height+.02:
                        continue
                elif .30 < max(extent[:2]) < .58 and min(extent[:2]) > .30 and extent[2] < .045:
                    kind = 'pad'
                else:
                    continue
                detection = dict(id=('box_' if kind=='box' else 'target_')+colour,
                                 kind=kind,colour=colour,position=centre.tolist(),extent=extent.tolist(),pixels=len(subset))
                if kind == 'pad':
                    # Minimum-area rectangle supplies the observed square's
                    # edge directions, allowing a face-normal table approach.
                    best = None
                    for yaw in np.linspace(0.,math.pi/2,91)[:-1]:
                        axes = np.array(((math.cos(yaw),-math.sin(yaw)),(math.sin(yaw),math.cos(yaw))))
                        projected = subset[:,:2]@axes
                        bounds = np.min(projected,axis=0),np.max(projected,axis=0)
                        area = float(np.prod(bounds[1]-bounds[0]))
                        if best is None or area < best[0]:
                            best = (area,float(yaw),(.5*(bounds[0]+bounds[1]))@axes.T)
                    detection['yaw'] = best[1]
                    detection['position'][:2] = best[2].tolist()
                result.append(detection)
    return result


class TransportPerception(Node):
    def __init__(self):
        super().__init__('transport_perception')
        qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        image_qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
        self.info = None
        self.colour = {}
        self.depth = {}
        self.imu,self.joints = deque(maxlen=200),deque(maxlen=200)
        self.objects = {}
        self.snapshot = None
        self.process_status = 'waiting_for_images'
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,self)
        self.pub = self.create_publisher(String,'/transport/observations',10)
        self.create_subscription(CameraInfo,'/camera/camera/color/camera_info',self._info,10)
        self.create_subscription(Image,'/camera/camera/color/image_raw',self._colour,image_qos)
        self.create_subscription(Image,'/camera/camera/aligned_depth_to_color/image_raw',self._depth,image_qos)
        self.create_subscription(Vector3Stamped,'/onboard/measured_rpy',lambda msg:self.imu.append((self.stamp(msg.header),(msg.vector.x,msg.vector.y,msg.vector.z))),qos)
        self.create_subscription(JointState,'/joint_states',lambda msg:self.joints.append((self.stamp(msg.header),dict(zip(msg.name,msg.position)))),qos)
        self.create_timer(.2,self._publish)

    def seconds(self):
        return self.get_clock().now().nanoseconds*1e-9

    @staticmethod
    def stamp(header):
        return header.stamp.sec+header.stamp.nanosec*1e-9

    def _info(self,msg):
        self.info = np.array(msg.k).reshape(3,3)

    def _colour(self,msg):
        if msg.encoding == 'rgb8':
            self.colour[self.stamp(msg.header)] = msg
            self.colour = dict(list(self.colour.items())[-5:])
            self._process(self.stamp(msg.header))

    def _depth(self,msg):
        self.depth[self.stamp(msg.header)] = msg
        self.depth = dict(list(self.depth.items())[-5:])
        self._process(self.stamp(msg.header))

    def _process(self,stamp):
        msg = self.depth.get(stamp)
        if msg is None:
            return
        colour = self.colour.get(stamp)
        if colour is None or self.info is None or not self.imu or not self.joints or msg.encoding != '16UC1':
            self.process_status = 'waiting_for_registered_pair_or_robot_state'
            return
        imu = min(self.imu,key=lambda p:abs(p[0]-stamp))
        joints = min(self.joints,key=lambda p:abs(p[0]-stamp))
        if abs(imu[0]-stamp)>.2 or abs(joints[0]-stamp)>.2:
            self.process_status = f'state_gap: imu={abs(imu[0]-stamp):.3f}, joints={abs(joints[0]-stamp):.3f}'
            return
        try:
            transform = self.tf.lookup_transform('odom','base_footprint',rclpy.time.Time.from_msg(msg.header.stamp)).transform
        except TransformException as exc:
            self.process_status = str(exc)
            return
        self.process_status = 'processed'
        rgb = np.frombuffer(colour.data,dtype=np.uint8).reshape(colour.height,colour.step)[:,:colour.width*3].reshape(colour.height,colour.width,3)
        depth = np.frombuffer(msg.data,dtype='<u2').reshape(msg.height,msg.step//2)[:,:msg.width].astype(float)*.001
        origin,lidar_R = lidar_extrinsics(imu[1],joints[1])
        torso_R = lidar_R@np.diag((1.,-1.,-1.))
        origin = origin+torso_R@np.array((0.,.035,0.))
        optical_R = np.array(((0.,0.,1.),(-1.,0.,0.),(0.,-1.,0.)))
        q = transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        world_R = rpy_matrix((0.,0.,yaw))
        offset = np.array((transform.translation.x,transform.translation.y,0.))
        diagnostics = []
        detections = colour_detections(rgb,depth,self.info,origin,torso_R@optical_R,diagnostics=diagnostics)
        self.snapshot = dict(rgb=rgb.copy(),depth=depth.copy(),stamp=stamp,origin=origin.tolist(),
                             rpy=list(imu[1]),regions=diagnostics,detections=detections)
        for item in detections:
            item['position'] = (world_R@item['position']+offset).tolist()
            if 'yaw' in item:
                item['yaw'] += yaw
            item['observed_stamp'] = stamp
            self.objects[item['id']] = item
        self.colour.pop(stamp,None)
        self.depth.pop(stamp,None)

    def save_snapshot(self,directory):
        if self.snapshot is None:
            return
        directory.mkdir(parents=True,exist_ok=True)
        snapshot = dict(self.snapshot)
        snapshot.update(process_status=self.process_status,colour_stamps=list(self.colour),depth_stamps=list(self.depth),
                        latest_imu_stamp=self.imu[-1][0] if self.imu else None,
                        latest_joint_stamp=self.joints[-1][0] if self.joints else None,
                        snapshot_written_at=self.seconds())
        rgb,depth = snapshot.pop('rgb'),snapshot.pop('depth')
        np.savez_compressed(directory/'rgbd.npz',rgb=rgb,depth=depth,K=self.info)
        height,width,_ = rgb.shape
        def chunk(kind,data):
            return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
        raw = b''.join(b'\0'+row.tobytes() for row in rgb)
        png = b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',width,height,8,2,0,0,0))
        (directory/'rgb.png').write_bytes(png+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))
        (directory/'perception_snapshot.json').write_text(json.dumps(snapshot,indent=2)+'\n')

    def _publish(self):
        now = self.seconds()
        for stamp in list(self.depth):
            if 0. <= now-stamp <= 1.5:
                self._process(stamp)
        self.objects = {key:value for key,value in self.objects.items() if now-value['observed_stamp'] < (60. if value['kind']=='pad' else 30.)}
        self.pub.publish(String(data=json.dumps(dict(frame='odom',stamp=now,objects=list(self.objects.values()),source='registered_rgbd_joint_imu'))))
