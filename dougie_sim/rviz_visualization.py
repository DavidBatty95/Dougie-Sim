"""Bounded ROS visualisation; no MuJoCo renderer or OpenGL context is created.

Static truth geometry is diagnostic only. Robot markers use physical primitive
geometry, rather than duplicating the heavy visual meshes into every message.
"""
import mujoco
import numpy as np
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class Visualization:
    def __init__(self, bridge, show_sources=False, map_layer='mean'):
        self.bridge = bridge
        self.map_layer = map_layer
        self.next_robot = 0.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene = bridge.node.create_publisher(MarkerArray, '/dougie/scene', qos)
        self.robot = bridge.node.create_publisher(MarkerArray, '/dougie/robot', 1)
        self.mission = bridge.node.create_publisher(MarkerArray, '/radiation/visualization', qos)
        self.dashboard = bridge.node.create_publisher(MarkerArray, '/radiation/dashboard', qos)
        sim = bridge.sim
        self.robot_ids = [i for i in range(sim.model.ngeom)
                          if sim.model.geom_bodyid[i] != 0 and sim.model.geom_group[i] != 2]
        markers = []
        for i in range(sim.model.ngeom):
            if sim.model.geom_bodyid[i] == 0:
                markers.extend(self.geom(i, 'scene'))
        if show_sources:
            for i in range(sim.model.nsite):
                if sim.model.site(i).name.startswith('radiation_source_'):
                    m = self.marker('sources', i, Marker.SPHERE)
                    self.position(m, sim.data.site_xpos[i])
                    m.scale.x = m.scale.y = m.scale.z = .18
                    m.color = ColorRGBA(**dict(zip(('r', 'g', 'b', 'a'), map(float, sim.model.site_rgba[i]))))
                    markers.append(m)
        self.scene.publish(MarkerArray(markers=markers))

    def marker(self, namespace, index, kind):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.bridge.stamp()
        m.ns, m.id, m.type = namespace, index, kind
        m.pose.orientation.w = 1.
        m.color = ColorRGBA(r=.2, g=.8, b=.9, a=1.)
        return m

    @staticmethod
    def position(marker, xyz):
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, xyz)

    def geom(self, index, namespace):
        sim = self.bridge.sim
        kind = sim.model.geom_type[index]
        types = {mujoco.mjtGeom.mjGEOM_BOX: Marker.CUBE,
                 mujoco.mjtGeom.mjGEOM_PLANE: Marker.CUBE,
                 mujoco.mjtGeom.mjGEOM_SPHERE: Marker.SPHERE,
                 mujoco.mjtGeom.mjGEOM_ELLIPSOID: Marker.SPHERE,
                 mujoco.mjtGeom.mjGEOM_CYLINDER: Marker.CYLINDER,
                 mujoco.mjtGeom.mjGEOM_CAPSULE: Marker.CYLINDER}
        if kind not in types:
            return []
        m = self.marker(namespace, index*3, types[kind])
        self.position(m, sim.data.geom_xpos[index])
        q = np.empty(4)
        mujoco.mju_mat2Quat(q, sim.data.geom_xmat[index])
        m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = map(float, q)
        size = sim.model.geom_size[index]
        scale = 2*size.copy()
        if kind == mujoco.mjtGeom.mjGEOM_PLANE:
            scale = [30., 30., .02]
            m.pose.position.z -= .01
        elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            scale = [2*size[0]]*3
        elif kind in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
            scale = [2*size[0], 2*size[0], 2*size[1]]
        m.scale.x, m.scale.y, m.scale.z = map(float, scale)
        m.color = ColorRGBA(**dict(zip(('r', 'g', 'b', 'a'), map(float, sim.model.geom_rgba[index]))))
        result = [m]
        if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
            axis = sim.data.geom_xmat[index].reshape(3, 3)[:, 2]
            for j, sign in enumerate((-1, 1), 1):
                cap = self.marker(namespace, index*3+j, Marker.SPHERE)
                self.position(cap, sim.data.geom_xpos[index]+sign*size[1]*axis)
                cap.scale.x = cap.scale.y = cap.scale.z = float(2*size[0])
                cap.color = m.color
                result.append(cap)
        return result

    def tick(self):
        stamp = self.bridge.sim.data.time
        if stamp+1e-9 < self.next_robot:
            return
        self.next_robot = stamp+.05
        if self.robot.get_subscription_count():
            self.robot.publish(MarkerArray(markers=[m for i in self.robot_ids for m in self.geom(i, 'robot')]))

    def update(self, volume, metrics, trail):
        markers = []
        if volume is not None:
            xyz, mean, std, _ = volume
            m = self.marker('gp', 0, Marker.SPHERE_LIST)
            m.scale.x = m.scale.y = m.scale.z = .12
            values = np.clip(std/2 if self.map_layer == 'uncertainty' else np.log1p(mean)/np.log(101), 0, 1)
            m.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in xyz]
            m.colors = [ColorRGBA(r=float(v), g=float(1-v), b=.3, a=.65) for v in values]
            markers.append(m)
        for index, (name, points, color) in enumerate((
                ('planned path', metrics.get('planned_path', []), ColorRGBA(r=.1, g=1., b=.2, a=1.)),
                ('travelled path', trail, ColorRGBA(r=.2, g=.6, b=1., a=1.)),
                ('return route', metrics.get('return_path', []), ColorRGBA(r=1., g=.7, b=.15, a=.8)))):
            m = self.marker(name, index, Marker.LINE_STRIP)
            m.scale.x, m.color = .04, color
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]+.08)) for p in points]
            if len(m.points) < 2:
                m.action = Marker.DELETE
            markers.append(m)
        target = self.marker('exploration target', 0, Marker.SPHERE)
        target.scale.x = target.scale.y = target.scale.z = .24
        target.color = ColorRGBA(r=.1, g=1., b=.2, a=1.)
        xyz = metrics.get('selected_target')
        if xyz is None:
            target.action = Marker.DELETE
        else:
            self.position(target, (xyz[0], xyz[1], xyz[2]+.15))
        markers.append(target)
        if volume is None:
            m = self.marker('gp', 0, Marker.SPHERE_LIST)
            m.action = Marker.DELETE
            markers.append(m)
        self.mission.publish(MarkerArray(markers=markers))

    def update_dashboard(self, metrics, sample, budget, rate_limit):
        """Small 2 Hz telemetry message, independent of the expensive map payload."""
        dose = sample.dose_usv if sample is not None else 0.
        rate = sample.rate_usv_h if sample is not None else 0.
        fraction = dose/budget if budget > 0 else 0.
        state = metrics.get('state', 'initial_survey')
        warning = fraction >= .8 or rate > rate_limit or state.startswith('return')
        color = ColorRGBA(r=1. if warning else .3, g=.7 if warning else 1., b=.2 if warning else .9, a=1.)
        text = self.marker('status', 0, Marker.TEXT_VIEW_FACING)
        text.pose.position.z = 2.5
        text.scale.z = .18
        text.color = color
        text.text = (f"DOUGIE | {state.replace('_', ' ').upper()} | {self.bridge.sim.data.time:.1f} s\n"
                     f"{metrics.get('decision_reason', '').replace('_', ' ')}\n"
                     f"Measured dose {dose:.3f} / {budget:g} uSv ({fraction:.0%})\n"
                     f"Measured rate {rate:.1f} / {rate_limit:g} uSv/h\n"
                     f"Estimated return reserve {metrics.get('return_reserve_usv', 0.):.3f} uSv\n"
                     f"Observed surface proxy {metrics.get('mapped_surface_area_m2', 0.):.1f} m^2")
        legend = self.marker('legend', 0, Marker.TEXT_VIEW_FACING)
        self.position(legend, (0., 0., 1.45))
        legend.scale.z = .12
        legend.color = ColorRGBA(r=.8, g=.85, b=.95, a=1.)
        layer = ('GP uncertainty: green low / red high (std log-rate 0..2)'
                 if self.map_layer == 'uncertainty' else 'GP mean: green low / red high (log scale 0..100 uSv/h)')
        legend.text = layer+'\nGreen: plan + target | Blue: travelled | Amber: estimated return'
        self.dashboard.publish(MarkerArray(markers=[text, legend]))
