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
from .radiation_display import CLOUD_ALPHA


class Visualization:
    def __init__(self, bridge, show_sources=False, map_layer='mean', cutaway=False,
                 highlights=None, field_display='detected'):
        from .radiation_display import RadiationHighlights
        self.bridge = bridge
        self.highlights = highlights or RadiationHighlights()
        self.field_display = field_display
        self.map_layer = map_layer
        self.status_height = 6. if cutaway else 2.5
        self.next_robot = 0.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scene = bridge.node.create_publisher(MarkerArray, '/dougie/scene', qos)
        self.robot = bridge.node.create_publisher(MarkerArray, '/dougie/robot', 1)
        self.mission = bridge.node.create_publisher(MarkerArray, '/radiation/visualization', qos)
        self.dashboard = bridge.node.create_publisher(MarkerArray, '/radiation/dashboard', qos)
        self.detections = bridge.node.create_publisher(MarkerArray, '/radiation/detections', qos)
        self.landmarks = bridge.node.create_publisher(MarkerArray, '/radiation/landmarks', qos)
        sim = bridge.sim
        self.robot_ids = [i for i in range(sim.model.ngeom)
                          if sim.model.geom_bodyid[i] != 0 and sim.model.geom_group[i] != 2]
        markers = []
        for i in range(sim.model.ngeom):
            if sim.model.geom_bodyid[i] == 0:
                geometry = self.geom(i, 'scene')
                if cutaway and sim.model.geom(i).name.startswith(('outer_', 'upper_floor_', 'room_partition_1_')):
                    for marker in geometry:
                        marker.color.a = .22
                markers.extend(geometry)
        if show_sources:
            for i in range(sim.model.nsite):
                if sim.model.site(i).name.startswith('radiation_source_'):
                    m = self.marker('sources', i, Marker.SPHERE)
                    self.position(m, sim.data.site_xpos[i])
                    m.scale.x = m.scale.y = m.scale.z = .5
                    m.color = ColorRGBA(**dict(zip(('r', 'g', 'b', 'a'), map(float, sim.model.site_rgba[i]))))
                    markers.append(m)
                    arrow = self.marker('source_arrows', i, Marker.ARROW)
                    p = sim.data.site_xpos[i]
                    annotation_z = self.source_annotation_height(p)
                    beacon = self.marker('source_beacons', i, Marker.SPHERE)
                    self.position(beacon, (p[0], p[1], annotation_z))
                    beacon.scale.x = beacon.scale.y = beacon.scale.z = .45
                    beacon.color = m.color
                    markers.append(beacon)
                    arrow.points = [Point(x=float(p[0]), y=float(p[1]), z=annotation_z),
                                    Point(x=float(p[0]), y=float(p[1]), z=float(p[2]+.2))]
                    arrow.scale.x, arrow.scale.y, arrow.scale.z = .08, .22, .22
                    arrow.color = m.color
                    markers.append(arrow)
                    label = self.marker('source_labels', i, Marker.TEXT_VIEW_FACING)
                    self.position(label, (p[0], p[1], annotation_z+.55))
                    label.scale.z = .26
                    label.color = ColorRGBA(r=1., g=.9, b=.65, a=1.)
                    kind = 'LEAK' if sim.model.nuser_site >= 3 and sim.model.site_user[i, 2] == 1 else 'HOTSPOT'
                    label.text = (kind+': '+sim.model.site(i).name.removeprefix('radiation_source_')+
                                  f'\nTRUE SOURCE / diagnostic / z={p[2]:.2f}m')
                    markers.append(label)
                    ring = self.marker('source_rings', i, Marker.LINE_STRIP)
                    ring.scale.x, ring.color = .055, m.color
                    ring.points = [Point(x=float(p[0]+.6*np.cos(a)), y=float(p[1]+.6*np.sin(a)), z=annotation_z)
                                   for a in np.linspace(0., 2*np.pi, 49)]
                    markers.append(ring)
        self.scene.publish(MarkerArray(markers=markers))

    def update_landmarks(self, observations):
        markers = []
        for index, observation in enumerate(sorted(observations, key=lambda item: item['identifier'])):
            marker = self.marker('discovered_landmarks', index, Marker.CUBE)
            self.position(marker, observation['position'])
            marker.scale.x = marker.scale.y = marker.scale.z = .16
            marker.color = ColorRGBA(r=.15, g=.9, b=.3 if observation['inspected'] else .95, a=1.)
            markers.append(marker)
            label = self.marker('landmark_labels', index, Marker.TEXT_VIEW_FACING)
            self.position(label, np.asarray(observation['position'])+[0., 0., .25])
            label.scale.z = .15
            label.text = 'TARGET '+observation['identifier']+(' / inspected' if observation['inspected'] else ' / discovered')
            markers.append(label)
        self.landmarks.publish(MarkerArray(markers=markers))

    def source_annotation_height(self, point):
        """Lift diagnostic labels above equipment enclosing an emitter.

        The arrow still points to its actual position; the elevated bead is an
        annotation, not another source or an autonomy input.
        """
        sim = self.bridge.sim
        height = float(point[2]+.75)
        for index in range(sim.model.ngeom):
            if sim.model.geom_bodyid[index] != 0 or sim.model.geom_group[index] == 2:
                continue
            kind, size = sim.model.geom_type[index], sim.model.geom_size[index]
            if kind in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
                half = size.copy()
            elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
                half = np.repeat(size[0], 3)
            elif kind in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
                half = np.array([size[0], size[0], size[1]+(size[0] if kind == mujoco.mjtGeom.mjGEOM_CAPSULE else 0.)])
            else:
                continue
            extent = np.abs(sim.data.geom_xmat[index].reshape(3, 3))@half
            centre = sim.data.geom_xpos[index]
            if np.all(np.abs(point-centre) <= extent+.01):
                height = max(height, float(centre[2]+extent[2]+.35))
        return height

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
            xyz, colors = self.highlights.cloud(volume, self.field_display, self.map_layer)
            m = self.marker('gp', 0, Marker.SPHERE_LIST)
            # RViz list-marker transparency is controlled by the marker alpha;
            # per-point alpha is not supported consistently across renderers.
            m.color = ColorRGBA(r=1., g=1., b=1., a=CLOUD_ALPHA)
            m.scale.x, m.scale.y, m.scale.z = .68, .68, .44
            m.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in xyz]
            m.colors = [ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.) for r, g, b, _ in colors]
            if not len(xyz):
                m.action = Marker.DELETE
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
        if sample is not None:
            self.highlights.observe(sample)
        xyz, measured_rates = self.highlights.confirmed()
        evidence = self.marker('above_background_counts', 0, Marker.CUBE_LIST)
        evidence.color = ColorRGBA(r=1., g=1., b=1., a=1.)
        evidence.scale.x = evidence.scale.y = evidence.scale.z = .14
        evidence.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in xyz]
        evidence.colors = [ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
                           for r, g, b, a in self.highlights.colors(measured_rates, .95)]
        if not len(xyz):
            evidence.action = Marker.DELETE
        self.detections.publish(MarkerArray(markers=[evidence]))
        rate = sample.rate_usv_h if sample is not None else 0.
        fraction = dose/budget if budget > 0 else 0.
        state = metrics.get('state', 'initial_survey')
        warning = fraction >= .8 or rate > rate_limit or state.startswith('return')
        color = ColorRGBA(r=1. if warning else .3, g=.7 if warning else 1., b=.2 if warning else .9, a=1.)
        text = self.marker('status', 0, Marker.TEXT_VIEW_FACING)
        text.pose.position.z = self.status_height
        text.scale.z = .18
        text.color = color
        text.text = (f"DOUGIE | {state.replace('_', ' ').upper()} | {self.bridge.sim.data.time:.1f} s\n"
                     f"{metrics.get('decision_reason', '').replace('_', ' ')}\n"
                     f"Measured dose {dose:.3f} / {budget:g} uSv ({fraction:.0%})\n"
                     f"Measured rate {rate:.1f} / {rate_limit:g} uSv/h\n"
                     f"Estimated return reserve {metrics.get('return_reserve_usv', 0.):.3f} uSv\n"
                     f"Observed surface proxy {metrics.get('mapped_surface_area_m2', 0.):.1f} m^2")
        legend = self.marker('legend', 0, Marker.TEXT_VIEW_FACING)
        self.position(legend, (0., 0., self.status_height-1.05))
        legend.scale.z = .12
        legend.color = ColorRGBA(r=.8, g=.85, b=.95, a=1.)
        layer = ('GP uncertainty (std log-rate 0..2)' if self.map_layer == 'uncertainty' else
                  'Radiation excess: yellow low / red high | translucent cloud = GP display, cubes = count evidence')
        if self.field_display == 'detected':
            layer += f'\nAbove background {self.highlights.background:g} uSv/h, near confirmed detections; blank is NOT proof of safety'
        legend.text = layer+'\nGreen: plan + target | Blue: travelled | Amber: estimated return'
        self.dashboard.publish(MarkerArray(markers=[text, legend]))
