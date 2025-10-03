#!/usr/bin/env python3
# Requires: ezdxf  (pip install ezdxf)
import math
import csv
from pathlib import Path
from typing import List, Tuple

import ezdxf
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32


# Map DXF entity types to small integer tags (used in orientation channels in the original logic)
ENTITY_TAG = {
    'LINE': 1,
    'LWPOLYLINE': 2,
    'POLYLINE': 3,
    'ARC': 4,
    'CIRCLE': 5,
    'SPLINE': 6,
}


class DXFPathEmitterCx(Node):
    def __init__(self) -> None:
        super().__init__('dxf_parser_node_cx')

        # ---------------- Params ----------------
        self.declare_parameter('dxf_file', '/home/hsdb/ros2_ws_2502/src/ms2r1p/ms2r1p/dxfs/traj_shape_amg.dxf')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('csv_out', '/home/hsdb/ros2_ws_2502/src/ms2r1p/ms2r1p/dxfs/dxf_waypoints_v5.csv')

        self._dxf_path = Path(self.get_parameter('dxf_file').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._csv_out = Path(self.get_parameter('csv_out').value)

        # ---------------- Pub ----------------
        self._pub_path = self.create_publisher(PathMsg, 'dxf_path', 10)
        self._pub_pc = self.create_publisher(PointCloud, 'dxf_pointcloud', 10)

        # ---------------- Run once on startup ----------------
        if not self._dxf_path.exists():
            self.get_logger().error(f'DXF not found: {self._dxf_path}')
            return

        self.get_logger().info(f'Parsing DXF: {self._dxf_path}')
        waypoints = self._read_dxf(self._dxf_path)

        self._export_csv(waypoints, self._csv_out)

        # Preserve original scaling: Path uses /100.0, PointCloud uses /10.0
        path_msg = self._to_path_msg(waypoints, self._frame_id, scale_xy=1/100.0, scale_z=1/100.0)
        pc_msg = self._to_pointcloud(waypoints, self._frame_id, scale_xyz=1/10.0)

        self._pub_path.publish(path_msg)
        self._pub_pc.publish(pc_msg)

        self.get_logger().info(f'Published path with {len(path_msg.poses)} poses; pointcloud with {len(pc_msg.points)} points.')
        self.get_logger().info(f'Waypoints exported to: {self._csv_out}')

    # ---------------- DXF parsing ----------------

    def _read_dxf(self, path: Path) -> List[Tuple[float, float, float, str, int]]:
        """Return list of (x,y,z,entity_type,entity_id) sampled from DXF entities."""
        doc = ezdxf.readfile(str(path))
        msp = doc.modelspace()

        wps: List[Tuple[float, float, float, str, int]] = []
        ent_id = 0

        for e in msp:
            etype = e.dxftype()
            try:
                if etype == 'LINE':
                    wps.append((e.dxf.start.x, e.dxf.start.y, 0.0, etype, ent_id))
                    wps.append((e.dxf.end.x,   e.dxf.end.y,   0.0, etype, ent_id))

                elif etype == 'LWPOLYLINE':
                    for x, y, *_ in e.get_points():
                        wps.append((x, y, 0.0, etype, ent_id))

                elif etype == 'POLYLINE':
                    for p in e.points():
                        wps.append((p.x, p.y, 0.0, etype, ent_id))

                elif etype == 'ARC':
                    for px, py in self._sample_arc(e.dxf.center.x, e.dxf.center.y, e.dxf.radius,
                                                   e.dxf.start_angle, e.dxf.end_angle, n=30):
                        wps.append((px, py, 0.0, etype, ent_id))

                elif etype == 'CIRCLE':
                    for px, py in self._sample_arc(e.dxf.center.x, e.dxf.center.y, e.dxf.radius,
                                                   0.0, 360.0, n=30):
                        wps.append((px, py, 0.0, etype, ent_id))

                elif etype == 'SPLINE':
                    for vec in e.flattening(0.5):
                        wps.append((vec.x, vec.y, 0.0, etype, ent_id))

                else:
                    self.get_logger().warn(f'Unrecognized entity: {etype}; skipping.')
            finally:
                ent_id += 1

        self.get_logger().info(f'Extracted {len(wps)} waypoints.')
        return wps

    @staticmethod
    def _sample_arc(cx: float, cy: float, r: float, a0_deg: float, a1_deg: float, n: int = 30):
        a0 = math.radians(a0_deg)
        a1 = math.radians(a1_deg)
        if a1 < a0:
            a1 += 2.0 * math.pi
        for i in range(n + 1):
            t = a0 + (a1 - a0) * i / n
            yield cx + r * math.cos(t), cy + r * math.sin(t)

    # ---------------- Message assembly ----------------

    def _to_path_msg(self, pts, frame_id: str, scale_xy: float = 1.0, scale_z: float = 1.0) -> PathMsg:
        msg = PathMsg()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y, z, etype, eid in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x * scale_xy
            ps.pose.position.y = y * scale_xy
            ps.pose.position.z = z * scale_z

            # Embed metadata similarly to the original (entity id/type)
            ps.pose.orientation.x = float(eid)
            ps.pose.orientation.y = float(ENTITY_TAG.get(etype, 0))
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0

            msg.poses.append(ps)
        return msg

    def _to_pointcloud(self, pts, frame_id: str, scale_xyz: float = 1.0) -> PointCloud:
        pc = PointCloud()
        pc.header.frame_id = frame_id
        pc.header.stamp = self.get_clock().now().to_msg()
        for x, y, z, _t, _id in pts:
            pc.points.append(Point32(x=x * scale_xyz, y=y * scale_xyz, z=z * scale_xyz))
        return pc

    # ---------------- CSV ----------------

    def _export_csv(self, pts, out_path: Path) -> None:
        try:
            out_path.parent.mkdir(parents=True, exist_ok=True)
            with out_path.open('w', newline='') as fh:
                w = csv.writer(fh)
                w.writerow(['x', 'y', 'z', 'entity_type', 'entity_id'])
                for x, y, z, etype, eid in pts:
                    w.writerow([x, y, z, etype, eid])
        except Exception as exc:
            self.get_logger().error(f'CSV export failed: {exc}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DXFPathEmitterCx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
