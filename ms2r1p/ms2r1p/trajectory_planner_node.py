#!/usr/bin/env python3
import csv
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped


Waypoint = Tuple[float, float, float, str, int]  # (x, y, z, entity_type, entity_id)


class TrajectoryPlannerCx(Node):
    def __init__(self) -> None:
        super().__init__('trajectory_planner_cx')

        # ---------- Parameters ----------
        self.declare_parameter('csv_waypoints',
            '/home/hsdb/ros2_ws_2502/src/ms2r1p/ms2r1p/dxfs/dxf_waypoints_v5.csv')
        self.declare_parameter('interpolation_type', 'cubic')  # linear | cubic | quintic
        # Timing knobs (seconds)
        self.declare_parameter('T_segment', 5.0)        # per edge
        self.declare_parameter('T_transition', 3.0)     # between entities (XY at z_hold)
        self.declare_parameter('T_prism_up', 2.0)       # lift duration
        self.declare_parameter('T_prism_down', 2.0)     # lower duration
        # Densities (number of samples)
        self.declare_parameter('N_line', 50)
        self.declare_parameter('N_poly', 50)
        self.declare_parameter('N_arc', 30)
        self.declare_parameter('N_circle', 30)
        self.declare_parameter('N_spline', 15)
        self.declare_parameter('N_transition', 30)
        self.declare_parameter('N_prism', 20)

        self._csv_path = Path(str(self.get_parameter('csv_waypoints').value))
        self._itype = str(self.get_parameter('interpolation_type').value)
        self._Tseg = float(self.get_parameter('T_segment').value)
        self._Txy = float(self.get_parameter('T_transition').value)
        self._Tup = float(self.get_parameter('T_prism_up').value)
        self._Tdn = float(self.get_parameter('T_prism_down').value)
        self._N_line = int(self.get_parameter('N_line').value)
        self._N_poly = int(self.get_parameter('N_poly').value)
        self._N_arc = int(self.get_parameter('N_arc').value)
        self._N_circle = int(self.get_parameter('N_circle').value)
        self._N_spline = int(self.get_parameter('N_spline').value)
        self._N_trans = int(self.get_parameter('N_transition').value)
        self._N_prism = int(self.get_parameter('N_prism').value)

        self._z_hold = 0.072  # same constant used elsewhere in your stack

        self._pub_path = self.create_publisher(PathMsg, 'planned_trajectory', 10)

        # ---------- Load, Plan, Publish ----------
        wps = self._load_waypoints(self._csv_path)
        self.get_logger().info(f'Waypoints cargados: {len(wps)}')

        final_pts = self._plan(wps)
        self.get_logger().info(f'Puntos interpolados generados: {len(final_pts)}')

        self._publish_path(final_pts)
        self._export_interpolated(final_pts,
            '/home/hsdb/ros2_ws_2502/src/ms2r1p/ms2r1p/dxfs/planned_trajectory.csv')

    # ================== CSV I/O ==================

    def _load_waypoints(self, path: Path) -> List[Waypoint]:
        out: List[Waypoint] = []
        try:
            with path.open('r', newline='') as fh:
                reader = csv.DictReader(fh)
                for row in reader:
                    x = float(row['x']); y = float(row['y']); z = float(row['z'])
                    et = str(row['entity_type']); eid = int(row['entity_id'])
                    out.append((x, y, z, et, eid))
        except Exception as exc:
            self.get_logger().error(f'Error reading waypoints CSV: {exc}')
        return out

    def _export_interpolated(self, pts: Sequence[Tuple[float,float,float,str]], out_path: str) -> None:
        try:
            p = Path(out_path)
            p.parent.mkdir(parents=True, exist_ok=True)
            with p.open('w', newline='') as fh:
                w = csv.writer(fh)
                w.writerow(['x', 'y', 'z', 'entity_type'])
                for x, y, z, etype in pts:
                    w.writerow([x, y, z, etype])
            self.get_logger().info(f' Puntos interpolados exportados a: {out_path}')
        except Exception as exc:
            self.get_logger().error(f' Error al exportar CSV: {exc}')

    # ================== Interpolation ==================

    def _interp_linear(self, q0: float, qf: float, T: float, N: int) -> np.ndarray:
        t = np.linspace(0.0, T, max(2, N))
        return q0 + (qf - q0) * (t / max(T, 1e-9))

    def _interp_cubic(self, q0: float, qf: float, T: float, N: int, v0: float = 0.0, vf: float = 0.0) -> np.ndarray:
        t = np.linspace(0.0, T, max(2, N))
        a0 = q0
        a1 = v0
        a2 = (3 * (qf - q0) / (T**2)) - (2*v0 + vf) / T
        a3 = (2 * (q0 - qf) / (T**3)) + (v0 + vf) / (T**2)
        return a0 + a1*t + a2*(t**2) + a3*(t**3)

    def _interp_quintic(self, q0: float, qf: float, T: float, N: int,
                         v0: float = 0.0, vf: float = 0.0, a0: float = 0.0, af: float = 0.0) -> np.ndarray:
        t = np.linspace(0.0, T, max(2, N))
        c0 = q0
        c1 = v0
        c2 = a0 / 2.0
        c3 = (20*(qf - q0) - (8*vf + 12*v0)*T - (3*a0 - af)*(T**2)) / (2*(T**3))
        c4 = (30*(q0 - qf) + (14*vf + 16*v0)*T + (3*a0 - 2*af)*(T**2)) / (2*(T**4))
        c5 = (12*(qf - q0) - (6*vf + 6*v0)*T - (a0 - af)*(T**2)) / (2*(T**5))
        return c0 + c1*t + c2*(t**2) + c3*(t**3) + c4*(t**4) + c5*(t**5)

    def _interp(self, q0: float, qf: float, T: float, N: int) -> np.ndarray:
        if self._itype == 'linear':
            return self._interp_linear(q0, qf, T, N)
        elif self._itype == 'quintic':
            return self._interp_quintic(q0, qf, T, N)
        else:
            return self._interp_cubic(q0, qf, T, N)

    # ================== Planning ==================

    def _density_for(self, entity_type: str) -> int:
        if entity_type in ('LINE', 'LWPOLYLINE', 'POLYLINE'):
            return self._N_line if entity_type == 'LINE' else self._N_poly
        if entity_type == 'CIRCLE':
            return self._N_circle
        if entity_type == 'ARC':
            return self._N_arc
        if entity_type == 'SPLINE':
            return self._N_spline
        return max(20, self._N_poly)

    def _plan(self, wps: List[Waypoint]) -> List[Tuple[float, float, float, str]]:
        final_pts: List[Tuple[float, float, float, str]] = []
        if not wps:
            return final_pts

        entity_start = wps[0]
        last_xyz = (entity_start[0], entity_start[1], entity_start[2])
        last_eid = entity_start[4]

        for i in range(len(wps) - 1):
            p1, p2 = wps[i], wps[i+1]
            id1, id2 = p1[4], p2[4]
            e_type = p1[3]

            if id1 == id2:
                N = self._density_for(e_type)
                x_vals = self._interp(p1[0], p2[0], self._Tseg, N)
                y_vals = self._interp(p1[1], p2[1], self._Tseg, N)
                z_vals = self._interp(p1[2], p2[2], self._Tseg, N)
                for x, y, z in zip(x_vals, y_vals, z_vals):
                    final_pts.append((x, y, z, e_type))
                last_xyz = (final_pts[-1][0], final_pts[-1][1], final_pts[-1][2])
            else:
                # Close entity back to its start if not already there
                if (p1[0], p1[1], p1[2]) != (entity_start[0], entity_start[1], entity_start[2]):
                    N_close = self._density_for(e_type)
                    x_vals = self._interp(p1[0], entity_start[0], self._Tseg, N_close)
                    y_vals = self._interp(p1[1], entity_start[1], self._Tseg, N_close)
                    z_vals = self._interp(p1[2], entity_start[2], self._Tseg, N_close)
                    for x, y, z in zip(x_vals, y_vals, z_vals):
                        final_pts.append((x, y, z, e_type))
                    self.get_logger().info(f' Cierre de entidad {id1}: +{N_close} pts')
                    last_xyz = (final_pts[-1][0], final_pts[-1][1], final_pts[-1][2])

                # Prism up from current z to z_hold
                if final_pts:
                    x_last, y_last, z_last = last_xyz
                    z_vals = self._interp(z_last, self._z_hold, self._Tup, self._N_prism)
                    for z in z_vals:
                        final_pts.append((x_last, y_last, z, 'PRISMATIC_UP'))
                    self.get_logger().info(f' Subida prismática: +{self._N_prism} pts')

                # XY transition at z_hold to the next entity start
                x_last, y_last, z_last = final_pts[-1][:3]
                x_vals = self._interp(x_last, p2[0], self._Txy, self._N_trans)
                y_vals = self._interp(y_last, p2[1], self._Txy, self._N_trans)
                for x, y in zip(x_vals, y_vals):
                    final_pts.append((x, y, self._z_hold, 'TRANSITION'))
                self.get_logger().info(f' Transición XY: +{self._N_trans} pts')

                # Prism down to next entity z
                z_vals = self._interp(self._z_hold, p2[2], self._Tdn, self._N_prism)
                for z in z_vals:
                    final_pts.append((p2[0], p2[1], z, 'PRISMATIC_DOWN'))
                self.get_logger().info(f' Bajada prismática: +{self._N_prism} pts')

                entity_start = p2
                last_xyz = (p2[0], p2[1], p2[2])
                last_eid = id2

        # Optional: ensure last entity closure back to its first point
        last_eid = wps[-1][4]
        first_of_last = next((p for p in wps if p[4] == last_eid), None)
        if first_of_last is not None and final_pts:
            lx, ly, lz, _ = final_pts[-1]
            if (lx, ly, lz) != (first_of_last[0], first_of_last[1], first_of_last[2]):
                N_close = self._density_for(first_of_last[3])
                x_vals = self._interp(lx, first_of_last[0], self._Tseg, N_close)
                y_vals = self._interp(ly, first_of_last[1], self._Tseg, N_close)
                z_vals = self._interp(lz, first_of_last[2], self._Tseg, N_close)
                for x, y, z in zip(x_vals, y_vals, z_vals):
                    final_pts.append((x, y, z, first_of_last[3]))
                self.get_logger().info(f' Cierre final de entidad {last_eid}: +{N_close} pts')

        return final_pts

    # ================== Publishing ==================

    def _publish_path(self, pts: Sequence[Tuple[float,float,float,str]]) -> None:
        msg = PathMsg()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        for x, y, z, _etype in pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x / 100.0
            ps.pose.position.y = y / 100.0
            ps.pose.position.z = z / 100.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self._pub_path.publish(msg)
        self.get_logger().info(f'Trayectoria publicada con {len(msg.poses)} puntos')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryPlannerCx()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
