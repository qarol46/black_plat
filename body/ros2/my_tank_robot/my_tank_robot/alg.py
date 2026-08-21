#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, JointState
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Duration

import math
import numpy as np

import sensor_msgs_py.point_cloud2 as pc2

# Вместо Float64 используем Float64MultiArray
from std_msgs.msg import Float64MultiArray

##############################################
# Вспомогательные функции (не меняем логику)
##############################################
def transform_points_np(arr2d, phi, h):
    c = math.cos(phi)
    s = math.sin(phi)
    shifted = arr2d + [0, h]
    R = np.array([[c, -s],
                  [s,  c]], dtype=float)
    return shifted @ R.T

def line_front_loc(x, t_deg, lb):
    return math.tan(math.radians(t_deg))*(x - lb/2)

def line_rear_loc(x, t_deg, lb):
    return -math.tan(math.radians(t_deg))*(x + lb/2)

# Параметры и веса (примерные)
w_theta = 1.0
w_y     = 1.0
w_s     = 1.0
w_t     = 100000.0
w_r     = 1.0

m_F = 1.0   # масса флиппера
m_B = 10.0  # масса базы

def total_cost(sol, prev_phi, prev_h, prev_t1, prev_t2,
               all_points, lb, lf):
    scenario_id, t1_deg, t2_deg, xf, yf, xr, yr, h, phi = sol

    C_theta = w_theta * abs(phi - prev_phi)

    points_transformed = transform_points_np(all_points, phi, h)
    avg_ground_y = np.mean(points_transformed[:, 1]) if len(points_transformed)>0 else 0.0
    Cy = w_y * abs(h - avg_ground_y)

    x_c1, x_c2 = -lb/2, lb/2
    Cs = w_s * (4 * m_F * lf / (m_B + 4 * m_F)) / max(1e-6, min(abs(x_c1), abs(x_c2)))

    Ct1 = 0
    Ct2 = 0
    if scenario_id in [2,4]:
        Ct1 = w_t * abs(xf - (lb/2)) / lf
    if scenario_id in [3,4]:
        Ct2 = w_t * abs(xr - (-lb/2)) / lf
    Ct = Ct1 + Ct2

    Cr = w_r * (abs(t1_deg - prev_t1)/10.0 + abs(t2_deg - prev_t2)/10.0)

    total_C = C_theta + Cy + Cs + Ct + Cr
    return total_C, {
        'C_theta': C_theta,
        'Cy': Cy,
        'Cs': Cs,
        'Ct': Ct,
        'Cr': Cr,
        'total': total_C
    }

def form_4_arrays_with_angles(shifted_points, t1_deg, t2_deg, lb, lf):
    t1_rad = math.radians(t1_deg)
    t2_rad = math.radians(t2_deg)

    x_com = (4 * m_F * lf / (m_B + 4 * m_F))

    # Задний флиппер: ИСПРАВЛЕНО — сравнение с rear_flip_min / rear_flip_max
    rf_x1 = -lb/2
    rf_x2 = -lb/2 - lf*math.cos(t2_rad) - 0.05
    rear_flip_min = min(rf_x1, rf_x2)
    rear_flip_max = max(rf_x1, rf_x2)
    TrearFlip = shifted_points[
        (shifted_points[:,0] >= rear_flip_min) &
        (shifted_points[:,0] <= rear_flip_max)
    ]

    TrearBase = shifted_points[
        (shifted_points[:,0] >= -lb/2) &
        (shifted_points[:,0] <= -x_com)
    ]

    ff_x1 = lb/2
    ff_x2 = lb/2 + lf*math.cos(t1_rad)
    front_flip_min = min(ff_x1, ff_x2)
    front_flip_max = max(ff_x1, ff_x2) + 0.05
    TfrontFlip = shifted_points[
        (shifted_points[:,0] >= front_flip_min) &
        (shifted_points[:,0] <= front_flip_max)
    ]

    TfrontBase = shifted_points[
        (shifted_points[:,0] >= x_com) &
        (shifted_points[:,0] <= lb/2)
    ]

    return TrearFlip, TrearBase, TfrontFlip, TfrontBase

def check_all_under_robot(h, phi, arr_4points, lb, lf, t1_deg, t2_deg):
    arr_after = transform_points_np(arr_4points, phi, h)
    for (xx, yy) in arr_after:
        if lb/2 <= xx <= (lb/2 + lf):
            frY = line_front_loc(xx, t1_deg, lb)
            if yy > frY:
                return False
        elif (-lb/2 - lf) <= xx <= -lb/2:
            rrY = line_rear_loc(xx, t2_deg, lb)
            if yy > rrY:
                return False
        else:
            if yy > 0:
                return False
    return True

def is_point_in_rear_base(point_after, lb, tol=1e-4):
    xx, yy = point_after
    if xx < -lb/2 or xx > lb/2:
        return False
    if abs(yy) > tol:
        return False
    return True

def is_point_in_front_base(point_after, lb, tol=1e-4):
    xx, yy = point_after
    if xx < -lb/2 or xx > lb/2:
        return False
    if abs(yy) > tol:
        return False
    return True

def is_point_in_rear_flipper(point_after, lb, lf, t_deg, tol=1e-4):
    xx, yy = point_after
    if xx < (-lb/2 - lf + 0.0012) or xx > -lb/2:
        return False
    ideal_y = line_rear_loc(xx, t_deg, lb)
    if abs(yy - ideal_y) > tol:
        return False
    return True

def is_point_in_front_flipper(point_after, lb, lf, t_deg, tol=1e-4):
    xx, yy = point_after
    if xx < lb/2 or xx > (lb/2 + lf - 0.0012):
        return False
    ideal_y = line_front_loc(xx, t_deg, lb)
    if abs(yy - ideal_y) > tol:
        return False
    return True

def compute_quartic_custom(xf, yf, xr, yr, k1, b1, k2, b2):
    X1= xf + k1*yf
    Y1= yf - k1*xf
    A1= Y1 + b1
    B1= -2*X1
    C1= -2*k1
    D1= b1 - Y1

    X2= xr + k2*yr
    Y2= yr - k2*xr
    A2= Y2 + b2
    B2= -2*X2
    C2= -2*k2
    D2= b2 - Y2

    a4= -C1**2 + 2*C1*C2 - C2**2
    a3= (A1*C1*C2 - A1*C2**2 - A2*C1**2 + A2*C1*C2
         - 2*B1*C1 + 2*B1*C2 + 2*B2*C1 - 2*B2*C2
         + C1**2*D2 - C1*C2*D1 - C1*C2*D2 + C2**2*D1)
    a2= (A1**2 - 2*A1*A2 + A1*B1*C2 + A1*B2*C1 - 2*A1*B2*C2
         - A1*C1*C2*D2 + A1*C2**2*D1 + 2*A1*D1 - 2*A1*D2
         + A2**2 - 2*A2*B1*C1 + A2*B1*C2 + A2*B2*C1
         + A2*C1**2*D2 - A2*C1*C2*D1 - 2*A2*D1 + 2*A2*D2
         - B1**2 + 2*B1*B2 + 2*B1*C1*D2 - B1*C2*D1 - B1*C2*D2
         - B2**2 - B2*C1*D1 - B2*C1*D2 + 2*B2*C2*D1
         + D1**2 - 2*D1*D2 + D2**2)
    a1= (-2*A1**2*D2 + 2*A1*A2*D1 + 2*A1*A2*D2
         + A1*B1*B2 - A1*B1*C2*D2 - A1*B2**2
         - A1*B2*C1*D2 + 2*A1*B2*C2*D1
         - 2*A1*D1*D2 + 2*A1*D2**2
         - 2*A2**2*D1 - A2*B1**2 + A2*B1*B2
         + 2*A2*B1*C1*D2 - A2*B1*C2*D1
         - A2*B2*C1*D1 + 2*A2*D1**2 - 2*A2*D1*D2
         + B1**2*D2 - B1*B2*D1 - B1*B2*D2 + B2**2*D1)
    a0= (A1**2*D2**2 - 2*A1*A2*D1*D2 - A1*B1*B2*D2
         + A1*B2**2*D1 + A2**2*D1**2 + A2*B1**2*D2
         - A2*B1*B2*D1)
    return [a4,a3,a2,a1,a0]

def formABCD(k, b, x0, y0):
    X = x0 + k*y0
    Y = y0 - k*x0
    A = Y + b
    B = -2*X
    C = -2*k
    D = b - Y
    return (A, B, C, D)

def process_robot_offset(points_global, offset, t1_vals, t2_vals, lb, lf):
    shifted = points_global.copy()
    shifted[:, 0] -= offset

    all_solutions = []

    for t1d in t1_vals:
        for t2d in t2_vals:
            TrearFlip, TrearBase, TfrontFlip, TfrontBase = form_4_arrays_with_angles(
                shifted, t1d, t2d, lb, lf
            )
            merged_4arrays = np.vstack([TrearFlip, TrearBase, TfrontFlip, TfrontBase])

            # Сценарий 1
            scenario_id = 1
            k1,b1 = 0,0
            k2,b2 = 0,0
            for (xr,yr) in TrearBase:
                for (xf,yf) in TfrontBase:
                    coeffs = compute_quartic_custom(xf,yf, xr,yr, k1,b1, k2,b2)
                    h_roots = np.roots(coeffs)
                    for h_candidate in h_roots:
                        if abs(h_candidate.imag) < 1e-10:
                            h_val = h_candidate.real
                            A1,B1,C1,D1 = formABCD(k1,b1, xf,yf)
                            A2,B2,C2,D2 = formABCD(k2,b2, xr,yr)
                            alpha = A1 - A2
                            beta  = (B1 - B2) + (C1 - C2)*h_val
                            gamma = (D1 - D2)
                            t_roots = np.roots([alpha, beta, gamma])
                            for rr2 in t_roots:
                                if abs(rr2.imag)<1e-10:
                                    real_r2 = rr2.real
                                    phi_val = 2.0*math.atan(real_r2)
                                    p_rear_after = transform_points_np(
                                        np.array([[xr,yr]]), phi_val, h_val
                                    )[0]
                                    if not is_point_in_rear_base(p_rear_after, lb):
                                        continue
                                    p_front_after = transform_points_np(
                                        np.array([[xf,yf]]), phi_val, h_val
                                    )[0]
                                    if not is_point_in_front_base(p_front_after, lb):
                                        continue
                                    if not check_all_under_robot(h_val, phi_val,
                                                                 merged_4arrays,
                                                                 lb, lf,
                                                                 t1d, t2d):
                                        continue
                                    all_solutions.append(
                                        (scenario_id, t1d, t2d,
                                         xf,yf, xr,yr,
                                         h_val, phi_val)
                                    )

            # Сценарий 2
            scenario_id = 2
            k2,b2 = 0,0
            k1    = math.tan(math.radians(t1d))
            b1    = -0.5*lb*k1
            for (xr,yr) in TrearBase:
                for (xf,yf) in TfrontFlip:
                    coeffs = compute_quartic_custom(xf,yf, xr,yr, k1,b1, k2,b2)
                    h_roots = np.roots(coeffs)
                    for h_candidate in h_roots:
                        if abs(h_candidate.imag) < 1e-10:
                            h_val = h_candidate.real
                            A1,B1,C1,D1 = formABCD(k1,b1, xf,yf)
                            A2,B2,C2,D2 = formABCD(k2,b2, xr,yr)
                            alpha = A1 - A2
                            beta  = (B1 - B2) + (C1 - C2)*h_val
                            gamma = (D1 - D2)
                            t_roots = np.roots([alpha, beta, gamma])
                            for rr2 in t_roots:
                                if abs(rr2.imag)<1e-10:
                                    phi_val = 2.0*math.atan(rr2.real)
                                    p_rear_after = transform_points_np(
                                        np.array([[xr,yr]]), phi_val, h_val
                                    )[0]
                                    if not is_point_in_rear_base(p_rear_after, lb):
                                        continue
                                    p_front_after = transform_points_np(
                                        np.array([[xf,yf]]), phi_val, h_val
                                    )[0]
                                    if not is_point_in_front_flipper(
                                        p_front_after, lb, lf, t1d):
                                        continue
                                    if not check_all_under_robot(h_val, phi_val,
                                                                 merged_4arrays,
                                                                 lb, lf,
                                                                 t1d, t2d):
                                        continue
                                    all_solutions.append(
                                        (scenario_id, t1d, t2d,
                                         xf,yf, xr,yr,
                                         h_val, phi_val)
                                    )
            # Сценарий 3 (закомментирован или нет — по вашему желанию)
            # ...
            # Сценарий 4
            scenario_id = 4
            k1 =  math.tan(math.radians(t1d))
            b1 = -0.5*lb*k1
            k2 = -math.tan(math.radians(t2d))
            b2 =  0.5*lb*k2
            for (xr,yr) in TrearFlip:
                for (xf,yf) in TfrontFlip:
                    coeffs = compute_quartic_custom(xf,yf, xr,yr, k1,b1, k2,b2)
                    h_roots = np.roots(coeffs)
                    for h_candidate in h_roots:
                        if abs(h_candidate.imag) < 1e-10:
                            h_val = h_candidate.real
                            A1,B1,C1,D1 = formABCD(k1,b1, xf,yf)
                            A2,B2,C2,D2 = formABCD(k2,b2, xr,yr)
                            alpha = A1 - A2
                            beta  = (B1 - B2) + (C1 - C2)*h_val
                            gamma = (D1 - D2)
                            t_roots = np.roots([alpha, beta, gamma])
                            for rr2 in t_roots:
                                if abs(rr2.imag)<1e-10:
                                    phi_val = 2.0*math.atan(rr2.real)
                                    p_rear_after = transform_points_np(
                                        np.array([[xr,yr]]), phi_val, h_val
                                    )[0]
                                    if not is_point_in_rear_flipper(
                                        p_rear_after, lb, lf, t2d):
                                        continue
                                    p_front_after = transform_points_np(
                                        np.array([[xf,yf]]), phi_val, h_val
                                    )[0]
                                    if not is_point_in_front_flipper(
                                        p_front_after, lb, lf, t1d):
                                        continue
                                    if not check_all_under_robot(h_val, phi_val,
                                                                 merged_4arrays,
                                                                 lb, lf,
                                                                 t1d, t2d):
                                        continue
                                    all_solutions.append(
                                        (scenario_id, t1d, t2d,
                                         xf,yf, xr,yr,
                                         h_val, phi_val)
                                    )

    return TrearFlip, TrearBase, TfrontFlip, TfrontBase, all_solutions


###############################################
# Узел ROS2, публикуем ОДИН угол (передний) как Float64MultiArray
# (элемент массива) на /geom_position_controller/commands
###############################################
class Alg(Node):
    def __init__(self):
        super().__init__('alg')

        # Подписчики
        self.odom_sub = self.create_subscription(
            Odometry, 'diff_drive_controller/odom', self.odom_callback, 10
        )
        self.pc_sub = self.create_subscription(
            PointCloud2, '/projected_pointcloud_xz_contour_robot',
            self.pointcloud_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Паблишер: ForwardCommandController (один джойнт, но принимает Float64MultiArray)
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/geom_position_controller/commands_before',
            10
        )

        # Память
        self.previous_position = None
        self.linear_speed = 0.0
        self.time_interval = 3.0
        self.pitch = 0.0

        self.current_theta1 = 0.0
        self.current_theta2 = 0.0

        # Геометрия
        self.l_b = 0.55
        self.l_f = 0.4

        self.prev_phi = 0.0
        self.prev_h   = 0.0
        self.prev_t1  = 0.0
        self.prev_t2  = 0.0

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.previous_position is not None:
            prev_x, prev_y, prev_time = self.previous_position
            dt = current_time - prev_time
            if dt > 0:
                dx = current_x - prev_x
                dy = current_y - prev_y
                self.linear_speed = math.sqrt(dx*dx + dy*dy) / dt

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        roll  = math.atan2(2.0*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        pitch = math.asin(2*(qw*qy - qz*qx))
        yaw   = math.atan2(2.0*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

        # Предположим, нас интересует pitch
        self.pitch = pitch

        self.previous_position = (current_x, current_y, current_time)

    def joint_state_callback(self, msg):
        try:
            idx_fl = msg.name.index('front_left_flipper_joint')
            idx_rr = msg.name.index('rear_left_flipper_jjoint')
            self.current_theta1 = math.degrees(msg.position[idx_fl])
            self.current_theta2 = math.degrees(msg.position[idx_rr])
        except ValueError:
            pass

    def pointcloud_callback(self, msg):
        points_2d = []
        for p in pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
            x_ = p[0]
            y_ = p[2]  # берём z за "высоту"
            points_2d.append([x_, y_])
        points_2d = np.array(points_2d, dtype=float)

        offset = self.linear_speed * self.time_interval
        self.get_logger().info(f"Using offset={offset:.3f}, speed={self.linear_speed:.2f}")

        angle_step = 10
        K = 3
        t1_min = max(-10, self.current_theta1 - K*angle_step)
        t1_max = min(40,  self.current_theta1 + K*angle_step)
        t2_min = max(-10, self.current_theta2 - K*angle_step)
        t2_max = min(40,  self.current_theta2 + K*angle_step)

        t1_vals = range(int(t1_min), int(t1_max)+1, angle_step)
        t2_vals = range(int(t2_min), int(t2_max)+1, angle_step)

        # Ищем решения
        _, _, _, _, solutions = process_robot_offset(
            points_2d, offset, t1_vals, t2_vals, self.l_b, self.l_f
        )

        if not solutions:
            self.get_logger().warn("Нет валидных решений.")
            return

        best_sol = min(
            solutions,
            key=lambda sol: total_cost(
                sol,
                self.prev_phi,
                self.prev_h,
                self.prev_t1,
                self.prev_t2,
                points_2d,
                self.l_b,
                self.l_f
            )[0]
        )

        scenario_id, t1d, t2d, xf, yf, xr, yr, h_val, phi_val = best_sol
        self.prev_phi = phi_val
        self.prev_h   = h_val
        self.prev_t1  = t1d
        self.prev_t2  = t2d

        # Публикуем ТОЛЬКО угол переднего флиппера (t1) в радианах
        front_angle_rad = math.radians(-t1d+270)

        # Формируем Float64MultiArray (массив из одного угла)
        msg = Float64MultiArray()
        # layout обычно можно оставить пустым
        msg.layout.dim = []
        msg.layout.data_offset = 0
        msg.data = [front_angle_rad]

        self.publisher.publish(msg)
        self.get_logger().info(f"Send front flipper => {t1d:.1f} deg ({front_angle_rad:.2f} rad)")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Alg()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
