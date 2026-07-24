"""
DSOR-фильтр облака точек для ROS2 / VLP-16.

Ключевое ускорение (v3):
  query_ball_point(radii)  →  query(k=min_neighbors+1)

  "Точка имеет ≥ k соседей в радиусе r" эквивалентно
  "расстояние до k-го ближайшего соседа ≤ r".

  tree.query возвращает готовый numpy-массив (N, k) —
  нет Python-цикла, нет list-of-lists, полная векторизация.

  Ожидаемая задержка: 5–15 мс вместо 180 мс.
"""

import time
import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField


# ─────────────────────────────────────────────────────────────────────────────
_FIELD_DTYPE = {
    PointField.INT8:    np.int8,
    PointField.UINT8:   np.uint8,
    PointField.INT16:   np.int16,
    PointField.UINT16:  np.uint16,
    PointField.INT32:   np.int32,
    PointField.UINT32:  np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


def _extract_xyz(msg: PointCloud2) -> np.ndarray:
    """Извлечь xyz из сырого буфера. Работает с любыми типами полей."""
    n    = msg.width * msg.height
    step = msg.point_step
    buf  = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
    offsets = {f.name: (f.offset, _FIELD_DTYPE[f.datatype]) for f in msg.fields}

    def _col(name: str) -> np.ndarray:
        off, dt = offsets[name]
        size = np.dtype(dt).itemsize
        return buf[:, off:off + size].copy().view(dt).reshape(-1).astype(np.float32)

    return np.column_stack([_col('x'), _col('y'), _col('z')])


def _make_msg(msg: PointCloud2, mask: np.ndarray) -> PointCloud2:
    """Собрать новый PointCloud2 из отфильтрованного буфера."""
    n    = msg.width * msg.height
    step = msg.point_step
    buf  = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)

    filtered_data = buf[mask].tobytes()
    n_out = int(mask.sum())

    out              = PointCloud2()
    out.header       = msg.header
    out.fields       = msg.fields
    out.is_bigendian = msg.is_bigendian
    out.point_step   = step
    out.is_dense     = msg.is_dense
    out.height       = 1
    out.width        = n_out
    out.row_step     = step * n_out
    out.data         = filtered_data
    return out


# ─────────────────────────────────────────────────────────────────────────────

class LidarPreprocessorNode(Node):
    def __init__(self):
        super().__init__('lidar_preprocessor')

        self.declare_parameter('min_neighbors',    2)
        self.declare_parameter('azimuth_res_deg',  0.625)
        self.declare_parameter('scale_factor',     1.0)
        self.declare_parameter('base_radius',      0.2)

        self.declare_parameter('use_open3d_sor',   False)
        self.declare_parameter('sor_nb_neighbors', 20)
        self.declare_parameter('sor_std_ratio',    2.0)

        self.declare_parameter('input_topic',      '/velodyne_points')
        self.declare_parameter('output_topic',     '/velodyne_points/filtered')
        self.declare_parameter('diff_topic',       '/velodyne_points/removed')
        self.declare_parameter('publish_diff',     False)
        self.declare_parameter('log_interval_sec', 10.0)

        in_topic   = self.get_parameter('input_topic').value
        out_topic  = self.get_parameter('output_topic').value
        diff_topic = self.get_parameter('diff_topic').value
        pub_diff   = self.get_parameter('publish_diff').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub      = self.create_subscription(PointCloud2, in_topic,  self.callback, qos)
        self.pub      = self.create_publisher(PointCloud2, out_topic, 10)
        self.pub_diff = self.create_publisher(PointCloud2, diff_topic, 10) if pub_diff else None

        self._log_interval = self.get_parameter('log_interval_sec').value
        self._frames = self._pts_before = self._pts_after = 0
        self._elapsed_ms = 0.0
        self._next_log_t = time.monotonic() + self._log_interval

        mode = 'SOR (Open3D)' if self.get_parameter('use_open3d_sor').value else 'DSOR (kNN-vectorized)'
        self.get_logger().info(f'LidarPreprocessor готов: {in_topic} → {out_topic}, режим={mode}')

    # ─────────────────────────────────────────────────────────────────────────
    def callback(self, msg: PointCloud2) -> None:
        n = msg.width * msg.height
        if n == 0:
            return

        xyz   = _extract_xyz(msg)
        valid = np.isfinite(xyz).all(axis=1)
        xyz_valid = xyz[valid] if not valid.all() else xyz

        if xyz_valid.shape[0] == 0:
            return

        t0 = time.monotonic()
        if self.get_parameter('use_open3d_sor').value:
            inlier_mask = self._apply_sor_open3d(xyz_valid)
        else:
            inlier_mask = self._apply_dsor(xyz_valid)
        dt_ms = (time.monotonic() - t0) * 1e3

        if not valid.all():
            full_mask = np.zeros(n, dtype=bool)
            full_mask[valid] = inlier_mask
        else:
            full_mask = inlier_mask

        n_after = int(full_mask.sum())
        self._frames     += 1
        self._pts_before += n
        self._pts_after  += n_after
        self._elapsed_ms += dt_ms

        if time.monotonic() >= self._next_log_t:
            avg_b  = self._pts_before / self._frames
            avg_a  = self._pts_after  / self._frames
            avg_dt = self._elapsed_ms / self._frames
            pct    = (avg_b - avg_a) / avg_b * 100 if avg_b else 0.0
            self.get_logger().info(
                f'[{self._frames} фреймов за {self._log_interval:.0f}с] '
                f'до: {avg_b:.0f}  после: {avg_a:.0f}  '
                f'удалено: {pct:.1f}%  задержка фильтра: {avg_dt:.1f} мс/фрейм'
            )
            self._frames = self._pts_before = self._pts_after = 0
            self._elapsed_ms = 0.0
            self._next_log_t = time.monotonic() + self._log_interval

        if n_after == 0:
            return

        self.pub.publish(_make_msg(msg, full_mask))

        if self.pub_diff is not None and (~full_mask).any():
            self.pub_diff.publish(_make_msg(msg, ~full_mask))

    # ─────────────────────────────────────────────────────────────────────────
    def _apply_dsor(self, xyz: np.ndarray) -> np.ndarray:
        """
        Векторизованный DSOR через kNN.

        Математическое обоснование замены:
          "точка имеет ≥ p_min соседей в радиусе r"
          эквивалентно
          "расстояние до p_min-го ближайшего соседа ≤ r".

          tree.query возвращает numpy-массив (N, k) расстояний —
          вся работа в C, нет Python-цикла, нет list-of-lists.
        """
        p_min  = self.get_parameter('min_neighbors').value
        p_az   = self.get_parameter('azimuth_res_deg').value
        p_sc   = self.get_parameter('scale_factor').value
        p_base = self.get_parameter('base_radius').value

        ranges = np.linalg.norm(xyz, axis=1)
        radii  = p_sc * ranges * np.tan(np.deg2rad(p_az)) + p_base

        tree = cKDTree(xyz)

        # k = p_min + 1: индекс 0 — сама точка (dist=0), далее соседи
        # distances shape: (N, p_min+1) — чистый numpy, без Python-цикла
        distances, _ = tree.query(xyz, k=p_min + 1, workers=-1)

        # distances[:, -1] — расстояние до p_min-го соседа
        # inlier если этот сосед попадает в динамический радиус
        return distances[:, -1] <= radii

    # ─────────────────────────────────────────────────────────────────────────
    def _apply_sor_open3d(self, xyz: np.ndarray) -> np.ndarray:
        import open3d as o3d
        nb  = self.get_parameter('sor_nb_neighbors').value
        std = self.get_parameter('sor_std_ratio').value

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
        _, idx = pcd.remove_statistical_outlier(nb_neighbors=nb, std_ratio=std)

        mask = np.zeros(len(xyz), dtype=bool)
        mask[np.asarray(idx)] = True
        return mask


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = LidarPreprocessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()