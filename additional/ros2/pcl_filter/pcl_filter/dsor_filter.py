import os
os.environ.setdefault('JOBLIB_TEMP_FOLDER', '/tmp')  # фолбэк если /dev/shm недоступен

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
from scipy.spatial import cKDTree
import open3d as o3d


class LidarPreprocessorNode(Node):
    """
    ROS2-нода статической фильтрации облака точек.

    Поддерживает два режима:
      - DSOR (Dynamic Statistical Outlier Removal) — радиус фильтрации
        масштабируется с дальностью точки, реализован через scipy.cKDTree.
      - SOR (Statistical Outlier Removal) — реализация Open3D,
        включается параметром use_open3d_sor:=true.

    ВАЖНО: все поля оригинального сообщения (x, y, z, intensity, ring, time и т.д.)
    сохраняются в выходном облаке, что необходимо для корректной работы LIO-SAM.
    """

    def __init__(self):
        super().__init__('lidar_preprocessor')

        # ── Параметры DSOR ──────────────────────────────────────────────────
        self.declare_parameter('min_neighbors',     2)
        self.declare_parameter('azimuth_res_deg',  0.625)
        self.declare_parameter('scale_factor',     1.0)
        self.declare_parameter('base_radius',      0.2)

        # ── Параметры Open3D SOR (альтернативный режим) ─────────────────────
        self.declare_parameter('use_open3d_sor',   True)
        self.declare_parameter('sor_nb_neighbors', 20)
        self.declare_parameter('sor_std_ratio',    2.0)

        # ── Топики ──────────────────────────────────────────────────────────
        self.declare_parameter('input_topic',  '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points/filtered')
        self.declare_parameter('diff_topic',   '/velodyne_points/removed')

        # Если False — diff-топик не создаётся, накладные расходы нулевые
        self.declare_parameter('publish_diff', False)

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
        self.pub      = self.create_publisher(PointCloud2,    out_topic, 10)
        self.pub_diff = self.create_publisher(PointCloud2, diff_topic, 10) if pub_diff else None

        # ── Метрики ─────────────────────────────────────────────────────────
        # Логируем агрегированную статистику раз в N секунд, чтобы не
        # засорять консоль (при 10 Гц фрейм приходит каждые 100 мс).
        self.declare_parameter('log_interval_sec', 10.0)
        self._log_interval = self.get_parameter('log_interval_sec').value

        self._frames      = 0       # счётчик фреймов за интервал
        self._pts_before  = 0       # сумма точек до фильтра
        self._pts_after   = 0       # сумма точек после фильтра
        self._elapsed_ms  = 0.0     # суммарное время фильтрации, мс
        self._next_log_t  = time.monotonic() + self._log_interval

        self.get_logger().info(
            f"LidarPreprocessor запущен: {in_topic} → {out_topic}"
            + (f" | diff → {diff_topic}" if pub_diff else "")
            + f", режим={'Open3D SOR' if self.get_parameter('use_open3d_sor').value else 'DSOR'}"
        )

    # ────────────────────────────────────────────────────────────────────────
    def callback(self, msg: PointCloud2) -> None:
        # Читаем ВСЕ поля сообщения (x, y, z, intensity, ring, time, …).
        # skip_nans=True отсекает точки с NaN в любом поле.
        gen = point_cloud2.read_points(msg, skip_nans=True)
        data = np.fromiter(gen, dtype=gen.dtype)  # структурированный массив (N,)

        if data.size == 0:
            return

        # Извлекаем xyz как непрерывную матрицу (N, 3) только для поиска соседей
        xyz = np.column_stack([
            data['x'].astype(np.float32),
            data['y'].astype(np.float32),
            data['z'].astype(np.float32),
        ])

        use_o3d = self.get_parameter('use_open3d_sor').value

        t0   = time.monotonic()
        mask = self._apply_sor_open3d(xyz) if use_o3d else self._apply_dsor(xyz)
        dt_ms = (time.monotonic() - t0) * 1e3

        # ── Накапливаем метрики ──────────────────────────────────────────────
        n_before = len(data)
        n_after  = int(mask.sum())
        self._frames     += 1
        self._pts_before += n_before
        self._pts_after  += n_after
        self._elapsed_ms += dt_ms

        if time.monotonic() >= self._next_log_t:
            avg_before  = self._pts_before / self._frames
            avg_after   = self._pts_after  / self._frames
            avg_removed = avg_before - avg_after
            pct_removed = avg_removed / avg_before * 100 if avg_before else 0.0
            avg_dt      = self._elapsed_ms / self._frames
            self.get_logger().info(
                f"[метрики за {self._log_interval:.0f}с | {self._frames} фреймов] "
                f"до фильтра: {avg_before:.0f} | "
                f"после: {avg_after:.0f} | "
                f"удалено: {avg_removed:.0f} ({pct_removed:.1f}%) | "
                f"задержка фильтра: {avg_dt:.1f} мс/фрейм"
            )
            # Сбрасываем аккумуляторы
            self._frames = self._pts_before = self._pts_after = 0
            self._elapsed_ms = 0.0
            self._next_log_t = time.monotonic() + self._log_interval

        filtered = data[mask]
        if filtered.size == 0:
            return

        # ── Публикуем отфильтрованное облако ────────────────────────────────
        filtered_msg = point_cloud2.create_cloud(
            header=msg.header,
            fields=msg.fields,
            points=filtered.tolist(),
        )
        self.pub.publish(filtered_msg)

        # ── Публикуем разностное облако (удалённые точки) ───────────────────
        if self.pub_diff is not None:
            removed = data[~mask]   # инверсия той же маски — без лишних вычислений
            if removed.size > 0:
                diff_msg = point_cloud2.create_cloud(
                    header=msg.header,
                    fields=msg.fields,
                    points=removed.tolist(),
                )
                self.pub_diff.publish(diff_msg)

    # ────────────────────────────────────────────────────────────────────────
    def _apply_dsor(self, xyz: np.ndarray) -> np.ndarray:
        """
        Dynamic Statistical Outlier Removal.

        Радиус окрестности для каждой точки масштабируется с её дальностью:
            r_i = scale_factor * range_i * tan(azimuth_res) + base_radius

        Точка считается выбросом, если в её окрестности меньше min_neighbors соседей.

        Использует scipy.cKDTree (быстрее sklearn, поддерживает массив радиусов
        и параллельный запрос через workers=-1).
        """
        p = {
            'min_neighbors':   self.get_parameter('min_neighbors').value,
            'azimuth_res_deg': self.get_parameter('azimuth_res_deg').value,
            'scale_factor':    self.get_parameter('scale_factor').value,
            'base_radius':     self.get_parameter('base_radius').value,
        }

        ranges = np.linalg.norm(xyz, axis=1)
        radii  = (
            p['scale_factor'] * ranges * np.tan(np.deg2rad(p['azimuth_res_deg']))
            + p['base_radius']
        )

        tree      = cKDTree(xyz)
        neighbors = tree.query_ball_point(xyz, r=radii, workers=-1)
        counts    = np.fromiter((len(n) for n in neighbors), dtype=np.int32, count=len(xyz))

        # -1: исключаем саму точку из счётчика соседей
        return (counts - 1) >= p['min_neighbors']

    # ────────────────────────────────────────────────────────────────────────
    def _apply_sor_open3d(self, xyz: np.ndarray) -> np.ndarray:
        """
        Statistical Outlier Removal через Open3D.

        Open3D убирает точки, у которых среднее расстояние до k ближайших соседей
        отклоняется от глобального среднего более чем на std_ratio * σ.
        Это классический SOR с фиксированным числом соседей (не DSOR).
        Удобен как быстрая альтернатива — реализован в C++ внутри Open3D.
        """
        nb  = self.get_parameter('sor_nb_neighbors').value
        std = self.get_parameter('sor_std_ratio').value

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)

        _, inlier_idx = pcd.remove_statistical_outlier(
            nb_neighbors=nb,
            std_ratio=std,
        )

        mask = np.zeros(len(xyz), dtype=bool)
        mask[np.asarray(inlier_idx)] = True
        return mask


# ────────────────────────────────────────────────────────────────────────────
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