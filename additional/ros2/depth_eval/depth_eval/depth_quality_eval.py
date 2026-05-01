#!/usr/bin/env python3
"""
depth_quality_eval.py
=====================
ROS2 нода сравнения качества карт глубины.

Архитектура:
  - Подписывается на два ИК-потока (raw и filtered) и запускает
    OpenCV StereoSGBM на обоих парах независимо.
  - Камерный depth (/camera/depth/...) используется только как
    дополнительный референс, не участвует в основном сравнении.
  - Параметры камеры (фокусное расстояние, baseline) берутся из
    топика CameraInfo.

Подписки (настраиваются через параметры):
  raw_ir1_topic      — сырой ИК левый  (infra1)
  raw_ir2_topic      — сырой ИК правый (infra2)
  filt_ir1_topic     — фильтрованный ИК левый
  filt_ir2_topic     — фильтрованный ИК правый
  camera_info_topic  — для fx и baseline (топик правой камеры)
  camera_depth_topic — (опционально) глубина камеры как референс
  frame_skip         — обрабатывать каждый N-й кадр (по умолчанию 1 = все)

Пример запуска:
  ros2 run <pkg> depth_quality_eval --ros-args \\
    -p raw_ir1_topic:=/camera/infra1/image_rect_raw \\
    -p raw_ir2_topic:=/camera/infra2/image_rect_raw \\
    -p filt_ir1_topic:=/camera/infra1/image_rect_filtered \\
    -p filt_ir2_topic:=/camera/infra2/image_rect_filtered \\
    -p camera_info_topic:=/camera/infra2/camera_info \\
    -p camera_depth_topic:=/camera/depth/image_rect_raw \\
    -p output_csv:=/data/depth_metrics.csv \\
    -p frame_skip:=10 \\
    -p verbose:=true
"""

import csv
import signal
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

# ---------------------------------------------------------------------------
# Зоны кадра: (y_start, y_end) как доля высоты изображения
# ---------------------------------------------------------------------------
ROI_ZONES = {
    "sky":     (0.00, 150/480),
    "mid":     (150/480, (480-250)/480),
    "asphalt": ((480-250)/480, 1.00),
    "full":    (0.00, 1.00),
}

# Flying pixels: порог отклонения глубины от локальной медианы
FP_KERNEL_SIZE = 5       # пикселей (нечётное)
FP_THRESH_MM   = 100.0   # мм = 10 см

# Минимальная площадь связной области пропусков (пиксели)
MIN_HOLE_AREA = 20

# ---------------------------------------------------------------------------
# Параметры StereoSGBM по умолчанию
# ---------------------------------------------------------------------------
_BS = 7
SGBM_DEFAULTS = dict(
    minDisparity      = 0,
    numDisparities    = 512,
    blockSize         = _BS,
    P1                = 8  * 3 * _BS ** 2,
    P2                = 32 * 3 * _BS ** 2,
    disp12MaxDiff     = 1,
    uniquenessRatio   = 10,
    speckleWindowSize = 100,
    speckleRange      = 32,
    preFilterCap      = 63,
    mode              = cv2.StereoSGBM_MODE_SGBM_3WAY,
)


# ---------------------------------------------------------------------------
class DepthQualityEval(Node):
    def __init__(self):
        super().__init__("depth_quality_eval")

        # ---- параметры ноды ------------------------------------------------
        self.declare_parameter("raw_ir1_topic",      "/camera/infra1/image_rect_raw")
        self.declare_parameter("raw_ir2_topic",      "/camera/infra2/image_rect_raw")
        self.declare_parameter("filt_ir1_topic",     "/camera/infra1/image_rect_filtered")
        self.declare_parameter("filt_ir2_topic",     "/camera/infra2/image_rect_filtered")
        self.declare_parameter("camera_info_topic",  "/camera/infra2/camera_info")
        self.declare_parameter("camera_depth_topic", "")
        self.declare_parameter("output_csv",         "/data/depth_metrics.csv")
        self.declare_parameter("verbose",            False)
        self.declare_parameter("frame_skip",         10)
        self.declare_parameter("sgbm_num_disparities", SGBM_DEFAULTS["numDisparities"])
        self.declare_parameter("sgbm_block_size",      SGBM_DEFAULTS["blockSize"])

        raw_ir1     = self.get_parameter("raw_ir1_topic").value
        raw_ir2     = self.get_parameter("raw_ir2_topic").value
        filt_ir1    = self.get_parameter("filt_ir1_topic").value
        filt_ir2    = self.get_parameter("filt_ir2_topic").value
        cam_info_t  = self.get_parameter("camera_info_topic").value
        cam_depth_t = self.get_parameter("camera_depth_topic").value
        self.csv_path   = Path(self.get_parameter("output_csv").value)
        self.verbose    = self.get_parameter("verbose").value
        self.frame_skip = max(1, self.get_parameter("frame_skip").value)

        nd = self.get_parameter("sgbm_num_disparities").value
        bs = self.get_parameter("sgbm_block_size").value
        sgbm_params = {**SGBM_DEFAULTS,
                       "numDisparities": nd,
                       "blockSize": bs,
                       "P1": 8 * 3 * bs ** 2,
                       "P2": 32 * 3 * bs ** 2}

        # ---- внутреннее состояние ------------------------------------------
        self.bridge     = CvBridge()
        self.stereo     = cv2.StereoSGBM_create(**sgbm_params)
        self.fx         = None
        self.baseline_m = None

        stream_names = ["raw", "filtered"]
        self.use_cam_depth = bool(cam_depth_t)
        if self.use_cam_depth:
            stream_names.append("camera_depth")

        self.stats       = {s: defaultdict(list) for s in stream_names}
        self.frame_rows  = []
        self.frame_count = 0   # сколько кадров реально обработано
        self.cb_count    = 0   # сколько раз вызван callback (для frame_skip)

        # ---- подписка на CameraInfo ----------------------------------------
        self.create_subscription(CameraInfo, cam_info_t,
                                 self._camera_info_cb, 1)

        # ---- синхронизация основных топиков --------------------------------
        subs = [
            Subscriber(self, Image, raw_ir1),
            Subscriber(self, Image, raw_ir2),
            Subscriber(self, Image, filt_ir1),
            Subscriber(self, Image, filt_ir2),
        ]
        if self.use_cam_depth:
            subs.append(Subscriber(self, Image, cam_depth_t))

        self.sync = ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.005)
        self.sync.registerCallback(self._callback)

        self.get_logger().info(
            "DepthQualityEval запущен.\n"
            f"  raw  IR: {raw_ir1}  +  {raw_ir2}\n"
            f"  filt IR: {filt_ir1}  +  {filt_ir2}\n"
            f"  SGBM: numDisparities={nd}, blockSize={bs}\n"
            f"  frame_skip={self.frame_skip} (обрабатывается каждый {self.frame_skip}-й кадр)\n"
            f"  camera depth reference: {'да' if self.use_cam_depth else 'нет'}\n"
            "Нажми Ctrl+C для вывода итогов."
        )

    # -----------------------------------------------------------------------
    def _camera_info_cb(self, msg: CameraInfo):
        if self.fx is not None:
            return
        self.fx = msg.k[0]
        tx = msg.p[3]
        if abs(tx) > 1e-6 and self.fx > 1e-6:
            self.baseline_m = abs(tx) / self.fx
        else:
            self.baseline_m = 0.050
            self.get_logger().warn(
                "Baseline из CameraInfo = 0. Убедись, что передаёшь топик "
                "правой камеры (infra2/camera_info). Используется 50 мм."
            )
        self.get_logger().info(
            f"CameraInfo: fx={self.fx:.1f} px, "
            f"baseline={self.baseline_m * 1000:.1f} mm"
        )

    # -----------------------------------------------------------------------
    def _callback(self, *msgs):
        # --- frame_skip: считаем все вызовы, обрабатываем каждый N-й -------
        self.cb_count += 1
        if self.cb_count % self.frame_skip != 0:
            return

        # --- ждём CameraInfo ------------------------------------------------
        if self.fx is None:
            self.get_logger().warn(
                "CameraInfo ещё не получен, кадр пропущен.",
                throttle_duration_sec=2.0,
            )
            return

        raw_ir1_msg, raw_ir2_msg, filt_ir1_msg, filt_ir2_msg = msgs[:4]
        cam_depth_msg = msgs[4] if self.use_cam_depth else None

        try:
            raw_ir1  = self.bridge.imgmsg_to_cv2(raw_ir1_msg,  "mono8")
            raw_ir2  = self.bridge.imgmsg_to_cv2(raw_ir2_msg,  "mono8")
            filt_ir1 = self.bridge.imgmsg_to_cv2(filt_ir1_msg, "mono8")
            filt_ir2 = self.bridge.imgmsg_to_cv2(filt_ir2_msg, "mono8")
        except Exception as e:
            self.get_logger().warn(f"Ошибка конвертации IR: {e}")
            return

        stamp = (raw_ir1_msg.header.stamp.sec
                 + raw_ir1_msg.header.stamp.nanosec * 1e-9)
        row = {"frame": self.frame_count, "timestamp": stamp}

        raw_depth_mm  = self._disp_to_depth(self._compute_disp(raw_ir1,  raw_ir2))
        filt_depth_mm = self._disp_to_depth(self._compute_disp(filt_ir1, filt_ir2))

        self._accumulate("raw",      raw_ir1,  raw_depth_mm,  row)
        self._accumulate("filtered", filt_ir1, filt_depth_mm, row)

        if cam_depth_msg is not None:
            try:
                cam_d = self.bridge.imgmsg_to_cv2(
                    cam_depth_msg, "passthrough").astype(np.float32)
                self._accumulate("camera_depth", raw_ir1, cam_d, row)
            except Exception as e:
                self.get_logger().warn(f"Ошибка конвертации camera depth: {e}")

        self.frame_rows.append(row)
        self.frame_count += 1

        if self.verbose:
            mr = row.get("raw/full/missing_ratio", 0)
            mf = row.get("filtered/full/missing_ratio", 0)
            fr = row.get("raw/full/flying_ratio", 0)
            ff = row.get("filtered/full/flying_ratio", 0)
            self.get_logger().info(
                f"Frame {self.frame_count:4d} (cb={self.cb_count}) | "
                f"missing  raw={mr:.3f} filt={mf:.3f} Δ={mf-mr:+.3f} | "
                f"flying   raw={fr:.3f} filt={ff:.3f} Δ={ff-fr:+.3f}"
            )

    # -----------------------------------------------------------------------
    def _compute_disp(self, ir1: np.ndarray, ir2: np.ndarray) -> np.ndarray:
        disp = self.stereo.compute(ir1, ir2).astype(np.float32) / 16.0
        disp[disp <= 0] = 0.0
        return disp

    def _disp_to_depth(self, disp: np.ndarray) -> np.ndarray:
        depth_mm = np.zeros_like(disp)
        valid = disp > 0
        depth_mm[valid] = (self.fx * self.baseline_m * 1000.0) / disp[valid]
        return depth_mm

    # -----------------------------------------------------------------------
    def _accumulate(self, stream: str, ir: np.ndarray,
                    depth_mm: np.ndarray, row: dict):
        h = ir.shape[0]
        for zone, (y0f, y1f) in ROI_ZONES.items():
            y0, y1 = int(h * y0f), int(h * y1f)
            m = _compute_metrics(ir[y0:y1], depth_mm[y0:y1])
            for k, v in m.items():
                key = f"{stream}/{zone}/{k}"
                self.stats[stream][f"{zone}/{k}"].append(v)
                row[key] = round(v, 5)

    # -----------------------------------------------------------------------
    def print_summary(self):
        streams = list(self.stats.keys())
        print("\n" + "=" * 95)
        print(f"  ИТОГОВЫЕ МЕТРИКИ — {self.frame_count} обработано "
              f"из {self.cb_count} кадров (frame_skip={self.frame_skip})")
        print("=" * 95)
        col_w = 20
        hdr = f"{'Метрика':<45}" + "".join(f"  {s:>{col_w}}" for s in streams)
        if "raw" in streams and "filtered" in streams:
            hdr += f"  {'Δ filt−raw':>12}"
        print(hdr)
        print("-" * 95)

        for key in sorted(self.stats["raw"].keys()):
            line = f"{key:<45}"
            means = {}
            for s in streams:
                arr = np.array(self.stats[s].get(key, [0.0]))
                means[s] = arr.mean()
                line += f"  {arr.mean():.4f}±{arr.std():.4f}".rjust(col_w + 2)
            if "raw" in means and "filtered" in means:
                d = means["filtered"] - means["raw"]
                line += f"  {d:>+.4f}".rjust(14)
            print(line)

        print("=" * 95 + "\n")

    def save_csv(self):
        if not self.frame_rows:
            return
        keys = sorted(self.frame_rows[0].keys())
        with open(self.csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(self.frame_rows)
        print(f"CSV сохранён: {self.csv_path}")


# ---------------------------------------------------------------------------
# Вычисление метрик для одного ROI
# ---------------------------------------------------------------------------
def _compute_metrics(ir: np.ndarray, depth_mm: np.ndarray) -> dict:
    total      = depth_mm.size
    valid_mask = depth_mm > 0
    n_valid    = int(valid_mask.sum())

    missing_ratio   = 1.0 - n_valid / total if total > 0 else 1.0
    flying_ratio    = _flying_pixel_ratio(depth_mm, valid_mask)
    missing_regions = _count_missing_regions(valid_mask)

    if n_valid > 0:
        d_m = depth_mm[valid_mask] / 1000.0
        depth_mean = float(d_m.mean())
        depth_std  = float(d_m.std())
        depth_p50  = float(np.median(d_m))
    else:
        depth_mean = depth_std = depth_p50 = 0.0

    ir_f = ir.astype(np.float32)
    return {
        "missing_ratio":   missing_ratio,
        "flying_ratio":    flying_ratio,
        "missing_regions": float(missing_regions),
        "depth_mean_m":    depth_mean,
        "depth_std_m":     depth_std,
        "depth_p50_m":     depth_p50,
        "ir_mean":         float(ir_f.mean()),
        "ir_std":          float(ir_f.std()),
        "ir_p25":          float(np.percentile(ir_f, 25)),
        "ir_p50":          float(np.percentile(ir_f, 50)),
        "ir_p75":          float(np.percentile(ir_f, 75)),
    }


def _flying_pixel_ratio(depth_mm: np.ndarray, valid_mask: np.ndarray) -> float:
    if valid_mask.sum() == 0:
        return 0.0
    med    = cv2.medianBlur(depth_mm.astype(np.float32), FP_KERNEL_SIZE)
    flying = (np.abs(depth_mm - med) > FP_THRESH_MM) & valid_mask
    return float(flying.sum()) / float(valid_mask.sum())


def _count_missing_regions(valid_mask: np.ndarray) -> int:
    missing  = (~valid_mask).astype(np.uint8)
    n_labels, labels = cv2.connectedComponents(missing, connectivity=8)
    return sum(
        1 for lbl in range(1, n_labels)
        if int((labels == lbl).sum()) >= MIN_HOLE_AREA
    )


# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    node = DepthQualityEval()

    def _shutdown(sig, frame):
        node.print_summary()
        node.save_csv()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass


if __name__ == "__main__":
    main()