import open3d as o3d
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.ticker as ticker
import argparse
import os
import json
from datetime import datetime

# ─── Глобальные настройки шрифта ─────────────────────────────────────────────
FONT_SIZE   = 30   # базовый: подписи осей, легенда
TITLE_SIZE  = 30   # заголовок
TICK_SIZE   = 30   # подписи делений осей
LABEL_SIZE  = 30   # метки пар на изображении

def apply_rcparams():
    plt.rcParams.update({
        "font.family":    "Times New Roman",
        "font.size":       FONT_SIZE,
        "axes.titlesize":  TITLE_SIZE,
        "axes.labelsize":  FONT_SIZE,
        "xtick.labelsize": TICK_SIZE,
        "ytick.labelsize": TICK_SIZE,
        "legend.fontsize": FONT_SIZE,
    })

# ─── Форматирование чисел с запятой ──────────────────────────────────────────
def fmt(value: float, decimals: int = 3) -> str:
    return f"{value:.{decimals}f}".replace(".", ",")

def fmt2(value): return fmt(value, 2)
def fmt1(value): return fmt(value, 1)

# ─── Аргументы ────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--data",      type=str,   default="default.pcd")
    p.add_argument("--zmin",      type=float, default=1.5)
    p.add_argument("--zmax",      type=float, default=2.5)
    p.add_argument("--res",       type=float, default=0.05)
    p.add_argument("--no-filter", action="store_true")
    p.add_argument("--out",       type=str,   default=".")
    p.add_argument("--pairs",     type=int,   default=0)
    return p.parse_args()

# ─── 1. Загрузка ──────────────────────────────────────────────────────────────
def load_and_filter(path, do_filter):
    if not os.path.exists(path):
        raise FileNotFoundError(f"Файл не найден: {path}")
    pcd = o3d.io.read_point_cloud(path)
    print(f"[PCD] Загружено точек: {len(pcd.points):,}")
    if do_filter:
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        print(f"[PCD] После фильтрации: {len(pcd.points):,}")
    return pcd

# ─── 2. Выравнивание ──────────────────────────────────────────────────────────
def align_to_ground(pcd):
    plane_model, _ = pcd.segment_plane(
        distance_threshold=0.05, ransac_n=3, num_iterations=2000)
    a, b, c, d = plane_model
    normal = np.array([a, b, c])
    normal /= np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal

    z_axis = np.array([0.0, 0.0, 1.0])
    v = np.cross(normal, z_axis)
    s = np.linalg.norm(v)
    c_cos = np.dot(normal, z_axis)

    if s < 1e-6:
        R = np.eye(3)
    else:
        vx = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
        R = np.eye(3) + vx + vx @ vx * ((1 - c_cos) / (s**2))

    T = np.eye(4); T[:3,:3] = R
    pcd = pcd.transform(T)
    pts = np.asarray(pcd.points)
    pts[:,2] -= np.percentile(pts[:,2], 5)
    pcd.points = o3d.utility.Vector3dVector(pts)
    print("[Align] Облако выровнено по плоскости земли")
    return pcd

# ─── 3. BEV ───────────────────────────────────────────────────────────────────
def build_bev(pcd, z_min, z_max, resolution):
    pts = np.asarray(pcd.points)
    mask = (pts[:,2] > z_min) & (pts[:,2] < z_max)
    sp = pts[mask]
    print(f"[BEV] Точек в срезе: {mask.sum():,}")
    if len(sp) == 0:
        raise ValueError("Срез пустой — проверьте --zmin / --zmax")

    x, y = sp[:,0], sp[:,1]
    x_min, y_min = x.min(), y.min()
    W = int(np.ceil((x.max()-x_min)/resolution)) + 1
    H = int(np.ceil((y.max()-y_min)/resolution)) + 1

    xi = np.clip(((x-x_min)/resolution).astype(int), 0, W-1)
    yi = np.clip(((y-y_min)/resolution).astype(int), 0, H-1)

    bev = np.full((H, W), 255, dtype=np.uint8)
    bev[yi, xi] = 0
    print(f"[BEV] {W}×{H} пкс = {fmt1(W*resolution)}×{fmt1(H*resolution)} м")
    return bev, (x_min, y_min), resolution

# ─── 4. Отображение ───────────────────────────────────────────────────────────
def show_bev_with_scalebar(bev, resolution, scalebar_m=10.0):
    apply_rcparams()
    fig, ax = plt.subplots(figsize=(11, 11))
    ax.imshow(bev, cmap="gray", origin="lower")
    ax.set_title(
        "Срез облака точек"
    )

    # Масштабная линейка
    bar_px = scalebar_m / resolution
    bar_x0, bar_y0 = 20, 20
    ax.add_patch(patches.FancyArrowPatch(
        (bar_x0, bar_y0), (bar_x0+bar_px, bar_y0),
        arrowstyle="-", color="red", linewidth=3))
    ax.text(bar_x0+bar_px/2, bar_y0+12,
            f"{fmt1(scalebar_m)} м", color="red",
            ha="center", fontsize=FONT_SIZE, fontweight="bold")

    # Деления осей в метрах с запятой
    ax.xaxis.set_major_formatter(
        ticker.FuncFormatter(lambda x, _: fmt1(x * resolution)))
    ax.yaxis.set_major_formatter(
        ticker.FuncFormatter(lambda y, _: fmt1(y * resolution)))

    ax.set_xlabel("X, м", labelpad=8)
    ax.set_ylabel("Y, м", labelpad=8)
    plt.tight_layout()
    return fig, ax

# ─── 5. Выбор точек ───────────────────────────────────────────────────────────
def select_control_pairs(fig, ax, bev, resolution, n_pairs):
    print("\n" + "="*60)
    print(f"Нужно выбрать {n_pairs} пар. Каждая пара — 2 клика.")
    print("="*60)

    colors = plt.cm.tab10(np.linspace(0, 1, n_pairs))
    pairs_px = []

    for i in range(n_pairs):
        print(f"\nПара {i+1}/{n_pairs}: кликните 2 точки")
        pts = plt.ginput(2, timeout=300)
        if len(pts) < 2:
            print(f"  [!] Пропущена пара {i+1}")
            pairs_px.append(None)
            continue

        p1, p2 = np.array(pts[0]), np.array(pts[1])
        pairs_px.append((p1, p2))

        ax.plot([p1[0],p2[0]], [p1[1],p2[1]],
                '-o', color=colors[i], linewidth=2, markersize=7)
        ax.text((p1[0]+p2[0])/2, (p1[1]+p2[1])/2+10,
                f"П{i+1}", color=colors[i],
                fontsize=LABEL_SIZE, fontweight="bold", ha="center")
        fig.canvas.draw()

        dist_m = np.linalg.norm(p2-p1) * resolution
        print(f"  Расстояние в BEV: {fmt(dist_m)} м")

    return pairs_px

# ─── 6. Ввод эталонных расстояний ────────────────────────────────────────────
def input_reference_distances(n_pairs, source="Google Earth Pro"):
    print(f"\nВВОД ЭТАЛОННЫХ РАССТОЯНИЙ (источник: {source})")
    refs = []
    for i in range(n_pairs):
        while True:
            try:
                val = input(f"  Пара {i+1} — расстояние [м]: ")
                refs.append(float(val.replace(",", ".")))
                break
            except ValueError:
                print("  [!] Введите число, например: 12,5 или 12.5")
    return refs

# ─── 7. Метрики ───────────────────────────────────────────────────────────────
def compute_metrics(pairs_px, refs_m, resolution):
    results = []
    for i, (pair, d_ref) in enumerate(zip(pairs_px, refs_m)):
        if pair is None: continue
        p1, p2 = pair
        d_map = np.linalg.norm(p2-p1) * resolution
        e     = abs(d_map - d_ref)
        delta = e / d_ref * 100.0 if d_ref > 0 else float("nan")
        results.append({"pair": i+1, "d_map": d_map,
                         "d_ref": d_ref, "error": e, "delta": delta})

    if not results:
        print("[!] Нет данных"); return {}

    errors = np.array([r["error"] for r in results])
    deltas = np.array([r["delta"] for r in results])
    return {
        "N": len(results),
        "RMSE":       float(np.sqrt(np.mean(errors**2))),
        "e_mean":     float(np.mean(errors)),
        "e_max":      float(np.max(errors)),
        "e_std":      float(np.std(errors)),
        "delta_mean": float(np.mean(deltas)),
        "pairs":      results,
    }

# ─── 8. Отчёт ─────────────────────────────────────────────────────────────────
def print_report(s):
    print("\n" + "="*65)
    print("РЕЗУЛЬТАТЫ ОЦЕНКИ ТОЧНОСТИ")
    print("="*65)
    print(f"{'Пара':>5}  {'d_карта [м]':>12}  {'d_эталон [м]':>13}  "
          f"{'e [м]':>9}  {'δ [%]':>8}")
    print("-"*60)
    for r in s["pairs"]:
        print(f"{r['pair']:>5}  {fmt(r['d_map']):>12}  {fmt(r['d_ref']):>13}  "
              f"{fmt(r['error']):>9}  {fmt2(r['delta']):>8}")
    print("-"*60)
    print(f"N      = {s['N']}")
    print(f"RMSE   = {fmt1(s['RMSE']*100)} см")
    print(f"e_max  = {fmt1(s['e_max']*100)} см")
    print(f"e_std  = {fmt1(s['e_std']*100)} см")
    print(f"δ_mean = {fmt2(s['delta_mean'])} %")
    print("="*65)

def save_results(summary, bev, fig, out_dir, pcd_name):
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(pcd_name))[0]
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")

    fig.savefig(os.path.join(out_dir, f"{stem}_bev_{ts}.png"),
                dpi=150, bbox_inches="tight")

    with open(os.path.join(out_dir, f"{stem}_metrics_{ts}.json"),
              "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    # CSV с разделителем «;» и запятыми в числах — сразу открывается в Excel/LibreOffice
    with open(os.path.join(out_dir, f"{stem}_pairs_{ts}.csv"),
              "w", encoding="utf-8") as f:
        f.write("пара;d_карта_м;d_эталон_м;e_м;delta_%\n")
        for r in summary["pairs"]:
            f.write(f"{r['pair']};{fmt(r['d_map'])};{fmt(r['d_ref'])};"
                    f"{fmt(r['error'])};{fmt2(r['delta'])}\n")

    print(f"[Out] Файлы сохранены в: {out_dir}")

# ─── main ─────────────────────────────────────────────────────────────────────
def main():
    args = parse_args()
    pcd  = load_and_filter(args.data, do_filter=not args.no_filter)
    pcd  = align_to_ground(pcd)
    bev, origin, res = build_bev(pcd, args.zmin, args.zmax, args.res)

    matplotlib.use("TkAgg")   # замените на "Qt5Agg" если нет Tk
    fig, ax = show_bev_with_scalebar(bev, res, scalebar_m=10.0)
    plt.show(block=False)
    plt.pause(0.5)

    n_pairs = args.pairs
    if n_pairs <= 0:
        while True:
            try:
                n_pairs = int(input("\nСколько контрольных пар? [≥5]: "))
                if n_pairs > 0: break
            except ValueError:
                pass

    pairs_px = select_control_pairs(fig, ax, bev, res, n_pairs)
    refs_m   = input_reference_distances(n_pairs)
    summary  = compute_metrics(pairs_px, refs_m, res)

    if summary:
        print_report(summary)
        save_results(summary, bev, fig, args.out, args.data)

    plt.show()

if __name__ == "__main__":
    main()