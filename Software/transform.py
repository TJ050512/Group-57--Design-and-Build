#!/usr/bin/env python3
"""
gridify_points.py (ray-cast enabled)

更新说明：
- 将栅格化改为默认启用射线更新（ray-cast），以区分已探索（observed）与未探索区域。
- 使用对数几率（log-odds）表示每个格子的置信度：命中格子使用 l_hit 增量，射线经过的格子使用 l_miss 递减。
- 提供 observed_mask（被观测到的格子）以便可视化区分 explored/unknown。
- 保持之前的分类可视化（未探索：灰，探索空闲：白，墙体按置信度黑色渐变）。

函数：
- load_polar_json(path)
- polar_to_cart(points, angle_unit='degree')
- bresenham_line(x0,y0,x1,y1)  # integer grid Bresenham
- gridify_json_to_grid(json_path, ..., do_raycast=True) -> (grid, meta, observed_mask)
- visualize_grid_classified(grid, meta=None, observed_mask=None, ...)  # unchanged接口

main()：示例/测试，若无 o.json 生成 demo 数据，运行栅格化并可视化。
"""

import json
import math
import os
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt


# ------------------ 辅助：加载与坐标转换 ------------------

def load_polar_json(path):
    with open(path, 'r', encoding='utf8') as f:
        obj = json.load(f)
    if isinstance(obj, list):
        pts = obj
    elif isinstance(obj, dict):
        if 'points' in obj and isinstance(obj['points'], list):
            pts = obj['points']
        elif 'data' in obj and isinstance(obj['data'], list):
            pts = obj['data']
        elif 'angle' in obj and isinstance(obj['angle'], list) and 'distance' in obj and isinstance(obj['distance'], list):
            pts = [{'angle': a, 'distance': d} for a, d in zip(obj['angle'], obj['distance'])]
        else:
            found = None
            for v in obj.values():
                if isinstance(v, list) and len(v) > 0 and isinstance(v[0], dict) and 'angle' in v[0] and 'distance' in v[0]:
                    found = v
                    break
            if found is None:
                raise ValueError(f"Unrecognized JSON structure in {path}")
            pts = found
    else:
        raise ValueError(f"Unsupported JSON structure in {path}")
    cleaned = []
    for p in pts:
        if isinstance(p, dict) and 'angle' in p and 'distance' in p:
            try:
                cleaned.append({'angle': float(p['angle']), 'distance': float(p['distance'])})
            except Exception:
                continue
    if len(cleaned) == 0:
        raise ValueError(f"No valid points found in {path}")
    return cleaned


def polar_to_cart(points, angle_unit='degree'):
    coords = []
    for p in points:
        a = float(p['angle'])
        r = float(p['distance'])
        theta = math.radians(a) if str(angle_unit).lower().startswith('deg') else a
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        coords.append([x, y])
    return np.array(coords, dtype=float)


# ------------------ Bresenham (integer grid line) ------------------

def bresenham_line(x0, y0, x1, y1):
    """Yield integer grid coordinates along a line from (x0,y0) to (x1,y1).
    Yields every cell along the line excluding the endpoint (useful for marking misses),
    and the caller should separately handle the endpoint as a hit.
    All inputs must be integers.
    """
    x0 = int(x0); y0 = int(y0); x1 = int(x1); y1 = int(y1)
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x1 >= x0 else -1
    sy = 1 if y1 >= y0 else -1
    if dx >= dy:
        err = dx // 2
        while x != x1:
            yield x, y
            x += sx
            err -= dy
            if err < 0:
                y += sy
                err += dx
    else:
        err = dy // 2
        while y != y1:
            yield x, y
            y += sy
            err -= dx
            if err < 0:
                x += sx
                err += dy
    return


# ------------------ 栅格化函数：射线更新 (ray-cast) + 点命中 ------------------

def gridify_json_to_grid(json_path,
                         res=50,
                         margin=1000,
                         angle_unit='degree',
                         use_logodds=True,
                         l0=0.0,
                         l_hit=0.9,
                         l_miss=-0.4,
                         l_min=-4.0,
                         l_max=4.0,
                         do_raycast=True,
                         save_npy=True,
                         out_npy_path=None,
                         return_observed_mask=True):
    """
    将 json_path 中的极坐标点栅格化；默认使用射线更新来区分 observed/unobserved。

    返回 (grid, meta, observed_mask) if return_observed_mask else (grid, meta).

    参数要点：
      - do_raycast: 如果 True，会对每个激光束从 sensor -> hit 进行 Bresenham 射线遍历，
                   对射线经过的格子应用 l_miss（表示 free observation），对命中格子应用 l_hit。
      - l_miss 应为负值，l_hit 为正值。

    注意：sensor 原点假定为 (0,0)（即极坐标转换的中心）。如果你的扫描中心不在 (0,0)，
    请传入适当的坐标变换并在调用前把点变换到期望参考系。
    """
    if not os.path.isfile(json_path):
        raise FileNotFoundError(json_path)

    polar = load_polar_json(json_path)
    pts = polar_to_cart(polar, angle_unit=angle_unit)

    if pts.shape[0] == 0:
        raise ValueError('No points to gridify')

    xs = pts[:, 0]
    ys = pts[:, 1]

    xmin = float(xs.min()) - margin
    xmax = float(xs.max()) + margin
    ymin = float(ys.min()) - margin
    ymax = float(ys.max()) + margin

    W = int(np.ceil((xmax - xmin) / res))
    H = int(np.ceil((ymax - ymin) / res))

    if W <= 0 or H <= 0:
        raise ValueError('Invalid grid dimensions')

    # world -> grid index
    def world_to_grid(x, y):
        i = int(math.floor((x - xmin) / res))
        j = int(math.floor((y - ymin) / res))
        return i, j

    # initialize grid (row-major: [y, x])
    if use_logodds:
        grid = np.full((H, W), l0, dtype=np.float32)
    else:
        grid = np.zeros((H, W), dtype=np.int32)

    observed = np.zeros((H, W), dtype=np.bool_)

    # sensor origin in world coords is (0,0)
    sx, sy = 0.0, 0.0
    si, sj = world_to_grid(sx, sy)
    # if sensor is outside grid, clip to boundary so Bresenham still works
    si = max(0, min(W - 1, si))
    sj = max(0, min(H - 1, sj))

    # iterate beams
    added = 0
    for (x, y) in pts:
        ti, tj = world_to_grid(x, y)
        # skip points outside grid
        if not (0 <= ti < W and 0 <= tj < H):
            continue
        if do_raycast:
            # iterate over cells along the ray excluding the endpoint
            for cx, cy in bresenham_line(si, sj, ti, tj):
                if 0 <= cx < W and 0 <= cy < H:
                    # miss update
                    if use_logodds:
                        grid[cy, cx] += l_miss
                        if grid[cy, cx] < l_min:
                            grid[cy, cx] = l_min
                        elif grid[cy, cx] > l_max:
                            grid[cy, cx] = l_max
                    else:
                        grid[cy, cx] -= 1
                    observed[cy, cx] = True
        # mark hit cell
        if use_logodds:
            grid[tj, ti] += l_hit
            if grid[tj, ti] < l_min:
                grid[tj, ti] = l_min
            elif grid[tj, ti] > l_max:
                grid[tj, ti] = l_max
        else:
            grid[tj, ti] += 1
        observed[tj, ti] = True
        added += 1

    meta = {
        'xmin': xmin, 'xmax': xmax, 'ymin': ymin, 'ymax': ymax,
        'res': res, 'W': W, 'H': H,
        'angle_unit': angle_unit,
        'created_at': datetime.utcnow().isoformat() + 'Z',
        'num_points_processed': int(pts.shape[0]),
        'num_points_added': int(added),
        'use_logodds': bool(use_logodds),
        'do_raycast': bool(do_raycast)
    }

    if save_npy:
        if out_npy_path is None:
            base = os.path.splitext(os.path.basename(json_path))[0]
            out_npy_path = f"map_{base}.npy"
        np.save(out_npy_path, grid)
        meta['saved_npy'] = out_npy_path

    if return_observed_mask:
        return grid, meta, observed
    return grid, meta


# ------------------ 分类可视化函数（快速） ------------------

def visualize_grid_classified(grid, meta=None, observed_mask=None, occ_threshold=0.6, out_png=None, show=True, max_display_size=800):
    """
    快速可视化栅格并区分：
      - 未探索（unobserved）: 淡灰色
      - 已探索 & 空闲（explored free）: 白色
      - 已探索 & 占据（wall/occupied）: 按置信度从深灰到黑渐变（置信度越高越黑）

    现在 observed_mask 在栅格化时由射线更新产生（miss 和 hit 都会标记为 observed）。
    """
    if not isinstance(grid, np.ndarray):
        grid = np.array(grid)

    H, W = grid.shape

    # 推断 observed mask
    if observed_mask is None:
        if isinstance(meta, dict) and meta.get('use_logodds', True):
            l0 = 0.0
            observed_mask = (grid != l0)
        else:
            observed_mask = (grid != 0)

    # 下采样步长（控制最大显示像素）
    max_side = max(H, W)
    step = max(1, math.ceil(max_side / float(max_display_size)))

    # 生成展示用数组（RGB）
    disp_h = (H + step - 1) // step
    disp_w = (W + step - 1) // step
    img = np.zeros((disp_h, disp_w, 3), dtype=np.uint8)

    # 计算概率图 p (0..1) 如果是 log-odds
    is_log = True
    if isinstance(meta, dict):
        is_log = bool(meta.get('use_logodds', True))
    if is_log:
        # map log-odds to p safely
        L = grid
        L_clipped = np.clip(L, -10.0, 10.0)
        p = 1.0 - 1.0 / (1.0 + np.exp(L_clipped))
    else:
        arr = grid.astype(np.float32)
        lo, hi = arr.min(), arr.max()
        if hi - lo < 1e-9:
            p = np.zeros_like(arr)
        else:
            p = (arr - lo) / (hi - lo)

    # Colors
    COLOR_UNOBS = np.array([180, 180, 180], dtype=np.uint8)  # 未探索：中灰
    COLOR_FREE = np.array([255, 255, 255], dtype=np.uint8)   # 自由区：白

    # 对每个下采样格子决定颜色（向下映射到原格子集合，采用简单取样）
    for yi in range(disp_h):
        for xi in range(disp_w):
            y0 = yi * step
            x0 = xi * step
            y1 = min((yi + 1) * step, H)
            x1 = min((xi + 1) * step, W)
            # 索引块
            block_obs = observed_mask[y0:y1, x0:x1]
            block_p = p[y0:y1, x0:x1]

            # 如果块内没有被观测到的格子 -> 未探索
            if not block_obs.any():
                img[yi, xi] = COLOR_UNOBS
                continue

            # 若有观测：优先判断是否存在占据格子（置信度高于阈值）
            block_p_obs = block_p[block_obs]
            if block_p_obs.size == 0:
                img[yi, xi] = COLOR_UNOBS
                continue

            max_p = float(block_p_obs.max())
            # 若最大的占据概率低于阈值，视为自由区（白色）
            if max_p <= occ_threshold:
                img[yi, xi] = COLOR_FREE
            else:
                # 否则视为墙体（occupied），按置信度从深灰到黑渐变
                intensity = int(max(0, min(255, (1.0 - max_p) * 255.0)))
                img[yi, xi] = np.array([intensity, intensity, intensity], dtype=np.uint8)

    # 翻转 Y 轴以便上方显示为 y 增加方向
    img_disp = np.flipud(img)

    plt.figure(figsize=(6, 6 * img_disp.shape[0] / max(1, img_disp.shape[1])))
    plt.imshow(img_disp, interpolation='nearest')
    plt.axis('off')
    plt.title(f"Occupancy Grid classified (size={H}x{W}, step={step})")

    if out_png:
        plt.imsave(out_png, img_disp)

    if show:
        plt.show()
    else:
        plt.show(block=False)
        plt.pause(0.001)

    # 检查并报告是否有已探索自由区
    explored_cells = int(observed_mask.sum())
    total = H * W
    has_explored_free = ((observed_mask) & (p <= occ_threshold)).any()
    print(f"Grid cells: {total}, explored cells: {explored_cells} ({explored_cells/total*100:.2f}%)")
    if not has_explored_free:
        print("注意：未检测到已探索自由区 (explored free cells)。如果你的传感器数据很稀疏，可能仍然看不到大量 free cells。")

    return img_disp


# ------------------ main: 测试 ------------------
if __name__ == '__main__':
    TEST_JSON = 'o.json'
    RES = 50
    MARGIN = 1000

    if not os.path.isfile(TEST_JSON):
        print(f"{TEST_JSON} not found — generating a demo polar JSON with circular arc.")
        demo = []
        for deg in range(0, 360, 2):
            r = 3.0 + 0.5 * math.sin(math.radians(deg * 3))
            demo.append({'angle': float(deg), 'distance': float(r)})
        with open(TEST_JSON, 'w', encoding='utf8') as f:
            json.dump({'points': demo}, f, ensure_ascii=False, indent=2)
        print(f"Wrote demo polar points to {TEST_JSON}")

    grid, meta, observed = gridify_json_to_grid(TEST_JSON, res=RES, margin=MARGIN, angle_unit='degree', save_npy=True, do_raycast=True)
    print('Meta:', json.dumps(meta, indent=2, ensure_ascii=False))

    # 可视化（分类渲染）
    img = visualize_grid_classified(grid, meta=meta, observed_mask=observed, occ_threshold=0.6, out_png=f"map_{os.path.splitext(os.path.basename(TEST_JSON))[0]}.png", show=True)
    print('Visualization done.')
