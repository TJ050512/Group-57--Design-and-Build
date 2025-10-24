#!/usr/bin/env python3
"""
match_icp_functions.py - 修改后的完整脚本

主要改动：
1) 改为以第一个参数（reference）为参考，把第二个参数（path_to_align）对齐到第一个。
   也就是说：在 main 中保持 SOURCE_PATH='o.json', TARGET_PATH='n.json' 时，
   调用 align_and_overwrite(SOURCE_PATH, TARGET_PATH) 将会把 n.json 对齐到 o.json 并覆盖写回 n.json。

2) 抽取了 do_align() helper 以复用加载/转换/ICP 逻辑，减少重复。

3) 在 __main__ 中增加了对 align_and_get_object_coords 的测试示例（直接运行脚本会打印两个函数的返回结果并保存对齐结果文件）。

4) 覆盖写回的 aligned 文件包含了 points 和 meta（包含 rotation_deg、translation、angle_unit、reference_file）。

说明（旋转改进）：
- 在 ICP 迭代得到最终对齐后的点云 A_aligned 之后，使用一次 best_fit_transform(A_original, A_aligned)
  来求解整体刚性变换 R_final, t_final，然后从 R_final 提取旋转角度并把 t_final 当作平移量返回。
- 增加 PCA 粗对齐并尝试 delta / delta+π 两种初始化以避免 180° 落入错误局部最优。
- 其余行为和接口保持不变（仅替换 do_align 的实现）。
"""

import json
import math
import os
import numpy as np
import matplotlib.pyplot as plt

# -------------------- 默认参数（可在调用时覆盖） --------------------
DEFAULT_MAX_ITER = 10000
# ------------------------------------------------------------------

try:
    from scipy.spatial import cKDTree as KDTree
    _HAVE_KDTREE = True
except Exception:
    _HAVE_KDTREE = False


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


def cart_to_polar(coords, angle_unit='degree'):
    pts = []
    for x, y in coords:
        r = math.hypot(x, y)
        theta = math.atan2(y, x)
        if str(angle_unit).lower().startswith('deg'):
            a = math.degrees(theta) % 360.0
        else:
            a = theta % (2 * math.pi)
        pts.append({'angle': float(a), 'distance': float(r)})
    return pts


def nearest_neighbors(src, dst):
    if _HAVE_KDTREE:
        tree = KDTree(dst)
        # scipy query returns (dists, idxs)
        dists, idxs = tree.query(src)
        return idxs.astype(int), dists.astype(float)
    else:
        idxs = []
        dists = []
        for s in src:
            d2 = np.sum((dst - s) ** 2, axis=1)
            idx = int(np.argmin(d2))
            idxs.append(idx)
            dists.append(math.sqrt(d2[idx]))
        return np.array(idxs, dtype=int), np.array(dists, dtype=float)


def best_fit_transform(A, B):
    assert A.shape == B.shape
    if A.shape[0] == 0:
        return np.eye(2), np.zeros(2)
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = centroid_B - R @ centroid_A
    return R, t


def icp(A, B, max_iterations=1000, tolerance=1e-8):
    src = A.copy()
    prev_error = float('inf')
    errors = []
    total_R = np.eye(2)
    total_t = np.zeros(2)
    for i in range(max_iterations):
        idxs, dists = nearest_neighbors(src, B)
        B_corr = B[idxs]
        R, t = best_fit_transform(src, B_corr)
        src = (R @ src.T).T + t
        total_R = R @ total_R
        total_t = R @ total_t + t
        mean_error = np.mean(dists) if len(dists) > 0 else 0.0
        errors.append(mean_error)
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
    # 返回累计变换与最终对齐点以及迭代误差列表
    return total_R, total_t, src, errors


def rotation_matrix_to_deg(R):
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def plot_results(A_orig, B_orig, A_aligned, t, out_path=None, show_plot=False):
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    ax1, ax2 = axes

    ax1.scatter(B_orig[:, 0], B_orig[:, 1], label='Reference (B)', marker='o')
    ax1.scatter(A_orig[:, 0], A_orig[:, 1], label='To-align (A)', marker='x')
    ax1.set_title('Original clouds')
    ax1.axis('equal')
    ax1.legend()
    ax1.grid(True, linestyle=':', alpha=0.4)

    ax2.scatter(B_orig[:, 0], B_orig[:, 1], label='Reference (B)', marker='o')
    ax2.scatter(A_aligned[:, 0], A_aligned[:, 1], label='Aligned -> Reference', marker='x')
    ax2.scatter([0.0], [0.0], marker='s', s=80, label='Reference origin (0,0)')
    ax2.scatter([t[0]], [t[1]], marker='D', s=80, label='To-align origin in reference coords')
    ax2.annotate('', xy=(t[0], t[1]), xytext=(0, 0), arrowprops=dict(arrowstyle='->', lw=2))
    ax2.text(t[0], t[1], f"  ({t[0]:.3f},{t[1]:.3f})")

    ax2.set_title('Aligned (To-align -> Reference)')
    ax2.axis('equal')
    ax2.legend()
    ax2.grid(True, linestyle=':', alpha=0.4)

    plt.tight_layout()
    if out_path:
        plt.savefig(out_path, dpi=200)
    if show_plot:
        plt.show()
    plt.close(fig)


# ----------------------- 复用的 helper -----------------------

def do_align(from_path, to_path, angle_unit='degree', max_iter=DEFAULT_MAX_ITER):
    """
    把 from_path 的点云对齐到 to_path（求变换 R,t 使得 R*from + t -> to），返回包含中间结果的字典。

    改进点（仅此处修改）：
      - 使用 PCA 粗对齐（主轴对齐）得到初始旋转 delta；
      - 处理 delta 与 delta+pi 两种情况（解决 180° 歧义）——对每种初始化跑 ICP；
      - 合并初始变换与 ICP 返回的局部变换得到最终 R_final,t_final；
      - 选取 RMS 更小的初始化结果作为最终输出。
    """
    src_pts = load_polar_json(from_path)
    tgt_pts = load_polar_json(to_path)

    A = polar_to_cart(src_pts, angle_unit=angle_unit)  # A = to-align
    B = polar_to_cart(tgt_pts, angle_unit=angle_unit)  # B = reference

    if A.shape[0] == 0 or B.shape[0] == 0:
        # 没点的情况回退到原实现（会在别处抛错）
        R_acc, t_acc, A_aligned, errors = icp(A, B, max_iterations=max_iter)
        # 计算 final dists
        _, final_dists = nearest_neighbors(A_aligned, B)
        rms_error = float(np.sqrt(np.mean(np.array(final_dists) ** 2))) if len(final_dists) > 0 else 0.0
        rot_deg = rotation_matrix_to_deg(R_acc)
        return {
            'A': A, 'B': B,
            'R': R_acc, 't': t_acc,
            'A_aligned': A_aligned,
            'errors': errors,
            'rotation_deg': float(rot_deg),
            'rms_error': float(rms_error),
            'num_points_A': int(A.shape[0]),
            'num_points_B': int(B.shape[0])
        }

    # 计算质心与中心化坐标
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    A_centered = A - centroid_A
    B_centered = B - centroid_B

    # PCA：求主方向（最大的特征值对应的特征向量）
    def principal_angle(points_centered):
        # 2x2 协方差
        C = np.cov(points_centered.T)
        # 特征向量
        try:
            w, v = np.linalg.eig(C)
            # 选择最大特征值对应的特征向量
            idx = int(np.argmax(w))
            vec = v[:, idx]
            ang = math.atan2(vec[1], vec[0])
            return ang
        except Exception:
            return None

    angle_A = principal_angle(A_centered)
    angle_B = principal_angle(B_centered)

    # 如果 PCA 失败，退回到不做粗对齐的 ICP
    init_candidates = []
    if angle_A is None or angle_B is None:
        init_candidates.append((np.eye(2), np.zeros(2)))  # 无初始变换
    else:
        delta = angle_B - angle_A
        for add in [0.0, math.pi]:  # delta, delta+pi 两种情况
            theta = delta + add
            R_init = np.array([[math.cos(theta), -math.sin(theta)],
                               [math.sin(theta),  math.cos(theta)]], dtype=float)
            t_init = centroid_B - (R_init @ centroid_A)
            init_candidates.append((R_init, t_init))

    best = None
    # 遍历每个初始化，跑 ICP（从 A_init 开始），合并变换并计算 RMS，选择最优的
    for R_init, t_init in init_candidates:
        # apply initial transform to A to get A_init
        A_init = (R_init @ A.T).T + t_init
        # run ICP starting from A_init (icp will treat A_init as "source")
        try:
            R_icp, t_icp, A_aligned, errors = icp(A_init, B, max_iterations=max_iter)
        except Exception:
            continue

        # 合并变换：整体变换 R_final, t_final，满足 R_final * A + t_final = A_aligned
        # 因为 A_init = R_init * A + t_init
        # ICP 返回 A_aligned = R_icp * A_init + t_icp = R_icp*(R_init*A + t_init) + t_icp
        # 所以 R_final = R_icp @ R_init ; t_final = R_icp @ t_init + t_icp
        R_final = R_icp @ R_init
        t_final = (R_icp @ t_init) + t_icp

        # 计算 A_aligned 到参考 B 的最近邻距离用于 RMS
        _, final_dists = nearest_neighbors(A_aligned, B)
        rms_error = float(np.sqrt(np.mean(np.array(final_dists) ** 2))) if len(final_dists) > 0 else float('inf')

        # 记录候选解
        cand = {
            'R_final': R_final,
            't_final': t_final,
            'A_aligned': A_aligned,
            'errors': errors,
            'rms': rms_error
        }
        if best is None or cand['rms'] < best['rms']:
            best = cand

    # 如果没有任何候选（极少见），退回直接用不初始化的 ICP
    if best is None:
        R_acc, t_acc, A_aligned, errors = icp(A, B, max_iterations=max_iter)
        _, final_dists = nearest_neighbors(A_aligned, B)
        rms_error = float(np.sqrt(np.mean(np.array(final_dists) ** 2))) if len(final_dists) > 0 else 0.0
        rot_deg = rotation_matrix_to_deg(R_acc)
        return {
            'A': A, 'B': B,
            'R': R_acc, 't': t_acc,
            'A_aligned': A_aligned,
            'errors': errors,
            'rotation_deg': float(rot_deg),
            'rms_error': float(rms_error),
            'num_points_A': int(A.shape[0]),
            'num_points_B': int(B.shape[0])
        }

    # 使用最佳候选结果作为输出
    R_out = best['R_final']
    t_out = best['t_final']
    A_aligned_out = best['A_aligned']
    errors_out = best['errors']
    rms_error_out = float(best['rms'])
    rot_deg = rotation_matrix_to_deg(R_out)

    return {
        'A': A, 'B': B,
        'R': R_out, 't': t_out,
        'A_aligned': A_aligned_out,
        'errors': errors_out,
        'rotation_deg': float(rot_deg),
        'rms_error': float(rms_error_out),
        'num_points_A': int(A.shape[0]),
        'num_points_B': int(B.shape[0])
    }


# ----------------------- 对外 API -----------------------

def align_and_overwrite(reference_path, path_to_align, angle_unit='degree', max_iter=DEFAULT_MAX_ITER, out_image=None, show_plot=False):
    """
    将 path_to_align 对齐到 reference_path（即把 path_to_align 的点云放到 reference 的坐标系），
    并**把对齐后的点云追加写入 reference_path**（保留 reference 原有数据）。返回对齐信息字典。

    行为说明：
      - 不再覆盖被对齐文件（path_to_align），而是把对齐结果追加到 reference_path 中的合适位置。
      - 如果 reference_path 是一个 dict 并包含一个合法的点云列表键（例如 'points'），对齐点将扩展到该列表中。
      - 如果无法在 reference JSON 中找到合适的点列表键，会创建一个新的键：
            'merged_points_from_<basename>' 并把对齐点写入其中。
      - 同时会在 reference JSON 中添加或扩展一个 '_merge_meta' 列表，记录合并来源与变换信息。
    """
    from datetime import datetime

    res = do_align(path_to_align, reference_path, angle_unit=angle_unit, max_iter=max_iter)

    # 将对齐后的笛卡尔点转换为极坐标
    polar_aligned = cart_to_polar(res['A_aligned'], angle_unit=angle_unit)

    # 读取 reference JSON，保留原有数据并追加
    try:
        with open(reference_path, 'r', encoding='utf8') as f:
            ref_obj = json.load(f)
    except Exception:
        # 如果读取失败（例如空文件），初始化一个 dict
        ref_obj = {}

    added_count = 0
    # 如果 reference 是列表，直接当作点列表并扩展
    if isinstance(ref_obj, list):
        ref_obj.extend(polar_aligned)
        added_count = len(polar_aligned)
    elif isinstance(ref_obj, dict):
        # 尝试找到已有的点列表键
        found_key = None
        for k, v in list(ref_obj.items()):
            if isinstance(v, list) and len(v) > 0 and isinstance(v[0], dict) and 'angle' in v[0] and 'distance' in v[0]:
                found_key = k
                break
        if found_key:
            ref_obj[found_key].extend(polar_aligned)
            added_count = len(polar_aligned)
        else:
            # 新建一个键来存放合并点
            key = f"merged_points_from_{os.path.basename(path_to_align)}"
            if key not in ref_obj or not isinstance(ref_obj.get(key), list):
                ref_obj[key] = []
            ref_obj[key].extend(polar_aligned)
            added_count = len(polar_aligned)

        # 维护或添加合并元信息
        meta = {
            'source_file': os.path.basename(path_to_align),
            'timestamp': datetime.utcnow().isoformat() + 'Z',
            'rotation_deg': res['rotation_deg'],
            'translation': {'x': float(res['t'][0]), 'y': float(res['t'][1])},
            'angle_unit': angle_unit,
            'num_points_added': len(polar_aligned)
        }
        if '_merge_meta' not in ref_obj or not isinstance(ref_obj.get('_merge_meta'), list):
            ref_obj['_merge_meta'] = []
        ref_obj['_merge_meta'].append(meta)
    else:
        # 其它类型：创建一个包含 points 的新 dict
        ref_obj = {
            'points': polar_aligned,
            '_merge_meta': [{
                'source_file': os.path.basename(path_to_align),
                'timestamp': datetime.utcnow().isoformat() + 'Z',
                'rotation_deg': res['rotation_deg'],
                'translation': {'x': float(res['t'][0]), 'y': float(res['t'][1])},
                'angle_unit': angle_unit,
                'num_points_added': len(polar_aligned)
            }]
        }
        added_count = len(polar_aligned)

    # 写回 reference_path（覆盖原文件，但保留其原有结构与字段，只是在其中追加数据）
    with open(reference_path, 'w', encoding='utf8') as f:
        json.dump(ref_obj, f, ensure_ascii=False, indent=2)

    if out_image or show_plot:
        plot_results(res['A'], res['B'], res['A_aligned'], res['t'], out_path=out_image, show_plot=show_plot)

    return {
        'rotation_deg': res['rotation_deg'],
        'translation': [float(res['t'][0]), float(res['t'][1])],
        'rms_error': res['rms_error'],
        'num_points_added': added_count,
        'written_path': reference_path
    }


def align_and_get_object_coords(global_map_path, object_path, angle_unit='degree', max_iter=DEFAULT_MAX_ITER):
    """
    将 object_path 对齐到 global_map_path，返回 object 原点在 global 地图坐标系下的坐标（即变换中的 t）。
    返回字典包含 'object_origin_in_global', 'rotation_deg', 'translation', 'rms_error', 等。
    """
    res = do_align(object_path, global_map_path, angle_unit=angle_unit, max_iter=max_iter)
    return {
        'object_origin_in_global': [float(res['t'][0]), float(res['t'][1])],
        'rotation_deg': res['rotation_deg'],
        'translation': [float(res['t'][0]), float(res['t'][1])],
        'rms_error': res['rms_error'],
        'num_points_object': res['num_points_A'],
        'num_points_global': res['num_points_B']
    }


# ----------------------- 脚本主流程（示例 / 测试） -----------------------
if __name__ == '__main__':
    # 编辑这两个路径后运行脚本：
    # - 会把 TARGET_PATH (n.json) 对齐到 SOURCE_PATH (o.json) 并覆盖写回 TARGET_PATH
    # - 会保存一张可视化图片 alignment_result.png
    SOURCE_PATH = 'o.json'   # reference (目标参考坐标系)
    TARGET_PATH = 'n.json'   # 需要对齐并覆盖写回的文件

    OUT_IMAGE = 'alignment_result.png'
    MAX_ITER = 100
    SHOW_PLOT = False

    # 检查文件存在
    if not os.path.isfile(SOURCE_PATH):
        print(f"Reference file not found: {SOURCE_PATH}")
    if not os.path.isfile(TARGET_PATH):
        print(f"To-align file not found: {TARGET_PATH}")

    try:
        print('=== Running align_and_overwrite: aligning TARGET -> SOURCE (n.json -> o.json) ===')
        out = align_and_overwrite(SOURCE_PATH, TARGET_PATH, angle_unit='degree', max_iter=MAX_ITER, out_image=OUT_IMAGE, show_plot=SHOW_PLOT)
        print(json.dumps(out, indent=2, ensure_ascii=False))

        print('=== Running align_and_get_object_coords (example): object relative to global map ===')
        info = align_and_get_object_coords(SOURCE_PATH, TARGET_PATH, angle_unit='degree', max_iter=MAX_ITER)
        print(json.dumps(info, indent=2, ensure_ascii=False))

        print(f"Visualization saved to: {OUT_IMAGE}")
    except Exception as e:
        print('Error during alignment:', e)
