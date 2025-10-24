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

使用示例（脚本直接运行）：
  - 编辑 SOURCE_PATH, TARGET_PATH 后运行，将会：
      * 把 TARGET_PATH (n.json) 对齐到 SOURCE_PATH (o.json)，并把对齐后的 n.json 覆盖写回（点云在 o.json 的原点下）
      * 保存可视化图像 alignment_result.png
      * 在终端打印对齐信息（包括 object 相对于 global 的位置示例）

函数 API：
- do_align(from_path, to_path, angle_unit='degree', max_iter=100)
    返回包含 A,B,R,t,A_aligned,errors,rotation_deg,rms_error 等字段的字典。

- align_and_overwrite(reference_path, path_to_align, angle_unit='degree', max_iter=100, out_image=None, show_plot=False)
    把 path_to_align 对齐到 reference_path 并覆盖写回 path_to_align。返回元信息字典（rotation_deg, translation,...）。

- align_and_get_object_coords(global_map_path, object_path, angle_unit='degree', max_iter=100)
    将 object 对齐到 global_map，返回 object 原点在 global 坐标系下的位置等信息。

"""

import json
import math
import os
import numpy as np
import matplotlib.pyplot as plt

# -------------------- 默认参数（可在调用时覆盖） --------------------
DEFAULT_MAX_ITER = 100
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
        dists, idxs = tree.query(src)
        return idxs, dists
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


def icp(A, B, max_iterations=50, tolerance=1e-7):
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
        mean_error = np.mean(dists)
        errors.append(mean_error)
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
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
    """
    src_pts = load_polar_json(from_path)
    tgt_pts = load_polar_json(to_path)

    A = polar_to_cart(src_pts, angle_unit=angle_unit)  # A = to-align
    B = polar_to_cart(tgt_pts, angle_unit=angle_unit)  # B = reference

    R, t, A_aligned, errors = icp(A, B, max_iterations=max_iter)
    rot_deg = rotation_matrix_to_deg(R)
    rms_error = (np.mean(np.array(errors) ** 2)) ** 0.5 if len(errors) > 0 else 0.0

    return {
        'A': A, 'B': B,
        'R': R, 't': t,
        'A_aligned': A_aligned,
        'errors': errors,
        'rotation_deg': float(rot_deg),
        'rms_error': float(rms_error),
        'num_points_A': int(A.shape[0]),
        'num_points_B': int(B.shape[0])
    }


# ----------------------- 对外 API -----------------------

def align_and_overwrite(reference_path, path_to_align, angle_unit='degree', max_iter=DEFAULT_MAX_ITER, out_image=None, show_plot=False):
    """
    修改版 align_and_overwrite：
    - 对齐 path_to_align -> reference_path（与之前相同）
    - 写回 reference_path 时：
        * 不会将任何原点（distance ~= 0）的点写入普通点列表
        * 保留 reference 原本的原点（如果存在），并以 `_origin_point` 字段单独记录
    - 额外行为：会**清理并覆盖写回 path_to_align 文件本身**，移除其中所有 distance ~= 0 的原点条目（保留其它内容与结构）
    - 返回与之前相同的元信息字典
    """
    from datetime import datetime

    ORIGIN_EPS = 1e-8  # 判定为原点的距离阈值

    # 先做对齐计算（保持原有行为）
    res = do_align(path_to_align, reference_path, angle_unit=angle_unit, max_iter=max_iter)

    # 将对齐后的笛卡尔点转换为极坐标
    polar_aligned = cart_to_polar(res['A_aligned'], angle_unit=angle_unit)

    # helper: 判定是否为原点
    def is_origin_point_dict(p):
        try:
            return float(p.get('distance', 0.0)) <= ORIGIN_EPS
        except Exception:
            return False

    # 过滤函数：给一个点列表，返回过滤掉原点后的新列表
    def filter_point_list(lst):
        if not isinstance(lst, list):
            return lst
        return [p for p in lst if not (isinstance(p, dict) and 'distance' in p and is_origin_point_dict(p))]

    # ---------- 1) 读取并清理 reference 文件（构建要写回的 ref_obj） ----------
    try:
        with open(reference_path, 'r', encoding='utf8') as f:
            ref_obj = json.load(f)
    except Exception:
        ref_obj = {}

    # 先把对齐结果中靠近原点的点过滤掉（不写入 reference）
    polar_aligned_filtered = [p for p in polar_aligned if float(p.get('distance', 0.0)) > ORIGIN_EPS]

    added_count = 0

    # 对 ref_obj 根据类型做处理：list / dict / 其它
    if isinstance(ref_obj, list):
        # 如果顶层是 list：先提取原有原点（若存在），然后把过滤后的 aligned 点追加到 points
        original_points = filter_point_list(ref_obj[:])  # 过滤掉原点
        origin = None
        # 尝试从原始 list 中找到一个原点（distance ~= 0）
        for item in ref_obj:
            if isinstance(item, dict) and 'distance' in item and is_origin_point_dict(item):
                origin = item
                break
        if origin is None:
            origin = {'angle': 0.0, 'distance': 0.0}
        # 合并：追加对齐点（不包含原点）
        original_points.extend(polar_aligned_filtered)
        added_count = len(polar_aligned_filtered)
        # 升级为 dict 以便同时保存 points 与 _origin_point
        ref_obj = {
            'points': original_points,
            '_origin_point': {
                'angle': float(origin.get('angle', 0.0)),
                'distance': float(origin.get('distance', 0.0)),
                'type': 'reference_origin',
                'source_file': os.path.basename(reference_path),
                'timestamp': datetime.utcnow().isoformat() + 'Z'
            }
        }
    elif isinstance(ref_obj, dict):
        # 在 dict 中寻找一个“点列表”键（含 angle/distance 的 dict 列表）
        found_key = None
        for k, v in list(ref_obj.items()):
            if isinstance(v, list) and len(v) > 0 and isinstance(v[0], dict) and 'angle' in v[0] and 'distance' in v[0]:
                found_key = k
                break

        origin = None
        if found_key:
            # 先过滤 reference 自身的原点并更新该列表
            cleaned = filter_point_list(ref_obj[found_key])
            # 尝试寻找并保留原点信息（若原列表中有）
            for item in ref_obj[found_key]:
                if isinstance(item, dict) and 'distance' in item and is_origin_point_dict(item):
                    origin = item
                    break
            # 将对齐后的点（已过滤原点）追加
            cleaned.extend(polar_aligned_filtered)
            ref_obj[found_key] = cleaned
            added_count = len(polar_aligned_filtered)
        else:
            # 没找到点列表，创建一个新键保存合并点（不包含原点）
            key = f"merged_points_from_{os.path.basename(path_to_align)}"
            if key not in ref_obj or not isinstance(ref_obj.get(key), list):
                ref_obj[key] = []
            ref_obj[key].extend(polar_aligned_filtered)
            added_count = len(polar_aligned_filtered)

        if origin is None:
            # 如果 reference 中没有显式原点，使用默认原点
            origin = {'angle': 0.0, 'distance': 0.0}

        # 写入或更新专用的 _origin_point 字段
        ref_obj['_origin_point'] = {
            'angle': float(origin.get('angle', 0.0)),
            'distance': float(origin.get('distance', 0.0)),
            'type': 'reference_origin',
            'source_file': os.path.basename(reference_path),
            'timestamp': datetime.utcnow().isoformat() + 'Z'
        }

        # 维护合并元信息（保留原有行为）
        meta = {
            'source_file': os.path.basename(path_to_align),
            'timestamp': datetime.utcnow().isoformat() + 'Z',
            'rotation_deg': res['rotation_deg'],
            'translation': {'x': float(res['t'][0]), 'y': float(res['t'][1])},
            'angle_unit': angle_unit,
            'num_points_added': len(polar_aligned_filtered)
        }
        if '_merge_meta' not in ref_obj or not isinstance(ref_obj.get('_merge_meta'), list):
            ref_obj['_merge_meta'] = []
        ref_obj['_merge_meta'].append(meta)
    else:
        # 其它类型：把现有点尽量归并到 points，并写入 _origin_point 与 _merge_meta
        existing_points = []
        # 尝试从 ref_obj 中抽取第一组可能的点列表作为基础
        for v in (ref_obj.values() if isinstance(ref_obj, dict) else []):
            if isinstance(v, list) and len(v) > 0 and isinstance(v[0], dict) and 'angle' in v[0] and 'distance' in v[0]:
                existing_points = filter_point_list(v[:])
                break
        existing_points.extend(polar_aligned_filtered)
        added_count = len(polar_aligned_filtered)
        ref_obj = {
            'points': existing_points,
            '_origin_point': {
                'angle': 0.0,
                'distance': 0.0,
                'type': 'reference_origin',
                'source_file': os.path.basename(reference_path),
                'timestamp': datetime.utcnow().isoformat() + 'Z'
            },
            '_merge_meta': [{
                'source_file': os.path.basename(path_to_align),
                'timestamp': datetime.utcnow().isoformat() + 'Z',
                'rotation_deg': res['rotation_deg'],
                'translation': {'x': float(res['t'][0]), 'y': float(res['t'][1])},
                'angle_unit': angle_unit,
                'num_points_added': len(polar_aligned_filtered)
            }]
        }

    # ---------- 2) 写回 reference_path ----------
    with open(reference_path, 'w', encoding='utf8') as f:
        json.dump(ref_obj, f, ensure_ascii=False, indent=2)

    # ---------- 3) 清理并写回 path_to_align 文件本身（移除其原点条目） ----------
    try:
        with open(path_to_align, 'r', encoding='utf8') as f:
            align_obj = json.load(f)
    except Exception:
        align_obj = None

    if align_obj is not None:
        # 如果顶层是 list，直接过滤并写回 list
        if isinstance(align_obj, list):
            cleaned_list = filter_point_list(align_obj)
            with open(path_to_align, 'w', encoding='utf8') as f:
                json.dump(cleaned_list, f, ensure_ascii=False, indent=2)
        elif isinstance(align_obj, dict):
            # 遍历 dict 中所有可能的点列表并过滤原点
            changed = False
            for k, v in list(align_obj.items()):
                if isinstance(v, list) and len(v) > 0 and isinstance(v[0], dict) and 'angle' in v[0] and 'distance' in v[0]:
                    new_list = filter_point_list(v)
                    if len(new_list) != len(v):
                        align_obj[k] = new_list
                        changed = True
            # 如果没有找到任何点列表（即没有角度/距离结构），也不做任何改动
            # 最后写回（如果 changed 或为了保证一致性也可以总是写回）
            with open(path_to_align, 'w', encoding='utf8') as f:
                json.dump(align_obj, f, ensure_ascii=False, indent=2)
        else:
            # 其它类型不改写（保持原样）
            pass

    # 如果需要绘图，则如前显示/保存（不变）
    if out_image or show_plot:
        plot_results(res['A'], res['B'], res['A_aligned'], res['t'], out_path=out_image, show_plot=show_plot)

    return {
        'rotation_deg': res['rotation_deg'],
        'translation': [float(res['t'][0]), float(res['t'][1])],
        'rms_error': res['rms_error'],
        'num_points_added': added_count,
        'written_path': reference_path,
        'cleaned_input_path': path_to_align
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
