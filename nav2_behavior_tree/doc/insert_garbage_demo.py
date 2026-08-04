#!/usr/bin/env python3

from __future__ import annotations

import copy
import itertools
import json
import math
import os
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

# WSL: DISPLAY may be IP:0 and matplotlib goes headless; prefer :0
def _ensure_gui_display() -> None:
  try:
    from matplotlib import _c_internal_utils as _ciu
  except Exception:
    return
  if _ciu.display_is_valid():
    return
  for candidate in (':0', os.environ.get('DISPLAY', '')):
    if not candidate:
      continue
    os.environ['DISPLAY'] = candidate
    if _ciu.display_is_valid():
      return

_ensure_gui_display()

import matplotlib
try:
  matplotlib.use('TkAgg')
except Exception:
  pass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from matplotlib.patches import FancyArrow, Rectangle
from matplotlib.transforms import Affine2D

# UI text uses ASCII only (avoid CJK tofu boxes on some fonts)
matplotlib.rcParams['font.family'] = 'DejaVu Sans'
matplotlib.rcParams['axes.unicode_minus'] = False
# Q/q is our "clear canvas"; disable matplotlib's default quit bindings
matplotlib.rcParams['keymap.quit'] = []
matplotlib.rcParams['keymap.quit_all'] = []

# ---------- mirror C++ InsertGarbagePose defaults ----------
CLIP_EXTEND_M = 1.0
CORNER_ANGLE_DEG = 30.0
GOALTOTAL_RANGE_M = 10.0
HEAD_DELETE_ROBOT_DIST_M = 4.0
ARRIVED_RADIUS = 0.5
# 垃圾离机器人超过该距离则本帧跳过，靠近后再插入（对齐 C++ 持续检测语义）
MAX_GARBAGE_ROBOT_DIST_M = 9.0
DEDUP_DISTANCE_M = 0.15

# ---------- map / robot / sim ----------
MAP_X_MIN, MAP_X_MAX = -20.0, 20.0
MAP_Y_MIN, MAP_Y_MAX = -15.0, 15.0
GRID_STEP = 1.0
ZOOM_STEP = 1.08  # wheel zoom factor per notch (small)
ROBOT_LENGTH_M = 2.0   # along yaw
ROBOT_WIDTH_M = 1.0
MIN_DRAG_M = 0.15      # below this -> default yaw 0

GARBAGE_FINISH_RADIUS_M = 0.6
ROBOT_SPEED = 1.0
REACH_GOAL_RADIUS = 0.25
DT = 0.05
STEP_POINT_MAX_FRAMES = 8000

Point = Tuple[float, float]
CaseTag = str  # '1' | '2' | '3' | '4'


def dist(a: Point, b: Point) -> float:
  return math.hypot(a[0] - b[0], a[1] - b[1])


def dist2(a: Point, b: Point) -> float:
  return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def squared_distance_point_to_segment(
  px: float, py: float,
  ax: float, ay: float,
  bx: float, by: float,
) -> Tuple[float, float]:
  abx, aby = bx - ax, by - ay
  apx, apy = px - ax, py - ay
  ab_len2 = abx * abx + aby * aby
  t = 0.0
  if ab_len2 > 1e-12:
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab_len2))
  qx, qy = ax + t * abx, ay + t * aby
  return (px - qx) ** 2 + (py - qy) ** 2, t


def project_point_to_infinite_line(
  px: float, py: float,
  ax: float, ay: float,
  bx: float, by: float,
) -> Point:
  abx, aby = bx - ax, by - ay
  ab_len2 = abx * abx + aby * aby
  if ab_len2 < 1e-12:
    return (ax, ay)
  t = ((px - ax) * abx + (py - ay) * aby) / ab_len2
  return (ax + t * abx, ay + t * aby)


def line_parameter_t(
  px: float, py: float,
  ax: float, ay: float,
  bx: float, by: float,
) -> float:
  abx, aby = bx - ax, by - ay
  ab_len2 = abx * abx + aby * aby
  if ab_len2 < 1e-12:
    return 0.0
  return ((px - ax) * abx + (py - ay) * aby) / ab_len2


def is_goal_not_corner(
  goals: Sequence[Point], idx: int, robot: Point, corner_angle_deg: float,
) -> bool:
  """对齐 C++ isGoalNotCorner：队首前驱用机器人，中间点用上一 goal。"""
  if not goals or idx >= len(goals) - 1:
    return True
  if idx > 0:
    prev = goals[idx - 1]
  else:
    prev = robot
  cur, nxt = goals[idx], goals[idx + 1]
  v1x, v1y = cur[0] - prev[0], cur[1] - prev[1]
  v2x, v2y = nxt[0] - cur[0], nxt[1] - cur[1]
  len1_sq = v1x * v1x + v1y * v1y
  len2_sq = v2x * v2x + v2y * v2y
  if len1_sq < 1e-6 or len2_sq < 1e-6:
    return True
  cos_theta = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / math.sqrt(len1_sq * len2_sq)))
  return math.acos(cos_theta) <= math.radians(corner_angle_deg)


def find_last_goal_within_path_range(
  goals: Sequence[Point], range_m: float,
) -> Optional[int]:
  """对齐 C++ findLastGoalWithinPathRange：从 goals[0] 沿路径量 range_m，返回末点下标。"""
  if len(goals) < 2 or range_m <= 0.0:
    return None
  accumulated = 0.0
  out_idx = 0
  for i in range(0, len(goals) - 1):
    accumulated += dist(goals[i], goals[i + 1])
    if accumulated > range_m:
      break
    out_idx = i + 1
  return out_idx


def find_first_corner_from_robot(
  goals: Sequence[Point], robot: Point, range_m: float, corner_angle_deg: float,
) -> Optional[int]:
  """对齐 C++ findFirstCornerFromRobot。"""
  if len(goals) < 2:
    return None
  range_end = find_last_goal_within_path_range(goals, range_m)
  if range_end is None:
    return None
  # C++: for (i = start_idx; i <= range_end && i < goals.size(); ++i)
  for i in range(0, len(goals)):
    if i > range_end:
      break
    if not is_goal_not_corner(goals, i, robot, corner_angle_deg):
      return i
  return None


def find_next_corner_after(
  goals: Sequence[Point], after_idx: int, robot: Point,
  range_m: float, corner_angle_deg: float,
) -> Optional[int]:
  """对齐 C++ findNextCornerAfter。"""
  if len(goals) < 2 or after_idx + 1 >= len(goals):
    return None
  range_end, accumulated = after_idx, 0.0
  for i in range(after_idx, len(goals) - 1):
    accumulated += dist(goals[i], goals[i + 1])
    if accumulated > range_m:
      break
    range_end = i + 1
  if range_end <= after_idx:
    return None
  for i in range(after_idx + 1, range_end + 1):
    if i >= len(goals):
      break
    if not is_goal_not_corner(goals, i, robot, corner_angle_deg):
      return i
  return None


def find_all_corners(
  goals: Sequence[Point], robot: Point, corner_angle_deg: float,
) -> List[Point]:
  """整条路径上算法判为角点的全部坐标（供可视化，不参与删点）。"""
  out: List[Point] = []
  for i in range(len(goals)):
    if not is_goal_not_corner(goals, i, robot, corner_angle_deg):
      out.append(goals[i])
  return out


# 参与排序点数 <= 该值时用暴力枚举求"总转角最小"的全局最优；更大则退回贪心
SWEEP_BRUTE_MAX_N = 7


def compute_sweep_order(
  robot: Point,
  robot_yaw: float,
  points: Sequence[Point],
) -> List[int]:
  """多堆垃圾清扫顺序：总转角最小的全局最优（点数 <= SWEEP_BRUTE_MAX_N 时）。

  角度模型：
  - 起点朝向 = 机器人当前朝向 robot_yaw（首段 R->第一堆 计入总角）；
  - 每段转向 = 当前朝向转到指向下一堆方向的"最小旋转角"
    （wrap 到 [-pi,pi] 取绝对值）；
  - 遍历所有排列，取"各段转向角总和"最小的那条作为清扫顺序。
  点数超过上限时退回贪心（首堆最近 + 每步最小转角）。
  返回按清扫先后排列的下标列表（与 points 一一对应）。
  """
  n = len(points)
  if n <= 1:
    return list(range(n))

  def route_total(perm: Sequence[int]) -> float:
    heading = robot_yaw
    pos = robot
    total = 0.0
    for idx in perm:
      q = points[idx]
      d = math.atan2(q[1] - pos[1], q[0] - pos[0]) - heading
      d = math.atan2(math.sin(d), math.cos(d))  # wrap to [-pi, pi]
      total += abs(d)
      heading = math.atan2(q[1] - pos[1], q[0] - pos[0])
      pos = q
    return total

  if n <= SWEEP_BRUTE_MAX_N:
    best_perm: Optional[Tuple[int, ...]] = None
    best_total = float('inf')
    for perm in itertools.permutations(range(n)):
      t = route_total(perm)
      if t < best_total - 1e-9:
        best_total = t
        best_perm = perm
    return list(best_perm or tuple(range(n)))

  # 退回贪心：n 大时 n! 算不动（首堆最近 + 每步最小转角）
  remaining = set(range(n))
  order: List[int] = []
  first = min(remaining, key=lambda i: dist2(robot, points[i]))
  order.append(first)
  remaining.discard(first)
  cur_pos = points[first]
  cur_yaw = math.atan2(cur_pos[1] - robot[1], cur_pos[0] - robot[0])
  while remaining:
    best_i = -1
    best_angle = float('inf')
    best_d2 = float('inf')
    for i in remaining:
      q = points[i]
      target_yaw = math.atan2(q[1] - cur_pos[1], q[0] - cur_pos[0])
      d = target_yaw - cur_yaw
      d = math.atan2(math.sin(d), math.cos(d))
      angle = abs(d)
      d2 = dist2(cur_pos, q)
      if (angle < best_angle - 1e-6) or \
         (abs(angle - best_angle) < 1e-6 and d2 < best_d2):
        best_angle = angle
        best_d2 = d2
        best_i = i
    order.append(best_i)
    remaining.discard(best_i)
    cur_yaw = math.atan2(
      points[best_i][1] - cur_pos[1], points[best_i][0] - cur_pos[0])
    cur_pos = points[best_i]
  return order


def plan_order_with_new_garbage(
  robot: Point,
  robot_yaw: float,
  cands: Sequence[int],
  new_idx: int,
  pts: Sequence[Point],
  prev_order: Optional[Sequence[int]] = None,
) -> Tuple[List[int], float]:
  """运行中新增垃圾时的重排（设计文档 4-1/4-2）。

  cands 为实际垃圾下标列表，pts 与 cands 一一对齐（均含新垃圾）。
  以 机器人位姿 -> 最近垃圾 为直线，算新垃圾投影参数 t：
  4-1  0<=t<=1（投影落在线段内）：新垃圾排第一（先生成 新垃圾->第一堆 路径），
       其余保持原计划顺序（prev_order，过滤已失效）；
  4-2  其余情况：排除当时判断的最近垃圾，其它点（含新垃圾）进入贪心最小转角
       重算角度、重算路径，最近垃圾仍留在队首。
  返回 (清扫顺序下标列表, 投影参数 t)。
  """
  n = len(cands)
  if n <= 1 or new_idx not in cands:
    sub = [pts[i] for i in range(n)]
    return [cands[k] for k in compute_sweep_order(robot, robot_yaw, sub)], 0.0

  # 在 cands 内部用位置下标运算，最后映射回实际垃圾下标
  new_pos = list(cands).index(new_idx)
  others_pos = [i for i in range(n) if i != new_pos]
  nearest_pos = min(others_pos, key=lambda i: dist2(robot, pts[i]))

  np_pt = pts[new_pos]
  nr_pt = pts[nearest_pos]
  foot = project_point_to_infinite_line(
    np_pt[0], np_pt[1], robot[0], robot[1], nr_pt[0], nr_pt[1])
  t = line_parameter_t(
    foot[0], foot[1], robot[0], robot[1], nr_pt[0], nr_pt[1])

  def greedy_pos(pos_list: Sequence[int]) -> List[int]:
    sub = [pts[i] for i in pos_list]
    return [pos_list[k] for k in compute_sweep_order(robot, robot_yaw, sub)]

  if 0.0 <= t <= 1.0:
    # 4-1: 新垃圾 -> 第一堆；其余保持原顺序（prev_order 过滤到候选）
    rest_pos: List[int] = []
    if prev_order:
      others_set = {cands[i] for i in others_pos}
      rest_pos = [
        list(cands).index(v) for v in prev_order if v in others_set]
    have = set(rest_pos)
    missing = [i for i in others_pos if i not in have]
    rest_pos += greedy_pos(missing)
    return [cands[p] for p in ([new_pos] + rest_pos)], t

  # 4-2: 排除最近垃圾，其余(含新垃圾)贪心重算，最近垃圾留队首
  rest_pos = [i for i in others_pos if i != nearest_pos] + [new_pos]
  return [cands[p] for p in ([nearest_pos] + greedy_pos(rest_pos))], t


# 调试：参与排序的点数 <=4 时打印所有排列的角度。
# 只在「新增了垃圾」或「最优顺序真的变了」时打印，避免刷屏。
_last_debug_cands_key: Optional[Tuple[int, ...]] = None  # 上次打印时的候选集合
_last_debug_order: Optional[Tuple[int, ...]] = None      # 上次打印时的清扫顺序


def debug_dump_sweep_orders(
  robot: Point,
  robot_yaw: float,
  cands: Sequence[int],
  pts: Sequence[Point],
  greedy_order: Optional[Sequence[int]] = None,
) -> None:
  """调试：把参与排序的点（<=4）所有访问顺序的转向角都打出来。

  角度模型与 compute_sweep_order 一致：
  - 起点朝向 = 机器人当前朝向 robot_yaw；首段 R->g0 计入总角；
  - 每段转向 = 当前朝向转到指向下一堆方向的"最小旋转角"
    （wrap 到 [-pi,pi] 取绝对值，单位度）。
  格式：perm g0->g1->g2 : R->g0=xx.x° g0->g1=xx.x°  total=xx.x°
  打印时按 total 从低到高排序，最上面就是全局最优；末尾附当前实际选中的路线。
  """
  n = len(cands)
  if n <= 1 or n > 4:
    return

  def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

  def turn_deg(heading: float, to_pt: Point, from_pt: Point) -> float:
    d = math.atan2(to_pt[1] - from_pt[1], to_pt[0] - from_pt[0])
    return math.degrees(abs(wrap_angle(d - heading)))

  print(
    f'[sweep-debug] {n} piles cands={list(cands)} '
    f'robot=({robot[0]:.2f},{robot[1]:.2f}) '
    f'yaw={math.degrees(robot_yaw):.0f}deg')
  entries: List[Tuple[float, str]] = []
  for perm in itertools.permutations(range(n)):
    idx = [cands[i] for i in perm]
    ord_pts = [pts[i] for i in perm]
    heading = robot_yaw
    pos = robot
    steps: List[str] = []
    total = 0.0
    for k in range(n):
      a = turn_deg(heading, ord_pts[k], pos)
      label = f'R->g{idx[0]}={a:.1f}' if k == 0 else f'g{idx[k - 1]}->g{idx[k]}={a:.1f}'
      steps.append(label)
      total += a
      heading = math.atan2(
        ord_pts[k][1] - pos[1], ord_pts[k][0] - pos[0])
      pos = ord_pts[k]
    line = (
      'perm ' + '->'.join(f'g{i}' for i in idx) + ' : ' +
      '  '.join(steps) + f'  total={total:.1f}')
    entries.append((total, line))
  # 按总角从低到高排序打印：最小排最前，对比最直观
  entries.sort(key=lambda e: e[0])
  for _, line in entries:
    print(line)
  best = entries[0][0]
  best_lines = [line for t, line in entries if abs(t - best) < 1e-9]
  print(
    f'[sweep-debug] min total={best:.1f} deg -> '
    + ' | '.join(best_lines))

  if greedy_order:
    gi = [cands.index(v) for v in greedy_order if v in cands]
    if len(gi) >= 1:
      ord_pts = [pts[i] for i in gi]
      heading = robot_yaw
      pos = robot
      gt = 0.0
      for k in range(len(gi)):
        gt += turn_deg(heading, ord_pts[k], pos)
        heading = math.atan2(
          ord_pts[k][1] - pos[1], ord_pts[k][0] - pos[0])
        pos = ord_pts[k]
      print(
        f'[sweep-debug] selected total={gt:.1f} deg '
        f'(order={list(greedy_order)})')


@dataclass
class RoundEvidence:
  """单轮 A–C 判定凭证（供 GUI 画线/标点）。"""
  round_i: int
  tag: CaseTag  # '1'|'2'|'3'|'4'
  a: Point      # 本轮线段起点（C++：goala=goals[0] 或上一段角点）
  c: Point      # 本轮角点 / goalc
  foot: Point   # 垃圾投到 A–C 无限直线上的垂足 goald
  garbage: Point
  t_d: float
  deleted: List[Point] = field(default_factory=list)
  note: str = ''


@dataclass
class ClipEvidence:
  """一次插入的完整凭证：删了什么、依据哪条 A–C、插到哪里。"""
  robot: Point
  garbage: Point
  goala: Point
  goald0: Point
  cases: List[CaseTag]
  rounds: List[RoundEvidence]
  deleted: List[Point]
  pre_deleted: List[Point]
  corners_kept: List[Point]
  path_corners: List[Point] = field(default_factory=list)  # 插点前整路角点（可视化）
  insert_idx: int = 0
  insert_after: Optional[Point] = None  # None=队首；否则插在该点后面
  insert_reason: str = ''
  far_from_head: bool = False
  queue_before: List[Point] = field(default_factory=list)
  queue_after_head: List[Point] = field(default_factory=list)


@dataclass
class InsertResult:
  goals: List[Point]
  deleted: List[Point]
  goald: Point
  garbage: Point
  insert_idx: int
  cases: List[CaseTag] = field(default_factory=list)
  case_detail: List[str] = field(default_factory=list)
  evidence: Optional[ClipEvidence] = None


def _match_point_index(goals: Sequence[Point], p: Point, tol: float = 0.08) -> Optional[int]:
  for i, g in enumerate(goals):
    if dist(g, p) < tol:
      return i
  return None


def refresh_corners_on_remaining(
  goals: Sequence[Point],
  robot: Point,
  delete_idx: set,
  protected: set,
  corners_kept: List[Point],
  corner_angle_deg: float,
  keep_idx: Optional[int] = None,
  also_keep_idx: Optional[int] = None,
) -> List[str]:
  """每轮前方延长线删点后：在剩余路径上重判角点。

  不再是角点的保护点：取消保护，并加入 delete_idx。
  keep_idx：本轮仍需保留的 goalc（当前 C）；不删它。
  also_keep_idx：下一轮投影起点 A（上一角点）；进入延长线投影前必须保住，
  否则 A 被当成“队首过期肘点”删掉后会缩成短边，误判 reverse 而停删。

  额外规则：若某保护点已成为剩余路径队首，且 keep_idx 仍指向更前方的角点，
  说明它的原前驱已被删、不再是“中间拐弯肘点”→ 视为过期（典型：L 形第一肘点 g2）。
  但若该点仍是当前 A（also_keep_idx），本轮先不删。
  """
  notes: List[str] = []
  remaining_idx = [i for i in range(len(goals)) if i not in delete_idx]
  if len(remaining_idx) < 2:
    return notes

  remaining_pts = [goals[i] for i in remaining_idx]
  old_to_new = {old: new for new, old in enumerate(remaining_idx)}
  keep_set = {i for i in (keep_idx, also_keep_idx) if i is not None}

  stale: List[int] = []
  for old_i in list(protected):
    if old_i in delete_idx:
      protected.discard(old_i)
      continue
    if old_i in keep_set:
      continue
    new_i = old_to_new.get(old_i)
    if new_i is None:
      protected.discard(old_i)
      continue

    lost_inbound = (old_i == 0) or ((old_i - 1) in delete_idx)
    became_head = (new_i == 0)
    # 队首 + 原前驱已断 + 后方还有要保留的角点 → 过期肘点
    # （当前 A/C 已在 keep_set，不会误伤下一轮投影边）
    if became_head and lost_inbound and keep_idx is not None and keep_idx not in delete_idx:
      stale.append(old_i)
      continue

    if is_goal_not_corner(remaining_pts, new_i, robot, corner_angle_deg):
      stale.append(old_i)

  for old_i in stale:
    protected.discard(old_i)
    delete_idx.add(old_i)
    pt = goals[old_i]
    corners_kept[:] = [c for c in corners_kept if dist(c, pt) >= 0.08]
    notes.append(
      f'stale corner idx={old_i} ({pt[0]:.1f},{pt[1]:.1f}) '
      f'no longer corner on remaining -> delete')

  return notes


def clip_and_insert_garbage(
  goals: Sequence[Point],
  robot: Point,
  garbage: Point,
  clip_extend_m: float = CLIP_EXTEND_M,
  corner_angle_deg: float = CORNER_ANGLE_DEG,
  goaltotal_range_m: float = GOALTOTAL_RANGE_M,
  head_delete_robot_dist_m: float = HEAD_DELETE_ROBOT_DIST_M,
) -> Optional[InsertResult]:
  """Demo 插删逻辑（相对 C++ 有两处刻意差异）:
  1) 垃圾始终在队列头（立刻去扫）；锚点仍按 C++ 算，锚点前的残留点丢掉，避免扫完折返；
  2) 每轮前方延长线删点后，在剩余路径上刷新角点；过期角点取消保护并删除。
  """
  goals = list(goals)
  if len(goals) < 2:
    return None

  gx, gy = garbage
  rx, ry = robot
  cases: List[CaseTag] = []
  case_detail: List[str] = []
  rounds: List[RoundEvidence] = []
  corners_kept: List[Point] = []
  path_corners = find_all_corners(goals, robot, corner_angle_deg)

  # --- gatherInsertInfo ---
  # goala = goals[0]（不用机器人）
  goala = goals[0]
  radius_m = dist(robot, garbage)
  if radius_m < 1e-6:
    return None

  c_idx = find_first_corner_from_robot(goals, robot, goaltotal_range_m, corner_angle_deg)
  if c_idx is None:
    c_idx = find_last_goal_within_path_range(goals, goaltotal_range_m)
    if c_idx is None:
      return None
    case_detail.append(f'no corner, use last within {goaltotal_range_m:.1f}m as goalc idx={c_idx}')

  goalc = goals[c_idx]
  if dist2(goala, goalc) < 1e-12:
    # robot off-path often marks goals[0] as corner; pick next usable goalc
    next_c = find_next_corner_after(goals, 0, robot, goaltotal_range_m, corner_angle_deg)
    if next_c is None:
      last = find_last_goal_within_path_range(goals, goaltotal_range_m)
      if last is not None and last > 0:
        next_c = last
      elif len(goals) >= 2:
        next_c = 1
    if next_c is None:
      return None
    c_idx = next_c
    goalc = goals[c_idx]
    case_detail.append(f'goala==goalc fallback, new goalc idx={c_idx}')
    if dist2(goala, goalc) < 1e-12:
      return None

  goald0 = project_point_to_infinite_line(gx, gy, goala[0], goala[1], goalc[0], goalc[1])
  case_detail.append(
    f'goala=goals[0]=({goala[0]:.2f},{goala[1]:.2f}) '
    f'goalc=({goalc[0]:.2f},{goalc[1]:.2f}) idx={c_idx}')

  # --- clipGoalsNearGarbage ---
  dist_to_head = dist(robot, goals[0])
  far_from_head = dist_to_head > head_delete_robot_dist_m
  hit_mid_case = False
  hit_forward_case = False
  delete_idx: set[int] = set()

  if far_from_head:
    case_detail.append(
      f'robot {dist_to_head:.2f}m from goals[0] > {head_delete_robot_dist_m:.1f}m '
      f'-> skip all deletes')
  else:
    arc_s = [0.0] * len(goals)
    for i in range(len(goals) - 1):
      arc_s[i + 1] = arc_s[i] + dist(goals[i], goals[i + 1])

    protected = {c_idx}

    def mark_between(left_idx: int, right_c_idx: int, left_is_head: bool) -> List[Point]:
      begin = left_idx if left_is_head else left_idx + 1
      newly: List[Point] = []
      for i in range(begin, right_c_idx):
        if i in protected:
          continue
        if i not in delete_idx:
          newly.append(goals[i])
        delete_idx.add(i)
      return newly

    def foot_arc_on_path_prefix(px: float, py: float, path_end_idx: int) -> float:
      best, seg, t = float('inf'), 0, 0.0
      last_seg = min(path_end_idx, len(goals) - 1)
      for i in range(0, last_seg):
        d2, ti = squared_distance_point_to_segment(
          px, py, goals[i][0], goals[i][1], goals[i + 1][0], goals[i + 1][1])
        if d2 < best:
          best, seg, t = d2, i, ti
      return arc_s[seg] + t * (arc_s[seg + 1] - arc_s[seg])

    first_round = True
    a_idx = 0
    ax, ay = goala
    dx, dy = goald0
    eps = 1e-6
    round_i = 0

    while True:
      round_i += 1
      cx, cy = goals[c_idx]
      a_pt = (ax, ay)
      c_pt = (cx, cy)
      foot = (dx, dy)
      t_d = line_parameter_t(dx, dy, ax, ay, cx, cy)

      # 非首轮进入本轮前：先刷新角点，删掉已过期的上一肘点（如 g2）
      # 但必须保住当前 A（上一角点）与 C，否则投影边被掐短 → 假 reverse 停删
      if not first_round and t_d > 1.0 + eps:
        for n in refresh_corners_on_remaining(
          goals, robot, delete_idx, protected, corners_kept, corner_angle_deg,
          keep_idx=c_idx, also_keep_idx=a_idx):
          case_detail.append(f'R{round_i}:pre-refresh {n}')
        if a_idx in delete_idx:
          new_a = 0
          for i in range(c_idx - 1, -1, -1):
            if i not in delete_idx:
              new_a = i
              break
          a_idx = new_a
          ax, ay = goals[a_idx]
          a_pt = (ax, ay)
          dx, dy = project_point_to_infinite_line(
            gx, gy, ax, ay, cx, cy)
          foot = (dx, dy)
          t_d = line_parameter_t(dx, dy, ax, ay, cx, cy)
          case_detail.append(
            f'R{round_i}: A was stale, rebase A idx={a_idx} t={t_d:.2f}')

      if t_d < -eps:
        cases.append('4')
        note = f'4 reverse t={t_d:.2f} (no delete, stop)'
        case_detail.append(f'R{round_i}:{note}')
        rounds.append(RoundEvidence(
          round_i=round_i, tag='4', a=a_pt, c=c_pt, foot=foot,
          garbage=garbage, t_d=t_d, deleted=[], note=note))
        break

      if t_d > 1.0 + eps:
        hit_forward_case = True
        cases.append('3')
        before = set(delete_idx)
        if first_round:
          mark_between(0, c_idx, True)
        else:
          if a_idx not in protected and a_idx not in delete_idx:
            delete_idx.add(a_idx)
          mark_between(a_idx, c_idx, False)
        newly = [goals[i] for i in sorted(delete_idx - before)]
        corners_kept.append(c_pt)
        protected.add(c_idx)
        note = (
          f'3 forward t={t_d:.2f} delete mid A–C '
          f'A=({a_pt[0]:.1f},{a_pt[1]:.1f}) C=({c_pt[0]:.1f},{c_pt[1]:.1f}) '
          f'del={len(newly)}')
        case_detail.append(f'R{round_i}:{note}')
        rounds.append(RoundEvidence(
          round_i=round_i, tag='3', a=a_pt, c=c_pt, foot=foot,
          garbage=garbage, t_d=t_d, deleted=newly, note=note))

        for n in refresh_corners_on_remaining(
          goals, robot, delete_idx, protected, corners_kept, corner_angle_deg,
          keep_idx=c_idx):
          case_detail.append(f'R{round_i}:refresh {n}')

        next_c = find_next_corner_after(
          goals, c_idx, robot, goaltotal_range_m, corner_angle_deg)
        if next_c is None:
          # 无下一角点：从当前 C 起沿路 goaltotal_range 内末点兜底再判一轮
          # （否则会漏掉「延长线后再落在下一段中部」的删除，如图二竖列）
          range_end, accumulated = c_idx, 0.0
          for i in range(c_idx, len(goals) - 1):
            accumulated += dist(goals[i], goals[i + 1])
            if accumulated > goaltotal_range_m:
              break
            range_end = i + 1
          if range_end <= c_idx:
            for n in refresh_corners_on_remaining(
              goals, robot, delete_idx, protected, corners_kept, corner_angle_deg,
              keep_idx=c_idx):
              case_detail.append(f'R{round_i}:final-refresh {n}')
            case_detail.append(f'R{round_i}:3 no next corner/range end -> stop')
            break
          next_c = range_end
          case_detail.append(
            f'R{round_i}:3 no next corner, fallback last-in-range idx={next_c} '
            f'({goals[next_c][0]:.2f},{goals[next_c][1]:.2f})')

        if c_idx not in delete_idx:
          a_idx = c_idx
        else:
          a_idx = 0
          for i in range(next_c - 1, -1, -1):
            if i not in delete_idx:
              a_idx = i
              break
          case_detail.append(
            f'R{round_i}: prev C deleted as stale, new A idx={a_idx}')

        c_idx = next_c
        protected.add(c_idx)
        ax, ay = goals[a_idx]
        dx, dy = project_point_to_infinite_line(
          gx, gy, ax, ay, goals[c_idx][0], goals[c_idx][1])
        first_round = False
        continue

      # 垂足在段中
      hit_mid_case = True
      s_d = foot_arc_on_path_prefix(dx, dy, c_idx)
      s_c = arc_s[c_idx]
      path_to_corner = s_c - s_d

      if clip_extend_m > 0.0 and path_to_corner > clip_extend_m:
        tag: CaseTag = '1'
        s_hi = s_d + clip_extend_m
        include_hi = True
        span = f'foot+{clip_extend_m:.1f}m'
      else:
        tag = '2'
        s_hi = s_c
        include_hi = False
        span = 'foot->corner'

      begin = 0 if first_round else a_idx
      before = set(delete_idx)
      for j in range(begin, len(goals)):
        if j == begin and not first_round:
          if j in protected:
            continue
          delete_idx.add(j)
          continue
        if include_hi:
          if arc_s[j] > s_hi + 1e-9:
            break
        else:
          if arc_s[j] >= s_hi - 1e-9:
            break
        if j in protected:
          continue
        if not is_goal_not_corner(goals, j, robot, corner_angle_deg):
          protected.add(j)
          continue
        delete_idx.add(j)
      newly = [goals[i] for i in sorted(delete_idx - before)]
      cases.append(tag)
      note = (
        f'{"1" if tag == "1" else "2"} mid t={t_d:.2f} {span} '
        f'(d2c={path_to_corner:.2f}m) A=({a_pt[0]:.1f},{a_pt[1]:.1f}) '
        f'C=({c_pt[0]:.1f},{c_pt[1]:.1f}) foot=({foot[0]:.1f},{foot[1]:.1f}) '
        f'del={len(newly)}')
      case_detail.append(f'R{round_i}:{note}')
      rounds.append(RoundEvidence(
        round_i=round_i, tag=tag, a=a_pt, c=c_pt, foot=foot,
        garbage=garbage, t_d=t_d, deleted=newly, note=note))
      # 段中删完也要刷新过期角点：上一轮 C（现为 A）在剩余路径上
      # 往往已不是角点，但仍在 protected，不刷就会漏删
      for n in refresh_corners_on_remaining(
        goals, robot, delete_idx, protected, corners_kept, corner_angle_deg,
        keep_idx=c_idx):
        case_detail.append(f'R{round_i}:mid-refresh {n}')
      break

  deleted = [goals[i] for i in sorted(delete_idx)]
  out = [g for i, g in enumerate(goals) if i not in delete_idx]

  # --- insert（对齐 C++ 锚点，再改成队首，避免扫完折返）---
  # C++：远队头→队首；段中→goala 后；前方延长线→最后保留角点后。
  # 若只把垃圾 insert(0) 而留下 out[:anchor]，扫完会先回到这些点=折返。
  # 正确队首插入：out = [garbage] + out[anchor:]，锚点前的点并入 deleted。
  insert_anchor = 0
  insert_after: Optional[Point] = None
  match_tol = 0.08
  if far_from_head:
    insert_reason = 'far from head -> head insert, no clip delete'
  elif hit_mid_case:
    for j, p in enumerate(out):
      if dist(p, goala) < match_tol:
        insert_anchor = j + 1
        insert_after = p
        break
    insert_reason = (
      f'mid: C++ would insert after goala (anchor={insert_anchor}); '
      f'head-insert keeps tail from anchor')
  elif hit_forward_case and corners_kept:
    for corner in reversed(corners_kept):
      for j, p in enumerate(out):
        if dist(p, corner) < match_tol:
          insert_anchor = j + 1
          insert_after = p
          break
      if insert_anchor != 0:
        break
    insert_reason = (
      f'forward: C++ would insert after last kept corner '
      f'(anchor={insert_anchor}); head-insert keeps tail from anchor')
  else:
    insert_reason = 'fallback head insert'

  skipped = out[:insert_anchor]
  resume_from = insert_anchor
  # 前方延长线：保留最后角点作接回，只丢掉角点之前的残留
  if hit_forward_case and insert_after is not None and insert_anchor > 0:
    resume_from = insert_anchor - 1
    skipped = out[:resume_from]
  if skipped:
    deleted.extend(skipped)
    case_detail.append(
      f'head-insert drop before resume x{len(skipped)} '
      f'{[(round(p[0], 1), round(p[1], 1)) for p in skipped[:8]]}')
  out = [garbage] + out[resume_from:]
  insert_idx = 0
  case_detail.append(
    f'insert idx=0 ({insert_reason}), remain={len(out)}')

  # 远队头跳过删点时仍保留一轮 A–C 几何，方便 GUI 画延长线
  if not rounds:
    t0 = line_parameter_t(goald0[0], goald0[1], goala[0], goala[1], goalc[0], goalc[1])
    tag0: CaseTag = '4' if t0 < -1e-6 else ('3' if t0 > 1.0 + 1e-6 else '1')
    rounds.append(RoundEvidence(
      round_i=1, tag=tag0, a=goala, c=goalc, foot=goald0,
      garbage=garbage, t_d=t0, deleted=[],
      note=f'geom-only t={t0:.2f} (far_from_head={far_from_head})'))

  evidence = ClipEvidence(
    robot=robot,
    garbage=garbage,
    goala=goala,
    goald0=goald0,
    cases=list(cases),
    rounds=rounds,
    deleted=deleted,
    pre_deleted=[],
    corners_kept=list(corners_kept),
    path_corners=list(path_corners),
    insert_idx=insert_idx,
    insert_after=insert_after,
    insert_reason=insert_reason,
    far_from_head=far_from_head,
    queue_before=list(goals),
    queue_after_head=out[:8],
  )

  return InsertResult(
    goals=out, deleted=deleted, goald=goald0, garbage=garbage,
    insert_idx=insert_idx, cases=cases, case_detail=case_detail,
    evidence=evidence)




# ---------- sim ----------
@dataclass
class SimState:
  original_goals: List[Point]
  goals: List[Point]
  deleted: List[Point] = field(default_factory=list)
  passed: List[Point] = field(default_factory=list)
  garbage_items: List[Tuple[Point, str]] = field(default_factory=list)
  published: set = field(default_factory=set)
  finished: set = field(default_factory=set)
  inserted_xy: List[Point] = field(default_factory=list)
  cases_hit: set = field(default_factory=set)
  last_evidence: Optional[ClipEvidence] = None
  last_sweep_order: List[int] = field(default_factory=list)
  # 最近一次运行中新增的垃圾下标：走 4-1/4-2 投影重排，发布/完成后清除
  priority_new_idx: Optional[int] = None
  robot: Point = (0.0, 0.0)
  yaw: float = 0.0
  done: bool = False
  status: str = 'running'
  log_lines: List[str] = field(default_factory=list)

  @property
  def garbage_xy(self) -> List[Point]:
    return [g for g, _ in self.garbage_items]

  def log(self, msg: str) -> None:
    print(msg, flush=True)
    self.log_lines.append(msg)
    if len(self.log_lines) > 12:
      self.log_lines = self.log_lines[-12:]


def pending_inserted_garbage(state: SimState) -> bool:
  for i in state.published:
    if i in state.finished:
      continue
    gxy = state.garbage_xy[i]
    for g in state.goals:
      if dist(g, gxy) < DEDUP_DISTANCE_M:
        return True
  return False


def maybe_publish_and_insert(state: SimState) -> None:
  global _last_debug_cands_key, _last_debug_order
  # 候选：未发布、未完成、且在距离范围内的垃圾（每帧动态过滤）。
  # 不再用 path-ahead 路径超前门槛：多堆排序应纳入 9m 内所有堆，
  # 否则几何上近、但对应路径靠后的垃圾会被漏扫（与 C++ 语义不一致）。
  cands: List[int] = []
  for i, (gxy, hint) in enumerate(state.garbage_items):
    if i in state.published or i in state.finished:
      continue
    gx, gy = gxy
    # 太远则本帧忽略，不标记 finished：机器人靠近后继续判断
    if MAX_GARBAGE_ROBOT_DIST_M > 0.0 and \
       dist(state.robot, (gx, gy)) > MAX_GARBAGE_ROBOT_DIST_M:
      continue
    cands.append(i)

  if not cands:
    return

  # 优先级：最近一次运行中新增的垃圾，用于 4-1/4-2 投影重排；已发布/完成则清除
  priority = state.priority_new_idx
  if priority is not None and priority not in cands:
    state.priority_new_idx = None
    priority = None

  cand_pts = [state.garbage_xy[i] for i in cands]

  # 4-1 时允许越过"待扫的第一堆"直接插队首（生成 新垃圾->第一堆 的路径）
  bypass_pending = False
  if priority is not None and len(cands) > 1:
    # 新垃圾：先投影判断 4-1/4-2，再决定清扫顺序
    sweep, t_proj = plan_order_with_new_garbage(
      state.robot, state.yaw, cands, priority, cand_pts,
      prev_order=state.last_sweep_order)
    state.last_sweep_order = list(sweep)
    if sweep[0] == priority:
      # 4-1：新垃圾排第一 → 插到队首，第一堆仍留在 goals 排在新垃圾之后
      bypass_pending = True
      state.log(
        f'  new-garbage[{priority}] re-plan 4-1 t={t_proj:.2f} -> '
        f'{state.last_sweep_order} (head insert, before first pile)')
    else:
      # 4-2：最近垃圾仍排第一 → 等它扫完再按重排结果插入
      state.log(
        f'  new-garbage[{priority}] re-plan 4-2 t={t_proj:.2f} -> '
        f'{state.last_sweep_order}')
    i = sweep[0]
  elif len(cands) > 1:
    # 多堆清扫顺序：从当前机器人位姿动态重算（总转角最小，全局最优）
    sweep = compute_sweep_order(state.robot, state.yaw, cand_pts)
    state.last_sweep_order = [cands[k] for k in sweep]
    i = cands[sweep[0]]
  else:
    i = cands[0]
    state.last_sweep_order = [i]

  # 调试：参与排序点数<=4。只在「新增了垃圾」或「最优顺序真的变了」时打印，避免刷屏
  if 1 < len(cands) <= 4:
    cur_order = tuple(state.last_sweep_order)
    # 候选里出现了上次打印时没有的下标 → 新增了垃圾
    added_new = (_last_debug_cands_key is None) or \
      bool(set(cands) - set(_last_debug_cands_key))
    changed = False
    if not added_new and _last_debug_order is not None:
      # 与上次打印时的顺序比较（都过滤到当前候选集合）
      prev_rel = tuple(v for v in _last_debug_order if v in cands)
      cur_rel = tuple(v for v in cur_order if v in cands)
      changed = (prev_rel != cur_rel)
    if added_new or changed:
      _last_debug_cands_key = tuple(sorted(cands))
      _last_debug_order = cur_order
      debug_dump_sweep_orders(
        state.robot, state.yaw, cands, cand_pts,
        greedy_order=state.last_sweep_order)

  # 已有垃圾在待扫队列里：普通插入等它扫完；4-1 新垃圾插队首除外
  if not bypass_pending and pending_inserted_garbage(state):
    return

  gx, gy = state.garbage_xy[i]
  hint = state.garbage_items[i][1]
  dist_robot = dist(state.robot, (gx, gy))

  for ix, iy in state.inserted_xy:
    if dist((gx, gy), (ix, iy)) < DEDUP_DISTANCE_M:
      state.published.add(i)
      return

  state.log(
    f'Publish garbage[{i}] ({gx:.2f},{gy:.2f}) '
    f'dist_robot={dist_robot:.2f}m | {hint}')
  state.log(f'  sweep_order={state.last_sweep_order}')

  result = clip_and_insert_garbage(state.goals, state.robot, (gx, gy))
  if result is None:
    # do not mark published: allow retry; usually goala==goalc before fallback fix
    state.log(f'Insert FAILED garbage[{i}] (clip returned None)')
    return

  state.published.add(i)

  state.goals = result.goals
  state.deleted.extend(result.deleted)
  state.inserted_xy.append((gx, gy))
  state.last_evidence = result.evidence
  for c in result.cases:
    state.cases_hit.add(c)
  state.log(
    f'Insert @({gx:.2f},{gy:.2f}) goald=({result.goald[0]:.2f},{result.goald[1]:.2f}) '
    f'del={len(result.deleted)} now={len(state.goals)} cases={"".join(result.cases)}')
  for d in result.case_detail:
    # keep ASCII in on-screen log: strip some CJK case markers if present
    state.log('  ' + d)
  state.log(f'  deleted_xy={[(round(p[0],2), round(p[1],2)) for p in result.deleted]}')
  state.log(
    f'  queue_head={[(round(p[0],2), round(p[1],2)) for p in state.goals[:6]]}')


def step_robot(state: SimState) -> bool:
  if state.done:
    return False
  if not state.goals:
    state.done = True
    state.status = 'done'
    state.log(
      f'Done. cases_hit={sorted(state.cases_hit)} '
      f'finished={sorted(state.finished)}/{len(state.garbage_items)}')
    return False

  target = state.goals[0]
  dx, dy = target[0] - state.robot[0], target[1] - state.robot[1]
  d = math.hypot(dx, dy)
  if d < 1e-9:
    reached = state.goals.pop(0)
    state.passed.append(reached)
    state.log(f'Passed goal ({reached[0]:.2f},{reached[1]:.2f})')
    _maybe_finish_garbage_at(state, reached)
    return True
  state.yaw = math.atan2(dy, dx)
  step = min(ROBOT_SPEED * DT, d)
  state.robot = (
    state.robot[0] + math.cos(state.yaw) * step,
    state.robot[1] + math.sin(state.yaw) * step,
  )
  if dist(state.robot, state.goals[0]) <= REACH_GOAL_RADIUS:
    reached = state.goals.pop(0)
    state.passed.append(reached)
    state.log(f'Passed goal ({reached[0]:.2f},{reached[1]:.2f})')
    _maybe_finish_garbage_at(state, reached)
    return True
  return False


def _maybe_finish_garbage_at(state: SimState, reached: Point) -> None:
  for i, gxy in enumerate(state.garbage_xy):
    if i in state.published and i not in state.finished:
      if dist(reached, gxy) <= GARBAGE_FINISH_RADIUS_M or dist(state.robot, gxy) <= GARBAGE_FINISH_RADIUS_M:
        state.finished.add(i)
        state.log(f'Finished garbage[{i}]')


def new_sim_state(
  goals: Sequence[Point],
  robot: Point,
  yaw: float,
  garbage_items: List[Tuple[Point, str]],
) -> SimState:
  goals_list = list(goals)
  st = SimState(
    original_goals=list(goals_list),
    goals=list(goals_list),
    garbage_items=list(garbage_items),
    robot=robot,
    yaw=yaw,
  )
  st.log(f'Start run: {len(st.goals)} goals, {len(garbage_items)} garbage')
  return st


def yaw_from_drag(origin: Point, tip: Point) -> float:
  dx, dy = tip[0] - origin[0], tip[1] - origin[1]
  if math.hypot(dx, dy) < MIN_DRAG_M:
    return 0.0
  return math.atan2(dy, dx)




def load_scenario(path) -> dict:
  with open(path, 'r', encoding='utf-8') as f:
    return json.load(f)


def scenario_to_edit(data: dict):
  goals = [(float(g['x']), float(g['y'])) for g in data['goals']]
  yaws = [math.radians(float(g.get('yaw_deg', 0.0))) for g in data['goals']]
  garbage = [(float(g['x']), float(g['y'])) for g in data.get('garbage', [])]
  # 运行中 RMB 加的垃圾：用 RT* 提示（对齐日志复刻）
  runtime_garbage = [
    (float(g['x']), float(g['y'])) for g in data.get('runtime_garbage', [])]
  robot = data['robot']
  robot_xy = (float(robot['x']), float(robot['y']))
  robot_yaw = math.radians(float(robot.get('yaw_deg', 0.0)))
  return goals, yaws, garbage, runtime_garbage, robot_xy, robot_yaw


def run_gui(replay: bool = False, scenario_path: Optional[str] = None) -> None:
  # edit buffers
  edit_goals: List[Point] = []
  edit_goal_yaws: List[float] = []
  edit_garbage: List[Point] = []
  edit_runtime_garbage: List[Point] = []
  robot_xy: Optional[Point] = None
  robot_yaw: float = 0.0
  undo_stack: List[str] = []  # 'goal' | 'robot' | 'garbage'
  replay_mode = bool(replay)  # no mouse edit in edit stages; run still allows pan/add
  runtime_garbage_stack: List[int] = []  # indices of garbage added during run

  mode = 'edit_goals'  # edit_goals | edit_garbage | running
  # pan: middle mouse (any mode); running mode LMB/RMB add temporary garbage
  pan_btn = False
  pan_xy0 = (0.0, 0.0)
  pan_xlim0 = (0.0, 0.0)
  pan_ylim0 = (0.0, 0.0)
  state: Optional[SimState] = None
  checkpoints: List[SimState] = []
  checkpoint_idx = 0
  paused = False

  drag_btn: Optional[int] = None  # 1=LMB 3=RMB
  drag_origin: Optional[Point] = None
  drag_tip: Optional[Point] = None

  fig = plt.figure(figsize=(12, 7))
  fig.canvas.manager.set_window_title(
    'InsertGarbagePose Demo | REPLAY' if replay_mode else
    'InsertGarbagePose Demo | LMB goals  RMB robot  MMB pan  Enter next  Bksp undo  Q clear')
  gs = GridSpec(1, 2, width_ratios=[3.2, 1.3], wspace=0.25)
  ax = fig.add_subplot(gs[0, 0])
  ax_info = fig.add_subplot(gs[0, 1])
  ax_info.axis('off')

  ax.set_xlim(MAP_X_MIN, MAP_X_MAX)
  ax.set_ylim(MAP_Y_MIN, MAP_Y_MAX)
  ax.set_aspect('equal')
  ax.set_xlabel('x (m)')
  ax.set_ylabel('y (m)')
  ax.set_xticks(np.arange(MAP_X_MIN, MAP_X_MAX + 0.1, GRID_STEP))
  ax.set_yticks(np.arange(MAP_Y_MIN, MAP_Y_MAX + 0.1, GRID_STEP))
  ax.grid(True, which='major', color='#cfd8dc', lw=0.6)
  ax.set_axisbelow(True)
  # map frame
  ax.add_patch(Rectangle(
    (MAP_X_MIN, MAP_Y_MIN), MAP_X_MAX - MAP_X_MIN, MAP_Y_MAX - MAP_Y_MIN,
    fill=False, edgecolor='#37474f', lw=2.0, zorder=1))

  path_line, = ax.plot([], [], '-', color='#6aa84f', lw=1.5, alpha=0.5, label='active path')
  draft_path, = ax.plot([], [], '-', color='#81c784', lw=1.2, alpha=0.7, zorder=2)
  active_sc = ax.scatter([], [], c='#22aa22', s=36, zorder=3, label='active goals')
  draft_sc = ax.scatter([], [], c='#43a047', s=40, zorder=4, label='draft goals')
  passed_sc = ax.scatter([], [], c='#e6b800', s=36, zorder=3, label='passed')
  deleted_sc = ax.scatter([], [], c='#222222', s=30, zorder=3, label='deleted')
  garbage_sc = ax.scatter([], [], c='#cc2222', s=36, zorder=4, label='garbage')
  robot_pt, = ax.plot([], [], 'o', color='#2266cc', ms=6, zorder=7)
  preview_arrow_art = None
  goal_arrow_arts: List = []
  robot_body = None
  robot_arrow = None
  evidence_artists: List = []

  ax.legend(loc='lower left', fontsize=7, framealpha=0.9)
  info_txt = ax_info.text(
    0.02, 0.98, '', transform=ax_info.transAxes, va='top', ha='left',
    fontsize=8, family='DejaVu Sans', wrap=True)

  status_log: List[str] = []

  def slog(msg: str) -> None:
    print(msg, flush=True)
    status_log.append(msg)
    if len(status_log) > 10:
      del status_log[:-10]

  if replay_mode:
    sp = scenario_path
    if not sp:
      sp = str(Path(__file__).resolve().with_name('replay_c_bend.json'))
    data = load_scenario(sp)
    edit_goals, edit_goal_yaws, edit_garbage, edit_runtime_garbage, robot_xy, robot_yaw = (
      scenario_to_edit(data))
    slog(f'Replay mode: {data.get("name", sp)}')
    slog(
      f'Loaded {len(edit_goals)} goals, {len(edit_garbage)} garbage, '
      f'{len(edit_runtime_garbage)} runtime_garbage')
    slog('No mouse edit. SPACE pause | A/D point | Enter restart | Esc quit')
  else:
    slog('Mode: edit goals. LMB=goal RMB=robot MMB=pan Enter=next Bksp=undo Q=clear')

  def in_axes(event) -> bool:
    return event.inaxes is ax and event.xdata is not None and event.ydata is not None

  def clear_list_artists(arts: List) -> None:
    for a in arts:
      try:
        a.remove()
      except Exception:
        pass
    arts.clear()

  def clear_evidence() -> None:
    clear_list_artists(evidence_artists)

  def set_robot_artists(xy: Point, yaw: float) -> None:
    nonlocal robot_body, robot_arrow
    if robot_body is not None:
      try:
        robot_body.remove()
      except Exception:
        pass
      robot_body = None
    if robot_arrow is not None:
      try:
        robot_arrow.remove()
      except Exception:
        pass
      robot_arrow = None
    robot_pt.set_data([xy[0]], [xy[1]])
    robot_body = Rectangle(
      (-ROBOT_LENGTH_M * 0.5, -ROBOT_WIDTH_M * 0.5),
      ROBOT_LENGTH_M, ROBOT_WIDTH_M,
      fill=True, facecolor='#90caf9', edgecolor='#1565c0', lw=1.5,
      alpha=0.55, zorder=6)
    robot_body.set_transform(
      Affine2D().rotate(yaw).translate(xy[0], xy[1]) + ax.transData)
    ax.add_patch(robot_body)
    robot_arrow = FancyArrow(
      xy[0], xy[1],
      0.55 * math.cos(yaw), 0.55 * math.sin(yaw),
      width=0.08, head_width=0.25, head_length=0.2,
      color='#0d47a1', zorder=7)
    ax.add_patch(robot_arrow)

  def redraw_goal_arrows() -> None:
    clear_list_artists(goal_arrow_arts)
    for (x, y), yaw in zip(edit_goals, edit_goal_yaws):
      arr = FancyArrow(
        x, y, 0.45 * math.cos(yaw), 0.45 * math.sin(yaw),
        width=0.05, head_width=0.18, head_length=0.14,
        color='#2e7d32', zorder=4)
      ax.add_patch(arr)
      goal_arrow_arts.append(arr)

  def set_preview(origin: Optional[Point], tip: Optional[Point], color: str) -> None:
    nonlocal preview_arrow_art
    if preview_arrow_art is not None:
      try:
        preview_arrow_art.remove()
      except Exception:
        pass
      preview_arrow_art = None
    if origin is None or tip is None:
      return
    yaw = yaw_from_drag(origin, tip)
    preview_arrow_art = FancyArrow(
      origin[0], origin[1],
      0.7 * math.cos(yaw), 0.7 * math.sin(yaw),
      width=0.06, head_width=0.2, head_length=0.16,
      color=color, alpha=0.8, zorder=8)
    ax.add_patch(preview_arrow_art)

  def refresh_draft_artists() -> None:
    if edit_goals:
      draft_path.set_data([p[0] for p in edit_goals], [p[1] for p in edit_goals])
      draft_sc.set_offsets(edit_goals)
    else:
      draft_path.set_data([], [])
      draft_sc.set_offsets(np.empty((0, 2)))
    redraw_goal_arrows()
    if edit_garbage:
      garbage_sc.set_offsets(edit_garbage)
    else:
      garbage_sc.set_offsets(np.empty((0, 2)))
    if robot_xy is not None:
      set_robot_artists(robot_xy, robot_yaw)
    else:
      robot_pt.set_data([], [])

  def undo_last() -> None:
    nonlocal robot_xy, robot_yaw
    if not undo_stack:
      slog('Undo: empty')
      return
    kind = undo_stack.pop()
    if kind == 'goal' and edit_goals:
      edit_goals.pop()
      edit_goal_yaws.pop()
      slog(f'Undo goal, remain={len(edit_goals)}')
    elif kind == 'robot':
      robot_xy = None
      robot_yaw = 0.0
      slog('Undo robot')
    elif kind == 'garbage' and edit_garbage:
      edit_garbage.pop()
      slog(f'Undo garbage, remain={len(edit_garbage)}')
    else:
      slog(f'Undo skipped ({kind})')
    refresh_draft_artists()

  def clear_all() -> None:
    nonlocal mode, state, checkpoints, checkpoint_idx, paused
    nonlocal robot_xy, robot_yaw, drag_btn, drag_origin, drag_tip
    nonlocal robot_body, robot_arrow
    edit_goals.clear()
    edit_goal_yaws.clear()
    edit_garbage.clear()
    undo_stack.clear()
    robot_xy = None
    robot_yaw = 0.0
    drag_btn = None
    drag_origin = None
    drag_tip = None
    state = None
    checkpoints = []
    checkpoint_idx = 0
    paused = False
    mode = 'edit_goals'
    clear_evidence()
    set_preview(None, None, '#000')
    clear_list_artists(goal_arrow_arts)
    path_line.set_data([], [])
    draft_path.set_data([], [])
    active_sc.set_offsets(np.empty((0, 2)))
    draft_sc.set_offsets(np.empty((0, 2)))
    passed_sc.set_offsets(np.empty((0, 2)))
    deleted_sc.set_offsets(np.empty((0, 2)))
    garbage_sc.set_offsets(np.empty((0, 2)))
    if robot_body is not None:
      try:
        robot_body.remove()
      except Exception:
        pass
      robot_body = None
    if robot_arrow is not None:
      try:
        robot_arrow.remove()
      except Exception:
        pass
      robot_arrow = None
    robot_pt.set_data([], [])
    slog('Cleared all. Mode: edit goals. LMB=goal RMB=robot MMB=pan Enter=next Q=clear')


  def add_runtime_garbage(xy: Point) -> None:
    nonlocal state
    if state is None or mode != 'running':
      return
    idx = len(state.garbage_items)
    state.garbage_items.append((xy, f'RT{idx}'))
    runtime_garbage_stack.append(idx)
    slog(f'Add runtime garbage[{idx}] ({xy[0]:.2f},{xy[1]:.2f})')
    # 新垃圾：置优先级，走 4-1/4-2 投影重排再插入
    state.priority_new_idx = idx
    maybe_publish_and_insert(state)
    push_checkpoint()

  def undo_runtime_garbage() -> None:
    nonlocal state
    if state is None or not runtime_garbage_stack:
      slog('Undo runtime: empty')
      return
    idx = runtime_garbage_stack[-1]
    if idx < 0 or idx >= len(state.garbage_items):
      runtime_garbage_stack.pop()
      slog('Undo runtime: stale')
      return
    gxy, hint = state.garbage_items[idx]
    if idx in state.finished:
      slog(f'Undo runtime: garbage[{idx}] already finished')
      return
    runtime_garbage_stack.pop()
    # drop from active queue if inserted
    state.goals = [g for g in state.goals if dist(g, gxy) >= DEDUP_DISTANCE_M]
    state.published.discard(idx)
    state.inserted_xy = [p for p in state.inserted_xy if dist(p, gxy) >= DEDUP_DISTANCE_M]
    if state.priority_new_idx == idx:
      state.priority_new_idx = None
    if idx == len(state.garbage_items) - 1:
      state.garbage_items.pop()
    else:
      # keep slot but skip forever
      state.published.add(idx)
      state.finished.add(idx)
    slog(f'Undo runtime garbage[{idx}] ({gxy[0]:.2f},{gxy[1]:.2f})')
    truncate_future()
    push_checkpoint()

  def start_run() -> None:
    nonlocal mode, state, checkpoints, checkpoint_idx, paused
    if len(edit_goals) < 2:
      slog('Need >= 2 goals')
      return
    if robot_xy is None:
      slog('Need robot pose (RMB)')
      return
    items = [((g[0], g[1]), f'G{i}') for i, g in enumerate(edit_garbage)]
    for j, g in enumerate(edit_runtime_garbage):
      items.append(((g[0], g[1]), f'RT{len(items)}'))
    state = new_sim_state(edit_goals, robot_xy, robot_yaw, items)
    checkpoints = [copy.deepcopy(state)]
    checkpoint_idx = 0
    paused = False
    mode = 'running'
    runtime_garbage_stack.clear()
    # hide draft path overlays; sim artists take over
    draft_path.set_data([], [])
    draft_sc.set_offsets(np.empty((0, 2)))
    clear_list_artists(goal_arrow_arts)
    slog('Running. MMB pan | LMB/RMB add garbage | Bksp undo | SPACE/A/D')

  def restart_run() -> None:
    nonlocal state, checkpoints, checkpoint_idx, paused
    if robot_xy is None or len(edit_goals) < 2:
      return
    items = [((g[0], g[1]), f'G{i}') for i, g in enumerate(edit_garbage)]
    for j, g in enumerate(edit_runtime_garbage):
      items.append(((g[0], g[1]), f'RT{len(items)}'))
    state = new_sim_state(edit_goals, robot_xy, robot_yaw, items)
    checkpoints = [copy.deepcopy(state)]
    checkpoint_idx = 0
    paused = False
    runtime_garbage_stack.clear()
    clear_evidence()
    slog('Restart run')

  def begin_pan(event) -> None:
    nonlocal pan_btn, pan_xy0, pan_xlim0, pan_ylim0
    pan_btn = True
    pan_xy0 = (float(event.x), float(event.y))
    pan_xlim0 = ax.get_xlim()
    pan_ylim0 = ax.get_ylim()

  def on_press(event) -> None:
    nonlocal drag_btn, drag_origin, drag_tip
    if not in_axes(event):
      return
    # Middle mouse: pan view in any mode (handy while placing goals)
    if event.button == 2:
      begin_pan(event)
      return
    # running: MMB pan; LMB/RMB add temporary garbage (即时重排路径)
    if mode == 'running' and state is not None:
      if event.button == 1 or event.button == 3:
        add_runtime_garbage((float(event.xdata), float(event.ydata)))
        fig.canvas.draw_idle()
      return
    if replay_mode:
      return
    if event.button not in (1, 3):
      return
    if mode == 'edit_garbage' and event.button == 3:
      return
    drag_btn = int(event.button)
    drag_origin = (float(event.xdata), float(event.ydata))
    drag_tip = drag_origin
    color = '#2e7d32' if drag_btn == 1 else '#1565c0'
    if mode == 'edit_garbage' and drag_btn == 1:
      set_preview(None, None, color)
    else:
      set_preview(drag_origin, drag_tip, color)
    fig.canvas.draw_idle()

  def on_motion(event) -> None:
    nonlocal drag_tip
    if pan_btn:
      if event.x is None or event.y is None:
        return
      bbox = ax.get_window_extent()
      if bbox.width <= 1 or bbox.height <= 1:
        return
      dx_pix = float(event.x) - pan_xy0[0]
      dy_pix = float(event.y) - pan_xy0[1]
      dx = -dx_pix * (pan_xlim0[1] - pan_xlim0[0]) / bbox.width
      dy = -dy_pix * (pan_ylim0[1] - pan_ylim0[0]) / bbox.height
      ax.set_xlim(pan_xlim0[0] + dx, pan_xlim0[1] + dx)
      ax.set_ylim(pan_ylim0[0] + dy, pan_ylim0[1] + dy)
      fig.canvas.draw_idle()
      return
    if drag_btn is None or drag_origin is None or not in_axes(event):
      return
    if mode == 'edit_garbage' and drag_btn == 1:
      return
    drag_tip = (float(event.xdata), float(event.ydata))
    color = '#2e7d32' if drag_btn == 1 else '#1565c0'
    set_preview(drag_origin, drag_tip, color)
    fig.canvas.draw_idle()

  def on_release(event) -> None:
    nonlocal drag_btn, drag_origin, drag_tip, robot_xy, robot_yaw, pan_btn
    if pan_btn and event.button in (1, 2):
      pan_btn = False
      return
    if drag_btn is None or drag_origin is None:
      return
    tip = drag_tip if drag_tip is not None else drag_origin
    if in_axes(event):
      tip = (float(event.xdata), float(event.ydata))
    btn = drag_btn
    origin = drag_origin
    drag_btn = None
    drag_origin = None
    drag_tip = None
    set_preview(None, None, '#000')

    if mode == 'edit_goals':
      yaw = yaw_from_drag(origin, tip)
      if btn == 1:
        edit_goals.append(origin)
        edit_goal_yaws.append(yaw)
        undo_stack.append('goal')
        slog(f'Add goal[{len(edit_goals)-1}] ({origin[0]:.2f},{origin[1]:.2f}) yaw={math.degrees(yaw):.0f}deg')
      elif btn == 3:
        robot_xy = origin
        robot_yaw = yaw
        undo_stack.append('robot')
        slog(f'Set robot ({origin[0]:.2f},{origin[1]:.2f}) yaw={math.degrees(yaw):.0f}deg')
    elif mode == 'edit_garbage' and btn == 1:
      edit_garbage.append(origin)
      undo_stack.append('garbage')
      slog(f'Add garbage[{len(edit_garbage)-1}] ({origin[0]:.2f},{origin[1]:.2f})')
    refresh_draft_artists()
    fig.canvas.draw_idle()

  def truncate_future() -> None:
    nonlocal checkpoints
    if checkpoint_idx + 1 < len(checkpoints):
      del checkpoints[checkpoint_idx + 1:]

  def push_checkpoint() -> None:
    nonlocal checkpoint_idx
    if state is None:
      return
    truncate_future()
    checkpoints.append(copy.deepcopy(state))
    checkpoint_idx = len(checkpoints) - 1

  def restore_checkpoint(idx: int) -> None:
    nonlocal state, checkpoint_idx
    checkpoint_idx = max(0, min(idx, len(checkpoints) - 1))
    state = copy.deepcopy(checkpoints[checkpoint_idx])

  def advance_one_point() -> None:
    if state is None or state.done:
      slog('Next: done')
      return
    for _ in range(STEP_POINT_MAX_FRAMES):
      maybe_publish_and_insert(state)
      if step_robot(state) or state.done:
        push_checkpoint()
        slog(f'Next point cp={checkpoint_idx}/{len(checkpoints)-1} goals={len(state.goals)}')
        return
    slog('Next: timeout')

  def on_key(event) -> None:
    nonlocal mode, paused, state, checkpoint_idx
    key = event.key
    if key in ('escape',):
      try:
        anim.event_source.stop()
      except Exception:
        pass
      plt.close(fig)
      return
    if key in ('q', 'Q'):
      if replay_mode:
        slog('Replay: Q disabled (use Enter to restart)')
      else:
        clear_all()
      fig.canvas.draw_idle()
      return
    if key in ('backspace',):
      if mode == 'running':
        undo_runtime_garbage()
      elif replay_mode:
        slog('Replay: edit undo disabled')
      elif mode in ('edit_goals', 'edit_garbage'):
        undo_last()
      fig.canvas.draw_idle()
      return

    if key == 'enter':
      if replay_mode:
        if mode != 'running':
          start_run()
        else:
          restart_run()
      elif mode == 'edit_goals':
        if len(edit_goals) < 2:
          slog('Need >= 2 goals before garbage mode')
        elif robot_xy is None:
          slog('Need robot (RMB) before garbage mode')
        else:
          mode = 'edit_garbage'
          slog('Mode: edit garbage. LMB=add garbage, Enter=start, Bksp=undo')
      elif mode == 'edit_garbage':
        start_run()
      elif mode == 'running':
        restart_run()
      fig.canvas.draw_idle()
      return

    if mode != 'running' or state is None:
      return

    if key == ' ':
      paused = not paused
      slog('Paused' if paused else 'Playing')
    elif key in ('a', 'A', 'left'):
      if checkpoint_idx <= 0:
        slog('Prev: at start')
      else:
        restore_checkpoint(checkpoint_idx - 1)
        paused = True
        slog(f'Prev cp={checkpoint_idx}/{len(checkpoints)-1}')
    elif key in ('d', 'D', 'right'):
      paused = True
      if checkpoint_idx < len(checkpoints) - 1:
        restore_checkpoint(checkpoint_idx + 1)
        slog(f'Next cp={checkpoint_idx}/{len(checkpoints)-1}')
      else:
        advance_one_point()
    else:
      return
    fig.canvas.draw_idle()

  def draw_evidence(ev: Optional[ClipEvidence]) -> None:
    clear_evidence()
    if ev is None:
      return
    colors = ['#00bcd4', '#7e57c2', '#26a69a', '#5c6bc0']

    # 1) 插点前整路角点：橙色菱形 + CORNER 标签（不受 corners_kept 刷新影响）
    for i, p in enumerate(ev.path_corners):
      sc = ax.scatter(
        [p[0]], [p[1]], c='#e65100', s=110, marker='D',
        edgecolors='white', linewidths=0.8, zorder=6)
      evidence_artists.append(sc)
      t = ax.annotate(
        f'CORNER{i + 1}', p, textcoords='offset points', xytext=(8, -16),
        fontsize=8, color='#e65100', fontweight='bold', zorder=7,
        bbox=dict(boxstyle='round,pad=0.15', fc='white', ec='#e65100', alpha=0.8))
      evidence_artists.append(t)

    # 2) 每轮 A–C 线段 + 两端虚线延长 + 垂足 + 标签
    for ri, rnd in enumerate(ev.rounds):
      col = colors[ri % len(colors)]
      ax_pt, c_pt, foot = rnd.a, rnd.c, rnd.foot
      vx, vy = c_pt[0] - ax_pt[0], c_pt[1] - ax_pt[1]
      L = math.hypot(vx, vy) + 1e-9
      ux, uy = vx / L, vy / L
      # 延长长度：至少 3m，或 A–C 长度的 0.6 倍
      ext = max(3.0, 0.6 * L)

      (ln,) = ax.plot(
        [ax_pt[0], c_pt[0]], [ax_pt[1], c_pt[1]],
        '-', color=col, lw=2.2, alpha=0.9, zorder=5)
      evidence_artists.append(ln)
      # 前方延长线（过 C）
      (fwd,) = ax.plot(
        [c_pt[0], c_pt[0] + ux * ext], [c_pt[1], c_pt[1] + uy * ext],
        ':', color=col, lw=1.6, alpha=0.75, zorder=4)
      evidence_artists.append(fwd)
      # 反向延长线（过 A）
      (rev,) = ax.plot(
        [ax_pt[0], ax_pt[0] - ux * ext], [ax_pt[1], ax_pt[1] - uy * ext],
        ':', color=col, lw=1.6, alpha=0.55, zorder=4)
      evidence_artists.append(rev)

      (perp,) = ax.plot(
        [rnd.garbage[0], foot[0]], [rnd.garbage[1], foot[1]],
        '--', color='#d500f9', lw=1.6, alpha=0.9, zorder=5)
      evidence_artists.append(perp)

      sa = ax.scatter(
        [ax_pt[0]], [ax_pt[1]], c=col, s=70, marker='s',
        edgecolors='white', linewidths=0.6, zorder=6)
      evidence_artists.append(sa)
      sc = ax.scatter(
        [c_pt[0]], [c_pt[1]], c=col, s=95, marker='D',
        edgecolors='white', linewidths=0.6, zorder=6)
      evidence_artists.append(sc)
      sf = ax.scatter(
        [foot[0]], [foot[1]], c='#ff6f00', s=70, marker='x', zorder=7)
      evidence_artists.append(sf)

      for lab, pt, off in (
          (f'A{rnd.round_i}', ax_pt, (-12, 8)),
          (f'C{rnd.round_i}', c_pt, (8, 8)),
          (f'F{rnd.round_i} t={rnd.t_d:.2f}', foot, (8, -14)),
      ):
        t = ax.annotate(
          lab, pt, textcoords='offset points', xytext=off,
          fontsize=8, color=col, fontweight='bold', zorder=7,
          bbox=dict(boxstyle='round,pad=0.15', fc='white', ec=col, alpha=0.8))
        evidence_artists.append(t)

      for p in rnd.deleted:
        circ = plt.Circle(p, 0.22, fill=False, ec='#ff6f00', lw=1.4, zorder=5)
        ax.add_patch(circ)
        evidence_artists.append(circ)

    # 3) 最终仍保留的角点（刷新后仍在 corners_kept）
    for i, p in enumerate(ev.corners_kept):
      t = ax.annotate(
        f'kept{i + 1}', p, textcoords='offset points', xytext=(-18, 12),
        fontsize=7, color='#bf360c', zorder=7)
      evidence_artists.append(t)

    # 4) 插点示意：始终队首；若有 C++ 锚点则标出继续点
    gx, gy = ev.garbage
    ring = plt.Circle((gx, gy), 0.45, fill=False, ec='#ff9800', lw=2.0, zorder=6)
    ax.add_patch(ring)
    evidence_artists.append(ring)
    t = ax.annotate(
      'INSERT@head', (gx, gy), textcoords='offset points', xytext=(10, 10),
      fontsize=8, color='#ef6c00', fontweight='bold', zorder=7,
      bbox=dict(boxstyle='round,pad=0.15', fc='white', ec='#ff9800', alpha=0.85))
    evidence_artists.append(t)
    if ev.insert_after is not None:
      ax_pt = ev.insert_after
      (ins,) = ax.plot(
        [ax_pt[0], gx], [ax_pt[1], gy],
        '-', color='#ff9800', lw=2.0, alpha=0.7, zorder=6)
      evidence_artists.append(ins)
      t2 = ax.annotate(
        'continue@', ax_pt, textcoords='offset points', xytext=(8, 10),
        fontsize=7, color='#ef6c00', zorder=7)
      evidence_artists.append(t2)

  def update(_frame: int):
    nonlocal robot_arrow, robot_body
    title = {
      'edit_goals': 'EDIT GOALS: LMB=goal+dir  RMB=robot  MMB=pan  Enter=garbage  Bksp=undo  Q=clear',
      'edit_garbage': 'EDIT GARBAGE: LMB=add  MMB=pan  Enter=start  Bksp=undo  Q=clear',
      'running': 'RUN: MMB pan  LMB/RMB add garbage  Bksp undo | SPACE/A/D | cyan=A-C+ext  orangeD=CORNER',
    }[mode]
    ax.set_title(title, fontsize=9)

    if mode != 'running' or state is None:
      # keep draft visible
      refresh_draft_artists()
      lines = [
        '=== mode ===',
        mode,
        '',
        '=== edit keys ===',
        'LMB drag = goal + yaw',
        'RMB drag = robot pose',
        'MMB drag = pan view',
        'Bksp     = undo',
        'Enter    = next stage',
        'Q        = clear all',
        'Esc      = quit',
        'Wheel   = zoom',
        '',
        '=== run keys ===',
        'SPACE = pause/play',
        'A / D = prev / next point',
        'Enter = restart run',
        'Q     = clear & re-edit',
        '',
        f'goals={len(edit_goals)} garbage={len(edit_garbage)}',
        f'robot={"set" if robot_xy else "none"}',
        f'robot_size={ROBOT_LENGTH_M:.0f}x{ROBOT_WIDTH_M:.0f}m',
        '',
        '--- log ---',
        *status_log[-6:],
      ]
      info_txt.set_text('\n'.join(lines))
      return path_line, active_sc, draft_sc, garbage_sc, robot_pt, info_txt

    if not paused and not state.done:
      maybe_publish_and_insert(state)
      if step_robot(state):
        push_checkpoint()

    if state.goals:
      path_line.set_data([p[0] for p in state.goals], [p[1] for p in state.goals])
      active_sc.set_offsets(state.goals)
    else:
      path_line.set_data([], [])
      active_sc.set_offsets(np.empty((0, 2)))

    deleted_sc.set_offsets(state.deleted if state.deleted else np.empty((0, 2)))
    passed_sc.set_offsets(state.passed if state.passed else np.empty((0, 2)))
    garbage_sc.set_offsets(state.garbage_xy if state.garbage_xy else np.empty((0, 2)))
    set_robot_artists(state.robot, state.yaw)
    draw_evidence(state.last_evidence)

    hit = ''.join(sorted(state.cases_hit)) or '-'
    ev = state.last_evidence
    ev_lines: List[str] = []
    if ev is not None:
      reason = ev.insert_reason
      # avoid non-ascii markers in panel
      reason = reason.replace('①', '1').replace('②', '2').replace('③', '3').replace('④', '4')
      reason = reason.replace('→', '->')
      ev_lines = [
        '=== last insert evidence ===',
        f'cases={"".join(ev.cases)}  insert_idx={ev.insert_idx}',
        reason[:52],
        f'path_corners={[(round(p[0],1), round(p[1],1)) for p in ev.path_corners]}',
        f'kept={[(round(p[0],1), round(p[1],1)) for p in ev.corners_kept]}',
        f'far={ev.far_from_head} round_del={sum(len(r.deleted) for r in ev.rounds)}',
        f'goala=({ev.goala[0]:.1f},{ev.goala[1]:.1f})',
      ]
      for rnd in ev.rounds:
        ev_lines.append(
          f'  R{rnd.round_i}:{rnd.tag} t={rnd.t_d:.2f} del={len(rnd.deleted)}')
        ev_lines.append(
          f'    A=({rnd.a[0]:.1f},{rnd.a[1]:.1f}) '
          f'C=({rnd.c[0]:.1f},{rnd.c[1]:.1f})')
      ev_lines.append('')

    lines = [
      '=== keys ===',
      'MMB   = pan map',
      'LMB/RMB = add garbage',
      'Bksp  = undo garbage',
      'SPACE = pause/play',
      'A/D   = prev/next point',
      'Enter = restart run',
      'Q     = clear & re-edit',
      'Esc   = quit',
      '',
      f'{"PAUSED" if paused else "PLAYING"}  '
      f'cp={checkpoint_idx}/{max(0, len(checkpoints) - 1)}',
      '',
      '=== legend ===',
      'green  = goals queue',
      'yellow = nav reached',
      'black  = clip-deleted',
      'orange D = path CORNER (judged)',
      'cyan   = A-C + dotted extension',
      'magenta= garbage->foot',
      'orange ring = insert@head',
      'blue box = robot 2x1m',
      '',
      *ev_lines,
      '=== insert rule (mirror C++) ===',
      f'>{HEAD_DELETE_ROBOT_DIST_M:.0f}m from goals[0]:',
      '  no delete, insert at head',
      f'<={HEAD_DELETE_ROBOT_DIST_M:.0f}m:',
      '  1/2 mid -> after A (A->G->C)',
      '  insert: always queue HEAD (go sweep now)',
      '  3 forward: refresh stale corners each round',
      'A=goals[0] (not robot)',
      '',
      f'hit so far: [{hit}]',
      f'garbage fin={sorted(state.finished)}/{len(state.garbage_items)}',
      f'sweep={state.last_sweep_order}',
      f'goals={len(state.goals)} del={len(state.deleted)}',
      f'robot=({state.robot[0]:.2f},{state.robot[1]:.2f})',
      '',
      '--- log ---',
      *(state.log_lines[-5:] if state.log_lines else status_log[-5:]),
    ]
    info_txt.set_text('\n'.join(lines))
    return path_line, active_sc, passed_sc, deleted_sc, garbage_sc, robot_pt, info_txt

  def on_scroll(event) -> None:
    if event.inaxes is not ax or event.xdata is None or event.ydata is None:
      return
    # up = zoom in, down = zoom out; keep cursor point fixed
    if event.button == 'up':
      scale = 1.0 / ZOOM_STEP
    elif event.button == 'down':
      scale = ZOOM_STEP
    else:
      return
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    cx, cy = float(event.xdata), float(event.ydata)
    new_w = (x1 - x0) * scale
    new_h = (y1 - y0) * scale
    # clamp view size
    min_span, max_span = 2.0, (MAP_X_MAX - MAP_X_MIN) * 2.0
    if new_w < min_span or new_h < min_span:
      return
    if new_w > max_span or new_h > max_span:
      return
    rx = (cx - x0) / (x1 - x0) if x1 != x0 else 0.5
    ry = (cy - y0) / (y1 - y0) if y1 != y0 else 0.5
    ax.set_xlim(cx - new_w * rx, cx + new_w * (1.0 - rx))
    ax.set_ylim(cy - new_h * ry, cy + new_h * (1.0 - ry))
    fig.canvas.draw_idle()

  fig.canvas.mpl_connect('button_press_event', on_press)
  fig.canvas.mpl_connect('motion_notify_event', on_motion)
  fig.canvas.mpl_connect('button_release_event', on_release)
  fig.canvas.mpl_connect('scroll_event', on_scroll)
  fig.canvas.mpl_connect('key_press_event', on_key)

  if replay_mode:
    refresh_draft_artists()
    start_run()

  anim = FuncAnimation(fig, update, interval=int(DT * 1000), blit=False, cache_frame_data=False)
  fig._anim = anim  # noqa: SLF001

  def on_close(_event) -> None:
    try:
      anim.event_source.stop()
    except Exception:
      pass

  fig.canvas.mpl_connect('close_event', on_close)

  if not hasattr(fig.canvas, 'manager') or fig.canvas.manager is None:
    raise SystemExit(
      f'no interactive window (backend={matplotlib.get_backend()}, '
      f'DISPLAY={os.environ.get("DISPLAY")!r}). try: export DISPLAY=:0')
  try:
    plt.show()
  finally:
    try:
      anim.event_source.stop()
    except Exception:
      pass
    plt.close('all')


if __name__ == '__main__':
  import argparse
  ap = argparse.ArgumentParser(description='InsertGarbagePose demo')
  ap.add_argument(
    '--replay', action='store_true',
    help='auto-run a recorded scenario (no mouse edit)')
  ap.add_argument(
    '--scenario', type=str, default=None,
    help='scenario json path (default: replay_c_bend.json next to this script)')
  args = ap.parse_args()
  run_gui(replay=args.replay, scenario_path=args.scenario)
