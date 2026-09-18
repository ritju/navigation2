# 覆盖 via 角色、角点扫掠 snap 与短边加严

本文是 `through_poses_guidance_path_scheme.md` 的增量方案：**只改 through-poses 分类 + FastPath 占用/走廊策略**，不改 Hybrid 插件，不改 `nav2_core`。碰撞仍只认 LETHAL(254)；253 / 原地转 / 动态障仍归局部。

公式用 `$...$` / `$$...$$`。分类 + 角点扫掠 snap + 短边加严已按第 11 节接入代码。

---

## 1. 目标与非目标

### 1.1 要保证

1. 行端拐角：局部转向时车体扫掠不进 254。
2. 短连接边：目标位姿完整 footprint 可占据，局部有空间跟完机动。
3. 长覆盖行内部：仍用现有半宽走廊，不被误标成短边、不被扫掠矩形吸到通道中心。
4. 中间 via 可丢；最后一点不可静默丢。

### 1.2 不保证 / 不做

- 全程矩形无 253。
- 角点锁作业 yaw。
- 沿边一维内收（墙在侧面时收不到自由位姿）。
- 全图无界「最近自由格」。
- 在 Hybrid 加密点列上判角点。
- 把扫掠代理多边形用于长边内部。

---

## 2. 模块边界

| 职责 | 落点 |
|---|---|
| 共线合并、标长短边、用折线内角标角点 | `computePlanThroughPoses` 入口，对**原始** `goal_poses` **一次** |
| 本段角色 → `FastPlanOptions` | 循环内、每次 `getPlan` 之前 |
| 占用 snap（含角点/短边扫掠形）、走廊、缩短/转线 | `FastPathPlanner::compute` |
| Hybrid | 走廊仍不通时；角点/短边的目标用 **snap 后的 $G'$**，不回滚到原始墙角 |

`ComputePathToPose` 无折线，不分类，行为与现有最后一段相同。

FastPath **禁止**自己用 $S$、$G$ 判角点（没有前一点/后一点）。

丢 via 后 **不重新分类**；剩余点沿用原始下标上的角色。

---

## 3. 折线内角（角点判据）

与参考实现相同：顶点 $i$ 用 $i-\mathrm{span}$、$i$、$i+\mathrm{span}$ 三点，余弦定理求**内角**（度）。直行 $\approx 180^\circ$，尖角更小；评不了则返回 180。

```text
polylineVertexAngleDeg(plan, idx, span) -> deg
  ia = idx - span
  ic = idx + span
  越界或 span < 1 → 180
  P, Pa, Pc = plan[idx], plan[ia], plan[ic]
  len_a = |P - Pa|      // 前点 → 当前
  len_b = |Pc - P|      // 当前 → 后点
  len_c = |Pc - Pa|
  len_a 或 len_b < 1e-6 → 180
  cos_t = (len_a^2 + len_b^2 - len_c^2) / (2 * len_a * len_b)
  return acos(clamp(cos_t, -1, 1)) * 180 / PI
```

分类对象必须是任务 via。`span` 默认 2。仍不得对 Hybrid `path.poses` 调用。

- **共线**（并入同一覆盖边）：内角 $\ge \theta_{colinear}$（默认 $135^\circ$）。
- **角点**：内角 $< \theta_{corner}$（默认 $120^\circ$，能抓住 $90^\circ$ 折返）。
- `via_angle_span` 默认 $2$。
- 两阈值之间（例如 $150^\circ$ 微折）：既不拆边、也不标 Corner。

---

## 4. via 角色

记 $G_0,\ldots,G_{N-1}$ 为任务点（不含机器人起点 $R$）。

每个 via **一个主角色**，优先级：`Last` > `Corner` > `Short` > `LongInterior`。

```text
classifyViaRoles(goals[0..N-1]):
  if N == 0: return []
  每个 i：vertex_ang = polylineVertexAngleDeg(goals, i, span)
           out_yaw = (i+1<N) ? atan2(G[i+1]-G[i]) : NAN

  // 4.1 共线合并成边（边长是覆盖行长度，不是相邻 via 间距）
  s = 0
  while s < N-1:
    e = s
    L = |G[s+1]-G[s]|
    while e+1 < N-1 and vertex_ang[e+1] >= theta_colinear:
      e += 1
      L += |G[e+1]-G[e]|
    edges.append({start:s, end:e+1, length:L})
    s = e+1

  // 4.2 短边
  L_short = short_edge_length  // 默认 2.0 m
  for E in edges:
    short = E.length < L_short
    if E.length < L_footprint:
      左端将是角点 或 右端将是角点 或 E.end==N-1 → short = true
    for i in E.start..E.end:
      先标 Short（若 short）否则 LongInterior
      记下 edge_id, edge_len

  // 4.3 角点：仅内部点 1..N-2（两端没有两侧边，不因夹角标 Corner）
  for i in 1..N-2:
    if vertex_ang[i] < theta_corner:
      role[i] = Corner

  // 4.4 最后一点
  role[N-1] = Last
  return roles
```

第一段 $R\to G_0$：目标角色跟 $G_0$；走廊按该角色，不把 $R$ 并进覆盖边长。

---

## 5. 角点扫掠代理多边形（仅 Corner 的占用/snap）

角点不查静止 footprint。差速在 via 附近近似绕 `base_footprint` 转：前角圆管端墙，后角圆管侧甩。用 **body 系矩形** 做轴对齐代理，再乘缩放 $k$。

从当前全局 footprint 多边形取顶点（相对 `base_footprint`）：

- 前左 / 前右：该侧 $x$ 最大的顶点（若无则用 $x_{\max}$ 且 $y$ 左正右负）
- 后左 / 后右：该侧 $x$ 最小的顶点

$$
R_{\mathrm{fl}} = \mathrm{hypot}(x_{\mathrm{fl}}, y_{\mathrm{fl}}),\quad
R_{\mathrm{fr}} = \mathrm{hypot}(x_{\mathrm{fr}}, y_{\mathrm{fr}})
$$

$$
R_{\mathrm{rl}} = \mathrm{hypot}(x_{\mathrm{rl}}, y_{\mathrm{rl}}),\quad
R_{\mathrm{rr}} = \mathrm{hypot}(x_{\mathrm{rr}}, y_{\mathrm{rr}})
$$

扩展矩形（$k=$ `corner_sweep_scale`，默认 $1.0$）：

$$
x_{\mathrm{front}}' = k \cdot \max(R_{\mathrm{fl}}, R_{\mathrm{fr}})
$$

$$
y_{\mathrm{left}}'  = k \cdot R_{\mathrm{rl}},\quad
y_{\mathrm{right}}' = k \cdot R_{\mathrm{rr}}
$$

$$
x_{\mathrm{rear}}' = x_b \quad \text{（后缘 }x\text{ 不乘 }k\text{、不后扩）}
$$

四点：$(x_{\mathrm{front}}', y_{\mathrm{left}}')$、$(x_{\mathrm{front}}', -y_{\mathrm{right}}')$、$(x_{\mathrm{rear}}, y_{\mathrm{left}}')$、$(x_{\mathrm{rear}}, -y_{\mathrm{right}}')$。无顶点时退回轴对齐 AABB 再按同样半径规则扩。

**半宽必须用后角半径**，不要用 $R_f$：前向加长已经覆盖鼻子打端墙；侧向再灌 $R_f$ 会把整车沿覆盖行法向灌肥，贴边行被吸走。鼻子转到正横，靠 **出边 yaw 上前 $x=R_f$ 的那一次检查**，不靠入边朝向的宽盒子。

角点占用判定：该多边形在指定 yaw 下扫格，只认 254；255 跳过；253 不算碰。

检查 yaw（至少两次，都要过才算该 $xy$ 自由；snap 环搜时每个候选都做）：

1. $\psi_{\mathrm{in}} = \mathrm{atan2}(G_i - S)$（本段入边；第一段 $S$ 为机器人）
2. $\psi_{\mathrm{out}}$：分类时记下的 `out_yaw`；最后一点或缺失时只用 $\psi_{\mathrm{in}}$（Last 不是 Corner 则走第 6 节静止 footprint）

实现可把「两次 yaw 都自由」收进 `pose_free` 的角点分支。

---

## 6. 占用调整：有界 snap，不沿边内收

角点与短边目标 **复用** `snapOccupiedGoal` 环搜，不实现 $G_0-t\hat{u}$ 内收。

### 6.1 碰撞形

| 角色 | snap 碰撞形 |
|---|---|
| `LongInterior` | 现状：半宽截面 |
| `Short` | **第 5 节扫掠矩形**（`corner_sweep_scale`），入/出两个 yaw 都要自由 |
| `Corner` | **第 5 节扫掠矩形**，入/出两个 yaw 都要自由 |
| `Last` | 现状：静止完整 footprint；若分类上 $G_{N-2}$ 为 Corner，本段 $G$ 只做 Last 前悬，不强制扫掠形 |

### 6.2 何时搜

现逻辑：原点已自由则不动。角点补一条：

- 扫掠形在 $\psi_{\mathrm{in}}$、$\psi_{\mathrm{out}}$ 已自由；但 $G$ 侧一车长窗口（扫掠形，只认 254）仍碰 → **视为占用**，继续环搜。

短边与角点共用扫掠矩形做占用 snap；原点该形自由且尾窗无 254 则不动。走廊仍关浅侵入豁免。

### 6.3 环搜约束

- 半径：`corner_snap_tolerance`（Corner/Short 共用；$0$ 表示 $\max(W/2+\delta,\ L_{\mathrm{footprint}}/2)$）。Last 仍用 `goal_occupied_tolerance`。
- 步长：现有 `goal_search_resolution`。
- 打分：现有 `dist + 0.25*|cross| + 0.05*|along|`，Corner 再减一项转向内侧（沿 $\psi_{\mathrm{in}}$、$\psi_{\mathrm{out}}$ 角平分指向内侧的位移加分）。
- 禁止无界全图最近。

半径内找不到：中间 via `GoalUnreachable` → server 丢点；Last 失败。**不要**把原始墙角交给 Hybrid。

### 6.4 与 stretch / 转线 / Hybrid

```text
snap（按角色选碰撞形）
走廊（按角色）
  通 → StraightOk
  不通且非 Corner 且 allow_stretch → 现有 tryStretchGoal
  不通且 allow_rotate → 现有转线
  仍不通 → NeedAstar
```

Corner / Short 本轮 **不做** `tryStretchGoal`（snap 已是占用调整，避免缩两次）。

`NeedAstar` 的 $G$：

| 情况 | Hybrid 目标 |
|---|---|
| Corner/Short snap 成功但走廊不通 | **保留 $G'$** |
| LongInterior 失败的 stretch/rotate | 回滚到占用 snap 后、调整前（现状） |
| snap 失败 | 不进 Hybrid，上层丢点/失败 |

---

## 7. 走廊（角色增量）

| 角色 | 中心线 | 边线 / 车体 | 浅侵入豁免 |
|---|---|---|---|
| `LongInterior` | 现状 | 半宽加深，只认 254 | 开（现状 $\delta$） |
| `Short` | 现状粗检 | $G$ 侧窗口按**扫掠矩形** | **关** |
| `Corner` | 现状粗检 | $G$ 侧窗口按**扫掠矩形**（$\psi_{\mathrm{in}}$ 或弦向）抽点 | **关** |
| `Last` | 现状 + 前悬 | 静止 footprint snap；前悬半宽截面（现状） | 近段规则不变 |

$s=0$ 仍不否决。长边内部禁止扫掠矩形。

近/远（`allow_rotate` / `allow_stretch`）仍用现有 `isNearSegment` / 最后一段例外。角色是加严，不替代近远。

`rewrite_via_yaw_to_approach` 不变。角点 `pose_free` 的 yaw 按第 5 节入/出边，**可与** rewrite 后的作业/来向 yaw 不同。

Hybrid 航向门禁：仅 `LongInterior` 且近段中间 via。`Corner` / `Short` / `Last` 关闭（短距大转角会拖死 A\*）。

start yaw 仍不改。

---

## 8. 流水线伪代码

```text
computePlanThroughPoses(start, goals[0..N-1]):
  roles = classifyViaRoles(goals)
  concat = empty
  for i in 0..N-1:
    S = concat.empty ? start : concat.back()
    G = goals[i]
    is_last = (i == N-1)
    opt = fillOptions(roles[i], S, G, is_last)
    path = getPlan(S, G, opt)
    if path empty:
      is_last ? fail : continue
    else:
      concat.append(path)

fillOptions(role, S, G, is_last):
  opt.strict_goal_footprint = role in {Last, Corner, Short}
  opt.use_corner_sweep     = (role == Corner or role == Short)
  opt.short_no_exempt      = (role == Short or role == Corner)
  opt.allow_stretch = (near or is_last) and enable_line_stretch and (role != Corner) and (role != Short)
  opt.allow_rotate  = near and enable_line_rotate
  opt.rewrite_goal_yaw_to_approach = 现规则
  opt.heading_gate = (role == LongInterior) and near_dist and not is_last
  opt.out_yaw = roles[i].out_yaw
  opt.snap_radius = (role in {Corner, Short}) ? corner_snap_tolerance
                   : goal_occupied_tolerance
  return opt

FastPath.compute(S, G, opt):
  按 rewrite 填 yaw_plan
  if not snapOccupiedGoal(S, G, opt):   // 见下
    return GoalUnreachable
  G_snap = G
  hit = checkCorridor(S, G, opt)
  if not hit.blocked: return StraightOk
  if opt.allow_stretch: tryStretch ...
  if opt.allow_rotate:  tryRotate ...
  NeedAstar：Corner/Short 保留当前 G，否则回滚 G_snap 策略同现状

snapOccupiedGoal(..., opt):
  free(xy, yaw_set) =
    if opt.use_corner_sweep:
      sweepPoly 在 psi_in 自由 且 (out_yaw 有效 → 也在 psi_out 自由)
    else if opt.strict_goal_footprint:
      静止 footprint 无 254
    else:
      半宽截面无 254

  psi_in = atan2(G-S)
  if free(G, {psi_in, out_yaw}):
    if (Corner or Short) and tailWindowHits254(S, G, opt):
      视为占用，进入环搜
    else:
      return true
  环 ring=1..ceil(snap_radius/res):
    每格候选：free(...) 则按 score 取最优
    有则写入 G 并 return true
  return false
```

---

## 9. 参数（`planner_server` 命名空间）

| 参数 | 建议默认 | 含义 |
|---|---|---|
| `via_angle_span` | 2 | 内角用的前后下标间隔 |
| `edge_colinear_angle_deg` | 135 | 内角 ≥ 此值并入同一条边 |
| `corner_angle_deg` | 120 | 内角 < 此值标 Corner |
| `short_edge_length` | 2.0 | 边长小于则短边 (m) |
| `corner_sweep_scale` | 1.2 | 扫掠矩形缩放 $k$ |
| `corner_snap_tolerance` | 1.5 | Corner/Short 环搜半径 (m) |
| `corner_snap_enable` | true | 角点/短边加严总开关；false 则分类只打日志 |

不与 `near_yaw_threshold`、`via_heading_tolerance`、`goal_occupied_tolerance`（Last/长边）复用。不写进 `GridBased.*`。

---

## 10. 调试

`[ThroughPoses]` 每 via：`role=long|short|corner|last` `ang=` `edge_len=` `dpsi_out=`。

`[FastPath]` snap：`snap_mode=halfwidth|footprint|sweep` `k=` `R_f=` `R_r=` `radius=` `in_yaw=` `out_yaw=`。

RViz：原始折线；角点 Marker；扫掠矩形（入/出 yaw 可各一）；snap 前后 $G$。session 仍仅 `endSession` 整包发出。

---

## 11. 实现顺序

1. `classifyViaRoles` + 日志 / Marker，**行为不变**。验收：密 via 长行是一条长边；行端 $90^\circ$ 为 Corner；行距连接为短边。
2. `FastPlanOptions` 增加扫掠/半径/out_yaw；Corner 用扫掠形有界 snap；snap 失败丢中间点；NeedAstar 保留 $G'$。
3. Short：与角点相同的扫掠矩形 snap（`k=` `corner_sweep_scale`，入/出 yaw）+ 关闭浅侵入豁免；不做 stretch。
4. 角点尾段碰也触发 snap；航向门禁仅 LongInterior。
5. 扫掠矩形画进 `planning_debug`（`ns=corner_sweep`：品红入边、青出边）。

本步不落地 `path_untangle`、远段推 G。

---

## 12. 验收

- 密 via 覆盖行：内部半宽直线，不整行变短边。
- 行端 $90^\circ$：snap 后扫掠矩形（入/出 yaw）不碰 254；下一段 $S$ 为 $G'$。
- 贴边长行不被 $R_f$ 半宽从墙边吸走。
- 短连接边：目标静止 footprint 可占据，局部能跟完。
- snap 半径内失败：中间缺 via，concat 不断；最后一点失败。
- 单点 NavigateToPose：无分类，与现在一致。
- 不出现短距锁作业 yaw 导致的 Hybrid 绕圈。

---

## 13. 与现方案条款的对照

| 现条款 | 本方案 |
|---|---|
| 中间 via 半宽走廊 | `LongInterior` 不变 |
| Last / NavigateToPose 完整 footprint + 前悬 | 不变 |
| 占用 `snapOccupiedGoal` | Corner/Short 换碰撞形与半径，算法仍是环搜 |
| 近段 stretch/rotate | Corner 不做 stretch；其余不变 |
| 改 yaw / 航向门禁 | 规则不变；门禁范围收窄到 LongInterior |
| Hybrid 弱引导 | 不改插件；仅 goal 用 snap 后位姿 |

**一句话：** server 对原始折线用内角分类；FastPath 按角色 snap（角点/短边：后缘不动、前 $x\leftarrow k R_f$、半宽 $\leftarrow k R_r$ 的扫掠矩形，入/出 yaw）；找不到则丢中间点，不沿边内收、不把墙角交给局部。
