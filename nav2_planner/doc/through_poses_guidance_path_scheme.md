# 覆盖多点导航：弱运动学 Hybrid 引导路径方案

本文固化 through-poses（`ComputePathThroughPoses`）全局规划的讨论结论与落地技术方案。  
**本期已落地：** 254 半宽走廊（中心线粗检 + 窗口加深）、单侧浅侵入净空豁免、近段仅在净空够且侵入较大时转线、最后一段 / NavigateToPose 中心线通过后补前悬、近段缩短、远段不转线、倒车直线、走廊 RViz（session Marker 延后 flush）。裁剪 / 远段推 G / 丢点比例仍待做。

公式统一用 `$...$`（行内）和 `$$...$$`（独立行）。下标写成 `$d_{near}$`，不要用 `\mathrm`、`\text`、`\,`；表格单元格里不要写 `$|...|$`（竖线会拆列），改用 `abs(...)`。不使用 `\(...\)`。

适用对象：矩形 footprint、差速底盘、覆盖作业（via-point 密、中间点可丢、终点不可静默丢）。  
局部规划（MPPI / TEB / DWB）负责矩形碰撞、原地转、253 膨胀层与动态障碍。

---

## 1. 目标与非目标

### 1.1 全局必须保证

1. **同伦正确**：中心线不穿 LETHAL(254)，不把局部送进死胡同。
2. **进度可定义**：路径大致沿覆盖前进，不把「多绕的一圈」交给局部去跟。
3. **偏离量在局部能力内**：折角、航向差、擦膨胀，局部差速可修。

### 1.2 全局不必保证

- 中间 via 的作业航向（yaw）对齐。
- 全程矩形 footprint 无 INSCRIBED(253)。
- 处处满足 Hybrid 最小转弯半径 $R$（$R$ 只是舒适半径）。
- 路径绝对平滑、无折点。

### 1.3 分层

```text
planner_server
  FastPath          snap + 直线 / 终点伸缩 / 转线（前置，非插件）
  GridBased/Hybrid  弱运动学引导线（现有 GlobalPlanner 插件）
  path_untangle     自交裁剪（库函数，非插件）
        ↓
局部 MPPI/TEB/DWB   真矩形 + v,ω（含原地转）
```

不把 Lattice-diff 作为默认全图全局规划器（分支与 footprint 检查过贵）。

---

## 2. 现有问题（方案要修的）

1. **snap 朝向用作业 yaw**：同一格子用来向检查可能碰撞，直线失败、后续段起点贴障。
2. **中间 via 锁死 yaw**：短距离 Hybrid 解析展开（Dubins CCC）为对齐航向绕圈打结。
3. **through-poses 失败 `continue` 不更新 concat**：后面所有点共用坏起点。
4. **直线失败立即 A\***：近段本可先伸缩终点或把线转开，不必上 Hybrid。
5. **打结路径拼进 concat**：局部会去跟踪那个圈。
6. **用弧长/直线距当打结**：绕岛、走 U 臂被误杀。

---

## 3. 模块边界（不要做成规划插件）

| 职责 | 落点 | 是否 pluginlib |
|---|---|---|
| 来向 snap、直线、终点伸缩、绕起点转线 | `FastPathPlanner`（现有类） | 否 |
| 近/远分流、改 yaw、丢中间点、推开 G、终点保护 | `PlannerServer::computePlanThroughPoses` | 否 |
| 自交检测 + 裁环 | 新建 `path_untangle.{hpp,cpp}` | 否 |
| Hybrid 搜索 | 现有 `SmacPlannerHybrid`（`GridBased`） | 已是插件，本方案尽量不改 |

**直线仍写在 FastPath。** 碰撞只认 LETHAL(254)：半宽走廊、snap 截面、伸缩/转线重检共用 `checkCorridor`。中间 via 不加前后悬；最后一段中心线通过后补前悬。不在 $S$ 原地转碰撞。文件过长时可同库拆 `fast_path_straight.cpp`，对外仍是一个 `FastPathPlanner`。

**打结裁剪单独源文件，不要做成 `nav2_core::GlobalPlanner`。**  
裁剪是对已有 `nav_msgs/Path` 的几何后处理，需要返回「新路径 + 是否裁过 + 是否仍自交」，`createPlan(start, goal) → Path` 装不下；也不应绑死在 Hybrid 内部。

调用点：Hybrid（或任意 GridBased）出路径之后、写入 `concat_path` 之前。推荐在 through-poses 循环里显式调用；单点 `ComputePathToPose` 的 `getPlan` 末尾也可同样裁一次。

---

## 4. 段分类

每一段：

- 起点 $S$：上一**成功**段终点（`concat_path.back()`）；没有则用机器人位姿。不要用原始 $G_{i-1}$。
- 目标 $G$：当前 via。

**近且朝向接近**（须同时满足，阈值参数化）：

- $abs(G - S) < d_{near}$
- $abs(yaw_G - yaw_S) < \theta_{near}$
- 作业 yaw 与连线方向 `atan2(G - S)` 也接近（作业方向已和扫线差很多时，不要硬保直线）

否则视为 **远段 / 大转向**：仍可走半宽走廊直线（含单侧浅侵入且净空够），但 **禁止转线与伸缩**，走廊不通则只 snap 目标后交给 Hybrid。

中间 via 的规划 yaw：`rewrite_via_yaw_to_approach:=true` 且 $d(S,G) \le d_{near}$ 时改为来向 `atan2(G - S)`；远段中间 via 保留作业 yaw。  
单点 `ComputePathToPose`、以及 through-poses **最后一个目标**：**不改 yaw**。找不到 snap / 裁空 / 仍结时不得静默 `continue`，应放大容差再试或整次失败。

---

## 5. 端到端流水线

```text
for 每个 via G:
  S = concat 终点（或机器人）；S 的 yaw 保持原方向，不改
  yaw_plan(G) := 近段中间 via 且开关开启 ? 来向 S→G : 原 yaw
                （单点 / 最后一点 / 远段永不改）
  最后一段 / NavigateToPose → 中心线 + 前悬；中间 via → 中心线 + 半宽
  两者都只认 254 为碰撞；253 仅作中心线提示；不在 S 原地转碰撞

  snap：
    中间 via：半宽截面 + yaw_plan
    最后一段 / NavigateToPose：完整 footprint + 原 yaw
    失败 → 中间 continue；终点 / NavigateToPose 整次失败

  走廊检查（中间=半宽，最后=半宽+前悬；S 后悬不否决）
    通 / 单侧浅侵入且对侧净空 >= 车宽 → 直线
    撞在 G 邻域 / 前悬 → 缩短 G（最后一段即使远也允许缩短）后重检
    近段单侧 254、净空够、侵入 >= δ → 绕 S 转线后重检
    单侧净空不够 / 中线 / 两侧 → 不转线
    远段中间 via：不转线、不缩短 → 浅侵入净空够仍直线，否则 Hybrid
    仍撞 → Hybrid(start_yaw=S 原方向, goal_yaw=yaw_plan)
    最后一段 G 上车体仍 254 且 snap 不到 → 失败，不准报成功

  仅成功段 insert concat
```

直线失败不是丢中间点的条件。NavigateToPose 整段按最后一段处理。

---

## 6. FastPath：254 半宽走廊 + 近段伸缩/转线 + 倒车直线

### 6.0 碰撞统一 254

全局直线 / snap / 伸缩 / 转线 **只认 LETHAL(254)**。253 由局部处理。未知格（255）不当墙。
半宽取 footprint 真实 $\max |y|$。中心线全部 $<253$ 即视为走廊可走，**不再扫边线**；边线略蹭 254 由局部处理。中间 via **不加** 前后悬。最后一段在中心线通过后再补 $G$ 前悬；$S$ 处截面（$s=0$）不否决（机器人已在 $S$）。

Hybrid 插件内部仍可用自己的 footprint 代价；走廊 254 通了不会进 Hybrid。

### 6.1 来向 snap

- 截面检查与环搜使用 **本段 `yaw_plan`**：近段中间 via 且开关开启时为来向，否则为 via 原 yaw（单点 / 最后一点 / 远段永不改成来向）。
- 点是否可 snap：该点处垂直检查朝向的半宽线段上无 254。
- 同等圈内略偏好沿连线、小横偏。
- 找不到：中间点丢；终点不丢。

### 6.2 走廊检查（替代整车沿线 + 原地转）

不在 $S$ 旋转碰撞，检查朝向直接取 `atan2(G-S)`。
几何：线段 $S$–$G$ 向两侧各扩半车宽的定向矩形。

实现分两级（$R_{ins}$ = 全局图 `getInscribedRadius()`，$w$ = 半宽）：

1. **中心线粗检**（仅当 $R_{ins} \ge w$）：沿 $S \to G$ 按分辨率读中心线。全部 $<253$（255 跳过）→ 走廊通过，**不扫边线**。代价为 253 或 254 才进入加深。
2. **窗口加深**：只认 254。以触发点为中心沿轨前后各扩 $\max(R_{ins} + res, s_{half})$，其中 $s_{half}$ 为半车长步长；扫垂直半宽截面；$s=0$ 不否决。窗口没有不可豁免的 254 则仍出直线（中心线 253 只是膨胀晕）。单侧浅侵入见下。
3. 若 $R_{ins} < w$，退回整段截面扫描（同样跳过 $s=0$）。

中线 254（$abs(d_y) \approx 0$）不转线、不豁免。转角用碰撞处余量除以 $s$，不是 `atan2(d_y, s)`，也不是除以 $L$。

记下沿轨 $s$、段长 $L$、横向 $d_y$（左正）、侵入 $\delta_{hit} = w - abs(d_y)$、对侧净空是否够、是否两侧都有 254。

半宽扫描碰到 254 **不立刻否决整段**。中线 / 两侧立即否决。单侧则只在该碰撞点（按地图分辨率沿法向走一趟，不扫整个窗口的每个无碰截面）量净空：

- 从该 254 格沿垂直于该 pose、背离碰撞侧的方向，搜索 $W + 2\delta$，$W$ 为车宽（footprint 的 $y_{max}-y_{min}$），$\delta$ 为 `corridor_intrusion_tol`。
- 先跨过连续 254，再计连续 **非 254** 长度 $L_{free}$（253、255 可过；出图当墙）。
- $L_{free} \ge W$ → 净空够。该点豁免后 **沿 $s$ 继续扫**，后面每个 254 再各探一次。后面若出现中线 / 两侧 / 净空不够，仍否决。

| 命中 | 近段 | 远段 |
|---|---|---|
| 无 254，或单侧浅侵入（$\delta_{hit} \le \delta$）且净空够 | 原直线 | 原直线 |
| 终点邻域 $s > L - L_{goal}$ | 先缩短 $G$ | **不**伸缩（最后一段除外），Hybrid |
| 单侧、净空够、$\delta_{hit} > \delta$ | 绕 $S$ 转线 | **不**转线，Hybrid |
| 单侧净空不够 / 中线 254 / 两侧夹住 | **不**转线，Hybrid | Hybrid |

转线是把中心线拧进 **已经存在** 的缝，不是去找新同伦。净空不够时小转角变不出车宽，直接放弃直线。侵入小于 $\delta$ 时局部能修横偏，不必转。远段即使净空够、侵入大，也不转线。

同一轮先伸后转，禁止正负来回。远段仍做 **一次** 走廊检查，通了或浅侵入豁免照样直线。  
缩短 / 转线只有走廊因此变通才采用 $G^{\prime}$。`NeedAstar` 时 Hybrid 的目标 **回滚到占用 snap 之后、调整之前**（yaw 为 `yaw_plan`），不要把半成品 $G^{\prime}$ 交给插件。

### 6.3 终点附近缩短（仅近段）

覆盖扫线顶到墙时，把 $G$ 沿射线收回比拧歪整条线更符合覆盖。

$$
G^{\prime} = S + \max(s_{hit} - res, 2 \cdot res) \cdot \hat{u}
$$

$abs(G^{\prime} - G_{original}) \le d_{stretch}$，否则不伸。默认不伸长（`line_stretch_allow_extend:=false`）。缩短后走廊仍不通 → 不把收回的 $G^{\prime}$ 交给 Hybrid。

### 6.4 绕起点转线（仅近段）

- **前置：** 只在单侧 254、对侧净空够、且 $\delta_{hit} > \delta$ 时转线。净空不够、中线、两侧 → 不转，直接 Hybrid。
- 只向远离 254 的一侧转。
- $s=0$ 截面贴 254 不作为转线输入（已在走廊扫描里跳过）。
- 每步转角 $\Delta\alpha = \max(w + res - abs(d_y), res) / \max(s, s_{min})$。杠杆臂是碰撞点到 $S$ 的距离 $s$，不是段长 $L$。$s_{min} = \max(s_{half}, 4 \cdot res)$，$s_{half}$ 为半车长步长，避免 $s$ 过小一步转爆。$abs(G^{\prime}-G)$ 仍受 `line_rotate_goal_shift_tol` 限制。
- $G^{\prime}$ = 新射线上离原 $G$ 最近且在 $S$ 前方的点。
- $abs(G^{\prime}-G)$ 超 `line_rotate_goal_shift_tol` → 放弃转线，Hybrid 用 snap 后的原 $G$。
- $\alpha$ 只往同一侧累加，最多 `line_rotate_max_iters`。转线未通走廊时不保留中间 $G^{\prime}$。转后重检若只剩浅侵入且净空够，视为通过（不必拧到半宽内零 254）。
- 转后出现对侧 / 两侧 / 净空不够 → 放弃转线。

### 6.5 倒车直线

半宽走廊与前进是 **同一条几何**，不必先转 $\pi$ 再查一遍。
走廊通了之后：若 `/enable_backward` 或窄通道，且 $S$ 的原 yaw 更接近来向 $+\pi$，则路径点 yaw 用来向 $+\pi$，否则用来向。局部按路径 yaw 进退。

### 6.6 Hybrid 朝向

- **start yaw 保持 concat / 机器人原方向**（FastPath 不得改 start）。
- **goal yaw：** `rewrite_via_yaw_to_approach:=false` 时全部保留原 yaw。`true` 时仅 **through-poses 近段中间 via**（$d \le d_{near}$，只看直线距离）改成 S→G 来向；**单点导航、最后一点、远段中间 via 不改**。该判定在 `getPlan` 里算完再传给 FastPath snap / NeedAstar，避免 server 与前置不一致。
- 短距大夹角仍可能绕圈，交给后续裁剪；不要为灭结去改 start yaw。

近/远 **只看** $d=\mathrm{hypot}(G-S)$ 与 `near_distance_threshold`，不用 `isNearSegment`（后者还比 yaw，拧反后会被误判成远段）。

**近段中间 via**（$d \le d_{near}$，且不是最后一段 / 单点）：

1. Hybrid 同一轮 A\*：XY 仍用 `GridBased.tolerance`（默认 0.5 m 先不收）。进入 XY 容差的节点还须 $|\Delta\psi| \le$ `via_heading_tolerance` 才允许提前返回；航向不合格继续搜。比较的是交给 Hybrid 的 goal yaw（pose orientation）。不重试、不挪 $G$。
2. 插件返回后再做航向裁尾：从终点往起点累加弧长，只在 `via_heading_trim_length` 窗口内取最后一个航向合格点，其后丢掉。
3. Hybrid 空路径，或窗口内没有合格点，或裁完只剩起点 → **丢掉该中间 via**（concat 不变）。

**单点 / 最后一段**（`strict_goal_footprint`）：Hybrid 搜索不加航向门禁（XY 容差即可提前返回），避免短距大夹角把 A\* 拖死或和参数回调抢锁。仍可做航向裁尾；裁不了就保留；空路径则失败。永不丢该点。

**远段**（$d > d_{near}$）：

- Hybrid 搜索维持 XY 容差提前返回（不加航向门禁）。
- 仍可裁尾（同一套航向容差 + 弧长窗口）；裁不了 **保留原路径**，不丢 via。

只处理 Hybrid 路径。代价尾裁 `GridBased.tail_trim_length` 仍在插件内先做，航向裁尾在 `planner_server` 拿到路径之后。`via_heading_tolerance` 不与 `near_yaw_threshold` 复用（后者只给转线/伸缩）。


### 6.7 最后一段 / NavigateToPose：中心线通过后补前悬

中间 via 仍用半宽走廊（不加前后悬），允许 via 略顶墙。

停靠段（`ComputePathToPose` 全程，或 through-poses **最后一个 via**）与中间 via 共用中心线粗检，**不要**再沿路重叠填整车 AABB。中心线通过后只补 $G$ 前悬：

- 前悬 = footprint 前端 $x_{front}$。沿来向扫 $[L, L + x_{front}]$ 的半宽截面，只认 254。内切圆盖不住车头，这一段不能省。
- $S$ 后悬**不否决**（机器人已在 $S$）。
- snap 仍用完整 footprint，避免搜到中心空、车体进墙的点。
- 中心线触发点靠近 $G$ 时，加深窗口延伸到 $L + x_{front}$。
- 命中在前悬（$s > L$）→ 缩短用 $s - x_{front}$ 作为目标弧长，避免把 $G$ 伸进障碍。
- 命中在终点邻域 / 前悬 → 缩短 $G$ 后重检（最后一段远距离也允许缩短）。
- 近段单侧、净空够、侵入大于 $\delta$ → 转线后重检；浅侵入净空够 → 原直线；净空不够不转线。远段不转线。
- $G$ 处 footprint 仍 254 且邻域 snap 失败 → `GoalUnreachable`，through-poses 对最后一点失败，NavigateToPose 失败。

### 6.8 走廊实现与性能

| 项 | 约定 |
|---|---|
| 中心线粗检 | $R_{ins} \ge w$ 时沿 $S \to G$ 读中心线；全部 $<253$ 即通过，不扫边线；253/254 才加深；255 跳过 |
| 加深 | 窗口内半宽截面，只认 254；$s=0$ 不否决；单侧碰撞点做法向净空探针，浅侵入且净空够则继续扫；硬否决（中线/两侧/净空不够）立即返回；侵入大于 $\delta$ 且净空够则作为转线输入（若后面还有硬否决，硬否决优先） |
| 净空探针 | 仅碰撞点；从 254 沿反法向搜 $W+2\delta$；先跨致死再计连续非 254；$L_{free} \ge W$ 为够 |
| 转线步长 | $(w + res - abs(d_y)) / \max(s, s_{min})$，杠杆臂用碰撞点弧长；仅净空够时转 |
| 中间 via | 中心线通过 → 直线；否则窗口加深。不加前后悬 |
| 最后一段 | 中心线通过 → 再扫 $G$ 前悬半宽带；$S$ 后悬不否决 |
| 退化 | $R_{ins} < w$ 时整段半宽扫描（最后一段含前悬） |
| Marker | session 下 `publishMarkers` 只入缓存；**仅 `endSession` 整包发出**（Hybrid 各段异色）。不要每 via flush |


---

## 7. 自交检测与裁剪（`path_untangle`）

只处理 **本段** `curr_path`，不要对整条 `concat_path` 做自交（覆盖折返会误报）。  
直线路径一般不裁。第一版只做 **真交叉 + 环长下限**。

### 7.1 遍历

点列 $P_0, \ldots, P_{n-1}$，边 $e_i = P_i P_{i+1}$。  
$n < 4$ 不可能自交。

```text
预计算前缀弧长 s[k]
for i = 0 .. n-4:
  for j = i+2 .. n-2:          // 跳过邻边
    loop_len = s[j] - s[i+1]
    if loop_len < L_min: continue
    if 边 e_i 与 e_j 真交叉:
      记录 (i, j, 交点 P, loop_len)
```

$L_{min}$ 建议 $\pi R$ 或 1～2 m，滤掉折线锯齿。

### 7.2 真交叉（参数法，同时拿交点）

$A = P_i$，$B = P_{i+1}$，$C = P_j$，$D = P_{j+1}$：

$$
A + t(B - A) = C + u(D - C)
$$

- $abs(den)$ 过小：平行/共线，不当真交叉。
- 真交叉当且仅当 $\varepsilon < t < 1 - \varepsilon$ 且 $\varepsilon < u < 1 - \varepsilon$（不含端点）。
- 交点 $P = A + t(B - A)$。

不要用弧长/欧氏距离比，不要用「同一格子走两次」当主判据，不要把路径首尾连成闭合多边形。

### 7.3 裁剪

每一趟只裁 **loop_len 最短且 ≥ $L_{min}$** 的一对：

- 保留 $P_0 \ldots P_i$，插入 $P$，再接 $P_{j+1} \ldots P_{n-1}$（丢掉两次经过交点之间的点）。
- 拼接点 yaw 用裁完后下一段切向。
- 对结果再跑检测，直到无真交叉或达到最大轮数。
- 环几乎是整条路径、裁完接近空 → 视为裁失败，不要硬裁。

裁完路径是原点列的子列（外加插值点 $P$），不新增穿障弦；仍建议对插值点做一次 254 检查。

### 7.4 建议接口（库函数，非插件）

```text
struct UntangleResult {
  nav_msgs::msg::Path path;
  bool clipped;                    // 是否裁过至少一环
  bool still_self_intersecting;    // 裁完仍有真交叉或裁失败
};

UntangleResult untangleSelfIntersections(
  const nav_msgs::msg::Path & path,
  double min_loop_length);
```

通过 `still_self_intersecting` 驱动近段丢点 / 远段推开，不要只返回 bool。

### 7.5 明确不做（第一版）

- 同向近切（棒棒糖相切环）：线上若仍多见再补「距离 < 1 格 且 航向同向 且 环长 ∈ $[L_{min}, L_{max}]$」。U 形近平行反向 **禁止** 当自交。
- 累计转角多 $2\pi$：仅可作近段兜底，不参与远段、不提供裁点。
- 远段不因「转得多 / 路长」丢点。

---

## 8. Hybrid 之后的分流

### 8.1 近且朝向接近

```text
Hybrid（yaw 已是来向）
  → untangle
  → 无自交 → 拼接
  → 仍自交 / 裁空 / 近段兜底仍多一圈
       → 丢该中间点，本段不写入 concat
       → 下一段从 concat.back() → G_{i+1}
```

近段不要先做「推开目标」。转线/伸缩已经在修终点贴障与侧向擦障；再结说明该点不适合当扫线锚点。

### 8.2 远或转向大

```text
Hybrid（来向 yaw）
  → untangle
  → 无自交 → 接受（绕岛、U 臂走这里）
  → 仍结（多为终点贴障进场圈）
       → P_obs = 距 G 最近的 LETHAL(254) 格
       → G' = G + d * normalize(G - P_obs)     // d ≈ 半车宽 + 1～2 格
       → G' 用来向做碰撞：撞了则不改 G，接受裁后路径
       → 不撞则只重规划一次 → 再 untangle
       → 仍结则接受，不丢远段点
```

推开只用 254，只一轮。修的是贴墙进场圈，修不了中段绕岛——绕岛本来就不该丢。

远段 Hybrid 前的「推开 G」与近段直线的「沿射线缩短」都是挪终点，但场景不同：近段保扫线、只沿当前射线动；远段是离开最近实体墙、给进场留空隙。不要混成一个函数乱调用。

### 8.3 终点

中间点可丢；**最后一个目标不能静默剔除**。snap 失败、裁空、仍结：无 yaw 再规划一次，或放大 snap 容差，或整次 through-poses 失败。

建议：丢掉超过一定比例（例如 30%）则整次失败并打日志（原因：占用无替代 / 打结）。

---

## 9. 落地文件与改动范围

均在 `nav2_planner` 内。`nav2_smac_planner` 第一版不改（不锁 yaw、不改解析展开也可先靠外层改 goal yaw + 裁剪兜住）。

| 文件 | 改动 |
|---|---|
| `include/nav2_planner/fast_path_planner.hpp` + `src/fast_path_planner.cpp` | 中心线粗检 + 半宽加深；最后一段补前悬；来向 snap；近段缩短/转线；倒车只改路径 yaw；`compute(options)` |
| 可选 `src/fast_path_straight.cpp` | 仅当直线+伸缩+转线把单文件撑太大，同库拆分 |
| **新建** `include/nav2_planner/path_untangle.hpp` | `UntangleResult` 与函数声明 |
| **新建** `src/path_untangle.cpp` | 真交叉、裁环、循环 |
| **新建** `include/nav2_planner/planning_debug_viz.hpp` + `src/planning_debug_viz.cpp` | 调试 Path / Marker 发布；日志前缀 `[PlanDbg]` |
| `src/planner_server.cpp` | through-poses：来向 yaw、近/远、直线失败不丢点、调 untangle、近段丢点、远段推 G、终点保护、失败 continue 起点不变；`[ThroughPoses]` / `[getPlan]` 日志；接入 debug viz |
| `CMakeLists.txt` | 把 `path_untangle.cpp`、`planning_debug_viz.cpp` 编进 `planner_server_core` |
| 参数 | 见下一节；动态参数按现有 `planner_server` 模式挂 |

**不要** 新增 `pluginlib` xml / `GlobalPlanner` 子类。  
**不要** 把裁剪写进 `smac_planner_hybrid.cpp`。  
**不要** 把转线/伸缩写成第二个规划插件。

### 9.1 `getPlan` 与 through-poses 的分工建议

- `getPlan(..., allow_stretch, allow_rotate, strict_goal_footprint)`：FastPath → 插件。start yaw 不改。goal yaw 仅在「总开关开启 + 非终点 + 近段距离」时改为来向。
  中间 via：半宽走廊，远段 stretch/rotate=false。
  最后一段 / NavigateToPose：`strict_goal_footprint=true`（中心线 + 前悬，snap 仍用完整 footprint，**不改 yaw**）；缩短在最后一段即使远也允许；转线仍仅近段。
- 近/远（转线/伸缩）仍用 `isNearSegment`；改 yaw / 航向门禁 / 丢 via 只用直线距离。航向门禁仅近段中间 via。
- FastPath 的 `NeedAstar`：插件目标 = 占用 snap 后的 $G$，不含失败的缩短/转线。
- FastPath 的 `GoalUnreachable`：through-poses 对中间点 `continue`，对最后一点失败。

现有起点占用 `pop_back` 恢复可保留，与「丢 via」是两条恢复路径，不要混成一次 `pop_back` 既丢路径点又丢作业点。

---

## 10. 建议参数（均在 `planner_server` 命名空间）

名称可在实现时微调，语义应保持稳定。

| 参数 | 建议默认 | 含义 |
|---|---|---|
| `near_distance_threshold` | 2.0 m | 近段距离阈值 $d_{near}$；航向门禁（仅中间 via）/裁尾/丢 via 只看该距离 |
| `near_yaw_threshold` | 0.35 rad（约 20°） | 近段转线/伸缩的航向差；**不**用于 Hybrid 到达或裁尾 |
| `via_heading_tolerance` | 0.35 rad | Hybrid 近段**中间 via** 到达门禁与航向裁尾的 $|\Delta\psi|$；单点/最后一段不开搜索门禁 |
| `via_heading_trim_length` | 1.0 m | 航向裁尾最大累计弧长；超出窗口不再往前裁 |
| `enable_line_rotate` | true | 近段直线转线 |
| `line_rotate_max_iters` | 5 | 转线最大圈数 |
| `line_rotate_goal_shift_tol` | 0.5 m | 转线后 abs(G'-G) 上限 |
| `corridor_intrusion_tol` | 0.08 m | 单侧 254 侵入阈值 $\delta$；小于该值且净空够则接受原直线，不转线 |
| `enable_line_stretch` | true | 近段终点沿射线伸缩 |
| `line_stretch_max` | 0.4 m | abs(dx) 与沿轨位移上限 $d_{stretch}$ |
| `line_stretch_goal_window` | 0.8 m | 终点邻域 $L_{goal}$ |
| `line_stretch_allow_extend` | false | 是否允许后端侵入时伸长；建议默认只缩短 |
| `rewrite_via_yaw_to_approach` | true | 总开关。true：仅多点近段中间 via 改成来向；单点 / 最后一点 / 远段不改。false：全部保留原 yaw。FastPath 跟同一判定 |
| `publish_planning_debug` | true | 发布 `planning_debug/*` 供 RViz |
| `planning_debug_keep_mode` | `session` | `session`：一次规划内累加所有 via；`current`：只显示当前段 |
| `planning_debug_footprint_stride` | 0 | session 下 0=不沿路画 footprint；>0 按步长累加 |

$R$ 沿用 Hybrid 的 `minimum_turning_radius`。

**未落地、暂不声明：** `untangle_*`、`goal_push_*`、`drop_via_max_ratio`（裁剪 / 远段推 G / 丢点比例）。落地时再加回 server。

**Hybrid 已删除：** `enable_straight_expand`、`goal_occupied_tolerance`、`goal_search_resolution`、`goal_close_to_obstacle_distance`、`footprint_extend_*`。snap / 直线只在 FastPath。尾裁用 `GridBased.tail_trim_length`：只在终点往回该弧长窗口内按车长抽路径点，中心线 `cost < 253`（255 跳过）才保留；窗口内全挡住则空路径。

可直接替换现场 yaml 的参考文件：[`planner_server_params.example.yaml`](planner_server_params.example.yaml)。  
已从旧配置中删除：`planner_accumulate_distance`（server 未读取）、整段注释掉的重复 `GridBased`、`debug_visualizations`（改用根参数 `publish_planning_debug`）、`goal_close_to_obstacle_distance`、Lattice 专用项（`rotation_penalty`、`allow_reverse_expansion`）、2D 专用 `cost_travel_multiplier`。

---

## 11. 实现顺序（建议分 PR / 分步，仍不在本文改代码）

1. **调试可视化骨架（已落地）**  
   `PlanningDebugViz` + 走廊矩形 `ns=corridor` + 直线/snap/Hybrid 原始路径。
2. **FastPath 254 走廊（已落地）**  
   中间 via：中心线粗检 + 窗口内半宽加深（只认 254）+ 单侧碰撞点净空探针。浅侵入且净空够接受原直线；近段仅净空够且侵入大于 $\delta$ 才转线。最后一段 / NavigateToPose：中心线通过后补前悬；snap 仍用完整 footprint。近段缩短；远段中间 via 不转线；倒车只改路径 yaw。session Marker 延后 flush。
3. **Hybrid 近段航向门禁 + 航向裁尾（本期已落地）**  
   仅近段中间 via：A\* 要 XY 容差 + `via_heading_tolerance` 才提前返回。单点 / 最后一段不加搜索门禁。随后按 `via_heading_trim_length` 裁尾。近段中间裁不掉丢 via；远段裁不了保留；最后一段不丢。
4. **`path_untangle` + 单测**（未落地）
5. **through-poses：近段仍结丢点 + 远段推 G + 丢点比例**（未落地）

每一步都应能单独合入、单独回退。

---

## 12. 测试要点

| 场景 | 期望 |
|---|---|
| 近段直线终点顶墙，前向侵入小于伸缩阈值 | 沿射线缩短 $G$，不上转线、不上 Hybrid |
| 近段直线终点前向侵入大于阈值 | 不硬缩短；有侧向则转线，否则 Hybrid |
| 近段直线擦单侧 254，侵入小于 $\delta$ 且对侧净空 >= 车宽 | 接受原直线，不转线、不上 Hybrid |
| 近段直线单侧侵入大于 $\delta$、净空够 | 不伸缩；转线成功，目标微移小于转线阈值 |
| 近段单侧但净空不够 | 不转线 → Hybrid |
| 远段单侧浅侵入且净空够 | 直线 |
| 远段单侧侵入大于 $\delta$ 即使净空够 | 不转线 → Hybrid |
| 近段两侧夹住 | 不转线 → Hybrid → 裁圈；仍结则丢中间点 |
| 中段撞障 | 不把 $G$ 缩到障碍前冒充成功 |
| 短距大航向差（旧打结日志形态） | 来向 yaw + 裁自交，不把圈拼进 concat |
| 绕矩形障碍 / U 臂 | 不自交，远段保留，不丢点 |
| 折线锯齿 | $L_{min}$ 挡住，不裁 |
| 占用 via、邻域无来向落点 | 中间 continue；最后一点失败 |
| 连续失败 | concat 起点不卡住同一坏位姿 |
| 单点 `ComputePathToPose` | 行为与现网兼容；可选只裁自交、不丢点 |

---

## 13. 与旧思路对照

| 旧 | 现方案 |
|---|---|
| 打结近段直接丢点 | 先裁自交，裁干净则保留该点 |
| 远段打结直接接受含圈路径 | 先裁圈；推开 G 仅作贴障备选 |
| 弧长比 / 「像结」 | 只认真交叉（第一版） |
| 直线失败或 Hybrid 空路径就跳过 | 直线失败 → 先伸缩再转线 → Hybrid；近段航向不合格先裁尾，裁不掉才丢中间 via；远段裁不了则保留；最后一段不丢 |
| 终点碰了就转整条线 | 终点纵向侵入先沿射线伸缩，不成再转 |
| 半宽内任意单侧 254 就转线 | 浅侵入且净空够走原直线；净空不够不转；仅净空够且侵入大于 $\delta$ 才转（仅近段） |
| 锁 via 作业 yaw | 中间点来向 yaw |
| 裁剪做成规划插件 | 库函数；直线留 FastPath |

---

## 14. 验收标准

- 覆盖 through-poses 不再把明显自交的 Hybrid 圈交给局部。
- 近段优先近似直线：终点顶墙先缩短；单侧浅蹭且净空够走原直线；侵入较大且净空够再转开；净空不够不上转线。不因此丢 via。
- 绕岛 / U 形走廊不被当成打结丢掉。
- 最后一个目标在 snap/打结失败时有明确失败，而不是静默少一个点。
- 不引入新的 GlobalPlanner 插件，不把几何后处理写进 Hybrid。

---

## 15. 日志与 RViz 可视化

所有调试话题挂在 `planner_server` 节点下（注意命名空间，常见为 `/planning_debug/...` 或 `/<ns>/planning_debug/...`）。  
参数 `publish_planning_debug:=false` 可关发布；日志仍输出。

### 15.1 日志前缀（便于 grep）

| 前缀 | 位置 | 内容 |
|---|---|---|
| `[ThroughPoses]` | `computePlanThroughPoses` | 总起终点、via 序号、段长、成功/失败、concat 点数（失败时 concat **不变**）；规划结束打 summary：目标点数、总时长、直线/Hybrid 平均耗时、失败目标坐标 |
| `[getPlan]` | `getPlan` | StraightOk / NeedAstar / GoalUnreachable / Hybrid yaw / heading trim / drop via（`hybrid_heading_rejected`） |
| `[Hybrid]` | `SmacPlannerHybrid` | 近段航向门禁 `goal_heading_tol`；失败时 `(XY+heading gate)` |
| `[FastPath]` | `FastPathPlanner` | start/goal/yaw、snap 偏移、走廊碰撞 $s$/$d_y$/侵入/净空、豁免切片数、前进/后退、StraightOk |
| `[PlanDbg]` | `PlanningDebugViz` | beginSession、配置 |
| `[Untangle]` | 裁剪落地后 | 边对 $(i,j)$、交点、loop_len、裁轮数、是否仍自交 |
| `[LineStretch]` / `[LineRotate]` | 伸缩/转线落地后 | iter、alpha、dx、dy、abs(G'-G)、成败原因 |
| `[GoalPush]` | 远段外推落地后 | $P_{obs}$、$G^{\prime}$、是否碰撞 |

通过-poses 每一段至少打一行：

```text
[ThroughPoses] via=i/N start=(x,y,yaw) goal=(x,y,yaw) dist=... concat_poses=...
[ThroughPoses] summary: goals=N concat_poses=M total=X.XXXs
  success=A failed=B straight=C avg=D.DDms hybrid=E avg=F.FFms
  failed goals: via=i (x, y) reason=...; ...
```

直线碰撞在现有实现里已带 `d=.. / L` 与 heading；伸缩落地后补 `dx` `dy`。
单点 `ComputePathToPose` 结束同样打 `[getPlan]` summary（goals=1）。

### 15.2 Path 话题（`nav_msgs/Path`）

RViz 各加一个 Path Display，颜色自定。这些话题**只保留当前段最后一次发布**（Path 不能按 via 累加）。看全程请用 Marker 里的 `path_*` LINE_STRIP。`session` 模式只在下次规划开始时清空；`current` 模式每 via 会发空路径。

| 话题 | 何时发布 | 含义 |
|---|---|---|
| `planning_debug/straight_iter_path` | 每次尝试直线（含伸缩/转线迭代） | 当前候选射线（先两点，算法落地后为整段采样） |
| `planning_debug/straight_path` | 直线最终成功 | 采用的直线（含终点伸缩后） |
| `planning_debug/rotated_path` | 转线成功 | 绕 $S$ 转开后的直线 |
| `planning_debug/hybrid_raw_path` | GridBased/Hybrid 返回后 | **裁剪前** 原始引导线 |
| `planning_debug/hybrid_clipped_path` | untangle 之后 | 去掉自交环后的路径 |
| `plan` | 原有 | 整次 through-poses 拼接结果 |

### 15.3 Marker 话题（`visualization_msgs/MarkerArray`）

单一话题：`planning_debug/markers`。用 **ns** 区分，RViz 一个 MarkerArray Display 即可。  
默认 `planning_debug_keep_mode:=session`：会话内缓存全部 Marker。`publishMarkers` **只写入缓存**；**仅规划结束 `endSession` 整包发出**（先 `DELETEALL` 再带上当前所有 via），一次性显示全部路径。不要每 via 全量 flush。QoS depth=1 仍能 latch 全程。Path 话题仍只显示当前段，看全程请用 Marker。  
`current` 模式仍立即发布，每段清屏，只看正在算的 via。

| ns | 类型 | 颜色（约定） | 含义 |
|---|---|---|---|
| `segment` | ARROW + TEXT | 蓝 / 青 | 本段 $S$、原始 $G$，文字 `via i/N` |
| `corridor` | LINE_STRIP 矩形 + TEXT | 绿=通 / 红=254 | 半宽走廊，不加前后悬 |
| `start` | ARROW | 蓝 | 规划起点 $S$ |
| `goal_original` | ARROW + TEXT `G_orig` | 青 | 作业 via / 未调整目标 |
| `goal_snapped` | ARROW + TEXT `G_snap` | 紫 | 占用环搜后的目标 |
| `goal_adjusted` | ARROW + TEXT `G' stretch\|rotate\|push` | 绿 | 伸缩、转线或远段推开后的 $G^{\prime}$ |
| `collision` | ARROW + footprint + SPHERE + TEXT | 红 / 橙 | 箭头和多边形是发生碰撞时的 **base_footprint**；红球是该位姿下踩到的 254 格 |
| `straight_iter` | TEXT + 起终点 footprint | 黄 | 当前直线迭代轮次（`forward/reverse/stretch/rotate`） |
| `footprint_straight` | LINE_STRIP 多边形 | 橙 | 成功直线上的 footprint（stride 控制密度） |
| `footprint_rotated` | LINE_STRIP | 橙 | 转线后路径上的 footprint |
| `intersect` | SPHERE + TEXT `X` | 品红 | 判定的自交点（裁剪用的 $P$） |
| `via_dropped` | SPHERE + TEXT `drop:...` | 灰 | 近段打结或占用无 snap 丢掉的 via |
| `path_straight_iter` | LINE_STRIP | 黄 | 当前 via 最后一次直线候选（session 下各 via 各一条） |
| `path_straight` | LINE_STRIP | 绿 | 成功直线，按 via 累加 |
| `path_rotated` | LINE_STRIP | 橙 | 转线成功路径，按 via 累加 |
| `path_hybrid_raw` | LINE_STRIP + TEXT `H via=i/N` | **按 via 轮换色相** | 该段 Hybrid 裁剪前路径；through-poses 结束一次画出全部 Hybrid 段 |
| `path_hybrid_clipped` | LINE_STRIP | 蓝 | Hybrid 裁剪后，按 via 累加 |

**转线 / 伸缩落地后必须再发：**

- 每一轮候选：`publishStraightCandidate(S, G_iter, iter, "stretch"|"rotate")` → `straight_iter_path` + 黄 footprint
- 碰撞：`publishCollision(pose, dx, dy, s, L)` → 红球 + 该位姿 footprint
- 调整后目标：`publishAdjustedGoal(G', "stretch"|"rotate")`
- 转线成功：`publishRotatedPath(path)` + `footprint_rotated`

**Hybrid 裁剪落地后必须再发：**

- `publishHybridRaw`（已接）
- `publishIntersections({P...})`
- `publishHybridClipped`
- 远段推开：`publishPushedGoal(G, G')`

### 15.4 建议对照关系（看图排障）

| 现象 | 先看 |
|---|---|
| 直线顶墙 | 红 `corridor` / `collision` 是否在终点附近；青 $G$ 与绿 $G^{\prime}$ 是否沿射线缩短 |
| 转线拧扫线 | 黄 `straight_iter_path` 相对青 $G$ 的侧向偏移是否超阈值；远段不应出现转线 |
| Hybrid 绕圈 | Marker `path_hybrid_raw` 各段异色是否自交；品红 `X` 是否在圈上；`hybrid_clipped_path` 是否变成近道 |
| 后续段同一坏起点 | `[ThroughPoses] concat_unchanged`；失败 via 未写入 concat |
| snap 用错朝向 | `G_orig` 与 `G_snap` 连线是否垂直于来向 |

### 15.5 RViz 添加清单

1. MarkerArray：`planning_debug/markers`（一次规划结束后画面会停住；下次规划才清）。through-poses 的 Hybrid 段在 `path_hybrid_raw` 里按 via 异色，一次画全。
2. Path ×5：上表 `planning_debug/*_path`，线宽 0.04，颜色互相区分  
   - 黄：iter 直线  
   - 绿：成功直线  
   - 橙：转线  
   - 红：Hybrid 原始（Path 话题只留最后一段；看全程用 Marker）  
   - 白/蓝：Hybrid 裁后  
3. 原有 `plan`：最终拼接
4. 全局代价地图：对照 254 / 253

### 15.6 代码落点（已实现 vs 待接）

| API | 状态 |
|---|---|
| `beginSession` / `setSegmentContext` | 已接 through-poses 与单点规划 |
| `publishStart` / `publishOriginalGoal` / `publishSnappedGoal` | 已接 FastPath |
| `publishStraightCandidate` / `publishCollision` / `publishStraightPath` / `publishCorridor` | 已接 FastPath 走廊 / 伸缩 / 转线 |
| `publishHybridRaw` | 已接 `getPlan` 插件返回 |
| `publishRotatedPath` / `publishAdjustedGoal` | 已接转线/缩短成功 |
| `publishIntersections` / `publishHybridClipped` / `publishPushedGoal` / `publishDroppedVia` | API 已写，等裁剪、丢点逻辑调用 |

实现几何算法时 **不要另开一套 publisher**，只调 `PlanningDebugViz`。

