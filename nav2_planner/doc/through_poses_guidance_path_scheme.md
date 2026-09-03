# 覆盖多点导航：弱运动学 Hybrid 引导路径方案

本文固化 through-poses（`ComputePathThroughPoses`）全局规划的讨论结论与落地技术方案。  
**本文档只描述方案，不随本文修改任何代码。**

公式用 `$...$`（行内）和 `$$...$$`（独立行），便于 GitHub / VS Code / Cursor 预览；不使用 `\(...\)`。

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

**直线仍写在 FastPath。** 伸缩、转线、侵入检测与现有 `isFree` / `tryStraightPath` / footprint 扩展共用。文件过长时可同库拆 `fast_path_straight.cpp`，对外仍是一个 `FastPathPlanner`。

**打结裁剪单独源文件，不要做成 `nav2_core::GlobalPlanner`。**  
裁剪是对已有 `nav_msgs/Path` 的几何后处理，需要返回「新路径 + 是否裁过 + 是否仍自交」，`createPlan(start, goal) → Path` 装不下；也不应绑死在 Hybrid 内部。

调用点：Hybrid（或任意 GridBased）出路径之后、写入 `concat_path` 之前。推荐在 through-poses 循环里显式调用；单点 `ComputePathToPose` 的 `getPlan` 末尾也可同样裁一次。

---

## 4. 段分类

每一段：

- 起点 $S$：上一**成功**段终点（`concat_path.back()`）；没有则用机器人位姿。不要用原始 $G_{i-1}$。
- 目标 $G$：当前 via。

**近且朝向接近**（须同时满足，阈值参数化）：

- $|G - S| < d_{\mathrm{near}}$
- $|yaw_G - yaw_S| < \theta_{\mathrm{near}}$
- 作业 yaw 与连线方向 $\mathrm{atan2}(G - S)$ 也接近（作业方向已和扫线差很多时，不要硬保直线）

否则视为 **远段 / 大转向**。

中间 via 的规划 yaw **一律改为来向** $\mathrm{atan2}(G - S)$。  
最后一个目标：可保留作业 yaw（对接需要时）；找不到 snap / 裁空 / 仍结时不得静默 `continue`，应放大容差再试或整次失败。

---

## 5. 端到端流水线

```text
for 每个 via G:
  S = concat 终点（或机器人）
  yaw_plan(G) := 来向 S→G     // 终点可保留作业 yaw

  若 G 占用:
      用来向做 snap；失败 → 中间点 continue（起点不变），终点失败

  若 近且朝向接近:
      FastPath 直线
        终点附近碰撞且 X 向可伸缩 → 沿射线缩短/伸长 G 后重检
        仍撞（侧向为主，或伸缩超阈值/失败）→ 绕 S 转线
      成功 → concat
      否则 Hybrid → untangle
          仍结 / 裁空 → 丢该中间点（终点则重试无 yaw 或失败）
  否则:  # 远或大转向
      Hybrid（yaw 仍是来向）→ untangle
      仍结 → 沿「最近 254 → G」外推约半车宽，碰撞检查后只重规划一次 → 再 untangle
      仍结 → 接受，不丢远段点

  仅成功段 insert concat；continue 时起点保持上一成功终点
```

直线失败（伸缩、转线都救不了）**不是**丢点条件，只进入 Hybrid。  
丢点只留给：占用无 snap，或近段 Hybrid 裁完仍是结。

---

## 6. FastPath：来向 snap + 直线伸缩 + 转线

### 6.1 来向 snap

- `isFree` / 环搜使用 **本段来向**，不用 via 自带作业 yaw。
- 同等圈内优先沿连线方向的偏移，再比欧氏距离。
- 碰撞策略与现有 FastPath 可先保持；弱引导方向上长期应避免用「完整矩形 + 253」把窄缝误判为不可 snap。
- 找不到：中间点丢；终点不丢。

### 6.2 近段直线：先分清撞在哪

将 $G$ 的 yaw 设为来向，试现有前进直线（允许倒车时再试后退直线）。  
碰撞时把命中格变到 `base_footprint`（航向 = 当前直线 heading）：

- $d_x$：沿车体 X（前进为正）。超出前端为正侵入，超出后端为负侵入。
- $d_y$：沿车体 Y。符号表示左侧还是右侧。

并看碰撞沿路径的位置 $s$（起点到该检测点的距离），段长 $L = |G - S|$。

| 碰撞位置 | 主导方向 | 优先手段 |
|---|---|---|
| 靠近终点（$s > L - L_{\mathrm{goal}}$）且 $|d_x|$ 为主 | 纵向 | **沿射线伸缩 $G$** |
| 任意位置，$d_y$ 为主，或两侧都有 $d_y$ | 侧向 | **绕 $S$ 转线** |
| 中段（$s \le L - L_{\mathrm{goal}}$） | 路上有障 | 不伸缩（缩短会跳过障碍，等于丢段）；转线或 Hybrid |
| 伸缩、转线都失败 | — | `NeedAstar`，不丢点 |

$L_{\mathrm{goal}}$ 建议取车长量级（或 0.5～1.0 m）。现有实现里终点 footprint 会单独查一次，终点-only 失败也算「靠近终点」。

**同一轮不要又伸又转。** 先伸缩并整段重检；仍有侧向碰撞再转。禁止正负伸缩来回拧。

### 6.3 终点附近伸缩（合理，且应先于转线）

覆盖扫线顶到墙/膨胀时，via 经常略伸进障碍。**沿原扫线把 $G$ 收回一点**，比把整条线拧歪更符合覆盖：同伦不变，只少扫末端一小段。

**是否可伸缩**（须同时满足）：

1. 中段无碰撞；碰撞只出现在终点邻域（或仅终点 footprint 失败）。
2. 纵向为主：例如 $|d_x| \ge |d_y|$，或 $|d_y|$ 小于半个膨胀格。
3. $|d_x| \le d_{\mathrm{stretch}}$（与原 via 的沿轨位移上限，建议 0.3～0.5 m）。超过则伸缩会把目标挪太远，改走转线或 Hybrid。

**怎么动**（沿单位方向 $\hat{u} = (G-S)/|G-S|$，与倒车 heading 解耦，用路径坐标系）：

$$
G' =
\begin{cases}
G - |d_x|\,\hat{u} & d_x > 0 \quad \text{前端侵入：缩短（G 向 S 收）} \\
G + |d_x|\,\hat{u} & d_x < 0 \quad \text{后端侵入：伸长（G 沿射线往外）}
\end{cases}
$$

约束：

- $G'$ 必须在 $S$ 前方，新段长大于约 2 个碰撞步长。
- $|G' - G_{\mathrm{original}}| \le d_{\mathrm{stretch}}$（相对**原始** via，不是相对上一轮 $G$）。
- 缩短后 $G'$ 不要相对作业前进方向倒退过多（覆盖倒退）；伸长不要越过墙。
- **伸长要整段重检**（含多出来的那一截）。前端撞墙时伸长会更糟，只允许 $d_x>0$ 缩短为主；伸长仅在明确是后端侵入、且中段确实无碰时启用。实践上可先只做缩短，伸长用参数关掉。
- 最多 1～2 次伸缩，失败则不再伸，转入转线。

伸缩解决不了（$|d_x|$ 超阈值、重检仍撞、或其实是侧向/中段）→ **仍然旋转**，不是直接 Hybrid。转线失败才 `NeedAstar`。

### 6.4 绕起点转线

在伸缩之后（或一开始就是侧向碰撞、不具备伸缩条件）执行：

- 只 +Y 或只 −Y：向远离障碍一侧转，$\alpha$ 取各点所需转角的最大（$\alpha \approx d_y / s$，$s$ 为该点到 $S$ 的距离）。
- **两侧都撞**：放弃直线，`NeedAstar`。
- 新目标 $G'$ = 新射线上离原 $G$ 最近、且在 $S$ 前方的点。
- $|G' - G| $ 超 `line_rotate_goal_shift_tol` → 放弃直线。
- $G'$ 不要相对原目标往后缩太多。
- 转完后 **整段重新碰撞检测**。最大迭代（建议 5）；$\alpha$ **只往同一侧累加**，禁止正负来回。
- 转开后若仅终点又前向侵入，允许再做 **一次** 缩短（转后的新射线上），不再进入新一轮大角度转。

转开失败 → `NeedAstar`，把（可能已伸缩/微转过的）目标交给 Hybrid，**不丢点**。

转线与伸缩都只在「近且朝向接近」时启用；远段 `allow_straight=false` 或跳过这两步。

### 6.5 近段直线调整顺序

```text
试直线 S → G
  通 → 采用
  撞在中段 → 转线（不伸缩）；转失败 → Hybrid
  撞在终点邻域:
      |dx| ≤ 阈值 且 纵向为主 → 伸缩 G 沿射线，整段重检
          通 → 采用
          仍撞 → 转线
      纵向超阈值或侧向为主 → 转线
  转线成功 → 采用（必要时再缩短一次终点）
  转线失败 → Hybrid，不丢点
```

---

## 7. 自交检测与裁剪（`path_untangle`）

只处理 **本段** `curr_path`，不要对整条 `concat_path` 做自交（覆盖折返会误报）。  
直线路径一般不裁。第一版只做 **真交叉 + 环长下限**。

### 7.1 遍历

点列 $P_0,\ldots,P_{n-1}$，边 $e_i = \overline{P_i P_{i+1}}$。  
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

$L_{\min}$ 建议 $\pi R$ 或 1～2 m，滤掉折线锯齿。

### 7.2 真交叉（参数法，同时拿交点）

$A = P_i$，$B = P_{i+1}$，$C = P_j$，$D = P_{j+1}$：

$$
A + t(B - A) = C + u(D - C)
$$

- $|\mathrm{den}|$ 过小：平行/共线，不当真交叉。
- 真交叉当且仅当 $\varepsilon < t < 1 - \varepsilon$ 且 $\varepsilon < u < 1 - \varepsilon$（不含端点）。
- 交点 $P = A + t(B - A)$。

不要用弧长/欧氏距离比，不要用「同一格子走两次」当主判据，不要把路径首尾连成闭合多边形。

### 7.3 裁剪

每一趟只裁 **loop_len 最短且 ≥ $L_{\min}$** 的一对：

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

- 同向近切（棒棒糖相切环）：线上若仍多见再补「距离 < 1 格 且 航向同向 且 环长 ∈ $[L_{\min},\, L_{\max}]$」。U 形近平行反向 **禁止** 当自交。
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
| `include/nav2_planner/fast_path_planner.hpp` + `src/fast_path_planner.cpp` | 来向 snap；近段终点伸缩 + 转线；`compute` 可由 server 只在近段 `allow_straight=true` |
| 可选 `src/fast_path_straight.cpp` | 仅当直线+伸缩+转线把单文件撑太大，同库拆分 |
| **新建** `include/nav2_planner/path_untangle.hpp` | `UntangleResult` 与函数声明 |
| **新建** `src/path_untangle.cpp` | 真交叉、裁环、循环 |
| `src/planner_server.cpp` | through-poses：来向 yaw、近/远、直线失败不丢点、调 untangle、近段丢点、远段推 G、终点保护、失败 continue 起点不变 |
| `CMakeLists.txt` | 把 `path_untangle.cpp` 编进 `planner_server_core` |
| 参数 | 见下一节；动态参数按现有 `planner_server` 模式挂 |

**不要** 新增 `pluginlib` xml / `GlobalPlanner` 子类。  
**不要** 把裁剪写进 `smac_planner_hybrid.cpp`。  
**不要** 把转线/伸缩写成第二个规划插件。

### 9.1 `getPlan` 与 through-poses 的分工建议

- `getPlan(start, goal, planner_id)`：仍是 FastPath → 插件。单点规划可在插件返回后做一次 untangle。
- 近/远、丢点、推 G、改 yaw：**只放** `computePlanThroughPoses`。这些是覆盖多点策略，不是单点规划语义。
- FastPath 的 `GoalUnreachable`：through-poses 对中间点 `continue`，对最后一点失败。

现有起点占用 `pop_back` 恢复可保留，与「丢 via」是两条恢复路径，不要混成一次 `pop_back` 既丢路径点又丢作业点。

---

## 10. 建议参数（均在 `planner_server` 命名空间）

名称可在实现时微调，语义应保持稳定。

| 参数 | 建议默认 | 含义 |
|---|---|---|
| `near_distance_threshold` | 2.0 m | 近段距离阈值 $d_{\mathrm{near}}$ |
| `near_yaw_threshold` | 0.35 rad（约 20°） | 近段航向差 |
| `enable_line_rotate` | true | 近段直线转线 |
| `line_rotate_max_iters` | 5 | 转线最大圈数 |
| `line_rotate_goal_shift_tol` | 0.5 m | 转线后 $|G'-G|$ 上限 |
| `enable_line_stretch` | true | 近段终点沿射线伸缩 |
| `line_stretch_max` | 0.4 m | $|d_x|$ 与沿轨位移上限 $d_{\mathrm{stretch}}$ |
| `line_stretch_goal_window` | 0.8 m | 终点邻域 $L_{\mathrm{goal}}$ |
| `line_stretch_allow_extend` | false | 是否允许后端侵入时伸长；建议默认只缩短 |
| `untangle_enable` | true | 自交裁剪开关 |
| `untangle_min_loop_length` | 1.0 m | $L_{\min}$，或 $\pi R$ |
| `untangle_max_rounds` | 8 | 裁环最大轮数 |
| `goal_push_enable` | true | 远段贴障外推 |
| `goal_push_distance` | 半车宽 | 沿离开最近 254 的偏移 |
| `drop_via_max_ratio` | 0.3 | 中间点丢弃比例上限 |
| `rewrite_via_yaw_to_approach` | true | 中间点 yaw 改来向 |

$R$ 沿用 Hybrid 的 `minimum_turning_radius`（若 server 读不到，用 `untangle_min_loop_length` 单独配）。

---

## 11. 实现顺序（建议分 PR / 分步，仍不在本文改代码）

1. **`path_untangle` + 单测**  
   构造自交折线、U 形不交、邻边假交、短环；断言裁点与剩余路径。
2. **through-poses：来向 yaw + 调 untangle + 近段仍结丢点 + 终点不丢**  
   先不转线、不伸缩、不推 G，也能消掉大部分进场圈。
3. **FastPath：snap 用来向；近段先终点伸缩、再转线**  
   保住扫线，减少无谓 Hybrid。
4. **远段推开 G 一次**  
   终点贴障进场圈的备选。
5. **日志与丢点比例**  
   便于线上对照覆盖稀疏程度。

每一步都应能单独合入、单独回退。

---

## 12. 测试要点

| 场景 | 期望 |
|---|---|
| 近段直线终点顶墙，前向侵入小于伸缩阈值 | 沿射线缩短 $G$，不上转线、不上 Hybrid |
| 近段直线终点前向侵入大于阈值 | 不硬缩短；有侧向则转线，否则 Hybrid |
| 近段直线擦单侧膨胀（中段或全程侧向） | 不伸缩；转线成功，目标微移小于转线阈值 |
| 近段两侧夹住 | 转线放弃 → Hybrid → 裁圈；仍结则丢中间点 |
| 中段撞障 | 不把 $G$ 缩到障碍前冒充成功 |
| 短距大航向差（旧打结日志形态） | 来向 yaw + 裁自交，不把圈拼进 concat |
| 绕矩形障碍 / U 臂 | 不自交，远段保留，不丢点 |
| 折线锯齿 | $L_{\min}$ 挡住，不裁 |
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
| 直线失败或 Hybrid 空路径就跳过 | 直线失败 → 先伸缩再转线 → Hybrid；只有近段裁不掉才丢中间点 |
| 终点碰了就转整条线 | 终点纵向侵入先沿射线伸缩，不成再转 |
| 锁 via 作业 yaw | 中间点来向 yaw |
| 裁剪做成规划插件 | 库函数；直线留 FastPath |

---

## 14. 验收标准

- 覆盖 through-poses 不再把明显自交的 Hybrid 圈交给局部。
- 近段优先近似直线：终点顶墙先缩短，单侧擦障再转开，不因此丢 via。
- 绕岛 / U 形走廊不被当成打结丢掉。
- 最后一个目标在 snap/打结失败时有明确失败，而不是静默少一个点。
- 不引入新的 GlobalPlanner 插件，不把几何后处理写进 Hybrid。
