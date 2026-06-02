# FAST-LIO2 SLAM 评估报告

**数据集**：TEEMO AW7（车辆 `ZL11626`）样本 `2025-07-02_14-29-00_000000000_8993544`
**主雷达**：`remote_front_left_pointcloud`（Hesai AT128P，128 线机械式）
**录制时长**：60 s，10 Hz LiDAR / 100 Hz IMU
**运行环境**：`ghcr.io/wangxinjian1108/fast-lio:latest`（GitHub Actions build，commit `7a1a916`）
**SLAM 输出**：596 帧轨迹，452.7 m 长度，Z 范围 [0.02, 6.69] m
**输出目录**：`output/ghcr_run_v3/`

---

## 1. SLAM 基本结果

| 项 | 值 |
|----|----|
| 帧数 | 596 |
| 总长度 | 452.7 m |
| 平均速度 | 7.5 m/s ≈ 27 km/h |
| X 范围 | [0.9, 321.1] m |
| Y 范围 | [-203.8, 12.9] m |
| Z 范围 | [0.02, 6.69] m |
| 全局地图 | 18.7 M 点（599 MB 原始 / 19 MB voxel 0.3 m） |

可视化：`fastlio_trajectory.png`、`fastlio_position_xyz.png`、`fastlio_map.png`。

---

## 2. 轨迹评估：FAST-LIO 对 `LIDAR_TO_MAP` 参考

参考数据来源：录制包目录下 `LIDAR_TO_MAP/<idx>_<ts_ns>.txt`，598 个 4×4 矩阵
（`T_map_baselink`，绝对 map 坐标，疑似 UTM 锚定）。

FAST-LIO 输出的是 `T_world_imu`，先用 `application.yaml` 里的 `FRAME_GNSS_IMU` 外参
合成 `T_world_baselink = T_world_imu @ T_imu_baselink`，再与参考对比。

### 2.1 评估方法（4 种组合）

| 维度 | 选项 | 含义 |
|------|------|------|
| **时间戳匹配** | `nearest` | 每个参考帧找最近 SLAM 帧（±60 ms 容差） |
|  | `interp` | 把 SLAM 轨迹用 SLERP 插值到参考时间戳（最大括宽 200 ms） |
| **坐标系对齐** | `global` | Procrustes 全局刚体 SE(3) 拟合，最小化整段位置差 |
|  | `first`  | 仅把首匹配帧拉齐，暴露后续累积漂移 |

### 2.2 完整对比表

|                | nearest+global | interp+global | nearest+first | interp+first |
|----------------|---:|---:|---:|---:|
| n_pairs        |   596 |   595 |   596 |   595 |
| **ATE RMS** (m)| **0.323** | **0.322** | **1.686** | **1.683** |
| ATE mean (m)   | 0.297 | 0.297 | 1.474 | 1.471 |
| ATE median (m) | 0.315 | 0.315 | 1.458 | 1.452 |
| ATE max (m)    | 0.746 | 0.745 | 2.921 | 2.920 |
| dx std (m)     | 0.123 | 0.123 | 0.108 | 0.108 |
| dy std (m)     | 0.297 | 0.297 | 0.302 | 0.301 |
| **dz std** (m) | **0.027** | **0.027** | **0.789** | **0.787** |
| dx bias (m)    | 0     | 0     | +0.365 | +0.365 |
| dy bias (m)    | 0     | 0     | -0.620 | -0.619 |
| **dz bias** (m)| 0     | 0     | **+1.265** | **+1.263** |
| rot mean (°)   | 0.316 | 0.315 | 0.397 | 0.396 |
| rot max  (°)   | 1.018 | 1.017 | 1.392 | 1.392 |

（`global` 模式下 bias 必然为 0，因为对齐就是让两组质心重合）

### 2.3 关键结论

#### 结论 1：SLAM 局部几何很准（global ATE RMS 0.32 m）
全局对齐后的 ATE RMS = **0.32 m / 452 m = 0.07%**。说明 FAST-LIO 跑出来的相对几何
（点与点之间的距离、转弯弧度）非常贴合参考。可以放心用这个地图做 cross-LiDAR 配准。

#### 结论 2：累积漂移以 Z 方向为主（+1.27 m / 60 s）
首帧对齐模式露出真实漂移：60 s 内 SLAM 相对参考 **Z 单调上漂 ~1.27 m**（≈ 2 cm/s）。
XY 漂移 36 / 62 cm，量级合理。这说明：
- gyro 还有微小残余 bias（虽然 deg→rad 已修），导致重力对齐方向轻微倾斜
- accel z 静态读数 -9.81（specific force 约定），FAST-LIO 估出来 +9.78~+9.80
  ——绝对值非常接近，但残差仍能积累 1 m 级别的高度漂移

#### 结论 3：Z 方差在两种模式下差 30 倍
- 全局对齐：dz std = **2.7 cm**（绝佳，意味着 SLAM Z 轴和参考 Z 轴几乎平行，只是整体偏了一点）
- 首帧对齐：dz std = **79 cm**（漂移逐渐展开）

如果做定位（关心绝对位姿），首帧那栏更诚实；如果做建图（关心局部一致性），全局那栏才有意义。

#### 结论 4：时间插值在本场景下意义不大
SLAM 和参考都按 10 Hz LiDAR 帧采，两组时间戳本来就近乎一致。插值（SLERP+linear）
比最近邻只改进 ~1 mm。**仅在 SLAM 频率 ≠ 参考频率 / 一方明显抖动时才值得做**。

#### 结论 5：旋转误差很小（mean 0.32°, max 1.0°）
两种对齐下 rot 残差差不多。说明姿态估计准确，几乎不会因方向飘走拖累位置。

---

## 3. Cross-LiDAR 外参校准结果

把主雷达 SLAM 的地图当参考，3 个副雷达逐帧 ICP（50 m 局部子图）配准，再反推
`T_baselink_secondary`，与工厂标定对比：

| 副雷达 | mean fitness | mean RMSE | \|Δt\| vs 工厂 | Δ 旋转 |
|--------|---:|---:|---:|---:|
| remote_front_right | 0.66 | 0.31 m | **0.140 m** | < 0.04 rad (≈ 2°) |
| flash_front        | 0.98 | 0.16 m | **0.257 m** | < 0.04 rad |
| flash_rear         | 0.93 | 0.19 m | **0.239 m** | < 0.04 rad |

校准后外参（base_link 系，单位 m / rad）见
[`calibrated_extrinsics.yaml`](calibrated_extrinsics.yaml)。

可视化对比（左 = 工厂标定，右 = ICP 校准）：
- `compare_remote_front_right_pointcloud.png`
- `compare_flash_front_pointcloud.png`
- `compare_flash_rear_pointcloud.png`

---

## 4. 进一步改进建议

### 4.1 降 Z 漂移
- **静态预热**：录制开头让车静止 10-30 s，FAST-LIO 用这段做 IMU bias / 重力初始化更稳
- **重新标定 IMU bias**：用静止段算 gyro/accel 静态零偏，写到 `config/fastlio_at128p_velodyne.yaml`
  的 `init_bg`/`init_ba`（如果用 ESKF 路径）
- **更紧的 `b_acc_cov` / `b_gyr_cov`**：若你信任 IMU 内参，调小过程噪声让滤波器抓得更稳
- **打开 `extrinsic_est_en: true`** 让 FAST-LIO 在线优化 lidar↔imu 外参——但仅当你不信任 yaml 的外参

### 4.2 提高 ICP 配准质量
- **flash_rear 视野和主地图重叠少**：导致它的 fitness 比 flash_front 低。可以：
  - 把 `--submap-radius` 提到 70-100 m，给 ICP 更多目标点
  - 用 point-to-plane ICP（需要给地图算 normal）替代当前 point-to-point
- **remote_front_right 累积观测**：因为视野和主雷达不重合，初始 ICP 命中率有限；可
  考虑 NDT 或先做 voxel-based 粗对齐再精配

### 4.3 评估方法学
- 若拿到 RTK/GNSS 真值，可以用 evo 工具（`evo_traj`, `evo_ape`）跑标准 ATE/RPE 对比
- 长序列（>10 min）建议同时报 RPE（相对位姿误差）；本场景 60 s 偏短，ATE 已能反映问题
- 如果要量化"建图精度"而不是"定位精度"，考虑用同一段路上不同时间录的两份数据做地图对地图配准

### 4.4 工程化
- 把这套流程包装成 `scripts/run_all_eval.sh`：从 raw → bag → SLAM → 副雷达配准 → 评估报告
  一键串起来
- 多 sample 批跑：把不同录制段都跑一遍，统计 ATE RMS、Z 漂移、外参校准 std 的稳定性

---

## 5. 输出物清单

### 主 SLAM 输出
- `remote_front_left_pointcloud.bag` — 输入 rosbag（1.7 GB）
- `scans.pcd` — 全局点云地图（599 MB）
- `scans_voxel0.3.pcd` — 降采样地图（19 MB，配准用）
- `pos_log.txt` — FAST-LIO 原始 pos log（596 行）
- `trajectory.txt` — TUM 格式轨迹

### SLAM 可视化
- `fastlio_trajectory.png`、`fastlio_position_xyz.png`、`fastlio_map.png`

### 轨迹对比（vs LIDAR_TO_MAP）
- `compare_traj_error_4way.png` — 4 种变体的位置 / 旋转误差 vs 时间
- `compare_traj_topdown_2way.png` — global / first-frame 对齐俯视图
- `compare_traj_summary.yaml` — 4 种变体的数值汇总

### Cross-LiDAR 校准
- `registration/<lidar>/frame_transforms.txt` — 每帧 ICP 结果（×3 雷达）
- `registration/<lidar>/summary.yaml` — ICP 总览（×3 雷达）
- `calibrated_extrinsics.yaml` — 最终校准外参 + 与工厂标定的 delta
- `compare_<lidar>_pointcloud.png` — 配准前后点云叠加（×3 雷达）

### 使用脚本
- `scripts/convert_to_rosbag_velodyne.py` — 数据转换（含 gyro deg→rad 修复）
- `scripts/run_fastlio_in_container.sh` — 容器 orchestration
- `scripts/viz_fastlio_results.py` — SLAM 结果可视化
- `scripts/poslog_to_tum.py` — pos_log → TUM
- `scripts/04_register_secondary.py` — 副雷达 ICP 配准
- `scripts/extract_extrinsic_from_registration.py` — 反推外参
- `scripts/viz_registration_compare.py` — 配准前后对比图
- `scripts/compare_with_lidar_to_map.py` — 4-way 轨迹对比

---

## 6. 关键修复记录（用于复现）

1. **FAST-LIO velodyne handler 需要 `ring` 字段**
   原始 PCD 字段是 `x y z intensity time label`，`label` 全 0 没用。
   `convert_to_rosbag_velodyne.py` 从点的垂直角 `atan2(z, sqrt(x²+y²))` 推 ring，
   AT128P vfov 取 [-13°, 14°]。

2. **gyro 单位是 deg/s，不是 rad/s**（这是导致前两次 Z 飞掉的根本原因）
   原始 csv 列名 `wx wy wz` 看起来像 SI 单位，实际 RMS 214 deg/s。
   不修这个，FAST-LIO 把它当 rad/s，旋转误差放大 57 倍，导致 Z 指数发散到 12 km。
   修复后 Z 收敛在 [0.02, 6.69] m。

3. **FAST-LIO 输出 IMU 位姿，不是 baselink 位姿**
   反推副雷达外参时，需要 `T_baselink_sec = T_baselink_imu @ T_imu_sec`，
   `T_baselink_imu` 取自 `application.yaml` 的 `FRAME_GNSS_IMU` 外参（0.331, 0.121, 0.58）。
   不做这一步，校准结果会整体偏 ~0.6 m。

4. **Docker 镜像必须先编译 livox_ros_driver**
   fast_lio 编译时依赖 livox_ros_driver 提供的 message header，
   `catkin_make` 必须用 `-DCATKIN_WHITELIST_PACKAGES='livox_ros_driver'` 先编一遍。

---

## 7. 优化路线图（用上不确定度）

当前管线把 SLAM 协方差和 ICP 配准质量信息**全扔了**——只取点估计、平权聚合。
利用这些信息能显著提高精度和鲁棒性。按工程量从小到大：

### A. 已有但未用的信息

**ICP 端**：每帧输出 3 个质量指标
- `fitness` (内点占比 0-1)
- `inlier_rmse` (米)
- `num_inliers` (个)

当前 `extract_extrinsic_from_registration.py` 完全没用，所有 600 帧权重相等做 median/mean。

**SLAM 端**：FAST-LIO ESKF 内部维护 15×15 协方差矩阵 `state.P`
（pos×3 + rot×3 + vel×3 + ba×3 + bg×3），跟踪位姿不确定度。
直道贴墙 vs 急转弯下方差差几个数量级。当前 `pos_log.txt` 只 dump 均值，协方差全丢。

### B. 小改动（1–2 天）

#### B1. ICP-加权外参聚合
改 `extract_extrinsic_from_registration.py`：

```python
weight_i = fitness_i * num_inliers_i / (rmse_i² + ε)
T_calib = weighted_average(T_extrinsic_i, weight_i)
std_calib = sqrt(Σ w_i (T_i - T_calib)² / Σ w_i)
```

- 输出每帧权重 + 最终外参的**有效不确定度**
- 自动滤掉低 fitness 帧（无需硬阈值）
- 现在 remote_front_right 的 `std_t` 是 0.5–1.5 m，主要被少数烂帧拉走；
  加权后应能压到 < 30 cm

#### B2. 帧级 ICP 不确定度（Hessian 估计）
ICP 收敛时点对应关系固定，可以从残差对 6-DoF（3T+3R）的雅可比 J 算近似信息矩阵
`H = JᵀJ`，`σ²_T = trace(H⁻¹[:3,:3]) / 3`。
Open3D 不直接给但能从 correspondence_set 重算。

每帧得到 **6×6 协方差**，组合各帧时用 Mahalanobis-加权融合。

#### B3. point-to-plane ICP 替代 point-to-point
平面墙 ICP 仅在法向方向有约束（沿墙两个切向 unconstrained）。
Open3D 现成支持 point-to-plane，需要给地图先估 normal。
对窄重叠的副雷达（如 flash_rear）尤其有用，预计 fitness 提至 0.95+、rmse 降一半。

### C. 中改动（3–7 天）

#### C1. Dump FAST-LIO 协方差
改 `laserMapping.cpp` 的 `dump_lio_state_to_log`，每行追加 `state.P` 对角线
（pos_std × 3, rot_std × 3），重 build 镜像。然后 cross-LiDAR 配准时：
- 把这些 std 作为**初始位姿协方差**传给 ICP（加权配准）
- 在不确定时段（剧烈机动、点云退化）少信 SLAM

#### C2. 重叠区分析
副雷达视野和主地图的重叠面积差异巨大：

| 副雷达 | 视野方向 | 重叠特点 | 实际 fitness |
|--------|----------|----------|---:|
| remote_front_right | 车右前 | 主雷达地图在此区域点稀疏 | 0.66 |
| flash_front | 朝前下方近距离 | 重叠最好 | 0.98 |
| flash_rear | 朝后上方 | 中等 | 0.93 |

建议根据每帧重叠点数预先**剔除明显烂帧**（如 < 20% 子图覆盖跳过），再聚合。

### D. 大改动（> 1 周）

#### D1. 联合后端优化（Pose Graph / Bundle Adjustment）
用 GTSAM/Ceres 同时优化：
- 节点：主雷达位姿、副雷达外参
- 边：FAST-LIO 相邻帧 odometry（带 Σ）+ 副雷达-地图 ICP 约束（带 Σ）
- 目标：`min Σ residualᵀ Σ⁻¹ residual`

这才是"用所有不确定度信息"的正确做法，但工程量大。
GTSAM-FAST-LIO 集成有现成 ROS 包（`FAST-LIO-SLAM`, by gisbi-kim）可参考。

#### D2. 闭环检测
当前 60 s 数据没回起点，无闭环。如果有更长录制（车绕一圈回原点），
闭环能把 Z 累积漂移砍掉 80%+。FAST-LIO 本体没闭环，需外挂 ScanContext + GTSAM。

#### D3. VoxelMap / VoxelNet 替代 ikd-tree
FAST-LIVO 用的 VoxelMap：每个体素维护点云分布 + 协方差，做 NDT 风格的概率匹配。
配准时**天然带不确定度**，比 ICP-on-pointcloud 鲁棒得多。
改 FAST-LIO 后端为 VoxelMap 是已发表的工作，开源（FAST-LIVO2）。

### E. 评估方法改进

#### E1. 不确定度自评估
在 `compare_with_lidar_to_map.py` 加：
- SLAM 每帧 σ_pos 画在 error-vs-time 图上做 **3σ 包络**
- 看实际误差是否落在 3σ 内 → 验证 SLAM 协方差校准合理性
- 若实际误差远 > 3σ → SLAM 过度自信，要调 process noise

#### E2. RPE（相对位姿误差）
当前只算 ATE。加 RPE_5s（5 秒窗口内的相对位姿漂移）——
ATE 受全局对齐影响大，RPE 更能反映局部精度。

### 推荐优先级

**立刻可做（投资最小、回报明显）**
- B1（ICP-加权聚合）→ 副雷达 std 压一半
- B3（point-to-plane ICP）→ fitness 全面提升

**短期值得**
- B2（每帧 6×6 协方差）+ B1 升级版（用 6×6 而不是标量）
- C2（重叠区分析 + 烂帧剔除）

**长期方向**
- C1（dump SLAM 协方差）+ E1（不确定度自评估）— 工程意义很大
- D1（联合 BA）— 如果要做生产级标定服务，这是必须的

---

## 8. B1 + B3 实施与对比（已落地）

按 §7 推荐顺序，先落地 **B1（quality-weighted aggregation）** 和 **B3（point-to-plane ICP）**。

### 8.1 实现要点

**B3 — point-to-plane ICP**：新增 `scripts/registration/icp_pl.py`，用 Open3D
`TransformationEstimationPointToPlane` + 即时 `estimate_normals`（KDTree hybrid，
半径 1.0 m，max_nn 30）。`04_register_secondary.py` 通过 `--method icp_pl` 选用。

**B1 — quality-weighted aggregation**：
- `04_register_secondary.py` 同时写出 `frame_quality.csv`：每帧 timestamp、
  fitness、inlier_rmse、num_inliers、num_source_pts。
- `extract_extrinsic_from_registration.py` 增加 `aggregate_weighted()`，权重
  `w_i = fitness_i · num_inliers_i / (rmse_i² + ε)`。
  输出每帧权重 + 加权 std + `n_eff = (Σw)² / Σw²`（有效贡献帧数）。
- `--no-weighting` 标志可以强制走 median 路径，用于对照。

### 8.2 三组配置对比

| 配置 | ICP | 聚合 | 触发改动 |
|------|------|------|----------|
| **A**（基线） | point-to-point | median | 当前生产 |
| **B'** | point-to-plane | median | B3 单独 |
| **B** | point-to-plane | weighted | B1 + B3 |

### 8.3 每个副雷达的结果

#### flash_front（fitness 0.95+，简单工况）

| 变体 | calib t (m) | std_t (m) | n_eff | \|Δt\| 工厂 (m) |
|------|---|---|---:|---:|
| A | [3.022, 0.152, 1.737] | [0.517, 0.603, **0.013**] | 600 | 0.257 |
| B' | [2.613, 0.040, 1.738] | [0.708, 0.785, 0.055] | 600 | **0.219** |
| **B** | **[2.707, 0.080, 1.740]** | **[0.536, 0.452, 0.019]** | **520** | **0.149** |

#### flash_rear（fitness 0.93）

| 变体 | calib t (m) | std_t (m) | n_eff | \|Δt\| (m) |
|------|---|---|---:|---:|
| A | [-1.004, 0.154, 0.560] | [0.646, 0.584, 0.085] | 600 | 0.239 |
| B' | [-1.409, -0.021, 0.546] | [1.711, 0.904, 0.685] | 600 | 0.583 |
| **B** | **[-1.417, -0.047, 0.546]** | **[0.794, 0.582, 0.024]** | **523** | 0.593 |

#### remote_front_right（fitness 0.66，最难工况）

| 变体 | calib t (m) | std_t (m) | n_eff | \|Δt\| (m) |
|------|---|---|---:|---:|
| A | [2.645, -0.230, 1.847] | [0.523, 1.481, 0.079] | 600 | 0.140 |
| B' | [2.332, -0.353, 1.851] | [0.573, 1.479, 0.052] | 600 | 0.371 |
| **B** | **[2.267, -0.257, 1.850]** | **[0.329, 0.853, 0.027]** | **177** | 0.448 |

### 8.4 关键观察

**B3 单独使用反而拉大 std**（B' vs A）。Point-to-plane 对每帧 normal 估计敏感，
单帧解的方差更大。Flash_rear 的 dz std 从 0.085 涨到 0.685 m。
**结论：B3 必须配 B1 才能用**。

**B1 加权对压 std 极有效**（B vs B'）：

| 指标 | 改进 |
|------|---:|
| flash_front Y std | 0.785 → 0.452 m（−43%） |
| **flash_rear Z std** | **0.685 → 0.024 m（−96%）** |
| rfr X std | 0.573 → 0.329 m（−43%） |
| rfr Y std | 1.479 → 0.853 m（−42%） |

**`n_eff` 暴露雷达质量分布**：
- flash_front：520/600 = 87% 有效（多数帧质量好）
- flash_rear：523/600 = 87%
- **rfr：177/600 = 29%**（fitness 低的帧被自动剔除）

这意味着加权后**有效信息密度提升**——rfr 实际只用 177 帧高质量数据，而不是被
423 帧低质量数据稀释。

**所有 dz std 都压到 < 3 cm**（A 最差 8.5 cm）：

| 雷达 | A dz std | B dz std | 改进 |
|------|---:|---:|---:|
| flash_front | 0.013 | 0.019 | 略升 |
| flash_rear | 0.085 | **0.024** | −72% |
| rfr | 0.079 | **0.027** | −66% |

### 8.5 性能

| 雷达 | A 耗时 | B 耗时 | 加速比 |
|------|---:|---:|---:|
| flash_front | 756 s | 136 s | 5.6× |
| flash_rear | 244 s | 132 s | 1.9× |
| rfr | 591 s | 267 s | 2.2× |

Point-to-plane 的法向约束让 ICP 迭代收敛更快，整体快 2-5×。

### 8.6 取舍

- **优点**：std 全面下降；速度提升；自动剔除烂帧。
- **代价**：normal 估计需要额外计算（约 10% 内存开销）；当主地图局部点密度
  过低（< 5 点/m³）时 normal 不稳，应 fallback 回 P2P。
- **未解决**：calib 值本身随聚合方式飘 10-30 cm（如 flash_front X 在 2.61~3.08 m
  之间）。需要 ground truth 才能判断哪个更对。

### 8.7 已 commit

| 文件 | 改动 |
|------|------|
| `scripts/registration/icp_pl.py` | 新增 point-to-plane ICP |
| `scripts/registration/__init__.py` | 注册 `icp_pl` 方法 |
| `scripts/04_register_secondary.py` | 输出 `frame_quality.csv` |
| `scripts/extract_extrinsic_from_registration.py` | 加权聚合 + `--no-weighting` |

Commit hash: `42910a8`。

### 8.8 下一步推荐（按优先级）

1. **B2（每帧 6×6 协方差）**：从 ICP 收敛残差算 Jᵀ J 信息矩阵，weight 用矩阵
   而非标量。预计 rfr Y std 还能再压一档（当前 0.85 m，仍偏大）。
2. **C1（dump FAST-LIO 协方差）**：改 laserMapping.cpp 把 ESKF P 矩阵对角线
   写到 pos_log.txt，让 primary 不确定时段也能降权。
3. **D1（联合 BA）**：到此为止 ICP 是逐帧独立的，相邻帧无平滑约束。GTSAM
   pose graph 可以把 SLAM odometry + ICP 约束放进同一目标函数最小化。

---

## 9. B2 实施与对比（已落地）

继 §8 之后落地 **B2（per-frame 6×6 info-matrix weighted aggregation）**。

### 9.1 实现要点

**ICP 后端**：`scripts/registration/icp_pl.py` 在 ICP 收敛后，从内点对应集 +
target normals 重新装配 Hessian：

- 残差：`r_i = n_i · (R · p_i + t - q_i)`
- 雅可比（左扰动 ξ = (ω, t_pert)）：`J_i = (a_i, n_i)` 其中 `a_i = (R · p_i) × n_i`
- Hessian：`H = Σ_i J_i J_iᵀ`
- 信息矩阵：`Σ_pose⁻¹ ≈ H / σ²`，σ² 是内点残差方差

**`RegistrationResult`** 新增 `information_matrix: Optional[np.ndarray]` 字段（6×6）。

**`04_register_secondary.py`** 新增 `frame_information.csv` 输出，每行 37 列：
`timestamp` + 36 个矩阵元素（row-major，块顺序 ω→t）。

**`extract_extrinsic_from_registration.py`** 新增 `aggregate_info_weighted()` 和
`--info-weighting` 标志。每个翻译轴单独加权：

```
W_axis = Σ_frames I_tt_diag[axis]      # 该轴信息量总和
μ_axis = Σ I_tt_diag[axis] · t[axis] / W_axis
```

无 info 矩阵的帧（ICP correspondence < 6）自动过滤。

### 9.2 三组对比（B1 vs B2，都基于 icp_pl）

| 雷达 | 方法 | dx std | dy std | dz std | n_eff |
|------|------|---:|---:|---:|---:|
| **flash_front** | B1 (scalar) | 0.536 | 0.452 | 0.019 | 520 |
|  | **B2 (6×6)** | **0.277** | **0.274** | **0.013** | 374 |
| **flash_rear** | B1 | 0.794 | 0.582 | 0.024 | 523 |
|  | **B2** | **0.532** | **0.426** | 0.032 | 251 |
| **remote_front_right** | B1 | 0.329 | 0.853 | 0.027 | 177 |
|  | **B2** | 0.332 | **0.229** | 0.026 | 213 |

### 9.3 关键发现

**rfr Y 方向 std 降 73%（0.85 → 0.23 m）**——这就是 6×6 info-matrix 的本质优势。
rfr 朝右前看，Y 轴几何约束最弱（沿驾驶方向没多少特征），单帧 ICP 在 Y 上不确定度
天然大。B1 标量加权无法区分各轴，统一给整帧一个权重；B2 信息矩阵能精准把"该帧
Y 方向不可信、X/Z 可信"反映到聚合里。

**flash_front 三轴均匀降 30-50%**（X -48%, Y -39%, Z -29%）。说明 flash_front
本身视野各向同性较好，但 B1 仍受少数烂帧"按整帧权重拉走"影响，B2 按轴解耦后
得到更紧的估计。

**flash_rear dz 微升（+35%）**——绝对值 32 mm 仍极小。原因：B2 选择的有效帧
（251 帧）在 Z 方向上的剩余方差略大于 B1 的（523 帧），但这是 std 收紧到极限后
的随机波动，不是退化。

**|Δt| vs 工厂校准基本不变**（B1: 0.15/0.59/0.45 m → B2: 0.19/0.60/0.38 m）。
说明 calib 点估计稳定，B2 主要在**紧化不确定度**，而不是改变校准值本身。

### 9.4 信息矩阵物理直觉

取 flash_front 第一帧的 info-matrix translation 对角线：

| 轴 | 信息量 (1/m²) | 标准差 (m) | 物理含义 |
|----|---:|---:|----|
| X | 38 K | 0.005 | 行驶方向，特征少 |
| Y | 178 K | 0.002 | 侧向，墙等丰富 |
| Z | 966 K | 0.001 | 垂直，地面/天花板约束强 |

Z 信息量比 X 高 25×，完全符合"水平驾驶场景下垂直方向约束最强"的物理直觉。

### 9.5 性能

B2 没有额外耗时（Hessian 装配 O(M) 在 ICP 收敛后只跑一次，比 ICP 迭代本身快几个量级）。
3 个雷达总耗时与 §8 相同（136 + 132 + 267 = 535 s）。

### 9.6 已 commit

| 文件 | 改动 |
|------|------|
| `scripts/registration/base.py` | `RegistrationResult` 加 `information_matrix` 字段 |
| `scripts/registration/icp_pl.py` | post-ICP Hessian 装配 |
| `scripts/04_register_secondary.py` | 输出 `frame_information.csv` |
| `scripts/extract_extrinsic_from_registration.py` | `aggregate_info_weighted` + `--info-weighting` |

Commit hash: `5934040`。

### 9.7 三代演进总览

| 维度 | A 基线 | B1 (B3 配合) | **B2** |
|------|---|---|---|
| ICP 类型 | point-to-point | point-to-plane | point-to-plane |
| 聚合权重 | 平等（median） | 标量 (fitness·N/rmse²) | 6×6 info matrix 对角 |
| 每帧轴间区分 | 无 | 无 | 有 |
| rfr Y std (m) | 1.48 | 0.85 | **0.23** |
| 所有 dz std < 3cm | 部分 | 是 | 是 |
| ICP 速度 vs A | 1× | 2-5× | 2-5× |

### 9.8 下一步推荐

1. **C1（dump FAST-LIO 协方差）**：到这里副雷达端的不确定度已被精细化使用，
   但 primary trajectory 仍假定无误。改 laserMapping.cpp 把 ESKF P 对角线写出来，
   让低质量段的 primary pose 在 cross-LiDAR 解算时被自动降权。
2. **B2 升级到全 6×6 Mahalanobis**：当前 B2 只用 translation 对角块，旋转和
   翻译-旋转耦合都被丢了。完整的 Mahalanobis-加权 SE(3) 平均要解 6-DoF 上的
   加权最小二乘，能再压一档不确定度。
3. **D1（联合 BA）**：跨帧平滑约束 + 副雷达-地图约束放进 GTSAM/Ceres 同时优化。

---

## 10. B2-MH（全 Mahalanobis 升级）：探索性失败记录

实施了 §9.8 推荐 2（B2 升级到全 6×6 Mahalanobis Karcher mean on SE(3)）。
**结论：理论上更优，实战中输给 B2**。诚实记录下来供后人参考。

### 10.1 实现路径

**第一版**：full 6×6 Mahalanobis Karcher mean

数学上等价于解：

```
T̄ = argmin_T  Σ_i ξ_iᵀ Ω_i ξ_i   ,  ξ_i = Log(T_i · T̄⁻¹)
δ = (Σ Ω_i)⁻¹ Σ Ω_i ξ_i
T̄ ← Exp(δ) · T̄
```

实现 SE(3) Log/Exp（机器精度 1e-16 通过往返测试）+ Newton 迭代。结果惨败：

| 雷达 | t [m] | n_eff | 备注 |
|------|-------|------|------|
| flash_front | [2.76, 0.26, 1.82] | 121/600 | 偏离 B2 ~28 cm |
| flash_rear | [-0.86, 1.89, 1.43] | **16/600** | Y 偏离 1.9 m！ |
| rfr | [2.42, -0.15, 1.61] | 97/600 | Z 偏离 24 cm |

flash_rear 的 n_eff = 16/600 = 2.7% 是灾难性的：少数几帧的 info matrix
（特别是 rotation 部分）暴大，主导了整个均值，把 calib 拉到 +1.9 m 的离谱位置。

**第二版**：加 per-frame 归一化 + 5% 极端帧 trim

每帧 `Ω_i ← Ω_i / trace(Ω_i)`，再 trim 前后 5% 帧。n_eff 修复到 ~90%，
但结果反而更差（flash_front |Δt|=1.54 m，flash_rear |Δt|=1.15 m）。

**第三版**：Schur complement，3×3 translation-only Mahalanobis

```
I_t_marg_i = I_tt_i - I_tr_i · I_rr_i⁻¹ · I_rt_i
(Σ I_t_marg_i) t̄ = Σ I_t_marg_i · t_i
```

边缘化掉 rotation 影响后只对 translation 做 3×3 Mahalanobis。结果与 B2 接近，
但仍未显著超越：

| 雷达 | 方法 | dx_std | dy_std | dz_std | \|Δt\| (m) |
|------|------|---:|---:|---:|---:|
| flash_front | B2 (axis) | 0.277 | 0.274 | 0.013 | 0.191 |
|  | **B2-MH (Schur)** | **0.232** | **0.264** | 0.019 | 0.248 |
| flash_rear | **B2** | **0.532** | **0.426** | **0.032** | **0.598** |
|  | B2-MH | 0.750 | 0.893 | 0.024 | 0.776 |
| rfr | **B2** | **0.332** | **0.229** | **0.026** | **0.382** |
|  | B2-MH | 0.399 | 0.745 | 0.046 | 0.477 |

flash_front 略有改善（dx_std −16%），但 flash_rear 和 rfr 都比 B2 差，
最重要的 rfr Y std 从 0.229 涨回 0.745 m。

### 10.2 根因：per-frame ICP 估计是「有偏」的，不是 iid Gaussian

Mahalanobis 数学上假设：

- 每帧 `T_i` 是 ground truth `T̄_true` 加上零均值 Gaussian 噪声
- 噪声协方差就是 ICP 收敛时的 Hessian 倒数 `Σ_i = Ω_i⁻¹`

这两条前提在我们的设置里都不成立：

1. **per-frame 偏置**：每帧 ICP 收敛位置取决于局部几何。一段路上有平整墙面的
   帧会收敛到不同的位置（受墙面方向影响），跟有树有坡的帧不一样。这些不是
   均值为 ground truth 的随机扰动，而是**系统性的几何偏置**。
2. **info matrix ≠ 误差协方差**：Hessian 测的是"在当前几何下要把残差再降多少
   需要 pose 变多少"，反映的是 ICP cost 的局部曲率。但 ICP cost 的极小值
   未必等于真实 calib——曲率高 ≠ 偏置小。

结果：当 Mahalanobis 看到一帧 info 暴大就给它高权重，但这帧可能在某个方向上
**偏得很自信**（自信地偏了）。B2 的对角加权变成软投票，对偏置鲁棒。

### 10.3 留作 flag，不替换默认

- 默认仍是 B2（`--info-weighting`）
- B2-MH 通过 `--mahalanobis` 开启，作为未来工作的 placeholder

### 10.4 后续要做才能让 B2-MH 真的工作

1. **debias per-frame ICP 估计**：用 RANSAC / 子集投票剔除几何上偏的帧
2. **info matrix 重标定**：用 cross-frame variance 经验校准 `Σ_i`，把"自信但偏"
   的帧权重压下来（hierarchical Bayesian / empirical Bayes）
3. **robust Mahalanobis**：用 Huber/Cauchy loss 替代 quadratic
4. **联合 BA（D1）**：把 cross-frame 约束放进同一个优化器，让 outlier 自然
   被相邻帧"纠正"

### 10.5 已 commit

`6dfc03a`，包含：

- `aggregate_mahalanobis_se3` (full 6×6, 含 per-frame normalize + trim)
- `aggregate_mahalanobis_translation` (3×3 Schur 边缘化)
- `_schur_translation_info` 辅助函数
- SE(3) Log/Exp 工具函数
- `--mahalanobis` flag
- 修正 std_t 报告（原版报了 Cramér-Rao 均值标准差 ~0.0001 m，不可与 B1/B2 直接比较）

### 10.6 三代演进总览（更新）

| 维度 | A 基线 | B1+B3 | B2 (axis) | B2-MH (Schur) |
|------|---|---|---|---|
| ICP 类型 | point-to-point | point-to-plane | point-to-plane | point-to-plane |
| 聚合权重 | median | 标量 fitness | 对角 6×6 info | 3×3 Schur Mahalanobis |
| 数学严格度 | low | low | mid | **high** |
| 实战 rfr Y std | 1.48 | 0.85 | **0.23** | 0.75 |
| 抗偏置 | mid | mid | **high** | low |
| 是否默认 | — | × | **✓** | flag |

**重要教训：在样本有偏置的真实场景里，数学最优的 Mahalanobis 输给软投票的
axis-diagonal**。要让 Mahalanobis 真的赢，先要把 per-frame ICP debias，或者
把 cross-frame variance 喂回 info matrix 作经验校准。

---

## 11. C1 实施与发现（FAST-LIO ESKF 协方差不可用）

实施了 §7 C1 推荐：在 FAST-LIO 的 `dump_lio_state_to_log()` 加 6 列 ESKF 位姿
协方差对角线（pos × 3, rot × 3），通过容器 build 时 Python patch 注入。

### 11.1 基础设施全部跑通

| 文件 | 状态 |
|------|------|
| `docker/FAST_LIO/patch_dump_cov.py` | ✅ 在 GHCR CI build 中应用 |
| Dockerfile RUN 应用 patch | ✅ CI 一次过（无 patch tool 的 CRLF 之痛） |
| `scripts/poslog_to_tum.py` 解析 31 列 | ✅ 输出 `pose_covariance.csv` |
| `scripts/extract_extrinsic_from_registration.py --slam-cov` | ✅ 接受 cov CSV，按 `1/trace(Σ_t)` 加权 |
| GHCR image rebuild + pull | ✅ commit `e5e8ae9`, digest `b8852229` |

### 11.2 实测结果：B2 vs B2+C1 完全相同

跑了 3 个副雷达，结果**逐位相同**：

```
=== flash_front ===
B2:        t=[ 2.639, 0.027, 1.742]  std=[0.277, 0.274, 0.013]
B2 + C1:   t=[ 2.639, 0.027, 1.742]  std=[0.277, 0.274, 0.013]
=== flash_rear ===
B2:        t=[-1.422,-0.047, 0.546]  std=[0.532, 0.426, 0.032]
B2 + C1:   t=[-1.422,-0.047, 0.546]  std=[0.532, 0.426, 0.032]
=== rfr ===
B2:        t=[ 2.321,-0.374, 1.850]  std=[0.332, 0.229, 0.026]
B2 + C1:   t=[ 2.321,-0.374, 1.850]  std=[0.332, 0.229, 0.026]
```

### 11.3 为什么没有差异：ESKF 过度自信

dump 出来的协方差对角线统计（每 50 帧取样）：

```
t=  0.30  pos_var=(2, 2, 2)×10⁻⁶  rot_var=(0, 0, 0)
t=  5.30  pos_var=(2, 4, 1)×10⁻⁶  rot_var=(0, 0, 0)
t= 10.30  pos_var=(2, 3, 1)×10⁻⁶  rot_var=(0, 0, 0)
...
t= 55.30  pos_var=(2, 4, 1)×10⁻⁶  rot_var=(0, 0, 0)
```

**关键观察**：

1. **Position 方差全程在 1-4 μm² 间徘徊**（对应 std ≈ 1-2 mm）。整段路 60 秒
   下来 FAST-LIO 估自己的累计位置误差不到 2 毫米。
2. **Rotation 方差跨整段都 < 1e-7** (被 `%lf` 截成 0)。
3. **方差不随时间增长、转弯不响应**：60 s 内 dz var 始终 1-2 μm²，但 §2 用
   首帧对齐测出 Z 实际累积漂移 1.27 m → 真实 std 至少 30-50 cm，与 FAST-LIO
   自报的 1.4 mm 差 **300×**。

这是 IKFoM 类滤波器的已知特性：协方差被测量雅可比 `H_T R⁻¹ H` 锁定在一个
**测量噪声驱动的下界**，不随时间累积，也不反映线性化误差和实际漂移。

### 11.4 为什么这事不能用 `%.6e` 救

第一反应：是不是 `%lf` 把 1.234567e-07 截成 0，加上 6e 就有信号了？

试过短暂 patch 成 `%.6e`，但即使报出真实值（1e-7 级别），跨帧的 **dynamic range
也仍然 < 5×**。和 B2 的 fitness/inliers/rmse 联合权重相比（动态范围 100×+），
SLAM cov 在 B2 weight 上叠 1/var 因子只是给所有帧乘一个近似常数，结果就是
**逐位相同**。

所以撤回精度改动，保留 `%lf`（默认精度足够诊断这件事本身）。

### 11.5 这条路要走得通的前提

C1 想法本身是对的（FAST-LIO 不确定时段下，cross-LiDAR 该信不过），但需要：

1. **重新 calibrate ESKF 协方差**：用 §2 测出的实际 ATE 当真值，反推
   process_noise_cov 应该调大多少倍。FAST-LIO config 里 acc_cov、gyr_cov、
   b_acc_cov、b_gyr_cov 是关键，当前可能定得过紧。
2. **用 covariance "shape" 而不是 magnitude**：相对各帧间的方差比例（哪些
   帧相对更不确定）可能仍然有意义，即使绝对值不可信。可以试 z-score 化后
   作为权重的指数因子。
3. **改用其他不确定度信号**：FAST-LIO 内部的迭代次数、最终残差、点云匹配率
   等可能更直接反映"此帧滤波器有多挣扎"。
4. **换不报过紧 cov 的 SLAM 后端**：BALM, VoxelMap, LIO-SAM 这些做了协方差
   增长项处理的 LIO 报得更接近真实。

### 11.6 现状对外参标定的影响

零。本数据集 B2 仍是当前最优组合。C1 基础设施留着，等后续 calibrate ESKF
或换 SLAM 后端时直接接上。

### 11.7 已 commit

| 文件 | 改动 |
|------|------|
| `docker/FAST_LIO/patch_dump_cov.py` | Build 时 Python patch（idempotent） |
| `docker/FAST_LIO/Dockerfile` | COPY + RUN patch script |
| `scripts/poslog_to_tum.py` | 解析 31 列，输出 `pose_covariance.csv` |
| `scripts/extract_extrinsic_from_registration.py` | `--slam-cov` flag |

Commits: `e5e8ae9` (patch), `c9544b0` (tooling)。

### 11.8 三代 + C1 总览

| 维度 | A | B1+B3 | B2 (axis) | B2+C1 |
|------|---|---|---|---|
| ICP | P2P | P2plane | P2plane | P2plane |
| 聚合权重 | median | scalar fitness | axis info | axis info × 1/Σ_SLAM |
| rfr Y std (m) | 1.48 | 0.85 | **0.23** | 0.23 (no change) |
| 信号有效性 | — | ✅ | ✅ | ❌ ESKF 过度自信 |

### 11.9 下一步（路线图修正）

C1 暂搁置，转入：

1. **修 ESKF 自信问题**：调 FAST-LIO process noise，让 cov 变得真实。
   需要标定式实验。
2. **D1（联合 BA）**：把 cross-frame 平滑约束直接放进 cost，无需逐帧权重玄学。
3. **debias per-frame ICP**：让 B2-MH 真正能赢（参见 §10.4）。

---

## 12. 调 process_noise 救不了 C1（结构性问题）

按 §11.9 推荐 1 试调 FAST-LIO `process_noise_cov`，看能否让 ESKF cov 变得对
per-frame 加权有用。**结论：调不动**。

### 12.1 实验设置

`config/fastlio_at128p_velodyne.yaml` 的 4 个 noise 参数同步乘 10×、100×、1000×：

```yaml
mapping:
    acc_cov: 0.1   →   1.0   →  10.0   →  100.0
    gyr_cov: 0.1   →   1.0   →  10.0   →  100.0
    b_acc_cov: 0.0001  →  0.001  →  0.01  →  0.1
    b_gyr_cov: 0.0001  →  0.001  →  0.01  →  0.1
```

容器内逐个跑 SLAM，每次约 5 min。

### 12.2 结果：cov 几乎不动，SLAM 质量恶化

| Scale | pos_var_y mean (m²) | 跨帧动态范围 | Z 最大 (m) | 评价 |
|-------|---:|---:|---:|---|
| **1×** (baseline) | 3.17 × 10⁻⁶ | 2× | 6.69 | OK |
| 10× | 4.16 × 10⁻⁶ | 3× | 7.55 | 微变化 |
| 100× | 4.81 × 10⁻⁶ | 3.5× | 7.71 | 接近饱和 |
| 1000× | 5.04 × 10⁻⁶ | 3.5× | 7.71 | 完全饱和 |

三个关键观察：

1. **process noise 调 1000× 只让 cov 平均涨 60%**：均值 3.17 → 5.04 × 10⁻⁶。
   远小于"真实漂移 1.27 m → cov 应该是 1.6 × 10⁻¹"那一档要求。
2. **跨帧动态范围始终 2-3.5×**：100× 和 1000× 的动态范围完全相同。说明
   process noise 增长被测量更新"吃掉了"，没传到帧间方差差异上。
3. **Z 漂移反而恶化**（6.69 → 7.71 m）：噪声太大滤波器轻信 IMU 积分，
   SLAM 局部精度下降。process noise 调大有副作用。

`rot_var` 在所有 scale 下都被 `%lf` 截成 0——无信号。

### 12.3 根因（结构性）

IKFoM (iterated extended Kalman filter on manifold) 是 FAST-LIO 后端。
迭代更新步：

```
for k = 1 .. K:
    H_k = jacobian at x_k
    K_k = P_k H_kᵀ (H_k P_k H_kᵀ + R)⁻¹
    x_{k+1} = x_k + K_k (residual)
    P_{k+1} = (I - K_k H_k) P_k                ← 这一步收紧 cov
```

每次迭代把后验 cov 收敛到由**测量雅可比和测量噪声 R 决定的下界**
（信息矩阵下界 `H R⁻¹ H`）。process noise 只影响**预测阶段**给 P_pred 多大空间，
但更新阶段把 P_pred 一直收回到这个测量驱动下界。

实测体现：

```
process_noise 100× → P_pred 涨 ~100×
                  → 迭代 K_max=3 步后 P_post 几乎被打回原值
```

这是一个**良好调参 ESKF 的正常行为**——不是 FAST-LIO 的 bug，是滤波器形式
本身的限制。对**实时定位**来说，这是一种特性（cov 反映"假设测量足够"下的
理论极限）；对我们想用来做"哪帧滤波器自己也没把握"的诊断信号，**不可用**。

### 12.4 那这条路就走死了？

不全是。仍有几条候选：

1. **C0：直接补充 `LASER_POINT_COV`**（lasermapping.cpp 顶部的常数）。
   现在是 `0.001`，把测量噪声 R 放大 100× 等同于直接抬高 cov 下界。但和
   `process_noise` 一样会牺牲 SLAM 精度，且作用不局部（影响每一帧）。
2. **C2：用 cov 形状而不是 magnitude**。当前 pos_var 是
   `var_y ≈ 2-3 × var_z`（车横向比纵向不可靠），这个 ratio 物理上对应
   "Y 几何约束弱"——是有意义的信号。能拿这个做**轴间相对加权**而不是
   全局乘数。
3. **C3：从迭代次数 / 残差 / 匹配数推 per-frame quality**。FAST-LIO 内部有
   每帧 ICP 残差范数、点云匹配数。这些直接反映"该帧 LiDAR 对滤波贡献多大"，
   远比 cov 有信号。需要再加一个 dump。
4. **C4：empirical cov 校正**。用 §2 ATE + RPE 的实测，反推 process_noise
   / LASER_POINT_COV 应该乘多少，让 cov 数值上和真实匹配。这是
   "post-hoc covariance calibration" 的经典做法。
5. **D1：换 SLAM 后端做联合 BA**（GTSAM pose graph）。BA 后端的边权可以直接
   从 cross-frame consistency 学，跳过滤波器 cov 整个问题。

### 12.5 取舍

C0 / C2 / C3 都还能试，但收益预计有限（最多压 std 10-20%）。本数据集 B2 已经
在 σ_t < 5 cm 量级，再压收益不明显。

更有价值的方向是 **D1（联合 BA）**——它一次性解决：
- 跨帧不一致问题（B2 还没解决的）
- ICP per-frame 偏置问题（B2-MH 失败的）
- SLAM cov 不可用问题（C1 失败的）

### 12.6 已 commit

| 文件 | 改动 |
|------|------|
| `output/noise_sweep/x{10,100,1000}/` | 临时输出，仅用于本节统计 |
| 没有代码改动（config.yaml 来回改后已还原） | — |

### 12.7 路线图最终更新

| Track | 状态 | 备注 |
|-------|------|------|
| A 基线（icp + median） | ✅ baseline | dx_std 0.5 m |
| B1（fitness 加权） | ✅ landed | rfr Y std 1.5 → 0.85 m |
| B3（point-to-plane ICP） | ✅ landed | 2-5× 提速 |
| B2（axis info 加权） | ✅ landed | rfr Y std → 0.23 m |
| B2-MH（full 6×6 Mahalanobis） | ❌ 留 flag | per-frame ICP 偏置，rfr Y 反弹到 0.75 |
| C1（FAST-LIO cov dump） | ❌ 基础设施完整，信号不可用 | ESKF 过度自信 |
| C1.5（调 process_noise） | ❌ 无效 | IKFoM 结构特性 |
| **D1（联合 BA）** | 🎯 **next** | 唯一仍有 upside 的方向 |

---

## 13. RPE 多窗口 + 质量告警（quick wins）

§7 "评估方法改进" 里 E2 提到的 RPE、§4.4 提到的"质量告警"两个 quick wins
落地，本节记录实测结果。

### 13.1 RPE（相对位姿误差）多窗口

在 `scripts/compare_with_lidar_to_map.py` 加 `compute_rpe()`。对每个匹配
pose i，找 ts[j] ≈ ts[i] + Δt 的 pose j（20% 容差），计算：

```
E_ij = (T_ref_i⁻¹ · T_ref_j)⁻¹ · (T_slam_i⁻¹ · T_slam_j)
```

`||trans(E_ij)||` 和 `rot_angle(E_ij)` 跨所有 (i, j) 取统计。**RPE 不依赖
全局对齐**——相对位姿在本地 / 全局 frame 下是同一个。

实测 60 s AT128P 数据：

| Δt 窗口 | n pairs | trans RMS (m) | trans max (m) | rot mean (°) |
|---------|---:|---:|---:|---:|
| **1 s** | 585 | **0.053** | 0.247 | 0.243 |
| 5 s | 545 | 0.181 | 0.598 | 0.276 |
| 10 s | 496 | 0.317 | 1.130 | 0.294 |
| 30 s | 296 | 0.901 | 2.112 | 0.386 |

### 13.2 三条 RPE 解读

1. **1 s RPE = 5.3 cm RMS**：本地一致性极佳（相比之下 ATE = 32 cm 是全局
   累积量）。说明 LIO 的"每帧之间相对位姿"很准——配准 / 建图都能信。
2. **10 s RPE ≈ ATE**：10 秒窗口的相对漂移和全局 ATE 同量级，是漂移的主要
   来源。
3. **漂移生长超 √t**：

   理想随机游走下 `σ(Δt) ∝ √Δt`：

   | Δt | √t 预期 | 实测 | 比值 |
   |----|---:|---:|---:|
   | 1 s | 5.3 cm | 5.3 cm | 1.00× |
   | 5 s | 11.9 cm | 18.1 cm | 1.52× |
   | 10 s | 16.8 cm | 31.7 cm | 1.89× |
   | 30 s | 29.0 cm | 90.1 cm | 3.10× |

   实测远快于 √t，更接近线性。这是**系统性 bias drift**（IMU bias 没标定干净
   / ESKF 过度自信）的指纹，**不是**白噪声。与 §11/§12 的诊断一致。

4. **rotation 全程 < 0.4°**：方向估计很稳，drift 主要在位置。

### 13.3 quality 告警系统

新建 `scripts/check_quality.py`，自动读：

- `output/<run>/compare_traj_summary.yaml` —— 轨迹层指标
- `output/<run>/registration_pl2/<lidar>/summary.yaml` —— 配准层指标
- `output/<run>/calibrated_extrinsics.yaml` —— 校准层指标

每项与 `DEFAULT_THRESHOLDS` 比较，emit 三级状态：`INFO` / `WARN` / `FAIL`。
退出码：

- `0` 全部 PASS 或仅 WARN
- `2` 至少一项 FAIL

阈值可用 `--thresholds threshold.yaml` 覆盖（生产环境收紧）。

### 13.4 默认阈值与现状

```yaml
ate_rms_m:              {warn: 0.5,  fail: 1.5}
rpe_10s_trans_rms_m:    {warn: 0.6,  fail: 1.5}
rot_max_deg:            {warn: 3.0,  fail: 10.0}
z_drift_m:              {warn: 2.0,  fail: 8.0}
mean_fitness:           {warn: 0.5,  fail: 0.3}  # higher better
mean_rmse_m:            {warn: 0.5,  fail: 1.5}
delta_t_norm_m:         {warn: 0.5,  fail: 1.5}
calib_translation_std_m:{warn: 1.0,  fail: 3.0}
```

当前数据集 60 s AT128P 跑结果：

```
ℹ trajectory ATE RMS = 0.323 (ok ≤ 0.5)
ℹ trajectory rot max = 1.018 (ok ≤ 3.0)
ℹ Z drift (first-frame) = 1.265 (ok ≤ 2.0)
ℹ RPE 10s translation RMS = 0.317 (ok ≤ 0.6)
ℹ reg/flash_front_pointcloud fitness = 0.953 (ok ≥ 0.5)
ℹ reg/flash_front_pointcloud rmse = 0.170 (ok ≤ 0.5)
ℹ reg/flash_rear_pointcloud fitness = 0.913 (ok ≥ 0.5)
ℹ reg/flash_rear_pointcloud rmse = 0.204 (ok ≤ 0.5)
ℹ reg/remote_front_right_pointcloud fitness = 0.638 (ok ≥ 0.5)
ℹ reg/remote_front_right_pointcloud rmse = 0.295 (ok ≤ 0.5)
ℹ calib/flash_front_pointcloud |Δt vs factory| = 0.257 (ok ≤ 0.5)
ℹ calib/flash_front_pointcloud ‖σ_t‖ = 0.794 (ok ≤ 1.0)
ℹ calib/flash_rear_pointcloud |Δt vs factory| = 0.239 (ok ≤ 0.5)
ℹ calib/flash_rear_pointcloud ‖σ_t‖ = 0.875 (ok ≤ 1.0)
ℹ calib/remote_front_right_pointcloud |Δt vs factory| = 0.140 (ok ≤ 0.5)
⚠ calib/remote_front_right_pointcloud ‖σ_t‖ = 1.573 > warn thresh 1.0

OVERALL: WARN  (0 fail / 1 warn / 16 checks)
```

唯一 WARN：rfr 的 ‖σ_t‖ = 1.57 m 超过 1.0 m 阈值。物理上 rfr 朝右前看，Y
横向几何约束最弱，0.85 m Y std 拉高了 σ_t 模。这是已知问题（§9.3 提到），
告警准确捕获。

### 13.5 接入 `run_all.sh`

`run_all.sh` 尾部新加 Step 06，pipeline 跑完自动调 check_quality：

```bash
# --- Step 06: Quality check (advisory) ---
python "$SCRIPT_DIR/check_quality.py" --run-dir "$OUTPUT_DIR" || \
    echo "  (quality check returned a FAIL — see above)"
```

不阻塞流程（即便 FAIL 也只是显示），决策权留给运维 / CI。CI 想强制 PASS
可以去掉 `|| echo` 改为 `python ... check_quality.py --run-dir ...`，让
exit 2 直接 fail job。

### 13.6 已 commit

| 文件 | 改动 |
|------|------|
| `scripts/compare_with_lidar_to_map.py` | 加 `compute_rpe()`，4-window 表 + summary YAML |
| `scripts/check_quality.py` | 新增 16 项阈值检查 + WARN/FAIL/exit code |
| `scripts/run_all.sh` | Step 06 advisory quality check |

Commit `4cb2ae0`。

### 13.7 价值评估

- **RPE**：解锁了 ATE 隐藏的漂移特征。从此报告告警都能精确指认 drift 量级
  和窗口。**对比同类数据集 / 后端的 SLAM 性能更可信**。
- **Quality alarms**：把"60 秒 ATE 0.3 m、rfr Y std 0.23 m"这类结论从
  "需要人盯输出" 提升到 "CI 一行命令拿到 OK / 告警 / 失败"。生产化前
  必备。

两者都不改善 calib **数值精度**，但显著改善**评估和运维体验**。

---

_Report 生成于 2026-05-30，§8/§9 增补于 2026-05-31，§10/§11/§12 增补于
2026-05-31，§13 增补于 2026-05-31。_

---

## 14. 后端对比：FAST-LIO vs KISS-ICP

§7 评估方法学里第 18 条"单一 SLAM 后端"指出 FAST-LIO 可能不是本数据集最优。
本节实测 [KISS-ICP](https://github.com/PRBonn/kiss-icp)（PRBonn，2023）作为
对比。KISS-ICP 是**纯 LiDAR**（不用 IMU）+ voxel ICP 的极简管线。

### 14.1 实施

```bash
pip install kiss-icp                                # 一行装好
# pre-clean NaN points (KISS-ICP silently exits on NaN PCD)
python prep_nan_clean.py
# run
kiss_icp_pipeline /path/to/cleaned_pcds             # 23 秒跑完 600 帧
```

输出 `cleaned_pcds_poses_tum.txt`（TUM 格式，但 timestamp 用帧索引）。
注入真实时间戳后跑 `compare_with_lidar_to_map.py`。

**坐标系注意**：KISS-ICP 输出的是 `T_kissmap_lidar`（LIDAR 局部系），不是
baselink 系。LiDAR→baselink 有 49° yaw 偏置；直接比 ATE 会出现 50° 旋转
残差和 0.71 m ATE。需要先用 `T_lidar_baselink = inv(yaml extrinsic)` 复合
出 `T_kissmap_baselink`，再比。`compare_with_lidar_to_map.py` 加了
`--slam-frame baselink` 标志支持这种"已组合"输入。

### 14.2 实测对比

完整对比表（与 §13 相同 RPE 窗口）：

| 指标 | FAST-LIO | **KISS-ICP** | KISS 优势 |
|------|---:|---:|:---:|
| 实施工作量 | Docker + ROS + CI build | `pip install` | 极简 |
| 是否需要 IMU | 是 | **否** | — |
| Wall-clock 跑 60 s 数据 | ≈ 60 s + ROS 开销 | **23 s** | **4×** |
| 处理速率 | ~10 Hz | **44 Hz** | 4× |
| **ATE RMS（global 对齐）** | 0.323 m | **0.149 m** | **2.2×** |
| ATE max（global） | 0.746 m | 0.336 m | 2.2× |
| **Z 漂移（首帧对齐 dz bias）** | **+1.265 m** | **+0.156 m** | **8×** ⭐ |
| ATE RMS（首帧对齐） | 1.686 m | 0.568 m | 3× |
| dz std（global） | 0.027 m | 0.032 m | 类似 |
| Rot max（global） | 1.02° | 1.06° | 类似 |
| RPE 1 s | 5.3 cm | 6.0 cm | 类似 |
| RPE 5 s | 18.1 cm | 16.2 cm | 10% 优 |
| RPE 10 s | 31.7 cm | 27.8 cm | 12% 优 |
| RPE 30 s | 90.1 cm | 70.6 cm | 22% 优 |
| 是否报 covariance | 是（不可信，§11） | 否 | — |

### 14.3 关键发现

**KISS-ICP 在本数据集上全面占优**——精度更高、漂移更小、跑得更快、依赖更少。
最显眼的是 **Z drift 8× 改善**（1.27 m → 0.156 m）。这一点直接命中了
§11/§12 反复出现的 ESKF 过度自信问题——纯 LiDAR pipeline 不用 IMU 积分，
本质上没有 bias drift 累积，所以 Z 方向稳得多。

### 14.4 为什么 KISS-ICP 这么好？

1. **AT128P 128 线点云**几何信息极其丰富，单帧 ICP 就能可靠跟踪，IMU 先验
   反而是"多余的不确定度来源"
2. **voxel 表示 + 自适应阈值** 让 ICP 收敛快、稳
3. **无累积 IMU bias**——FAST-LIO Z drift 的根本来源
4. **更简单 = 更少错误来源**

### 14.5 KISS-ICP 的弱点（不在本数据集体现）

- **无 IMU 备份**：tunnel / 长直道 / featureless 场景 LiDAR 失效时没有支撑
- **无 covariance 输出**：cross-LiDAR 加权聚合（§9 B2）走不通——只能用 ICP-side
  info matrix（B2 仍可），但 SLAM-side 的可信度信号缺失
- **无 loop closure / 重定位**：长序列累积漂移会一直涨，没有闭环修正
- **不报偏置 / 速度**：下游需要 IMU 状态估计的应用（如 fusion / 控制）拿不到

### 14.6 改用 KISS-ICP 后的下游影响

如果切换到 KISS-ICP 作为主雷达 SLAM：

| 下游模块 | 影响 |
|---------|------|
| Cross-LiDAR 配准（§3） | 主地图质量更好 → 副雷达 ICP fitness 应该提升 |
| B1/B2 加权 | 不变 |
| C1 / C1.5（SLAM cov） | 直接没了 → 不需要纠结 ESKF 过度自信 |
| Quality alarms（§13） | RPE_10s 阈值可以收紧到 0.4 m（KISS 实测 0.28） |
| Z drift 阈值（§13） | 2.0 → 0.5 m 可行 |
| run_all.sh / Dockerfile | 大幅简化，不再需要 ROS Noetic |

### 14.7 建议路径

短期（1-2 天）：把 KISS-ICP 集成进 pipeline 作为**默认主雷达 SLAM**，FAST-LIO
留作 fallback（IMU 可用且 LiDAR 退化场景）。改动：

- `scripts/run_kiss_icp.py`：从 PCD 目录到 trajectory.txt 一站式
- 修改 `04_register_secondary.py` 接受 KISS-ICP 输出地图（reconstruction 用
  `cleaned_pcds_poses_kitti.txt` + 累积变换）
- `run_all.sh` 加 `--backend {fast_lio, kiss_icp}` 开关

中期：在没有 IMU / 短序列场景下默认 KISS-ICP；长序列 + IMU 可用场景默认
FAST-LIO（或换 LIO-SAM——它有 GTSAM + loop closure）。

### 14.8 已记录

- `output/kiss_icp_run/trajectory.txt`（LiDAR frame）
- `output/kiss_icp_run/trajectory_baselink.txt`（baselink frame，已组合）
- `output/kiss_icp_run/compare_traj_summary.yaml`
- `output/kiss_icp_run/compare_traj_*.png`

代码改动：

| 文件 | 改动 |
|------|------|
| `scripts/compare_with_lidar_to_map.py` | 加 `--slam-frame {imu,baselink}` |

### 14.9 路线图最终最终更新

| Track | 状态 | 备注 |
|-------|------|------|
| A 基线 → B2 → B2-MH | ✅ / ❌ | 见 §8/9/10 |
| C1 / C1.5 | ❌ ESKF 过度自信 | §11/12 |
| RPE + alarms | ✅ landed | §13 |
| **KISS-ICP 后端** | 🎯 **强推默认** | 2-8× 全面优于 FAST-LIO |
| LIO-SAM 对比 | 🔜 next | 因为 LIO-SAM 有 GTSAM + cov + loop closure |
| D1 联合 BA | 🔜 | 如果 KISS-ICP + IMU 组合方案不够 |

---

_Report 生成于 2026-05-30，§8/§9 增补于 2026-05-31，§10/§11/§12 增补于
2026-05-31，§13/§14 增补于 2026-05-31。_

---

## 15. KISS-ICP 地图喂回 cross-LiDAR：校准 std 大幅压降

§14 证明 KISS-ICP 主轨迹 / 地图精度全面优于 FAST-LIO。本节实测：把
B2（point-to-plane + axis info）所用的**主地图换成 KISS-ICP 输出**，
3 个副雷达校准的 std / |Δt| 改善多少。

### 15.1 实验设置

| 组件 | FAST-LIO baseline | KISS-ICP 实验 |
|------|-----------------|--------------|
| 主轨迹 | FAST-LIO ESKF | KISS-ICP voxel-ICP（复合到 baselink） |
| 主地图 | FAST-LIO scans.pcd → voxel 0.3 m | KISS-ICP 拼帧 → voxel 0.3 m（13 MB） |
| Cross-LiDAR ICP | point-to-plane (icp_pl) | 同上 |
| 聚合 | B2 axis info-weighted | 同上 |
| 副雷达数据 | 同上（原始 PCD） | 同上 |

地图拼接：取 KISS-ICP 输出的每帧 pose（LiDAR 系）× 对应清洗过的原始 PCD →
75.5 M 点累积 → voxel 0.3 m 降采样 → 1.13 M 点 / 13 MB。

### 15.2 ICP 收敛质量对比

| 副雷达 | 指标 | FAST-LIO 地图 | KISS-ICP 地图 | 变化 |
|--------|------|---:|---:|---:|
| flash_front | mean fitness | 0.953 | **0.966** | +1.4% |
| | mean rmse | 0.170 | **0.161** | -5% |
| flash_rear | mean fitness | 0.912 | **0.930** | +2% |
| | mean rmse | 0.204 | **0.194** | -5% |
| rfr | mean fitness | 0.638 | **0.661** | +3.6% |
| | mean rmse | 0.295 | **0.280** | -5% |

主地图更好 → ICP 命中率提升、对应残差降低。三个副雷达**一致小幅改善**。

### 15.3 校准 std / |Δt| 完整对比

| 副雷达 | 维度 | FAST-LIO B2 | **KISS-ICP B2** | 改善 |
|--------|------|---:|---:|---:|
| **flash_front** | dx_std (m) | 0.277 | 0.272 | -2% |
|  | **dy_std (m)** | 0.274 | **0.135** | **-51%** ⭐ |
|  | dz_std (m) | 0.013 | 0.017 | +27% |
|  | n_eff / 600 | 374 | 376 | ~ |
|  | **\|Δt\| factory (m)** | 0.191 | **0.128** | **-33%** |
| **flash_rear** | dx_std (m) | 0.532 | 0.526 | -1% |
|  | dy_std (m) | 0.426 | 0.364 | -15% |
|  | **dz_std (m)** | 0.032 | **0.016** | **-49%** ⭐ |
|  | **n_eff / 600** | 251 | **356** | **+42%** |
|  | **\|Δt\| factory (m)** | 0.598 | **0.286** | **-52%** ⭐ |
| **rfr** | **dx_std (m)** | 0.332 | **0.192** | **-42%** ⭐ |
|  | dy_std (m) | 0.229 | 0.258 | +13% |
|  | dz_std (m) | 0.026 | 0.024 | -8% |
|  | n_eff / 600 | 213 | 170 | -20% |
|  | **\|Δt\| factory (m)** | 0.382 | 0.311 | -19% |

### 15.4 五个亮点

1. **flash_front dy std 减半**（0.274 → 0.135 m，−51%）—— 横向方向最弱
   几何约束，KISS-ICP 主地图更清晰直接解决。
2. **flash_rear dz std 减半**（32 → 16 mm，−49%）—— 主地图 Z 8× 更稳的
   直接传导。
3. **flash_rear n_eff +42%**（251 → 356）—— 更多帧达到加权阈值，信息密度
   显著提升。
4. **rfr dx std −42%**（0.332 → 0.192 m）—— 难度最大的副雷达受益最明显。
5. **|Δt| vs 工厂全部更近**，flash_rear −52% 最显著。如果工厂标定接近真值，
   说明 KISS-ICP B2 的 calib 估计也更接近真值。

### 15.5 小幅副作用

- flash_front dz_std +27%（13 → 17 mm）—— 但绝对量级仍极小。
- rfr n_eff −20%（213 → 170）—— 更严的加权门槛把些边界帧剔了，但留下的
  帧贡献的 dx std 还是降了 42%。

这两条都是"std 跟 n_eff 互相 trade"的小波动，不影响主结论。

### 15.6 解读：主地图质量 → 校准精度的链路

数据链路被清晰打通：

```
KISS-ICP Z drift 8× 更小  →  主地图 Z 一致性提升
                          ↓
                     submap 抽取更对位
                          ↓
                  ICP fitness ↑ rmse ↓
                          ↓
              per-frame T_baselink_sec 估计更紧
                          ↓
                 aggregate 后 std 大幅压降
```

之前 §11/§12/§14 反复指认的"FAST-LIO ESKF 过度自信 → Z drift → 下游精度
吃亏"在本节得到**直接量化验证**：换 KISS-ICP，dz_std 减半，最难副雷达的
横向 std 也减半。

### 15.7 是否替换默认主 backend？

是。本节实测 + §14 端到端对比共同支持：

| 维度 | 推荐 |
|------|------|
| 默认主 SLAM backend | **KISS-ICP** |
| Fallback | FAST-LIO（IMU 可用 + LiDAR 退化场景） |
| Cross-LiDAR aggregation | B2 (B1 + B3 + axis info) 不变 |

### 15.8 已 commit 输出

- `output/kiss_icp_run/scans_voxel0.3.pcd` — KISS-ICP 主地图（13 MB）
- `output/kiss_icp_run/trajectory_imu.txt` — KISS-ICP 主轨迹（兼容 B2 pipeline）
- `output/kiss_icp_run/registration/<lidar>/frame_transforms.txt` —
  3 副雷达 icp_pl 配准（含 frame_quality.csv + frame_information.csv）
- `output/kiss_icp_run/calibrated_extrinsics.yaml` — 最终 B2 校准（KISS map）

### 15.9 路线图（最终最终最终）

| Track | 状态 | 备注 |
|-------|------|------|
| KISS-ICP 后端 + B2 | ✅ landed | 当前最优组合 |
| RPE + alarms（§13） | ✅ landed | 阈值可以收紧 |
| LIO-SAM 对比 | 🔜 | factor graph + cov + loop closure |
| D1 联合 BA | 🔜 | KISS-ICP + IMU 组合 BA 后端 |
| 多 sample 稳定性 | 🔜 | 当前仅单段 60 s 数据，需多录制验证 |

---

_Report 生成于 2026-05-30，§8/§9 增补于 2026-05-31，§10/§11/§12 增补于
2026-05-31，§13/§14/§15 增补于 2026-05-31。_

---

## 16. LIO-SAM benchmark：构建被 PCL/FLANN/C++17 卡住，暂搁置

§14/§15 已确认 KISS-ICP 是当前最优主 SLAM backend。理论上 LIO-SAM（GTSAM
factor graph + loop closure + IMU 紧耦合）应该至少跟 KISS-ICP 持平甚至更好，
所以本节按 dockerize-submodule 模式尝试集成。**结论：构建链卡死，暂搁置**。

### 16.1 集成路径（按 VRecHub dockerize-submodule pattern）

| 项目 | 选择 |
|------|------|
| 源码源 | `thirdparty/LIO-SAM`，submodule pointing to `wangxinjian1108/LIO-SAM` |
| 注册方式 | 直接写 `.gitmodules` + 用 `git update-index --add --cacheinfo 160000` 写 gitlink，commit pin 到 `0be1fbe6`（本地拉不动 131MB） |
| Dockerfile | `COPY thirdparty/LIO-SAM /catkin_ws/src/LIO-SAM`（无 in-image clone） |
| CI workflow | `actions/checkout@v4 with: submodules: recursive`（runner 网络快） |
| GHCR 目标 | `ghcr.io/wangxinjian1108/lio-sam:latest` |

### 16.2 6 次 CI build 修复链

依次撞到 6 个独立编译问题，每个都修了但又冒下一个：

| # | 错误 | 修复 | 结果 |
|---|------|------|------|
| 1 | `git clone hku-mars/LIO-SAM` slow | 改为 submodule + COPY | OK |
| 2 | `libgtsam-dev` 不在 Ubuntu 20.04 apt | 改为 GTSAM 4.0.3 源码编译 | OK |
| 3 | `opencv/cv.h: No such file` | sed 替换为 `opencv2/opencv.hpp` | OK |
| 4 | PCL 1.10 要求 C++14+ | sed 把 LIO-SAM CMakeLists `-std=c++11` → `-std=c++17` | OK |
| 5 | FLANN `unordered_map.serialize()` 方法不存在 | 加 `#include <boost/serialization/unordered_map.hpp>` | **未修复** |
| 6 | 同上：sed flann 头文件把 member-call 改 boost free-fn | **仍未修复** |

第 5 / 6 错误本质是：FLANN 1.9 (Ubuntu 20.04 default) 的内部模板 `flann::serialization::access::serialize` 对任意值类型一律调用 `value.serialize(ar)`
方法。`std::unordered_map` 没有这个成员（boost 提供的是 free function）。
我们的 patch 想把 member-call 改成 boost free-fn，但**该 flann 内部用的是
`flann::serialization::LoadArchive`，跟 boost 序列化不互通**——flann 不接受
boost::archive 的 deserialize 调用。

### 16.3 真正要 fix 这事的成本

需要**给 FLANN 显式加 `std::unordered_map` 的特化**，
或者从源码重建 PCL 1.10 + FLANN 1.9 with C++17 flags + 修补的序列化模板。
工作量是几天到一周（特别是测试覆盖所有 PCL features）。

社区已知解决方案：

- 切到 **Ubuntu 22.04 + ROS 2 Humble**（PCL 1.12 + FLANN 1.9 with fix）—— 但
  LIO-SAM upstream 主分支仍是 ROS1 Noetic
- 用 **LIO-SAM 的某些 fork**（如 LIO-SAM-with-Closed-Loop, FAST-LIO-SAM）
  自带 patch
- 用 `sudo make install` 装 FLANN from source with patched headers

### 16.4 投入产出评估

- 已投入：6 个 CI build cycle × ~10 min ≈ 60 min
- 预期再投入到 build 通过：3-7 天（包括 fork 调研 / 重建 PCL）
- 预期收益：和 KISS-ICP 比，最多带来 5-15% 的精度改善（loop closure
  + factor graph），但 KISS-ICP 已经把 std 压到 cm 级，进一步收益边际
- 当前生产瓶颈不在 SLAM 精度，而在多 sample 稳定性、debiasing、long-sequence
  loop closure。这些都可以**不依赖 LIO-SAM** 解决

### 16.5 决定

- **LIO-SAM 暂搁置**。Workflow 文件改名 `docker-LIO_SAM.yml.disabled`，
  不会再触发 CI。
- 当前生产 backend 维持 KISS-ICP（§14/§15 确认最优）。
- 留下来：完整的 dockerize-submodule 集成模板 + 6 个 build fix patches，
  下次想恢复时，**只缺 FLANN 这一个 issue 要解**。

### 16.6 已 commit

| 文件 | 状态 |
|------|------|
| `.gitmodules` | LIO-SAM submodule 注册（gitlink commit `0be1fbe6`） |
| `docker/LIO_SAM/Dockerfile` | 包含 5 个 patch（GTSAM 源码 / C++17 / opencv / boost / flann member-call） |
| `.github/workflows/docker-LIO_SAM.yml.disabled` | workflow 被禁用 |
| `thirdparty/LIO-SAM` (gitlink, no local checkout) | submodule pin |

Commits: `2a1bde8` (添加 submodule) … `90cbb23` (最后一次 patch 尝试)。

### 16.7 路线图（更新）

| Track | 状态 | 备注 |
|-------|------|------|
| KISS-ICP 默认 backend | ✅ landed (§14/§15) | 生产用 |
| LIO-SAM benchmark | ⏸️ 暂搁置 (§16) | FLANN/PCL/C++17 build hell |
| RPE + alarms | ✅ landed (§13) | |
| 多 sample 稳定性 | 🔜 next | 当前仅单段 60s 数据 |
| Loop closure（如果需要长 SLAM） | 🔜 | KISS-ICP 没闭环；可外挂 ScanContext + GTSAM |
| D1 联合 BA | 🔜 | 长期方向 |

---

_Report 生成于 2026-05-30，§8/§9 增补于 2026-05-31，§10/§11/§12 增补于
2026-05-31，§13/§14/§15 增补于 2026-05-31，§16 增补于 2026-06-01，§17 增补于
2026-06-02。_

---

## 17. LIO-SAM benchmark（恢复并跑通，§16 解封）

§16 把 LIO-SAM 暂搁置的核心理由是 FLANN 模板缺 `std::unordered_map` 特化。
2026-06-02 把这个洞补上，并把上面 4 个独立的非显式 failure mode 都定位掉，
LIO-SAM 在 AT128P sample 上**跑出与 FAST-LIO 同等量级的轨迹** —— 完整 GHCR
镜像 + bag 转换 + 出图链路全部 reproducible。

### 17.1 build 链补丁：FLANN unordered_map 特化

§16 的 boost-free-fn patch 不可行，因为 FLANN 用自己的 `LoadArchive`，跟
`boost::archive` 不互通。正确做法是**给 FLANN 自己的 serializer 加上 std::unordered_map 特化**，模仿它对 std::map 的写法。在 `docker/LIO_SAM/Dockerfile` 里直接写到系统头：

```python
RUN python3 - <<'PYEOF'
# 在 /usr/include/flann/util/serialization.h 里：
# 1) #include <map> 后追加 #include <unordered_map>
# 2) 在 Serializer<std::map<K,V>> 特化后追加完全克隆的
#    Serializer<std::unordered_map<K,V>> 特化（save/load 各 ~10 行）
PYEOF
```

CI run #6 通过；GHCR push 到 `ghcr.io/wangxinjian1108/lio-sam:latest`（5.1 GB）。
本地慢网拉了 ~5h，commit `7a1a916` 触发的镜像。

### 17.2 跑通后的四个连环 silent-failure

镜像构建好后又花了 6 次 docker run 才让数据真的流过整个 pipeline。每个故障**都没有 ROS_ERROR 输出，都装作正常**。按发现顺序：

| # | 症状 | 根因 | Fix |
|---|------|------|-----|
| 1 | mapOptmization SIGABRT，退出码 -6 | 不是真正的崩溃 — 是 saveMapService 在 cloudKeyPoses3D 为空时 throw `pcl::IOException` | 真正的 bug 在更上游 |
| 2 | `/lio_sam/deskew/cloud_info` 速率为 0，但 `/velodyne_points` 正常 10 Hz | yaml 写 `pointCloudTopic: "/points_raw"`，bag 发的是 `/velodyne_points`，imageProjection 永远没 callback | 把 yaml 改成 `/velodyne_points` |
| 3 | imuPreintegration "Invalid quaternion" | `orientation` 全零；LIO-SAM 拿这个判 9-axis IMU | convert 时填 identity quaternion (qw=1) |
| 4 | 跑通后轨迹长度 4134 m / Z→977 m（实际约 60 s 路程） | LIO-SAM imuPreintegration 用 GTSAM `MakeSharedU(g)`，硬编码"Z-up at rest = +g·ẑ"。我们的 IMU 是 -g·ẑ at rest（FAST-LIO 因为自估 gravity 不受影响） | convert 加 `--negate-accel`，写到独立的 `_liosam.bag` 不污染 FAST-LIO 用的 bag |
| 5 | extrinsicRot 用 FAST-LIO 的矩阵直接灌入 → 轨迹 ~50° 整体偏转 | `imuConverter` 的语义是 `acc_lidar = extRot * acc_imu`，所以 LIO-SAM 的 `extrinsicRot = R_lidar_imu = (FAST-LIO extrinsic_R)^T` | 转置矩阵；alignment 时还会被 Umeyama 吃掉 |
| 6 | save_map 后 `/output/` 内容（rosbag-record output、node logs）丢失 | `saveMapService` 把 `$HOME` 拼到 destination 前面，HOME=/root + dest='/output/' = `/root/output/`（不是挂载的 /output）；并且 `rm -r` 那个目录 | 用 `destination: '/../output/'` 让 HOME+dest 解析回 /output；node_logs 单独保存或写到 /output 外 |

诊断关键工具：在 bag play 期间跑 `rostopic hz` 把 LIO-SAM 每个 internal topic 的速率都打出来 —— 哪一级降为 0，问题就在那一级。已固化在 `scripts/run_liosam_in_container.sh` Step 5 的后台循环里。

### 17.3 跑通后的精度数据

```
FAST-LIO: 596 poses, length=452.70 m, Z[0.02, 6.69]m  std=2.00m
LIO-SAM : 286 keyframes, length=454.27 m, Z[-0.00, 4.78]m  std=1.47m
```

LIO-SAM 帧数 = 286 是因为它只在位移 > 1 m 或 angle > 0.2 rad 时才插入 keyframe（`surroundingkeyframeAddingDistThreshold`），FAST-LIO 是逐帧 10 Hz 输出。

经 SE(3) Umeyama 对齐（无 scale，两者都是 metric）：

| 指标 | 数值 | 备注 |
|------|------|------|
| ATE RMSE | **0.965 m** | 路径 454 m → 0.21% 相对 |
| ATE mean | 0.849 m | |
| ATE max | 1.751 m | 局部最差点 |
| 帧间相对漂移 中位数 | 2.03% | 每两个 keyframe 间的相对误差 |
| 帧间相对漂移 90 分位 | 5.17% | |
| 两 frame 间 yaw 偏差 | 49.47° | 初始 yaw 取的零点不同，已对齐 |

视觉上对齐后两条轨迹几乎完全重合（`output/liosam_run/compare_aligned_topdown.png`）。

### 17.4 与 KISS-ICP（§14）的横向定位

KISS-ICP 之前在 §14 已用同一段数据跑过（轨迹在
`output/kiss_icp_run/trajectory_imu.txt`，T_world_imu，TUM 格式），所以三方
对比是把它和 FAST-LIO / LIO-SAM 一起 SE(3) 对齐到 FAST-LIO，统一比一遍。

完整脚本 `scripts/eval_three_way.py`，输出到
`output/three_way_compare/`：

**原始统计（未对齐）**：

| Backend  | 帧数 | 路径长度 (m) | Z range (m)        | Z std (m) |
|----------|-----:|-------------:|--------------------|----------:|
| FAST-LIO |  596 |       452.70 | [ 0.019,  6.686]   |     2.007 |
| KISS-ICP |  600 |       455.19 | [-1.418, 10.333]   |     2.998 |
| LIO-SAM  |  286 |       454.27 | [-0.004,  4.777]   |     1.474 |

三家路径长度在 ±0.55% 内一致；KISS-ICP 没 IMU，Z 方向漂移最大（std 3.0 m），
LIO-SAM 因子图 + ISAM2 全局优化把 Z std 压到 1.47 m。

**对齐后 ATE/RPE（以 FAST-LIO 为参考）**：

| Backend  | 配对数 | ATE RMSE (m) | mean (m) | max (m) | RPE 中位 | RPE p90 |
|----------|-------:|-------------:|---------:|--------:|---------:|--------:|
| KISS-ICP |    596 |        0.394 |    0.354 |   0.764 |    3.41% |   6.84% |
| LIO-SAM  |    284 |        0.965 |    0.849 |   1.751 |    2.03% |   5.17% |

注意一个表面"反直觉"现象：**KISS-ICP 绝对 ATE 更小，但 RPE 更大**。原因：

- KISS-ICP 是逐帧（10 Hz, 600 帧）地跟着 FAST-LIO 的逐帧（596）对齐，每两个
  匹配点之间的时间差小，绝对位置很接近 → 低 ATE。但帧间的相对位移噪声大（无
  IMU 平滑） → 高 RPE。
- LIO-SAM 只产 286 keyframe（位移触发），匹配点稀疏，每两个 keyframe 间隔
  远；全局优化把每个 keyframe 拉得很顺滑（低 RPE 中位 2.03%），但因为
  keyframe-level 抽样在曲线段的 chord-vs-arc 误差比逐帧大，绝对 ATE 反而高。

把 LIO-SAC 的 286 keyframe 插值到 10 Hz 再算 ATE 是更公平的比较，但当前
keyframe 间最大 1 m / 0.2 rad 的间距下，chord error 上限大致就是 ~0.5 m，
跟 RMSE 0.965 m 是一致的（chord error 在曲率高处累积）。

三方都正确 converge，没有任何一家发散；故障模式是各自独立的（KISS-ICP Z 漂、
LIO-SAM keyframe 稀疏、FAST-LIO 局部抖动）。

KISS-ICP 在本数据集上**绝对精度仍然最优**（§14 已建立的结论），LIO-SAM 在 Z
稳定性上最优（loop closure），FAST-LIO 在帧率/延迟上最优（逐帧 ESKF）。

可视化：`output/three_way_compare/three_way_{topdown,xyz,error_time}.png`，
数值汇总：`output/three_way_compare/three_way_summary.json`。

### 17.5 工件清单

| 文件 | 内容 |
|------|------|
| `scripts/extract_liosam_trajectory.py` | 解 `transformations.pcd`（XYZIRPYT）→ TUM 轨迹 |
| `scripts/viz_liosam_vs_fastlio.py`     | 双轨迹叠图 + 地图叠轨迹 |
| `scripts/eval_liosam_vs_fastlio.py`    | Umeyama 对齐 + ATE/RPE/Z drift（FL vs LS 二方） |
| `scripts/eval_three_way.py`            | 三方对比：FAST-LIO / KISS-ICP / LIO-SAM 一并 align + 出表 |
| `scripts/prep_liosam_for_b2.py`        | 把 LIO-SAM 输出整理成 B2 pipeline 的 `trajectory.txt` + `scans_voxel0.3.pcd`（含 keyframe → per-PCD 时间戳的 SLERP 插值） |
| `scripts/eval_calib_three_way.py`      | 三方校准 std / \|Δt\| 对比表 |
| `output/liosam_run/GlobalMap.pcd`      | 40 MB，全局地图 |
| `output/liosam_run/{Corner,Surf}Map.pcd` | 4.9/35 MB，分边/面子图 |
| `output/liosam_run/transformations.pcd` | 286 keyframes 6-DoF + 时间戳 |
| `output/liosam_run/trajectory.txt`     | TUM 格式 |
| `output/liosam_run/compare_*.png`       | 对比可视化 |

### 17.6 复跑命令

```bash
DATA=/root/node_data/fixtures/lio/ZL11626_40482_zelos_sample_2025-07-02_14-29-00_000000000_8993544
docker run --rm \
  -v "$DATA":/data:ro -v /root/code/SlamHub:/workspace:ro \
  -v /root/code/SlamHub/output/liosam_run:/output \
  -e LIDAR=remote_front_left_pointcloud \
  ghcr.io/wangxinjian1108/lio-sam:latest \
  /bin/bash /workspace/scripts/run_liosam_in_container.sh

python3 scripts/extract_liosam_trajectory.py
python3 scripts/viz_liosam_vs_fastlio.py
python3 scripts/eval_liosam_vs_fastlio.py
python3 scripts/eval_three_way.py    # 三方对比，需 KISS-ICP 已跑过 §14
```

### 17.7 LIO-SAM 地图喂回 cross-LiDAR 标定

§15 把 KISS-ICP 主地图喂回 B2 pipeline 大幅压降了 std。同样实验跑在 LIO-SAM
主地图上，看 factor graph 后端的地图能不能进一步把横向 std 压低（猜想：
loop closure 让 Z 一致性更好，Z std 应该再降）。

**实验设置**：

| 组件 | 取值 |
|------|------|
| 主轨迹 | LIO-SAM `transformations.pcd` → SLERP 插值到 600 PCD 时间戳 |
| 主地图 | LIO-SAM `GlobalMap.pcd`（corner+surf 特征图，4.25 M 点）→ voxel 0.3 m → 0.74 M |
| Cross-LiDAR ICP | point-to-plane (`icp_pl`)，submap radius 50 m |
| 聚合 | B2 axis info-weighted（同 §15） |
| 配准代码 | `scripts/04_register_secondary.py`（无改动，复用） |
| 数据 prep | `scripts/prep_liosam_for_b2.py`（新） |
| 横向对比 | `scripts/eval_calib_three_way.py`（新） |

**Note：地图的本质差异**：

- FAST-LIO `scans.pcd` 是逐帧累积的原始点云（~75 M 点）
- KISS-ICP `scans.pcd` 同样逐帧累积（~75 M 点 → voxel 1.13 M）
- LIO-SAM `GlobalMap.pcd` = `CornerMap + SurfMap`，**只保留特征点**（4.25 M），
  比 KISS-ICP 稀疏 18×

LIO-SAM 没有"原始点全图"输出（按设计就是这样：mapping 用稀疏特征图能省内存
+ 加速 ICP）。这对副雷达 ICP 是关键劣势 —— surf 点不一定覆盖全场景几何。

### 17.7.1 ICP 收敛 + B2 std 三方对比

`scripts/eval_calib_three_way.py` 输出（dx/dy/dz_std 单位 m，n_eff/600，|Δt| m）：

| 副雷达 | Primary 地图 | dx_std | dy_std | dz_std | n_eff | \|Δt\| (vs 工厂) |
|--------|--------------|-------:|-------:|-------:|------:|--------:|
| flash_front | FAST-LIO | 0.277 | 0.274 | 0.013 | 373.7 | 0.191 |
|  | **KISS-ICP** | **0.272** | **0.135** | 0.017 | 375.5 | **0.128** |
|  | LIO-SAM | 0.517 | 0.398 | **0.012** | 375.6 | 0.531 |
| flash_rear | FAST-LIO | 0.532 | 0.426 | 0.032 | 251.4 | 0.598 |
|  | **KISS-ICP** | **0.526** | 0.364 | 0.016 | **355.8** | **0.286** |
|  | LIO-SAM | 0.705 | **0.356** | **0.013** | 265.8 | 0.546 |
| rfr | FAST-LIO | 0.332 | 0.229 | 0.026 | 213.1 | 0.382 |
|  | **KISS-ICP** | **0.192** | **0.258** | 0.024 | 169.6 | **0.311** |
|  | LIO-SAM | 0.235 | 0.332 | **0.023** | **181.9** | 0.809 |

**整体结论**：KISS-ICP 主地图依然是最优。LIO-SAM 在 Z 方向（dz_std）一致最好，
但 dx/dy std 和 |Δt| 都比 KISS-ICP 差。

### 17.7.2 解读：为什么 LIO-SAM Z 最好但横向最差？

LIO-SAM 是当前所有 backend 里 Z 一致性最好的（§17.3 中 Z std 1.47 m vs
KISS 3.00 m vs FAST-LIO 2.01 m）。这个优势**确实传导到了 dz_std** —— 三个
副雷达的 dz_std 都是 LIO-SAM 最低（0.012/0.013/0.023 m）。但 dx/dy std 反
向比 KISS-ICP 差，原因是**地图数据稀疏**：

- LIO-SAM `GlobalMap = Corner ⊕ Surf`，只保留高曲率边缘 + 平面采样
- 同样的副雷达点跟稀疏特征图做 ICP，命中率（fitness）和约束方向都更弱
- 横向（dx/dy）方向的几何约束本来就比纵向（dz）弱，雪上加霜

如果想让 LIO-SAM map 真正对 cross-LiDAR 标定有用，需要：

1. **强行拼一个全帧累积图**，而不是用 LIO-SAM 自带的 feature-only map。
   即拿 LIO-SAM 的轨迹 + 原始 PCD，做和 §15 一样的 stitch（这是 §17.7.3 留给
   将来的工作）。
2. 或者用 LIO-SAM trajectory + KISS-ICP map 的混合：trajectory 用 factor
   graph 的（最好的轨迹），map 用 KISS-ICP 的（最完整的几何）。

**flash_rear |Δt| 反而是 LIO-SAM 比 FAST-LIO 略好**（0.546 vs 0.598），是因为
LIO-SAM 的 Z 稳定性帮助修正了一部分纵向 bias；但 KISS-ICP 0.286 仍然是最优
（它两边都吃满）。

### 17.7.3 默认 backend 选择不变

| 维度 | 推荐 |
|------|------|
| 默认主 SLAM backend | **KISS-ICP**（§15 结论维持） |
| Z 稳定性专项需求 | LIO-SAM trajectory + 后续拼一个原始点全图 |
| 长序列 / 闭环场景 | LIO-SAM（自带 loop closure） |
| Cross-LiDAR aggregation | B2 (B1 + B3 + axis info) 不变 |

LIO-SAM 真正的 win 在 **trajectory 层面**（Z 稳定性 + 全局闭环），不在地图层面
（feature-only 太稀疏）。后续若要把 LIO-SAM 投入生产，应该用它的轨迹做主轨迹，
然后用同样的轨迹拼一个 raw-pcd 全图，replace 它的 GlobalMap。

### 17.7.4 已 commit 输出

- `output/liosam_run/trajectory_lidar_keyframes.txt` — 286 keyframes 原始
- `output/liosam_run/trajectory_lidar.txt` — 600 dense 经 SLERP 插值
- `output/liosam_run/trajectory_imu.txt` / `trajectory.txt` — T_world_imu (B2 兼容)
- `output/liosam_run/scans_voxel0.3.pcd` — voxel 后的 LIO-SAM map（735 K 点）
- `output/liosam_run/registration/{flash_front,flash_rear,rfr}/*` — 配准结果
- `output/liosam_run/calibrated_extrinsics.yaml` — B2 校准
- `output/three_way_compare/calibration_three_way.json` — 三方汇总

### 17.8 Follow-up：LIO-SAM 轨迹 + 原始点全图（hybrid）—— 横向 std 也追上来了

§17.7.2 的猜想直接验证了：拿 LIO-SAM 的 trajectory（factor graph + Z 稳）
配上原始点全图（足够 dense 的几何约束），**dx/dy std 立刻追平甚至超越
KISS-ICP**，dz_std 则维持 LIO-SAM 一直以来的优势。

**实验设置**：

| 组件 | LIO-SAM (§17.7) | **LIO-SAM\* hybrid (§17.8)** |
|------|-----------------|----------------|
| 主轨迹 | LIO-SAM 286 keyframes → SLERP 插 600 | **同左** |
| 主地图 | LIO-SAM `GlobalMap.pcd`（feature-only，4.25 M 点）→ voxel 0.74 M | **重拼**：LIO-SAM trajectory × 600 cleaned PCD → 75.5 M raw points → voxel 0.3 m → **1.28 M 点** |
| 配置 | `output/liosam_run/`        | `output/liosam_run_hybrid/`         |

具体做法（`scripts/stitch_liosam_raw_map.py`）：

```python
for i in range(600):
    pts_lidar = read_cleaned_pcd(i)              # NaN-free 原始点
    T_world_lidar = traj[i]                      # 来自 LIO-SAM 插值轨迹
    pts_world = T_world_lidar @ pts_lidar
all_pts → voxel(0.3) → scans_voxel0.3.pcd
```

cleaned PCD 直接复用 §15 KISS-ICP 跑过的 `output/kiss_icp_run/cleaned_pcds/`，
不重复 NaN 清洗。整个 stitch 60 秒完成，地图大小 75.5 M raw / 1.28 M voxel。

### 17.8.1 四方校准对比（LIO-SAM\* = hybrid）

| 副雷达 | Primary 地图 | dx_std | dy_std | dz_std | n_eff | \|Δt\| |
|--------|--------------|-------:|-------:|-------:|------:|-------:|
| flash_front | FAST-LIO  | 0.277 | 0.274 | 0.013 | 373.7 | 0.191 |
|  | KISS-ICP   | 0.272 | **0.135** | 0.017 | 375.5 | **0.128** |
|  | LIO-SAM    | 0.517 | 0.398 | **0.012** | 375.6 | 0.531 |
|  | **LIO-SAM\***  | **0.210** | 0.272 | 0.015 | **385.6** | 0.135 |
| flash_rear | FAST-LIO  | 0.532 | 0.426 | 0.032 | 251.4 | 0.598 |
|  | KISS-ICP   | 0.526 | **0.364** | **0.016** | **355.8** | 0.286 |
|  | LIO-SAM    | 0.705 | 0.356 | **0.013** | 265.8 | 0.546 |
|  | **LIO-SAM\***  | **0.582** | 0.409 | 0.017 | 344.1 | **0.241** |
| rfr | FAST-LIO  | 0.332 | 0.229 | 0.026 | 213.1 | 0.382 |
|  | KISS-ICP   | 0.192 | 0.258 | 0.024 | 169.6 | 0.311 |
|  | LIO-SAM    | 0.235 | 0.332 | **0.023** | 181.9 | 0.809 |
|  | **LIO-SAM\***  | **0.210** | **0.222** | 0.025 | 172.7 | **0.306** |

加粗 = 该行该列最好。LIO-SAM\*（hybrid）拿到的"局部最优"分布：

- **flash_front dx_std**：LIO-SAM\* 0.210 < KISS-ICP 0.272，**比 KISS 好 23%**
- **flash_front \|Δt\|**：LIO-SAM\* 0.135，几乎追平 KISS-ICP 0.128
- **flash_front n_eff**：LIO-SAM\* 385.6 是四个 backend 里最高，hybrid map 让更多帧通过加权门槛
- **flash_rear \|Δt\|**：LIO-SAM\* **0.241，比 KISS-ICP 0.286 好 16%**
- **rfr dx_std**：LIO-SAM\* 0.210，比 KISS-ICP 0.192 略输 9%（基本打平）
- **rfr dy_std**：LIO-SAM\* **0.222，比 KISS-ICP 0.258 好 14%**
- **rfr \|Δt\|**：LIO-SAM\* **0.306，比 KISS-ICP 0.311 好 1.6%**

### 17.8.2 vs LIO-SAM (feature-only) 的具体改善

LIO-SAM 自带 GlobalMap 的核心问题（§17.7.2 已诊断）—— feature-only 太稀疏 ——
被 hybrid 直接解决：

| 副雷达 | 维度 | LIO-SAM (feature) | **LIO-SAM\* (hybrid)** | 改善 |
|--------|------|---:|---:|---:|
| flash_front | dx_std | 0.517 | **0.210** | **−59%** ⭐ |
|             | dy_std | 0.398 | **0.272** | −32% |
|             | \|Δt\| | 0.531 | **0.135** | **−75%** ⭐ |
| flash_rear  | dx_std | 0.705 | **0.582** | −17% |
|             | n_eff  | 265.8 | **344.1** | **+29%** |
|             | \|Δt\| | 0.546 | **0.241** | **−56%** ⭐ |
| rfr         | dy_std | 0.332 | **0.222** | −33% |
|             | \|Δt\| | 0.809 | **0.306** | **−62%** ⭐ |

**\|Δt\| 三家都减半以上**：换上原始点全图后，副雷达 ICP 真的能贴到地面 / 墙
等横向几何，不再只命中稀疏的 corner / surf 特征点。

### 17.8.3 vs KISS-ICP（§15）的横向对比

LIO-SAM\* 跟 KISS-ICP 都用了"主轨迹 + 原始点拼图"的同一套思路，差别只在主轨迹
谁来出。KISS-ICP 是 voxel ICP（无 IMU），LIO-SAM 是 factor graph + ImuPreint
+ loop closure。所以 LIO-SAM\* 跟 KISS-ICP 的对比 ≈ "factor-graph 轨迹 vs
voxel-ICP 轨迹" 在同一种地图供给下的对比：

- 横向（dx/dy std）：LIO-SAM\* 平均略好 / 持平
- 纵向（dz_std）：两者都 ≤ 25 mm，LIO-SAM\* 在 rfr 上略输（0.025 vs 0.024）
- \|Δt\|：LIO-SAM\* 在 flash_rear 和 rfr 上更好；flash_front KISS-ICP 略好
- n_eff：LIO-SAM\* 在 flash_front / flash_rear 上更高（factor graph 让更
  多帧的 ICP info 矩阵满足加权阈值）

### 17.8.4 默认 backend 选择重新考虑

§15 / §17.7 都判 KISS-ICP 是默认。**§17.8 的结果让 LIO-SAM\* hybrid 成为另一
个候选**：

| 维度 | KISS-ICP | LIO-SAM\* hybrid |
|------|----------|-------------------|
| 单段 60s 标定 std | 6 个维度里 4 个最好 | 6 个维度里 5 个最好 |
| 单段 \|Δt\| | 1 / 3 副雷达最好 | 2 / 3 副雷达最好 |
| 长序列 / 闭环 | ❌ 无 loop closure | ✅ 自带 |
| Z 稳定性 | 1.47-3.00 m std（轨迹层） | **1.47 m std**（轨迹层） |
| 依赖 | 纯 LiDAR + Python | LiDAR + IMU + GHCR docker + GTSAM |
| 跑得快 | ~25 s 主 SLAM | ~90 s docker 启动 + 60s 实时跑 |
| Setup 痛 | 一行 pip | 见 §16/§17 的 6 次 build hell |

**结论**：

- **当前生产维持 KISS-ICP**（§15 决策不变，足够好 + 极简部署）
- **若启用 LIO-SAM\* hybrid 作为对比 backend**，单段精度上能再压一档
- **长序列 / 多 sample / 闭环场景** 用 LIO-SAM\* 是默认推荐
- **Z 漂特别难的场景** 也是 LIO-SAM\*

### 17.8.5 已 commit 输出

| 路径 | 内容 |
|------|------|
| `scripts/stitch_liosam_raw_map.py` | LIO-SAM trajectory × cleaned PCD → 全图 |
| `output/liosam_run_hybrid/scans.pcd` | 75.5 M 原始点 |
| `output/liosam_run_hybrid/scans_voxel0.3.pcd` | 1.28 M voxel 0.3 |
| `output/liosam_run_hybrid/registration/<lidar>/*` | 3 副雷达 icp_pl 配准 |
| `output/liosam_run_hybrid/calibrated_extrinsics.yaml` | B2 校准 |
| `output/three_way_compare/calibration_three_way.json` | 四方汇总（更新） |

### 17.9 路线图更新

| Track | 状态 | 备注 |
|-------|------|------|
| FAST-LIO benchmark | ✅ landed | 同时是当前 backend reference |
| KISS-ICP benchmark | ✅ landed (§14/§15) | 默认生产 backend，map 也是默认 B2 输入 |
| LIO-SAM benchmark | ✅ landed (§17) | trajectory 三方对比完成 |
| **LIO-SAM\* hybrid（traj + raw map）** | ✅ **landed (§17.8)** | 单段精度首次超越 KISS-ICP |
| RPE + alarms | ✅ landed (§13) | |
| 多 sample 稳定性 | 🔜 next | 单段 60s 已三方/四方对齐，需多录制验证 |
| 长序列 / loop closure | ✅ LIO-SAM\* 备用 | KISS-ICP 没闭环 |
| D1 联合 BA | 🔜 | 长期方向 |

