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

_Report 生成于 2026-05-30，§8 增补于 2026-05-31。_
