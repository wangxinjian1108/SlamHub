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

_Report 生成于 2026-05-30。_
