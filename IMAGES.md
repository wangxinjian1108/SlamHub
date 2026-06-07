# SlamHub Images

> 自动生成，请勿手动编辑。运行 `list-images` skill 刷新。

## ROS1 C++ Backends

| 项目 | Docker 镜像 | 算法类型 | 论文 | 备注 |
|------|------------|---------|------|------|
| FAST_LIO | `ghcr.io/wangxinjian1108/fast-lio:latest` | ESKF + IKD-Tree LIO | [RA-L 2022](https://arxiv.org/abs/2107.06829) | 实时 LiDAR-Inertial Odometry，紧耦合 ESKF |
| LIO_SAM | `ghcr.io/wangxinjian1108/lio-sam:latest` | GTSAM factor graph + ImuPreint + loop closure | [IROS 2020](https://arxiv.org/abs/2007.00258) | 紧耦合 LiDAR-IMU SLAM，闭环优化 |
| FAST_LIVO2 | `ghcr.io/wangxinjian1108/fast-livo2:latest` | ESKF LIO + 直接法 VIO（patch-based） | [RA-L 2024](https://arxiv.org/abs/2408.14035) | LiDAR-Visual-Inertial Odometry，相机畸变内置（Pinhole 4 系数 / Fisheye 4 系数）|

## Pure-Python Backends

| 项目 | Docker 镜像 | 算法类型 | 论文 | 备注 |
|------|------------|---------|------|------|
| genz-icp | `ghcr.io/wangxinjian1108/genz-icp:latest` | adaptive-weighted voxel ICP | [RA-L 2025](https://arxiv.org/abs/2411.06766) | 退化场景鲁棒（走廊 / 隧道）|
| mad-icp | `ghcr.io/wangxinjian1108/mad-icp:latest` | matching-data voxel ICP（minimal）| [RA-L 2024](https://ieeexplore.ieee.org/document/10669999) | 跨样本 \|Δt\| 最稳，§22 默认 backend |

## 共 5 个镜像（3 ROS1 + 2 Pure-Python）

KISS-ICP（pip-installable, 无 docker 镜像）也作为 backend 内置在 `scripts/run_kiss_icp.py`，性能基准见 `docs/reports/2026-05-30-fastlio-at128p-evaluation.md` §14。
