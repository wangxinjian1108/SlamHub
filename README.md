# SlamHub

SLAM / 定位 / 建图相关开源模型的容器化中心。每个 model 以 submodule 形式接入 `thirdparty/`，自动生成 runtime + dev 双镜像并推到 GHCR。所有镜像目录见 [IMAGES.md](IMAGES.md)。

## 镜像架构

Base 镜像（`docker/base/`）：

| Base | 用途 | Tag |
|------|------|-----|
| `Dockerfile.cuda` | 纯视觉 SLAM、深度学习方法 | `ghcr.io/wangxinjian1108/slamhub-base:cuda` |
| `Dockerfile.cuda-ros1` | 需要 ROS1 Noetic 的经典 LiDAR SLAM 方法 | `ghcr.io/wangxinjian1108/slamhub-base:cuda-ros1` |
| `Dockerfile.cuda-ros2` | 需要 ROS2 Humble 的新方法 | `ghcr.io/wangxinjian1108/slamhub-base:cuda-ros2` |

每个方法的镜像分两类：
- **Runtime 镜像** `ghcr.io/wangxinjian1108/<name>:latest` — 烧好 conda env、依赖、模型权重，能直接跑 inference。
- **Dev 镜像** `ghcr.io/wangxinjian1108/<name>:dev` — 在 runtime 上加 JupyterLab + sshd（s6 管），用于 K8s Notebook 部署。

## 目录结构

```
SlamHub/
├── config/          # token、密钥配置
├── docker/
│   ├── base/        # base 镜像（CUDA-only / CUDA+ROS2）
│   └── <model>/    # 每个方法的 Dockerfile
├── thirdparty/      # 开源方法（git submodule）
├── scripts/         # 数据转换脚本（KITTI/TUM/EuRoC/NuScenes 互转）
├── .github/workflows/  # CI/CD
└── codex-skills/    # 工程化 skill
```

## 数据格式支持

scripts 目录提供常见 SLAM 数据集格式之间的转换：
- KITTI
- TUM RGB-D
- EuRoC MAV
- NuScenes
- Waymo Open Dataset
- 自定义格式

## 快速开始

### 新增一个方法

```bash
# 添加 submodule
git submodule add <repo-url> thirdparty/<name>

# 创建 Dockerfile（选择合适的 base）
mkdir docker/<name>
# 编写 docker/<name>/Dockerfile

# 生成 workflow
# 使用 add-repo skill 或手动创建 .github/workflows/docker-<name>.yml
```
