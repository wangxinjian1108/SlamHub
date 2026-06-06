罗列所有已推送到 GitHub Container Registry 的 SLAM Docker 镜像及其对应论文/算法分类，以表格形式输出到 `IMAGES.md`。

## Steps

1. **收集镜像信息** — 扫描 `.github/workflows/docker-*.yml` 文件，提取每个镜像名称（在 `tags:` 块的 `ghcr.io/wangxinjian1108/<image-name>:latest`）。注意：image-name 通常是 dockerfile 目录名的小写连字符形式（如 `FAST_LIO` → `fast-lio`，`LIO_SAM` → `lio-sam`）。

2. **收集 backend 类型** — 对每个 `docker/<name>/Dockerfile`，根据 `FROM` 行 + 文件内容判断算法类型：
   - **ROS1 C++ SLAM**：`FROM ros:noetic-*` —— 标记 `algorithm = "ROS1 C++ + IMU/LiDAR factor graph or ESKF"`
   - **ROS2 C++ SLAM**：`FROM ros:humble-*` —— 标记 `algorithm = "ROS2 C++"`
   - **Pure-Python LiDAR odometry**：`FROM ubuntu:22.04` + `pip install` —— 标记 `algorithm = "voxel ICP"`（具体读 README 看是 KISS / GenZ adaptive-weighted / MAD matching-data 等等）

3. **收集论文信息** — 对每个镜像对应的 submodule（`thirdparty/<name>`），读取其 `README.md` 提取：
   - 论文标题
   - arXiv / DOI / IEEE Xplore 链接（若有）
   - 算法核心特征一句话（不超过 30 字）

4. **生成表格** — 写入 `IMAGES.md`，6 列，按 backend 类型分组（ROS1 → ROS2 → Pure-Python），组内按字母排序：

   ```markdown
   # SlamHub Images

   > 自动生成，请勿手动编辑。运行 `list-images` skill 刷新。

   ## ROS1 C++ Backends

   | 项目 | Docker 镜像 | 算法类型 | 论文 | 备注 |
   |------|------------|---------|------|------|
   | FAST_LIO | `ghcr.io/wangxinjian1108/fast-lio:latest` | ESKF + IKD-Tree | [RA-L 2022](https://...) | 实时 LiDAR-Inertial Odometry |

   ## ROS2 C++ Backends
   ...

   ## Pure-Python Backends

   | 项目 | Docker 镜像 | 算法类型 | 论文 | 备注 |
   |------|------------|---------|------|------|
   | GenZ-ICP | `ghcr.io/wangxinjian1108/genz-icp:latest` | adaptive-weighted voxel ICP | [RA-L 2025](https://...) | 退化场景鲁棒 |
   ```

   - 如果某个 submodule 没有独立论文，备注列标注"无独立论文，参考 [<related>](url)"
   - 文件末尾附一行统计：共 N 个镜像，分别在 X/Y/Z 类

5. **更新 README.md links（可选）** — 如果 SlamHub 顶层 `README.md` 引用了 IMAGES.md 的 backend 列表，扫描 README 并刷新引用。

6. **输出确认** — 打印 `IMAGES.md updated`，列出本次新增/移除/重命名的镜像，以及镜像总数。
