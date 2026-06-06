---
name: list-images
description: Rebuild IMAGES.md for the SlamHub repository by scanning Docker workflows, Dockerfiles, and submodule READMEs to produce a per-backend image + algorithm + paper index table. Use when the image catalog needs to be refreshed.
---

# List Images

Use this skill when the user asks to regenerate `IMAGES.md`.

## Workflow

1. Scan `.github/workflows/docker-*.yml` to identify all published runtime image names.
   The image tag is whatever appears in `tags:` under `ghcr.io/wangxinjian1108/<image-name>:latest`.
2. For each image, inspect `docker/<name>/Dockerfile` to determine algorithm category:
   - `FROM ros:noetic-*` → ROS1 C++ SLAM (typically IMU+LiDAR fusion: ESKF, factor graph)
   - `FROM ros:humble-*` → ROS2 C++ SLAM
   - `FROM ubuntu:22.04` + `pip install` → pure-Python LiDAR odometry (voxel ICP family)
3. Inspect the corresponding submodule README under `thirdparty/<name>` and extract:
   - paper title (e.g. "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry")
   - arXiv / IEEE Xplore / DOI link
   - a one-line algorithm-feature description (≤30 chars, e.g. "ESKF + IKD-Tree", "adaptive-weighted voxel ICP")
4. Generate `IMAGES.md` grouped by algorithm category, alphabetic within group:

   ```markdown
   # SlamHub Images

   > 自动生成，请勿手动编辑。运行 `list-images` skill 刷新。

   ## ROS1 C++ Backends
   | 项目 | Docker 镜像 | 算法类型 | 论文 | 备注 |
   |------|------------|---------|------|------|

   ## ROS2 C++ Backends
   | 项目 | Docker 镜像 | 算法类型 | 论文 | 备注 |

   ## Pure-Python Backends
   | 项目 | Docker 镜像 | 算法类型 | 论文 | 备注 |
   ```

5. If the project has no standalone paper, state that plainly in the paper column with a reference to the closest related work.
6. Add a final stat line: `共 N 个镜像，分别在 X / Y / Z 类`.

## Guardrails

- Prefer repo files over memory
- Keep existing formatting style if `IMAGES.md` already exists
- Do not invent paper metadata when the README is ambiguous; mark it as unknown instead
- Don't include backends that exist as scripts but have no Docker image (e.g. KISS-ICP if it's pure-pip with no container)

## Done criteria

- `IMAGES.md` is updated
- The total image count matches the number of discovered workflows or the gap is explained
- Each backend has a concrete algorithm category (not just "C++ SLAM")
