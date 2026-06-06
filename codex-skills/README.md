# codex-skills

SlamHub 的 Codex skills，覆盖从添加 SLAM repo、构建镜像、到 benchmark、审查、通知的全流程。**适配自 [VRecHub](../VRecHub/codex-skills/)**，但核心栈从 CUDA + conda + Python ML training 改成 ROS C++ + cmake + pure-Python LiDAR odometry。每个 skill 都有：

- `<name>/SKILL.md` — skill 主文档（Codex 读这个文件）
- `<name>/agents/openai.yaml` — interface 元数据（display_name 等）

## Skill 索引

| Skill | 用途 |
|-------|------|
| [add-repo](add-repo/SKILL.md) | 给 SlamHub 加新的 SLAM submodule，自动 dockerize + 写 CI workflow + 监控 build |
| [dockerize-submodule](dockerize-submodule/SKILL.md) | 给已有 submodule 生成 Dockerfile + workflow（template A: ROS C++，template B: pip-installable Python）|
| [list-images](list-images/SKILL.md) | 扫 workflows + Dockerfiles + READMEs，重建 IMAGES.md 索引（按算法类别分组）|
| [run-benchmark](run-benchmark/SKILL.md) | **SlamHub 专属** — 跑多 backend SLAM + cross-LiDAR 标定 benchmark，输出 per-cell \|Δt\| 表 |
| [update-submodule](update-submodule/SKILL.md) | 把 submodule 升到 upstream HEAD，可选触发镜像 rebuild + 重跑 benchmark |
| [ship](ship/SKILL.md) | Commit + push + 开 PR（如果不在 master）+ 监控 CI 最多 3 次修复 |
| [review](review/SKILL.md) | Code review，bug-finding 优先 |
| [security-review](security-review/SKILL.md) | Security review，secret 泄露 / supply chain / 容器配置 |
| [simplify](simplify/SKILL.md) | 减少复杂度而不改变行为 |
| [notify](notify/SKILL.md) | 输出简洁状态通知（Slack 等）|

## 为啥 SlamHub 跟 VRecHub 不同

- VRecHub 的 docker 模板 = CUDA + conda + PyTorch ML training；SlamHub = ROS C++ / cmake (FAST_LIO, LIO-SAM) 或 pip-installable LiDAR odometry (KISS-ICP, GenZ-ICP, MAD-ICP)
- VRecHub 用 train-test 跑一个 epoch sanity；SlamHub 用 run-benchmark 跑 6-backend × 7-sample 多样本 benchmark + B2 cross-LiDAR 标定
- VRecHub 镜像分 runtime 和 dev image 两个 tag；SlamHub 只发 runtime（每个 backend 一个）
- IMAGES.md schema 也不同：VRecHub 是 `项目 / Docker 镜像 / CUDA / PyTorch / 论文 / 描述`；SlamHub 按算法类别分组（ROS1 / ROS2 / Pure-Python），列是 `项目 / Docker 镜像 / 算法类型 / 论文 / 备注`

## 使用方式

Codex 自动读 SKILL.md。手动调用：

```bash
$ codex run --skill add-repo --arg "https://github.com/cocel-postech/genz-icp.git"
```

或者从 Claude Code 触发对应 `.claude/commands/<skill>.md` 版本（slightly different format，但语义对齐）。
