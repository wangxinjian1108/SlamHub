---
name: add-repo
description: Add a new SLAM repo as a git submodule to the SlamHub benchmarking hub, generate its Dockerfile + GitHub Actions image workflow, update the image index, commit, push, and watch CI. Use when the user gives a repo SSH/HTTPS URL or asks for the full add-repo pipeline.
---

# Add Repo

Use this skill when the user wants the full onboarding flow for a new SLAM/LiDAR-odometry submodule.

## Inputs

- A git SSH or HTTPS URL such as `git@github.com:wangxinjian1108/LIO-SAM.git` or `https://github.com/cocel-postech/genz-icp.git`

## Repo assumptions

- Main repo branch is `master`
- Submodules live under `thirdparty/<name>`
- Dockerfiles live under `docker/<name>/Dockerfile`
- Image workflows live under `.github/workflows/docker-<name>.yml`
- Image names: lowercase + dash form under `ghcr.io/wangxinjian1108/<name>`
  (`FAST_LIO` → `fast-lio`, `LIO_SAM` → `lio-sam`, `genz-icp` stays `genz-icp`)
- SlamHub does NOT have a separate dev-image pattern; one runtime image per model is enough

## Guardrails

- Read `AGENTS.md` first if it exists
- Never stage secrets such as `.env`, `config/token.yaml`, or large binaries
- Do not revert unrelated user changes in the worktree
- For repos that need patches (most ROS C++ SLAM), prefer the user fork
  (`wangxinjian1108/<name>`) over upstream so we can push patches in

## Workflow

1. Resolve `<name>` from the URL by taking the last path segment without `.git`.
2. Check `.gitmodules` and `thirdparty/<name>`.
3. If the submodule already exists, update with
   `git submodule update --init --remote thirdparty/<name>`.
4. If it does not exist:
   - **Normal network**: `git submodule add <url> thirdparty/<name>` then
     `git submodule update --init --recursive thirdparty/<name>`.
   - **Slow / blocked network** (clone times out): write `.gitmodules`
     manually + register the gitlink without local checkout:
     ```
     git config -f .gitmodules submodule.thirdparty/<name>.path thirdparty/<name>
     git config -f .gitmodules submodule.thirdparty/<name>.url <url>
     git update-index --add --cacheinfo 160000,<sha>,thirdparty/<name>
     ```
     Get `<sha>` from `gh api repos/<owner>/<name>/git/refs/heads/<default-branch>`.
     CI runner pulls the actual code via `submodules: recursive` checkout.
5. Generate `docker/<name>/Dockerfile` following the dockerize-submodule
   workflow (template A for ROS C++ SLAM, template B for pip-installable
   pure-Python LiDAR odometry).
6. Generate one workflow at `.github/workflows/docker-<name>.yml`.
7. Refresh `IMAGES.md` by following the list-images workflow.
8. Stage the submodule gitlink, `.gitmodules`, Dockerfile, workflow, and
   `IMAGES.md`.
9. Commit with `feat: add <name> Dockerfile and CI workflow` unless the
   user asked for different wording.
10. Push to master.
11. Watch the triggered CI run. If it fails, inspect failed logs, fix
    Dockerfile/workflow issues, amend, force-push, retry up to 3 rounds.
12. Optionally wire the new backend into `scripts/run_all_backends.sh` and
    add a `run_<name>_in_container.sh` (ROS docker) or `run_<name>.py`
    (pure Python) launcher that follows the existing pattern. Do not
    auto-run the benchmark — that's the run-benchmark skill.

## Dockerization requirements

- Build context must be the repo root (so all submodules are available)
- Copy source from local submodules with `COPY thirdparty/<name> ...`,
  never `git clone` during image build
- Prefer the existing SlamHub Dockerfiles as style references:
  - ROS C++ SLAM: `docker/FAST_LIO/Dockerfile`, `docker/LIO_SAM/Dockerfile`
  - Pure-Python: `docker/genz_icp/Dockerfile`, `docker/mad_icp/Dockerfile`
- Workflow reference: `.github/workflows/docker-LIO_SAM.yml`
- Build the runtime image with `docker/build-push-action@v5` using
  `push: ${{ github.event_name != 'pull_request' }}` — push streamed to
  GHCR. Tags include `latest` and `${{ github.sha }}`.
- Don't create a separate dev image — SlamHub doesn't use that pattern

## Done criteria

- Submodule gitlink exists at `thirdparty/<name>`
- Dockerfile and workflow exist and are coherent
- `IMAGES.md` lists the new image with correct algorithm category
- Changes are committed and pushed to `master`
- CI passed (or remaining blockers called out explicitly)
- (Optional) `scripts/run_all_backends.sh` knows about the new backend
