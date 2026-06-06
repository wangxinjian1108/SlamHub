Add a SLAM repo as a git submodule, dockerize it, create a GitHub Actions workflow, commit, push, and monitor CI until success.

## Argument
`$ARGUMENTS` is a git SSH or HTTPS URL (e.g. `https://github.com/cocel-postech/genz-icp.git` or `git@github.com:wangxinjian1108/LIO-SAM.git`).

## Steps

1. **Add or update submodule**
   - Extract `<name>` from the URL (last path segment without `.git`)
   - Check if `thirdparty/<name>` already exists in `.gitmodules`:
     - If yes: run `git submodule update --init --remote thirdparty/<name>` to pull the latest commit
     - If no: there are two options depending on local network state:
       - **Normal**: `git submodule add <url> thirdparty/<name>` then `git submodule update --init --recursive thirdparty/<name>`
       - **Slow / blocked network** (we can't clone the repo locally): write `.gitmodules` manually + register the gitlink without checkout:
         ```
         git config -f .gitmodules submodule.thirdparty/<name>.path thirdparty/<name>
         git config -f .gitmodules submodule.thirdparty/<name>.url <url>
         git update-index --add --cacheinfo 160000,<sha>,thirdparty/<name>
         ```
         Get `<sha>` via `gh api repos/<owner>/<name>/git/refs/heads/<default-branch>`. CI runner pulls the actual code via `submodules: recursive` checkout.

2. **Optional: prefer user fork** — if the user has not already forked, ask via AskUserQuestion. SlamHub's convention (mirroring §17/§18/§19 of the eval report): user-forks for repos we expect to patch (LIO-SAM, GenZ-ICP, MAD-ICP, FAST_LIO have all been forked); upstream for those we just consume (livox_ros_driver, ikd-Tree). If the user has a fork at `https://github.com/wangxinjian1108/<name>`, point `.gitmodules` at it instead.

3. **Dockerize** — invoke the `/dockerize-submodule` skill with `thirdparty/<name>` as the argument. This handles:
   - Detecting project type (ROS1 C++ vs ROS2 C++ vs pure-Python)
   - Picking the right Dockerfile template (template A for ROS, template B for pip-installable)
   - Source-build heavy deps (GTSAM, ceres, etc.) when needed
   - Apply known patches (C++17 standard, OpenCV header, FLANN serialization)
   - Creating `docker/<name>/Dockerfile` + `.github/workflows/docker-<name>.yml`

4. **Commit & push**
   - Stage: the new submodule gitlink (`thirdparty/<name>`), `.gitmodules`, `docker/<name>/Dockerfile`, `.github/workflows/docker-<name>.yml`
   - Do NOT stage `.env`, `config/token.yaml`, secrets, or large binaries
   - Commit message: `feat: add <name> Dockerfile and CI workflow`
   - Push to master: `git push`

5. **Monitor CI (iterate up to 3 times on failure)**
   - Find the triggered workflow run for `Docker – <name>`
   - Wait for it to complete
   - If success → report and stop
   - If failure:
     a. Fetch logs: `gh run view <run-id> --log-failed`
     b. Diagnose. Common failures specific to SLAM Dockerfiles:
        - "PCL requires C++14+" → add `sed -i 's/c++11/c++17/' CMakeLists.txt`
        - "opencv/cv.h: No such file" → `sed -i 's|opencv/cv.h|opencv2/opencv.hpp|'`
        - GTSAM not in apt → add source-build block (4.0.3 is the LIO-SAM-tested version)
        - FLANN unordered_map → see `docker/LIO_SAM/Dockerfile`'s Python script patch
        - mad-icp pip install needs ninja → `pip install ninja` first
        - typer/click incompat → `pip install "click<8.2"`
     c. Fix in Dockerfile, amend, push: `git add docker/<name>/Dockerfile && git commit --amend --no-edit && git push --force-with-lease`
     d. Wait for new run
   - After 3 failed attempts, stop and report what's broken

6. **Update image list** — invoke the `/list-images` skill to regenerate `IMAGES.md` with the new image included. Then stage and amend the last commit:
   ```
   git add IMAGES.md && git commit --amend --no-edit && git push --force-with-lease
   ```

7. **Add to multi-backend benchmarking (optional)** — if the new submodule is a SLAM backend (not e.g. a sensor driver), ask via AskUserQuestion whether to wire it into `scripts/run_all_backends.sh`. If yes, add the appropriate launcher (mirror the existing pattern: a `run_<name>_in_container.sh` for ROS-docker backends, or a `run_<name>.py` wrapper for pure-Python backends). Do NOT auto-run the benchmark; that's a separate `run-benchmark` skill.

8. **Report** — one or two sentences: workflow status, image name (e.g. `ghcr.io/wangxinjian1108/<name>:latest`), and whether it was added as a benchmarkable backend.
