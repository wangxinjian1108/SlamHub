Update an existing SLAM submodule to its latest upstream commit and rebuild its Docker image.

## Argument
`$ARGUMENTS` is the submodule name or path (e.g. `LIO-SAM` or `thirdparty/LIO-SAM`).

## Steps

1. **Locate the submodule** — search `.gitmodules` to resolve the full path if only a name was given. Verify the submodule directory exists.

2. **Update to latest** — pull the latest commit from the submodule's tracked branch:
   ```
   git submodule update --init --remote thirdparty/<name>
   ```

3. **Check for changes** — run `git diff --submodule thirdparty/<name>` to confirm the submodule pointer moved. If nothing changed, report "already up to date" and stop.

4. **Rebuild image (optional)** — use AskUserQuestion to ask whether to also rebuild the Docker image. If yes:
   - Check if `docker/<name>/Dockerfile` exists. If not, suggest running `/dockerize-submodule` first.
   - Trigger the workflow manually: `gh workflow run docker-<name>.yml`
   - Or, if the user prefers, commit and push to trigger it via the paths filter.

5. **Commit & push**
   - Stage the submodule pointer change: `git add thirdparty/<name>`
   - Commit: `feat: update <name> to latest`
   - Push: `git push`

6. **Monitor CI** — if a workflow was triggered:
   - Wait for the run to complete
   - If success → report and stop
   - If failure:
     a. Fetch logs: `gh run view <run-id> --log-failed`
     b. If the failure is in the Dockerfile (e.g. upstream changed an API or added a new system dep), fix it, amend, and force-push
     c. Common patterns when an SLAM submodule moves forward: new ROS message types in package.xml (need new ros-${ROS_DISTRO}-* apt pkg), new C++ build flag, new Python dep added to pyproject.toml
     d. Iterate up to 3 times
   - After 3 failed attempts, stop and report

7. **Re-validate against AT128P benchmark (optional)** — for SLAM backends that are wired into `scripts/run_all_backends.sh`, ask via AskUserQuestion whether to re-run the multi-sample benchmark to confirm the upstream change didn't regress trajectory accuracy or cross-LiDAR \|Δt\|. If yes, invoke `/run-benchmark` with the appropriate sample.

8. **Report** — one or two sentences: new submodule commit hash, CI status, and (if benchmark was rerun) any change in cross-backend agreement.

