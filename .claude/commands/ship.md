Commit staged/unstaged changes, push to remote, and monitor CI until success.

## Argument
`$ARGUMENTS` is an optional commit message. If omitted, generate one from the diff.

## Steps

1. **Sanity check** — run `git status` and `git diff HEAD` to understand what has changed. If there is nothing to commit, report that and stop.

2. **Stage & commit**
   - Stage all tracked modifications: `git add -u`
   - Also stage any new untracked files that are clearly part of the work (e.g. new Dockerfiles, workflow files, source files). Do NOT stage `.env`, secrets, or large binaries.
   - Commit with the provided message, or generate a concise one from the diff if none was given:
     ```
     git commit -m "<message>"
     ```
   - If a pre-commit hook fails, fix the reported issue, re-stage, and create a **new** commit (never use `--no-verify`).

3. **Push**
   - Determine the current branch: `git branch --show-current`
   - Push: `git push -u origin HEAD`

4. **Open PR (only if NOT on master)**
   - If the current branch is `master`, skip PR creation — go directly to step 5.
   - Otherwise, open a PR to `master` with `gh pr create --base master --fill` and print the PR URL.

5. **Watch the CI run**
   - Wait for the workflow run triggered by the push to appear (may take a few seconds).
   - Monitor with `gh run watch` or poll `gh run list` until all relevant runs complete.
   - If all checks pass → report success and stop.

6. **Fix failures (iterate up to 3 times)**
   If any check fails:
   a. Fetch the failed job logs: `gh run view <run-id> --log-failed`
   b. Analyse the error. Common patterns:
      - Dockerfile build error → fix the relevant Dockerfile
      - Lint / format error → fix the reported file
      - Missing secret → note it for the user, do not retry
      - Flaky network (e.g. download timeout) → retry the run with `gh run rerun <run-id> --failed` without changing code
   c. Apply the fix to the relevant file(s).
   d. Stage the fix and amend the last commit (keeps history clean):
      ```
      git add -u
      git commit --amend --no-edit
      git push --force-with-lease
      ```
   e. Wait for the new CI run to finish (go back to step 5).
   f. After 3 failed fix attempts, stop and summarise what is still broken so the user can decide.

7. **Report** — one or two sentences: branch/PR URL, final CI status, and any remaining issues that need manual attention.
