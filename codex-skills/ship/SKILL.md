---
name: ship
description: Commit relevant local changes in a SlamHub-style repo, push them, optionally open a PR when not on master, and monitor CI with up to three fix rounds. Use when the user asks to ship current work.
---

# Ship

Use this skill when the user wants the current work committed, pushed, and checked in CI.

## Inputs

- Optional commit message

## Guardrails

- Run `git status` and inspect the diff before staging anything
- Do not stage `.env`, `config/token.yaml`, secrets, or unrelated large artifacts
- Never use `--no-verify`
- Do not revert user changes you did not make

## Workflow

1. Inspect `git status` and `git diff HEAD`.
2. If there is nothing meaningful to commit, report that and stop.
3. Stage tracked changes with `git add -u`.
4. Stage new files only when they are clearly part of the requested work.
5. Commit with the provided message, or derive a concise message from the diff.
6. Detect the current branch with `git branch --show-current`.
7. Push with upstream tracking if needed.
8. If not on `master`, open a PR to `master` unless the user said not to.
9. Watch the triggered workflow runs with `gh`.
10. If CI fails:
    - inspect failed logs
    - distinguish code errors from missing secrets or flaky infra
    - fix actionable issues
    - amend or create a follow-up commit according to the repo’s requested history style
    - push again
    - retry up to 3 rounds
11. If the failure is only missing credentials or external infrastructure, stop and report that rather than thrashing.

## Reporting

Report:

- branch name
- PR URL if one was opened
- final CI status
- remaining blockers requiring human action
