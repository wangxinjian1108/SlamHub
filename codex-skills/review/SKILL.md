---
name: review
description: Review code changes in a SlamHub-style repository with a bug-finding mindset, prioritizing correctness risks, regressions, bad assumptions, and missing tests over summaries. Use when the user asks for a review.
---

# Review

Use this skill when the user asks for a review of code, scripts, Dockerfiles, workflows, or a diff.

## Review stance

- Default to code review, not explanation
- Prioritize findings over praise
- Focus on bugs, regressions, operational risks, and missing coverage

## What to inspect

- correctness of code changes
- Docker build behavior
- workflow trigger correctness
- image naming and publish logic
- dependency installation order
- unsafe assumptions about submodules, branches, or paths
- missing validation or tests

## Workflow

1. Read the diff and enough surrounding code to understand intent.
2. Look for behavior changes that conflict with repo conventions in `AGENTS.md`.
3. Check edge cases, failure handling, and state transitions.
4. Verify that changes to workflows or Dockerfiles will still build from repo root and still reference the correct paths.
5. Verify that changes do not accidentally stage or expose secrets.
6. Note missing tests or validation where the risk justifies it.

## Output format

Findings first, ordered by severity.

For each finding include:

- severity
- file and line reference when available
- concrete issue
- why it matters

After findings, include:

- open questions or assumptions
- brief overall summary

## If no findings

State that explicitly and mention residual risk areas or validation gaps.
