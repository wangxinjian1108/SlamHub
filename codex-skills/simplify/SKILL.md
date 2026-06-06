---
name: simplify
description: Simplify code, scripts, Dockerfiles, workflows, or repository structure in a SlamHub-style project without changing intended behavior. Use when the user asks to reduce complexity, remove duplication, or make a change easier to maintain.
---

# Simplify

Use this skill when the user wants a simpler implementation rather than a feature rewrite.

## Goals

- Reduce complexity
- Remove duplication
- Keep behavior stable unless the user asked for functional changes
- Preserve existing repository conventions

## Scope

This applies to:

- Dockerfiles
- GitHub Actions workflows
- shell scripts
- Python utilities
- repo documentation tied to workflows

## Workflow

1. Read the relevant files and understand the current behavior first.
2. Identify unnecessary branching, repeated logic, dead code, over-parameterization, or duplicated config.
3. Prefer the smallest change that materially improves readability or maintainability.
4. Preserve interfaces, file paths, image names, and workflow triggers unless there is a clear reason to change them.
5. If the simplification would alter behavior, state that explicitly and get confirmation unless the change is obviously intended.
6. After editing, run the most relevant validation available.

## Guardrails

- Do not turn repo-specific logic into abstract frameworks without a concrete payoff
- Do not “simplify” by dropping required validation, CI triggers, or dependency handling
- In this repo, keep the submodule, Docker, and workflow conventions aligned with `AGENTS.md`

## Reporting

Summarize:

- what was simplified
- what duplication or complexity was removed
- whether behavior changed
- what validation was run
