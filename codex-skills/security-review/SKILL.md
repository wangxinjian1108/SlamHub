---
name: security-review
description: Review a SlamHub-style repository or diff for security problems, secret handling mistakes, supply-chain risk, unsafe Docker or CI behavior, and permission issues. Use when the user asks for a security review.
---

# Security Review

Use this skill when the user wants a security-focused review rather than a general code review.

## Primary concerns

- secret leakage
- unsafe credential handling
- overly broad GitHub Actions permissions
- risky container build behavior
- untrusted downloads during build
- command injection or path injection
- accidental publication to the wrong registry
- writable mounts or privileged runtime usage without justification

## Workflow

1. Read the relevant diff and surrounding files.
2. Inspect secrets usage in:
   - GitHub Actions workflows
   - Dockerfiles
   - shell scripts
   - config files
3. Check for:
   - secrets written into image layers
   - tokens echoed to logs
   - `curl | sh` or similar unsafe bootstrap steps
   - unpinned or unauthenticated remote downloads when trust assumptions are weak
   - excessive workflow permissions
   - unsafe use of `docker run`, mounts, or privileged flags
4. For this repo specifically, verify:
   - `HF_TOKEN` is only used via BuildKit secrets for gated models
   - `GITHUB_TOKEN` and registry credentials are not misused
   - `config/token.yaml` or other sensitive files are never staged or copied into images
5. Distinguish between confirmed vulnerabilities and hardening suggestions.

## Output format

List confirmed findings first, ordered by severity.

For each finding include:

- severity
- location
- issue
- exploit or impact
- recommended fix

After findings, include:

- notable hardening suggestions
- unresolved assumptions

## If no findings

State that explicitly, then note residual exposure areas you could not fully validate.
