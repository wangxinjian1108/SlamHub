---
name: notify
description: Send a concise status notification for SlamHub work, typically to Slack channel #cc or another configured destination, after summarizing what changed, CI status, and required human attention. Use when the user asks to notify or broadcast results.
---

# Notify

Use this skill when the user wants a concise outbound status message after work is done.

## Important constraint

This skill depends on whatever notification integration exists in the current environment.

- If Slack or another notifier is configured, use it
- If no integration is available, draft the exact message and state that it was not sent

## Message goals

- concise
- factual
- easy to scan
- includes action items when needed

## Recommended content

- task or model name
- branch or PR link if relevant
- image or workflow status if relevant
- success or failure summary
- blockers needing human attention

## Workflow

1. Determine the destination from the user request or repo convention.
2. Gather the final status from the work just completed.
3. Draft a short message suitable for Slack or similar chat ops tools.
4. If a notifier tool or script exists, send it.
5. If not, return the drafted message and say it was prepared but not sent.

## Guardrails

- Never include secrets, tokens, or raw credentials
- Do not send noisy logs unless the user explicitly asks
- Prefer links and one-line status over long prose

## Output

Provide:

- destination used or intended
- whether the notification was actually sent
- the final message text
