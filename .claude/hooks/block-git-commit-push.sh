#!/usr/bin/env bash
# PreToolUse hook on Bash: denies any command that would commit or push.
# The user commits and pushes themselves, after hardware verification (see
# CLAUDE.md, "Git"). Reads the tool call as JSON on stdin; matches "git ... commit"
# or "git ... push" within one pipeline segment, so "git log | grep commit" passes.
command_text=$(jq -r '.tool_input.command // empty')
if grep -qE '(^|[;&|[:space:]])git\b[^|;&]*\b(commit|push)\b' <<<"$command_text"; then
  jq -n '{
    hookSpecificOutput: {
      hookEventName: "PreToolUse",
      permissionDecision: "deny",
      permissionDecisionReason: "Blocked by .claude/hooks/block-git-commit-push.sh: this project never commits or pushes from Claude Code. Leave the change in the working tree; the user commits after hardware verification."
    }
  }'
fi
exit 0
