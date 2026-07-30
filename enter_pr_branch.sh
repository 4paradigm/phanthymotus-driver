#!/usr/bin/env bash
# enter_pr_branch.sh — Fetch and checkout a PR's pre-merge branch for local testing.
# Usage: ./enter_pr_branch.sh [PR_NUMBER]
#   If PR_NUMBER is omitted, lists open PRs for interactive selection.
#
# Dependencies: git
# Note: Uses git ls-remote instead of GitHub API (no rate limit, no token needed).

set -euo pipefail

REMOTE="origin"
BRANCH_PREFIX="pr-merged"

# --- Helpers ---

die() { echo "❌ $*" >&2; exit 1; }
info() { echo "ℹ️  $*"; }
warn() { echo "⚠️  $*"; }

# Detect repo owner/name from git remote
detect_repo() {
  local url
  url=$(git remote get-url "$REMOTE" 2>/dev/null) || die "Remote '$REMOTE' not found."

  # Handle SSH (git@github.com:owner/repo.git) and HTTPS (https://github.com/owner/repo.git)
  if [[ "$url" =~ github\.com[:/]([^/]+)/([^/.]+)(\.git)?$ ]]; then
    REPO_OWNER="${BASH_REMATCH[1]}"
    REPO_NAME="${BASH_REMATCH[2]}"
  else
    die "Cannot parse GitHub repo from remote URL: $url"
  fi
}

check_deps() {
  if ! git rev-parse --is-inside-work-tree &>/dev/null 2>&1; then
    die "Not inside a git repository."
  fi
}

# Check for uncommitted changes
check_dirty() {
  if ! git diff-index --quiet HEAD -- 2>/dev/null; then
    warn "You have uncommitted changes in the current branch."
    read -rp "Continue anyway? [y/N] " ans
    [[ "$ans" =~ ^[Yy]$ ]] || exit 0
  fi
}

# List unmerged PRs using git ls-remote (no API needed)
select_pr() {
  info "Fetching PRs from ${REPO_OWNER}/${REPO_NAME} (via git)..."

  # Fetch all PR head refs with their SHAs
  local pr_data
  pr_data=$(git ls-remote "$REMOTE" 'refs/pull/*/head' 2>/dev/null) || \
    die "Failed to list PRs via git ls-remote."

  if [[ -z "$pr_data" ]]; then
    die "No PRs found."
  fi

  # Get commits already in main to filter out merged PRs
  git fetch "$REMOTE" --quiet 2>/dev/null
  local main_branch
  main_branch=$(git symbolic-ref refs/remotes/origin/HEAD 2>/dev/null | sed 's@^refs/remotes/origin/@@') || main_branch="main"

  echo ""
  echo "Unmerged PRs:"
  echo "─────────────────────────────────────────"
  local found=0
  while IFS=$'\t' read -r sha ref; do
    local num="${ref#refs/pull/}"
    num="${num%/head}"
    # Skip if this commit is already in main (i.e., PR was merged)
    if ! git merge-base --is-ancestor "$sha" "refs/remotes/origin/$main_branch" 2>/dev/null; then
      echo "  #$num"
      found=1
    fi
  done <<< "$pr_data"
  echo "─────────────────────────────────────────"

  if [[ "$found" -eq 0 ]]; then
    die "No unmerged PRs found."
  fi

  echo ""
  read -rp "Enter PR number: #" pr_num

  if ! [[ "$pr_num" =~ ^[0-9]+$ ]]; then
    die "Invalid PR number: '$pr_num'"
  fi
  PR_NUMBER="$pr_num"
}

# Verify PR exists via git ls-remote (no API needed)
verify_pr() {
  info "Checking PR #$PR_NUMBER..."
  local head_ref="refs/pull/${PR_NUMBER}/head"
  if ! git ls-remote "$REMOTE" "$head_ref" 2>/dev/null | grep -q "$head_ref"; then
    die "PR #$PR_NUMBER not found. Check that the PR exists."
  fi
  info "PR #$PR_NUMBER exists (ref found)."
}

# Fetch the pre-merge ref and checkout
fetch_and_checkout() {
  local local_branch="${BRANCH_PREFIX}-${PR_NUMBER}"
  local merge_ref="refs/pull/${PR_NUMBER}/merge"

  # Check if local branch already exists
  if git show-ref --verify --quiet "refs/heads/$local_branch"; then
    echo ""
    warn "Local branch '$local_branch' already exists."
    echo "  [u] Update — delete and re-fetch"
    echo "  [c] Checkout — switch to existing branch as-is"
    echo "  [a] Abort"
    read -rp "Choose [u/c/a]: " choice
    case "$choice" in
      [Uu])
        # If we're on that branch, switch away first
        if [[ "$(git branch --show-current)" == "$local_branch" ]]; then
          if ! git diff-index --quiet HEAD -- 2>/dev/null; then
            die "Cannot update: you have uncommitted changes on '$local_branch'. Commit or stash first."
          fi
          local default_branch
          default_branch=$(git symbolic-ref refs/remotes/origin/HEAD 2>/dev/null | sed 's@^refs/remotes/origin/@@') || default_branch="main"
          git checkout "$default_branch" --quiet || die "Failed to switch away from '$local_branch'. Stash your changes first."
        fi
        git branch -D "$local_branch"
        info "Deleted old branch '$local_branch'."
        ;;
      [Cc])
        git checkout "$local_branch" --quiet
        info "Switched to existing branch '$local_branch'."
        echo ""
        info "Done! You are now on: $(git branch --show-current)"
        return
        ;;
      *)
        info "Aborted."
        exit 0
        ;;
    esac
  fi

  echo ""
  info "Fetching merge ref for PR #$PR_NUMBER..."
  if ! git fetch "$REMOTE" "$merge_ref:$local_branch" 2>/dev/null; then
    # GitHub may not have a merge ref if there are conflicts
    warn "Failed to fetch merge ref. The PR may have merge conflicts."
    info "Trying to fetch the PR head branch instead..."
    local head_ref="refs/pull/${PR_NUMBER}/head"
    local_branch="pr-head-${PR_NUMBER}"

    if git show-ref --verify --quiet "refs/heads/$local_branch"; then
      warn "Branch '$local_branch' already exists."
      read -rp "Delete and re-fetch? [y/N] " ans
      if [[ "$ans" =~ ^[Yy]$ ]]; then
        if [[ "$(git branch --show-current)" == "$local_branch" ]]; then
          if ! git diff-index --quiet HEAD -- 2>/dev/null; then
            die "Cannot update: uncommitted changes on '$local_branch'. Commit or stash first."
          fi
          git checkout main --quiet || die "Failed to switch away. Stash your changes first."
        fi
        git branch -D "$local_branch"
      else
        exit 0
      fi
    fi

    git fetch "$REMOTE" "$head_ref:$local_branch" || \
      die "Failed to fetch PR #$PR_NUMBER. Check that the PR exists and you have access."
    warn "Note: This is the PR head (not pre-merged). You may need to rebase/merge manually."
  fi

  git checkout "$local_branch" --quiet
  echo ""
  info "Done! You are now on: $(git branch --show-current)"
  info "Tip: When finished, run 'git checkout main' to go back."
}

# --- Main ---

check_deps
detect_repo
check_dirty

if [[ "${1:-}" =~ ^#?([0-9]+)$ ]]; then
  PR_NUMBER="${BASH_REMATCH[1]}"
else
  select_pr
fi

verify_pr
fetch_and_checkout
