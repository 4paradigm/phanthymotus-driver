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

# Fetch PR head, merge with origin/main, abort on conflict
fetch_and_checkout() {
  local local_branch="${BRANCH_PREFIX}-${PR_NUMBER}"
  local head_ref="refs/pull/${PR_NUMBER}/head"

  # Remember current branch/commit so we can restore on failure
  local original_branch
  original_branch=$(git branch --show-current 2>/dev/null)
  if [[ -z "$original_branch" ]]; then
    original_branch=$(git rev-parse HEAD)  # detached HEAD
  fi

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
          git checkout "$original_branch" --quiet 2>/dev/null || git checkout main --quiet
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

  # Ensure we have latest origin/main
  local main_branch
  main_branch=$(git symbolic-ref refs/remotes/origin/HEAD 2>/dev/null | sed 's@^refs/remotes/origin/@@') || main_branch="main"
  info "Fetching latest origin/$main_branch..."
  git fetch "$REMOTE" "$main_branch" --quiet || die "Failed to fetch origin/$main_branch."

  # Fetch PR head into a temporary local ref (avoid FETCH_HEAD ambiguity)
  # Use + prefix to force-update in case PR was force-pushed (non-fast-forward)
  echo ""
  info "Fetching PR #$PR_NUMBER head..."
  local tmp_pr_ref="refs/tmp/pr-${PR_NUMBER}-head"
  git update-ref -d "$tmp_pr_ref" 2>/dev/null  # clean up stale ref from prior runs
  git fetch "$REMOTE" "+$head_ref:$tmp_pr_ref" || \
    die "Failed to fetch PR #$PR_NUMBER. Check that the PR exists and you have access."

  # Create branch from origin/main, then merge PR into it
  info "Creating branch '$local_branch' from origin/$main_branch and merging PR #$PR_NUMBER..."
  git checkout -b "$local_branch" "refs/remotes/origin/$main_branch" --quiet || {
    git update-ref -d "$tmp_pr_ref" 2>/dev/null
    die "Failed to create branch '$local_branch'."
  }

  if ! git merge "$tmp_pr_ref" --no-edit --quiet 2>/dev/null; then
    # Conflict — abort merge, restore original state, clean up
    git merge --abort 2>/dev/null
    git checkout "$original_branch" --quiet 2>/dev/null
    git branch -D "$local_branch" 2>/dev/null
    git update-ref -d "$tmp_pr_ref" 2>/dev/null
    die "PR #$PR_NUMBER has merge conflicts with origin/$main_branch. Resolve conflicts in the PR first."
  fi

  # Clean up temp ref
  git update-ref -d "$tmp_pr_ref" 2>/dev/null

  echo ""
  info "Done! You are now on: $(git branch --show-current)"
  info "This branch = origin/$main_branch + PR #$PR_NUMBER (merged locally)."
  info "Tip: When finished, run 'git checkout $original_branch' to go back."
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
