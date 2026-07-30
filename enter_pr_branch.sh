#!/usr/bin/env bash
# enter_pr_branch.sh — Fetch and checkout a PR's pre-merge branch for local testing.
# Usage: ./enter_pr_branch.sh [PR_NUMBER]
#   If PR_NUMBER is omitted, lists open PRs for interactive selection.
#
# Dependencies: git, curl, (optional) jq
# For private repos, set GITHUB_TOKEN environment variable.

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

# GitHub API call via curl
github_api() {
  local endpoint="$1"
  local args=(-s -f -L)
  if [[ -n "${GITHUB_TOKEN:-}" ]]; then
    args+=(-H "Authorization: token $GITHUB_TOKEN")
  fi
  args+=(-H "Accept: application/vnd.github+json")
  curl "${args[@]}" "https://api.github.com/repos/${REPO_OWNER}/${REPO_NAME}${endpoint}"
}

# Parse JSON value — uses jq if available, otherwise basic grep/sed fallback
json_val() {
  local json="$1" key="$2"
  if command -v jq &>/dev/null; then
    echo "$json" | jq -r ".$key"
  else
    # Fallback: works for simple flat string/number fields
    echo "$json" | grep -o "\"$key\"[[:space:]]*:[[:space:]]*\"[^\"]*\"" | head -1 | sed 's/.*: *"//;s/"$//' ||
    echo "$json" | grep -o "\"$key\"[[:space:]]*:[[:space:]]*[0-9]*" | head -1 | sed 's/.*: *//'
  fi
}

# Parse JSON array of PRs (minimal parser for listing)
parse_pr_list() {
  local json="$1"
  if command -v jq &>/dev/null; then
    echo "$json" | jq -r '.[] | "#\(.number)  \(.title)  (\(.user.login), \(.head.ref))"'
  else
    # Fallback: extract number and title pairs
    echo "$json" | grep -oP '"number"\s*:\s*\K[0-9]+' | while read -r num; do
      echo "#$num"
    done
    warn "(Install jq for better PR listing with titles)"
  fi
}

check_deps() {
  if ! git rev-parse --is-inside-work-tree &>/dev/null 2>&1; then
    die "Not inside a git repository."
  fi
  if ! command -v curl &>/dev/null; then
    die "'curl' not found."
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

# List open PRs and let user pick one
select_pr() {
  info "Fetching open PRs from ${REPO_OWNER}/${REPO_NAME}..."
  local response
  response=$(github_api "/pulls?state=open&per_page=20") || \
    die "Failed to fetch PRs. For private repos, set GITHUB_TOKEN."

  if [[ "$response" == "[]" || -z "$response" ]]; then
    die "No open PRs found."
  fi

  local pr_list
  pr_list=$(parse_pr_list "$response")

  if [[ -z "$pr_list" ]]; then
    die "No open PRs found (or failed to parse response)."
  fi

  echo ""
  echo "Open PRs:"
  echo "─────────────────────────────────────────"
  echo "$pr_list"
  echo "─────────────────────────────────────────"
  echo ""
  read -rp "Enter PR number: #" pr_num

  if ! [[ "$pr_num" =~ ^[0-9]+$ ]]; then
    die "Invalid PR number: '$pr_num'"
  fi
  PR_NUMBER="$pr_num"
}

# Verify PR exists via API
verify_pr() {
  info "Checking PR #$PR_NUMBER..."
  local response
  response=$(github_api "/pulls/$PR_NUMBER" 2>/dev/null) || \
    die "PR #$PR_NUMBER not found or not accessible. For private repos, set GITHUB_TOKEN."

  local title state head_branch
  title=$(json_val "$response" "title")
  state=$(json_val "$response" "state")
  head_branch=$(json_val "$response" "ref")  # nested under head, try fallback

  # For head.ref we need nested parsing
  if command -v jq &>/dev/null; then
    head_branch=$(echo "$response" | jq -r '.head.ref')
    local merged
    merged=$(echo "$response" | jq -r '.merged')
  else
    head_branch=$(echo "$response" | grep -o '"ref"[[:space:]]*:[[:space:]]*"[^"]*"' | head -1 | sed 's/.*"ref"[[:space:]]*:[[:space:]]*"//;s/"$//')
    local merged="false"
    echo "$response" | grep -q '"merged"[[:space:]]*:[[:space:]]*true' && merged="true"
  fi

  echo ""
  info "PR #$PR_NUMBER: $title"
  info "Branch: $head_branch | State: $state"

  if [[ "${merged:-false}" == "true" ]]; then
    warn "This PR is already merged."
    read -rp "Still want to fetch the merge ref? [y/N] " ans
    [[ "$ans" =~ ^[Yy]$ ]] || exit 0
  elif [[ "$state" == "closed" ]]; then
    warn "This PR is closed (not merged)."
    read -rp "Still want to fetch the merge ref? [y/N] " ans
    [[ "$ans" =~ ^[Yy]$ ]] || exit 0
  fi
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
