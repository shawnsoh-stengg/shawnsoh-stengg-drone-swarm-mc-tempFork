#!/usr/bin/env bash
# Bring the team repo's latest main into this repo (stand-in for a fork,
# since AI-DA-STC has forking disabled). See claudeShawn.md, "Repo setup".
#
# Folders:
#   TEAM_DIR = local clone of the team repo (AI-DA-STC/drone-swarm-mc)
#   MY_DIR   = this repo (shawnsoh-stengg-drone-swarm-mc-tempFork), found
#              from this script's own location
#
# Branches in this repo:
#   upstream-main = exact copy of the team's main, never edit it
#   main          = upstream-main + your own work
#
# Each run:
#   1. pulls the team's main into TEAM_DIR
#   2. copies it (git fetch from TEAM_DIR, history included) into this
#      repo's upstream-main and pushes that to GitHub
#   3. merges upstream-main into main and pushes main to GitHub
#
# Override any setting with an env var, e.g. TEAM_DIR=~/elsewhere ./sync-from-team.sh
set -euo pipefail

MY_DIR="$(git -C "$(dirname "$(readlink -f "$0")")" rev-parse --show-toplevel)"
TEAM_DIR="${TEAM_DIR:-$HOME/SW/cpp/drone-swarm-mc}"
TEAM_BRANCH="${TEAM_BRANCH:-main}"              # branch to follow on the team repo
MIRROR_BRANCH="${MIRROR_BRANCH:-upstream-main}" # this repo's copy of it
MY_BRANCH="${MY_BRANCH:-main}"                  # your work, on top of the copy

# --- 1. team repo -> TEAM_DIR -------------------------------------------

echo "==> [team] pulling $TEAM_BRANCH in $TEAM_DIR"
git -C "$TEAM_DIR" fetch --prune origin
if [ "$(git -C "$TEAM_DIR" symbolic-ref --short -q HEAD || true)" = "$TEAM_BRANCH" ]; then
  git -C "$TEAM_DIR" merge --ff-only "origin/$TEAM_BRANCH"
else
  # Fast-forward only: stops the script if the local branch has commits
  # the team's branch doesn't.
  git -C "$TEAM_DIR" fetch . "origin/$TEAM_BRANCH:$TEAM_BRANCH"
fi

# --- 2. TEAM_DIR -> this repo's upstream-main ---------------------------

cd "$MY_DIR"

if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  echo "!!  uncommitted changes in $MY_DIR."
  echo "    Commit or stash them, then run this script again."
  exit 1
fi

start_branch="$(git symbolic-ref --short -q HEAD || true)"
# git fetch can't update the branch that is checked out.
if [ "$start_branch" = "$MIRROR_BRANCH" ]; then
  git checkout -q "$MY_BRANCH"
fi

echo "==> [mine] copying $TEAM_BRANCH from $TEAM_DIR into $MIRROR_BRANCH"
git fetch --prune origin
git fetch "$TEAM_DIR" "refs/heads/$TEAM_BRANCH:refs/heads/$MIRROR_BRANCH"

echo "==> [mine] pushing $MIRROR_BRANCH to GitHub"
git push origin "$MIRROR_BRANCH:refs/heads/$MIRROR_BRANCH"

# --- 3. upstream-main -> main -------------------------------------------

git checkout -q "$MY_BRANCH"

# Pick up anything pushed to GitHub from elsewhere (web UI, another
# machine) first, so the push below never rejects.
if git rev-parse --verify -q "origin/$MY_BRANCH" >/dev/null; then
  echo "==> [mine] merging origin/$MY_BRANCH into $MY_BRANCH"
  git merge --no-edit "origin/$MY_BRANCH"
fi

echo "==> [mine] merging $MIRROR_BRANCH into $MY_BRANCH"
if ! git merge --no-edit "$MIRROR_BRANCH"; then
  echo "!!  merge hit a conflict. Fix the files, then:"
  echo "      git add <files> && git commit --no-edit"
  echo "    (or 'git merge --abort' to undo) and run this script again."
  exit 1
fi

echo "==> [mine] pushing $MY_BRANCH to GitHub"
git push origin "$MY_BRANCH:refs/heads/$MY_BRANCH"

if [ -n "$start_branch" ] && [ "$start_branch" != "$MY_BRANCH" ] &&
  [ "$start_branch" != "$MIRROR_BRANCH" ]; then
  git checkout -q "$start_branch"
fi

echo "==> done. $MY_DIR is on: $(git symbolic-ref --short -q HEAD || git rev-parse --short HEAD)"
