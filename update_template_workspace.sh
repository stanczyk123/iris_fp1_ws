#!/usr/bin/env bash
# Fetch the latest template workspace and rebase the current branch on top of it.
set -euo pipefail

TEMPLATE_URL="https://github.com/Internship-IRIS-ITS/iris_fp1_ws"
TEMPLATE_REMOTE="template"
TEMPLATE_BRANCH="main"

cd "$(git rev-parse --show-toplevel)"

if git remote get-url "$TEMPLATE_REMOTE" &>/dev/null; then
    git remote set-url "$TEMPLATE_REMOTE" "$TEMPLATE_URL"
else
    git remote add "$TEMPLATE_REMOTE" "$TEMPLATE_URL"
fi

git fetch "$TEMPLATE_REMOTE" "$TEMPLATE_BRANCH"

# Stash uncommitted changes around the rebase so it can run on a dirty tree
if ! git rebase --autostash "$TEMPLATE_REMOTE/$TEMPLATE_BRANCH"; then
    git rebase --abort || true
    echo "error: rebase onto $TEMPLATE_REMOTE/$TEMPLATE_BRANCH hit a merge conflict; aborted, branch left unchanged" >&2
    exit 1
fi

# Check out the submodule commits recorded by the rebased branch
git submodule sync --recursive
git submodule update --init --recursive
