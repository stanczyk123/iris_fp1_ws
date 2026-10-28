#!/usr/bin/env bash
# Reset all submodules to match upstream exactly, update them to the latest
# upstream commit, and commit the new submodule pointers.
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

git submodule sync --recursive
git submodule update --init --recursive

# Discard local modifications and untracked/ignored files so submodules are 1:1 with upstream
git submodule foreach --recursive 'git reset --hard && git clean -ffdx'

# Move top-level submodules to the tip of their upstream branch,
# then check out nested submodules at the commits upstream records
git submodule update --init --remote --force
git submodule foreach 'git submodule update --init --recursive --force'

mapfile -t paths < <(git config --file .gitmodules --get-regexp '\.path$' | awk '{print $2}')
git add .gitmodules "${paths[@]}"

if git diff --cached --quiet; then
    echo "Submodules already up to date."
    exit 0
fi

git commit -m "chore(submodules): update to latest upstream"
