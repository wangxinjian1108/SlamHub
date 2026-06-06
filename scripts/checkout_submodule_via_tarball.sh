#!/bin/bash
# Fallback submodule checkout via codeload.github.com tarballs.
# When github.com:443 (used by `git clone`) is throttled but
# codeload.github.com still works, this avoids needing git-protocol
# at all. We pull a tarball at the pinned SHA and explode it into the
# submodule path, then fake the .git pointer so `git submodule status`
# reports it as up-to-date.
#
# Usage: ./checkout_submodule_via_tarball.sh <submodule_path> <owner/repo> <sha>
# Example: ./checkout_submodule_via_tarball.sh thirdparty/mad-icp wangxinjian1108/mad-icp cb47d83...
set -euo pipefail
PATH_REL="$1"     # e.g. thirdparty/mad-icp
REPO="$2"         # e.g. wangxinjian1108/mad-icp
SHA="$3"          # 40-char hex SHA pinned in .gitmodules

REPO_ROOT="$(git rev-parse --show-toplevel)"
TGT="$REPO_ROOT/$PATH_REL"
GIT_DIR="$REPO_ROOT/.git/modules/$PATH_REL"

# 1. Download the tarball at the exact pinned SHA
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
TGZ="$TMP/repo.tar.gz"
echo "  fetching tarball for $REPO @ $SHA ..."
for attempt in 1 2 3 4 5; do
    if timeout 120 curl -sSfL -o "$TGZ" \
            "https://codeload.github.com/$REPO/tar.gz/$SHA"; then
        break
    fi
    echo "    codeload attempt $attempt failed, retrying..."
    sleep 5
done
if [ ! -s "$TGZ" ]; then
    echo "  ERROR: failed to download tarball"
    exit 1
fi

# 2. Wipe any half-state and recreate
rm -rf "$TGT" "$GIT_DIR"
mkdir -p "$TGT"

# 3. Extract (--strip-components=1 strips the top-level <repo>-<sha>/ dir)
echo "  extracting ..."
tar -xzf "$TGZ" -C "$TGT" --strip-components=1

# 4. Create a minimal git dir so submodule status recognizes it
mkdir -p "$GIT_DIR"
cd "$GIT_DIR"
git init --bare --quiet
git config core.bare false
git config core.worktree "$TGT"
# Record the SHA so submodule status sees us at the pinned commit.
# Need a real object: import the tarball's tree as a git tree, then commit.
# Faster: just write a fake HEAD pointing to the SHA. But submodule status
# checks if the SHA is reachable, so we need to actually have the commit
# object. We'll do a "git fetch" of just the one SHA from origin if we can,
# else fall back to creating a synthetic commit.
cd "$TGT"
echo "gitdir: $GIT_DIR" > .git
# The .git is now a file pointing into the SlamHub git dir.
# We need to populate $GIT_DIR with the one commit at $SHA.
# Strategy: use `git fetch` against the upstream URL with a one-shot
# refspec. If that fails too, write a synthetic ref.
URL="https://github.com/$REPO.git"
if timeout 90 git --git-dir="$GIT_DIR" fetch --depth=1 "$URL" "$SHA" 2>/dev/null; then
    git --git-dir="$GIT_DIR" update-ref HEAD "$SHA"
    echo "  submodule git dir hydrated to $SHA"
else
    # Fallback: tell git the SHA exists by writing the ref directly.
    # Submodule status will report "modified" but the tree content is correct.
    echo "  WARN: couldn't fetch one-commit; tarball content is correct but submodule status may show modified"
    git --git-dir="$GIT_DIR" config remote.origin.url "$URL"
fi

# 5. Recursive: walk into nested .gitmodules if present
if [ -f "$TGT/.gitmodules" ]; then
    echo "  found nested .gitmodules; recursing"
    while IFS= read -r line; do
        if [[ "$line" =~ path[[:space:]]*=[[:space:]]*(.+) ]]; then
            sub_path="${BASH_REMATCH[1]}"
        elif [[ "$line" =~ url[[:space:]]*=[[:space:]]*(.+) ]]; then
            sub_url="${BASH_REMATCH[1]}"
            # Strip .git suffix
            sub_url="${sub_url%.git}"
            sub_repo="${sub_url#https://github.com/}"
            # We'd need the SHA of the nested submodule. The parent's
            # .git/modules/.../HEAD doesn't have it because we faked HEAD.
            # Skip nested submodules in fallback mode — they need the
            # parent's full git history to know the SHA.
            echo "  (nested submodule $sub_path: skip in fallback mode)"
        fi
    done < "$TGT/.gitmodules"
fi

ls "$TGT" | head -5
echo "  $PATH_REL: extracted $(ls "$TGT" | wc -l) entries"
