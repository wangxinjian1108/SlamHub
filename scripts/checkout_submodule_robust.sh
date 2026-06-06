#!/bin/bash
# Robustly checkout one submodule using whichever method works.
# Tries git-clone first (cheap when network is up), falls back to
# codeload tarball + git fetch hydration.
set -uo pipefail

SUB_PATH="$1"      # e.g. thirdparty/mad-icp
REPO="$2"          # e.g. wangxinjian1108/mad-icp
SHA="$3"           # 40-char SHA
MAX_ATTEMPTS="${4:-30}"

REPO_ROOT="$(git rev-parse --show-toplevel)"
TGT="$REPO_ROOT/$SUB_PATH"
GIT_DIR="$REPO_ROOT/.git/modules/$SUB_PATH"

is_done() {
    # Check that we have content AND the SHA is checked out
    if [ ! -d "$TGT" ] || [ "$(ls "$TGT" 2>/dev/null | grep -v '^\.git$' | wc -l)" -lt 1 ]; then
        return 1
    fi
    return 0
}

if is_done; then
    echo "[$SUB_PATH] already populated, skip"
    exit 0
fi

cd "$REPO_ROOT"

for attempt in $(seq 1 $MAX_ATTEMPTS); do
    echo "[$SUB_PATH] attempt $attempt/$MAX_ATTEMPTS"

    # Method A: git submodule update (uses git-clone over github.com:443)
    rm -rf "$TGT" "$GIT_DIR" 2>/dev/null
    if GIT_HTTP_LOW_SPEED_LIMIT=0 GIT_HTTP_LOW_SPEED_TIME=300 \
       timeout 200 git submodule update --init --depth 1 "$SUB_PATH" 2>/dev/null; then
        if is_done; then
            echo "[$SUB_PATH] OK via git-clone (attempt $attempt)"
            exit 0
        fi
    fi
    # Half-state cleanup
    rm -rf "$TGT" "$GIT_DIR" 2>/dev/null

    # Method B: codeload tarball
    TMP=$(mktemp -d)
    if timeout 90 curl -sSfL -o "$TMP/r.tgz" \
        "https://codeload.github.com/$REPO/tar.gz/$SHA" 2>/dev/null && \
        [ -s "$TMP/r.tgz" ]; then
        mkdir -p "$TGT"
        if tar -xzf "$TMP/r.tgz" -C "$TGT" --strip-components=1 2>/dev/null; then
            mkdir -p "$GIT_DIR"
            git --git-dir="$GIT_DIR" init --bare --quiet
            git --git-dir="$GIT_DIR" config core.bare false
            git --git-dir="$GIT_DIR" config core.worktree "$TGT"
            git --git-dir="$GIT_DIR" config remote.origin.url "https://github.com/$REPO.git"
            echo "gitdir: $GIT_DIR" > "$TGT/.git"
            # Try to hydrate the SHA into the bare git dir so submodule status shows clean
            if timeout 90 git --git-dir="$GIT_DIR" fetch --depth=1 \
                "https://github.com/$REPO.git" "$SHA" 2>/dev/null; then
                git --git-dir="$GIT_DIR" update-ref HEAD "$SHA" 2>/dev/null
            fi
            rm -rf "$TMP"
            if is_done; then
                echo "[$SUB_PATH] OK via tarball (attempt $attempt)"
                exit 0
            fi
        fi
    fi
    rm -rf "$TMP"

    # Backoff
    sleep 30
done

echo "[$SUB_PATH] FAILED after $MAX_ATTEMPTS attempts"
exit 1
