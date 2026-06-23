#!/usr/bin/env bash
# 3pnw: pack + publish the prebuilt release. Run from inside the BUILT release tree (cwd = $TARGET_DIR,
# already a git repo on an orphan branch with the source committed by build_stripped.sh, and just
# compiled in place by system/manager/build.py). Strips build intermediates, keeps the compiled
# artifacts, marks `prebuilt`, and force-pushes to $RELEASE_BRANCH.
#
# EXPERIMENTAL — see 3PNW-RELEASE.md. aarch64 (GitHub ARM) != comma larch64; panda fw unsigned.
set -ex

: "${RELEASE_BRANCH:?RELEASE_BRANCH not set}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
# git identity for the release commit (release/ is stripped from the tree, so source it from the repo)
source "$SCRIPT_DIR/identity.sh" 2>/dev/null || { git config user.email "ci@pnw-pilot"; git config user.name "pnw-ci"; }

# drop build intermediates; keep the compiled outputs (.so etc.) that make this a prebuilt
find . -name '*.a' -delete
find . -name '*.o' -delete
find . -name '*.os' -delete
find . -name '*.pyc' -delete
find . -name 'moc_*' -delete
find . -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true
rm -f .sconsign.dblite

# host-arch libs that have no business on the device
find third_party/ -name '*x86*'    -exec rm -rf {} + 2>/dev/null || true
find third_party/ -name '*Darwin*' -exec rm -rf {} + 2>/dev/null || true

# guard: GitHub rejects files > 100 MB
BIG_FILES="$(find . -type f -not -path './.git/*' -size +95M)"
if [ -n "$BIG_FILES" ]; then
  echo "Files exceed GitHub's 100MB limit:"; echo "$BIG_FILES"; exit 1
fi

touch prebuilt   # tell the device this tree is already built (no on-device rebuild)

VERSION="$(awk -F\" '/COMMA_VERSION/{print $2}' common/version.h)"
git add -f .
git commit -q -m "3pnw v${VERSION} aarch64 prebuilt (CI, experimental)

Built on a GitHub ARM runner (generic aarch64, NOT comma larch64). Panda fw unsigned/skipped.
Not guaranteed device-deployable; the authoritative release is release/build_release.sh on-device."

# authenticate the push with the workflow token (the copied .git may not carry CI auth)
if [ -n "${GITHUB_TOKEN:-}" ] && [ -n "${GITHUB_REPOSITORY:-}" ]; then
  git remote set-url origin "https://x-access-token:${GITHUB_TOKEN}@github.com/${GITHUB_REPOSITORY}.git"
fi

git push -f origin "HEAD:${RELEASE_BRANCH}"
echo "[-] pushed prebuilt to ${RELEASE_BRANCH}"
