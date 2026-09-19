#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
#
# release_guard.sh — refuse to publish firmware artifacts that carry secrets.
#
# Every value in the git-ignored sdkconfig.credentials (Tailscale auth key,
# WiFi credentials, admin password, OTA URL/API key, management-server IP) is
# compiled into BOTH the .bin and the .elf as a plain string. Only a build made
# WITHOUT a credentials file may ever be attached to a public release.
#
#   tools/release_guard.sh <artifact>...
#
# Exit 0 = clean. Exit 1 = a secret or secret-shaped string was found (the
# offending pattern is named, the value is never printed).

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ $# -ge 1 ]] || { echo "usage: $0 <artifact>..." >&2; exit 2; }

fail=0

# Values that are already public (Kconfig defaults in the tracked tree, e.g. the
# documented default admin password) are not secrets and must not trip the guard.
public_defaults="$(grep -hoE 'default "[^"]+"' "$REPO"/components/microlink/Kconfig \
  "$REPO"/firmware/components/*/Kconfig "$REPO"/machn/main/Kconfig* 2>/dev/null | sed -E 's/^default "//; s/"$//' || true)"

# 1. Literal values from every credentials file we can find (private build inputs).
declare -a values=()
for f in "$REPO"/firmware/sdkconfig.credentials "$REPO"/machn/sdkconfig.credentials; do
  [[ -f "$f" ]] || continue
  while IFS= read -r line; do
    [[ "$line" =~ ^CONFIG_[A-Z0-9_]+=\"(.*)\"$ ]] || continue
    v="${BASH_REMATCH[1]}"
    # skip empty / boolean / short values that would match everywhere
    [[ -n "$v" && "$v" != "y" && "$v" != "n" && ${#v} -ge 6 ]] || continue
    grep -qxF -- "$v" <<< "$public_defaults" && continue
    values+=("${line%%=*}=$v")
  done < "$f"
done

for art in "$@"; do
  [[ -f "$art" ]] || { echo "MISSING $art" >&2; fail=1; continue; }
  for kv in "${values[@]}"; do
    k="${kv%%=*}"; v="${kv#*=}"
    if grep -qF -- "$v" "$art"; then
      echo "LEAK  $art: value of $k present"; fail=1
    fi
  done
  # 2. Secret-shaped strings regardless of any credentials file.
  if grep -qE -- 'tskey-(auth|client|api)-[A-Za-z0-9]+' "$art"; then
    echo "LEAK  $art: Tailscale key pattern present"; fail=1
  fi
  if grep -qE -- '-----BEGIN [A-Z ]*PRIVATE KEY' "$art"; then
    echo "LEAK  $art: PEM private key present"; fail=1
  fi
  if grep -qE -- '(^|[^A-Za-z])(Bearer|Basic) [A-Za-z0-9+/=_-]{16,}' "$art"; then
    echo "LEAK  $art: literal HTTP auth token present"; fail=1
  fi
  [[ $fail -eq 0 ]] && echo "clean $art"
done

exit $fail
