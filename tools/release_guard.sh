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
# Three independent layers, so the verdict does not depend on which
# credentials file happens to be on the machine running the guard:
#   1. build brand   — any image compiled with a credentials file present carries
#                      the ML-BUILD-WITH-CREDENTIALS marker (CMakeLists + ml_app.c)
#   2. local values  — every value of a credentials file found in the tree
#   3. secret shapes — tskey-…, PEM private keys, literal HTTP auth tokens
#
# Exit 0 = every artifact clean. Exit 1 = a leak (pattern named, value never
# printed). Exit 2 = usage.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ $# -ge 1 ]] || { echo "usage: $0 <artifact>..." >&2; exit 2; }

# Values that are already public (Kconfig defaults in the tracked tree, e.g. the
# documented default admin password) are not secrets and must not trip the guard.
public_defaults="$(grep -hoE 'default "[^"]+"' "$REPO"/components/microlink/Kconfig \
  "$REPO"/firmware/components/*/Kconfig 2>/dev/null | sed -E 's/^default "//; s/"$//' || true)"

# Kconfig writes strings with \" and \\ escaped; the compiler embeds the unescaped bytes.
kconfig_unescape() { printf '%s' "$1" | sed -E 's/\\(["\\])/\1/g'; }
# Escape a literal for grep -P.
re_escape() { printf '%s' "$1" | sed -E 's/[][\\.^$*+?(){}|/-]/\\&/g'; }

# Layer 2 inputs: "KEY<TAB>value" lines from every credentials file in the tree.
values=""
for f in "$REPO"/firmware/sdkconfig.credentials "$REPO"/machn/sdkconfig.credentials; do
  [[ -f "$f" ]] || continue
  while IFS= read -r line; do
    [[ "$line" =~ ^(CONFIG_[A-Z0-9_]+)=\"(.*)\"$ ]] || continue
    k="${BASH_REMATCH[1]}"; v="$(kconfig_unescape "${BASH_REMATCH[2]}")"
    [[ -n "$v" && "$v" != "y" && "$v" != "n" ]] || continue
    grep -qxF -- "$v" <<< "$public_defaults" && continue
    values+="$k"$'\t'"$v"$'\n'
  done < "$f"
done
values="$(printf '%s' "$values" | sort -u)"

overall=0
for art in "$@"; do
  if [[ ! -f "$art" ]]; then echo "MISSING $art" >&2; overall=1; continue; fi
  leak=0

  # 1. build brand
  if grep -qF -- 'ML-BUILD-WITH-CREDENTIALS' "$art"; then
    echo "LEAK  $art: built with a credentials file (private build brand present)"; leak=1
  fi

  # 2. local credential values. Short values would match by accident anywhere,
  #    so they are required as a whole NUL-terminated C string.
  while IFS=$'\t' read -r k v; do
    [[ -n "$k" ]] || continue
    if [[ ${#v} -ge 6 ]]; then
      grep -qF -- "$v" "$art" && { echo "LEAK  $art: value of $k present"; leak=1; }
    else
      grep -qaP -- "\x00$(re_escape "$v")\x00" "$art" && { echo "LEAK  $art: value of $k present"; leak=1; }
    fi
  done <<< "$values"

  # 3. secret-shaped strings regardless of any credentials file
  grep -qaE -- 'tskey-(auth|client|api)-[A-Za-z0-9]+' "$art" && { echo "LEAK  $art: Tailscale key pattern present"; leak=1; }
  # header followed by a base64 body — the bare header is an mbedTLS parser constant
  grep -qazP -- '-----BEGIN [A-Z ]*PRIVATE KEY-----\s*[A-Za-z0-9+/=]{40,}' "$art" && { echo "LEAK  $art: PEM private key present"; leak=1; }
  grep -qaE -- '(^|[^A-Za-z])(Bearer|Basic) [A-Za-z0-9+/=_-]{16,}' "$art" && { echo "LEAK  $art: literal HTTP auth token present"; leak=1; }

  if [[ $leak -eq 0 ]]; then echo "clean $art"; else overall=1; fi
done

exit $overall
