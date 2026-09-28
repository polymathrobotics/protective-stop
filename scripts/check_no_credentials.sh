#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Polymath Robotics
# SPDX-License-Identifier: Apache-2.0
#
# Fails if the git index holds credentials: private keys, real Tailscale keys,
# secret CONFIG_ML_* values other than the example's placeholders, or credential
# files forced past .gitignore. Searches every staged/tracked path itself, so
# pre-commit's global exclude (vendored and archived code) does not hide any.
set -uo pipefail

status=0
git --no-pager grep --cached -n -I -P \
  -e '-----BEGIN [A-Z ]*PRIVATE KEY-----' \
  -e 'tskey-(auth|client|api)-(?!X{5})[A-Za-z0-9]{5,}-[A-Za-z0-9]{8,}' \
  -e '^CONFIG_ML_ADMIN_PASSWORD="(?!(your-admin-password)?")' \
  -e '^CONFIG_ML_TAILSCALE_AUTH_KEY="(?!(tskey-auth-X{5}[X-]*)?")' \
  -e '^CONFIG_ML_(WIFI_PASSWORD|OTA_API_KEY)="(?!")'
# 1 means no match; 0 is a match and anything else a failed search, so both fail.
[ "$?" -eq 1 ] || status=1
tracked=$(git ls-files --cached) || status=1
if grep -E '(^|/)(sdkconfig|sdkconfig\.credentials|primary\.pem|backup\.pem|fe_master\.txt|\.?netrc)$' <<<"$tracked"; then
  status=1
fi
[ "$status" -eq 0 ] || echo 'credentials must not be committed (scripts/check_no_credentials.sh)' >&2
exit "$status"
