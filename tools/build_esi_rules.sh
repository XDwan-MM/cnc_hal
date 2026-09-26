#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output="${1:-${root}/build/esi_rules}"
mkdir -p "$(dirname "${output}")"
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -O2 -I"${root}/src" \
    "${root}/tools/esi_rules.c" "${root}/src/common/devdict.c" -o "${output}"
printf '%s\n' "${output}"
