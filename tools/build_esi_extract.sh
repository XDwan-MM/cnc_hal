#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output="${1:-${root}/build/esi_extract}"
mkdir -p "$(dirname "${output}")"

"${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror -O2 \
    -ffunction-sections -fdata-sections \
    -I"${root}/third_party/kickcat_esi/lib/include" \
    -I"${root}/third_party/kickcat_esi/lib/include/kickcat" \
    -I"${root}/third_party/tinyxml2" \
    "${root}/tools/esi_extract.cpp" \
    "${root}/third_party/kickcat_esi/lib/src/ESI/Parser.cc" \
    "${root}/third_party/kickcat_esi/lib/src/CoE/OD.cc" \
    "${root}/third_party/kickcat_esi/lib/src/protocol.cc" \
    "${root}/third_party/tinyxml2/tinyxml2.cpp" \
    -Wl,--gc-sections -o "${output}"
printf '%s\n' "${output}"
