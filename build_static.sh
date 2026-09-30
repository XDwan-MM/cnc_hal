#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${project_root}/build/static"

cmake -S "${project_root}" -B "${build_dir}" \
    -DHAL_BUILD_STATIC=ON \
    -DHAL_BUILD_HARDWARE_TEST=OFF \
    -DBUILD_TESTING=OFF
cmake --build "${build_dir}" --target cnc_hal_static --parallel "${JOBS:-2}"

printf 'HAL 静态库: %s\n' "${build_dir}/libcnc_hal.a"
