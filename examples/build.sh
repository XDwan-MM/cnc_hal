#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${project_root}/build/examples"

cmake -S "${project_root}" -B "${build_dir}" \
    -DHAL_BUILD_STATIC=OFF \
    -DHAL_BUILD_HARDWARE_TEST=ON \
    -DBUILD_TESTING=OFF
cmake --build "${build_dir}" --target hal_hardware_test --parallel "${JOBS:-2}"

printf 'examples 可执行文件: %s\n' "${build_dir}/hal_hardware_test"
printf '无需硬件的自检: %s selftest\n' "${build_dir}/hal_hardware_test"
