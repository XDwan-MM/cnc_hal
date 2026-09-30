#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${project_root}/build/test"

cmake -S "${project_root}" -B "${build_dir}" \
    -DHAL_BUILD_STATIC=OFF \
    -DHAL_BUILD_HARDWARE_TEST=OFF \
    -DBUILD_TESTING=ON
cmake --build "${build_dir}" --target hal_public_link --parallel "${JOBS:-2}"
python3 "${project_root}/test/run_driver_tests.py" \
    --build-only --output-dir "${build_dir}/cases"

printf 'test 可执行文件: %s/hal_public_link, %s/cases/\n' "${build_dir}" "${build_dir}"
printf '运行离线用例: ctest --test-dir %s --output-on-failure\n' "${build_dir}"
