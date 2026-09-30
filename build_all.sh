#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${project_root}/build/all"

cmake -S "${project_root}" -B "${build_dir}" \
    -DHAL_BUILD_STATIC=ON \
    -DHAL_BUILD_HARDWARE_TEST=ON \
    -DBUILD_TESTING=ON
cmake --build "${build_dir}" --parallel "${JOBS:-2}"
python3 "${project_root}/test/run_driver_tests.py" \
    --build-only --output-dir "${build_dir}/test_cases"

printf '静态库: %s/libcnc_hal.a\n' "${build_dir}"
printf '共享库: %s/libcnc_hal.so\n' "${build_dir}"
printf 'test: %s/hal_public_link、%s/hal_static_link 和 %s/test_cases/\n' \
    "${build_dir}" "${build_dir}" "${build_dir}"
printf 'examples: %s/hal_hardware_test\n' "${build_dir}"
printf '运行离线用例: ctest --test-dir %s --output-on-failure\n' "${build_dir}"
