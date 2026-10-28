#!/usr/bin/env bash

set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
writable_root="${CAPELLA_COLCON_ROOT:-/tmp/capella_coverage_server_colcon}"

mkdir -p "${writable_root}"

exec colcon \
  --log-base "${writable_root}/log" \
  build \
  --base-paths "${workspace_root}" \
  --build-base "${writable_root}/build" \
  --install-base "${writable_root}/install" \
  --test-result-base "${writable_root}/test_results" \
  --merge-install \
  --symlink-install \
  --parallel-workers "${CAPELLA_COLCON_WORKERS:-1}" \
  "$@"
