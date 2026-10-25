#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.9"
fi

export STAGING_ROOT="/data/safe_staging"

# 2022 Asia-Pacific Outback (angle-based EyeSight): its ECU versions aren't in the fingerprint list,
# so force the platform the same way jacobwaller's branch does. A car picked in sunnypilot's
# vehicle settings (CarPlatformBundle) takes priority over this.
export FINGERPRINT="SUBARU_CROSSTREK_2025"
