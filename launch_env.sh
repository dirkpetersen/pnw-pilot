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
  export AGNOS_VERSION="17.2"
fi

export STAGING_ROOT="/data/safe_staging"

# connect2pnw: self-hosted upload gateway (AWS API Gateway -> Lambda presign -> s3://comma-connect).
# Belt-and-suspenders alongside the common/api.py default. If unset, openpilot falls back to comma's
# api.commadotai.com, which 412s every proactive upload -> files get stamped "uploaded" without ever
# reaching S3 (silent data loss). See CONNECT2XNOR.md / DEVICE-STATE.md.
export API_HOST="https://jh69za4byd.execute-api.us-west-2.amazonaws.com"
