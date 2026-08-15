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
  export AGNOS_VERSION="19.6"
fi

export STAGING_ROOT="/data/safe_staging"

# connectsel2pnw: connect backend selector (ConnectBackend param; see common/connect_backend.py).
# openpilot reads API_HOST / ATHENA_HOST everywhere it talks to a backend (common/api.py,
# system/athena/athenad.py, registration, uploader), so exporting them here redirects all of it in
# one place. Takes effect on reboot.
#   0 / unset = PNW self-hosted (default — MUST keep exporting the gateway below, see the 412 note)
#   1         = Konik Stable (api.konik.ai / athena.konik.ai)
#   2         = Custom https:// base URL from ConnectCustomUrl (falls back to PNW if unset/invalid)
#   3         = Offline Mode (RFC 2606 .invalid hosts — uploads/athena can never egress)
CONNECT_BACKEND="$(cat /data/params/d/ConnectBackend 2>/dev/null)"
CONNECT_CUSTOM_URL="$(cat /data/params/d/ConnectCustomUrl 2>/dev/null)"
# connectsel2pnw: this validation must accept/reject IDENTICALLY to common/connect_backend.py's
# valid_custom_url() (finding #2) -- no leading/trailing whitespace, literal https:// prefix, non-
# empty host -- or the two sides can pick different backends for the same stored param value. A
# literal `${VAR#https://}` prefix match already rejects LEADING whitespace on its own (no match),
# but does nothing about TRAILING whitespace or an empty host ("https://" or "https:///"), so both
# are checked explicitly here. Trim via bash's leading/trailing [[:space:]] glob-strip idiom (no
# external `sed`/`awk` dependency) and compare to the untrimmed value: any difference means
# whitespace was present, matching Python's `url != url.strip()` check.
CONNECT_CUSTOM_URL_TRIMMED="${CONNECT_CUSTOM_URL#"${CONNECT_CUSTOM_URL%%[![:space:]]*}"}"
CONNECT_CUSTOM_URL_TRIMMED="${CONNECT_CUSTOM_URL_TRIMMED%"${CONNECT_CUSTOM_URL_TRIMMED##*[![:space:]]}"}"
CONNECT_CUSTOM_HOST="${CONNECT_CUSTOM_URL#https://}"
CONNECT_CUSTOM_HOST="${CONNECT_CUSTOM_HOST%%/*}"
if [ "$CONNECT_BACKEND" = "1" ]; then
  export API_HOST="https://api.konik.ai"
  export ATHENA_HOST="wss://athena.konik.ai"
elif [ "$CONNECT_BACKEND" = "3" ]; then
  export API_HOST="https://api.invalid"
  export ATHENA_HOST="wss://athena.invalid"
elif [ "$CONNECT_BACKEND" = "2" ] && [ "$CONNECT_CUSTOM_URL" = "$CONNECT_CUSTOM_URL_TRIMMED" ] \
     && [ "${CONNECT_CUSTOM_URL#https://}" != "$CONNECT_CUSTOM_URL" ] && [ -n "$CONNECT_CUSTOM_HOST" ]; then
  # athena lives on the custom URL's host (retropilot-style backends serve /ws/v2/ off the same host)
  export API_HOST="${CONNECT_CUSTOM_URL%/}"
  export ATHENA_HOST="wss://${CONNECT_CUSTOM_HOST}"
else
  # connect2pnw: self-hosted upload gateway (AWS API Gateway -> Lambda presign -> s3://comma-connect).
  # Belt-and-suspenders alongside the common/api.py default. If unset, openpilot falls back to comma's
  # api.commadotai.com, which 412s every proactive upload -> files get stamped "uploaded" without ever
  # reaching S3 (silent data loss). See CONNECT2XNOR.md / DEVICE-STATE.md.
  # connectsel2pnw: ATHENA_HOST deliberately NOT exported here — stock default preserved (unchanged behavior).
  export API_HOST="https://jh69za4byd.execute-api.us-west-2.amazonaws.com"
fi
