#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")"
python3 server.py --host 0.0.0.0 --port "${PORT:-8080}"

