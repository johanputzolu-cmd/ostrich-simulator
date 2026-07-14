#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

exec python3 "${SCRIPT_DIR}/honda_datalog_simulator_gui.py" "$@"
