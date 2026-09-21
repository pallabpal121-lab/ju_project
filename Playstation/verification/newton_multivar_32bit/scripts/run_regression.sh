#!/bin/bash
# ==============================================================================
# Script: Run Complete UVM Regression Suite
# Usage : ./scripts/run_regression.sh
# ==============================================================================
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${ROOT_DIR}"
make regression
