#!/bin/bash
# ==============================================================================
# Script: Run Full UVM Test Suite Regression
# Usage : ./scripts/run_regression.sh
# ==============================================================================
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${ROOT_DIR}"
make regression
