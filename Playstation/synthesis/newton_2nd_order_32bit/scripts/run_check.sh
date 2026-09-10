#!/bin/bash
# ==============================================================================
# Script: Run RTL Analyze, Elaborate, Link and Lint Check
# Usage : ./scripts/run_check.sh
# ==============================================================================
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${ROOT_DIR}"
make check_design
