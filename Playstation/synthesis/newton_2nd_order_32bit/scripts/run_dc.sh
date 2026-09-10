#!/bin/bash
# ==============================================================================
# Script: Run Synthesis Batch Flow using Synopsys Design Compiler
# Usage : ./scripts/run_dc.sh [CLK_PERIOD] [TECH_NODE] [CORNER]
# ==============================================================================
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CLK_PERIOD="${1:-10.0}"
TECH_NODE="${2:-SCL180}"
CORNER="${3:-typical}"

cd "${ROOT_DIR}"
make syn CLK_PERIOD="${CLK_PERIOD}" TECH_NODE="${TECH_NODE}" CORNER="${CORNER}"
