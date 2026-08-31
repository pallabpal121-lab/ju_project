#!/bin/bash
# ==============================================================================
# Script: Launch Synopsys Verdi Debugger
# Usage : ./scripts/run_verdi.sh
# ==============================================================================
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${ROOT_DIR}"
make verdi
