#!/bin/bash
# ==============================================================================
# Script: Launch Synopsys Design Vision GUI
# Usage : ./scripts/run_gui.sh
# ==============================================================================
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "${ROOT_DIR}"
make gui
