#!/usr/bin/env bash
# =============================================================================
# Launch Cadence ViVA Waveform Viewer for Simulation Dataset
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
source "$WS_DIR/config/env_virtuoso.sh"

CELL_NAME="${1:-compressor_4to2}"
RAW_PATH="$WS_DIR/outputs/${CELL_NAME}.raw"

if [ ! -d "$RAW_PATH" ]; then
    echo "[INFO] No waveform dataset found at: $RAW_PATH"
    echo "[INFO] Running simulation first..."
    bash "$WS_DIR/scripts/run_cell_sim.sh" "$CELL_NAME"
fi

echo "=========================================================================="
echo " Opening Cadence ViVA Waveform Viewer: $CELL_NAME"
echo " Dataset: $RAW_PATH"
echo "=========================================================================="

cd "$WS_DIR/work"
viva -raw "$RAW_PATH" &
echo "[SUCCESS] ViVA waveform viewer launched in background."
