#!/usr/bin/env bash
# =============================================================================
# Cadence Spectre Automated Simulation Runner for Custom Cells
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

source "$WS_DIR/config/env_virtuoso.sh"

CELL_NAME="${1:-booth_selector_cell}"

echo "=========================================================================="
echo " Cadence Spectre Transient Simulation: $CELL_NAME"
echo "=========================================================================="

mkdir -p "$WS_DIR/logs" "$WS_DIR/reports" "$WS_DIR/outputs" "$WS_DIR/work"

# Locate testbench
TB_PATH=$(find "$WS_DIR" -name "tb_*.spice" -o -name "tb_*.scs" -o -name "tb_*.sp" | grep -i "$CELL_NAME" | head -n 1 || true)

if [ -z "$TB_PATH" ]; then
    echo "[ERROR] Testbench not found for cell: $CELL_NAME"
    exit 1
fi

TB_DIR="$(dirname "$TB_PATH")"
TB_FILE="$(basename "$TB_PATH")"

RUN_DECK="$WS_DIR/work/${CELL_NAME}_run.spice"
LOG_FILE="$WS_DIR/logs/${CELL_NAME}_spectre.log"
REPORT_FILE="$WS_DIR/reports/${CELL_NAME}_report.txt"
RAW_DIR="$WS_DIR/outputs/${CELL_NAME}.raw"

# Assemble top-level simulation deck with models
cat << EOF > "$RUN_DECK"
simulator lang=spice
.include "$WS_DIR/config/models.spice"
.include "$TB_PATH"
.end
EOF

echo "Testbench Source : $TB_PATH"
echo "Simulation Deck  : $RUN_DECK"
echo "Spectre Log      : $LOG_FILE"
echo "Waveform Output  : $RAW_DIR"
echo "Analysis Report  : $REPORT_FILE"

cd "$WS_DIR/work"

# Run Spectre
spectre -64 \
    +log "$LOG_FILE" \
    -format psfbin \
    -raw "$RAW_DIR" \
    "$RUN_DECK"

# Generate Measurement & Verification Report
echo "==========================================================================" > "$REPORT_FILE"
echo " CADENCE SPECTRE SIMULATION REPORT" >> "$REPORT_FILE"
echo " Cell Design     : $CELL_NAME" >> "$REPORT_FILE"
echo " Timestamp       : $(date)" >> "$REPORT_FILE"
echo " Simulator       : Cadence Spectre (21.1.0 64-bit)" >> "$REPORT_FILE"
echo " Testbench File  : $TB_PATH" >> "$REPORT_FILE"
echo " Log Location    : $LOG_FILE" >> "$REPORT_FILE"
echo " Waveform Data   : $RAW_DIR" >> "$REPORT_FILE"
echo "==========================================================================" >> "$REPORT_FILE"
echo "" >> "$REPORT_FILE"
echo "=== Simulation Metrics ===" >> "$REPORT_FILE"
grep -E "Number of accepted tran steps|Transient Analysis|Maximum value achieved|Intrinsic tran analysis time" "$LOG_FILE" >> "$REPORT_FILE" || true
if [ -f "$WS_DIR/work/${CELL_NAME}_run.measure" ]; then
    echo "" >> "$REPORT_FILE"
    echo "=== Timing & Delay Measurements (.measure) ===" >> "$REPORT_FILE"
    grep "=" "$WS_DIR/work/${CELL_NAME}_run.measure" | grep -v -E "date|design|version|Measurement|Analysis" >> "$REPORT_FILE" || true
fi
echo "" >> "$REPORT_FILE"
echo "=== Status ===" >> "$REPORT_FILE"
grep -E "spectre completes with" "$LOG_FILE" >> "$REPORT_FILE" || true

echo "--------------------------------------------------------------------------"
echo "[SUCCESS] Simulation completed successfully for $CELL_NAME!"
echo "Log file: $LOG_FILE"
echo "Report  : $REPORT_FILE"
