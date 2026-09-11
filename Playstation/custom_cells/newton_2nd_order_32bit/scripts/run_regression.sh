#!/usr/bin/env bash
# =============================================================================
# Automated Spectre Simulation Regression Runner for All Custom Cells
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

source "$WS_DIR/env_virtuoso.sh"

echo "=========================================================================="
echo " Cadence Spectre Automated Simulation Regression"
echo " Target: All 12 Full-Custom Datapath Standard Cells"
echo "=========================================================================="

ALL_CELLS=(
    "booth_encoder"
    "booth_selector"
    "compressor_4to2"
    "csa_3to2_slice"
    "kogge_stone_cells"
    "cas_divider_slice"
    "srt_radix4_stage"
    "bitcell_8t_2r1w"
    "sram_6t_cell"
    "dynamic_overflow_detect"
    "fast_comparator"
    "tg_mux"
)

TOTAL=${#ALL_CELLS[@]}
PASSED=0
FAILED=0

for i in "${!ALL_CELLS[@]}"; do
    c="${ALL_CELLS[$i]}"
    idx=$((i + 1))
    echo "--------------------------------------------------------------------------"
    echo "[$idx/$TOTAL] Simulating cell: $c"
    echo "--------------------------------------------------------------------------"
    if bash "$SCRIPT_DIR/run_cell_sim.sh" "$c"; then
        PASSED=$((PASSED + 1))
    else
        echo "[ERROR] Simulation failed for $c"
        FAILED=$((FAILED + 1))
    fi
done

echo "=========================================================================="
echo " Regression Run Completed: $PASSED / $TOTAL Passed ($FAILED Failed)"
echo "=========================================================================="

# Compile reports & update signoff deliverable
echo "Generating comprehensive sign-off reports..."
python3 "$SCRIPT_DIR/gen_report.py" "$WS_DIR/work" "$WS_DIR/logs" "$WS_DIR/reports"

if [ "$FAILED" -eq 0 ]; then
    echo "[PASS] 100% Custom Cell Simulation Regression Achieved."
    exit 0
else
    echo "[FAIL] Regression completed with $FAILED failures."
    exit 1
fi
