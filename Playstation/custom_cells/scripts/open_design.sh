#!/usr/bin/env bash
# =============================================================================
# Open Specific Schematic or Layout View in Cadence Virtuoso GUI
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
source "$WS_DIR/config/env_virtuoso.sh"

CELL_INPUT="${1:-compressor_4to2}"
VIEW_TYPE="${2:-schematic}"

LIB_NAME="custom_cells_oa"

# Map cell name to actual top-level subcircuit name in custom_cells_oa
case "$CELL_INPUT" in
    *booth_selector*)
        TARGET_CELL="BOOTH_SELECTOR"
        ;;
    *booth_encoder*)
        TARGET_CELL="BOOTH_ENCODER"
        ;;
    *compressor*)
        TARGET_CELL="COMPRESSOR_4TO2"
        ;;
    *csa*|*fused*)
        TARGET_CELL="CSA_3TO2_SLICE"
        ;;
    *cas*)
        TARGET_CELL="CAS_CELL"
        ;;
    *srt*)
        TARGET_CELL="CAS_CELL"
        ;;
    *kogge*|*black*)
        TARGET_CELL="BLACK_CELL"
        ;;
    *gray*)
        TARGET_CELL="GRAY_CELL"
        ;;
    *pg_gen*)
        TARGET_CELL="PG_GEN_CELL"
        ;;
    *sum_cell*)
        TARGET_CELL="SUM_CELL"
        ;;
    *8t*|*bitcell*)
        TARGET_CELL="BITCELL_8T_2R1W"
        ;;
    *sram*|*6t*)
        TARGET_CELL="SRAM_6T_CELL"
        ;;
    *overflow*)
        TARGET_CELL="DYNAMIC_OVERFLOW_DETECT"
        ;;
    *comparator*)
        TARGET_CELL="CMP_BITSLICE"
        ;;
    *mux*)
        TARGET_CELL="TG_MUX2_1BIT"
        ;;
    *)
        TARGET_CELL=$(echo "$CELL_INPUT" | tr '[:lower:]' '[:upper:]')
        ;;
esac

echo "=========================================================================="
echo " Opening Cadence Virtuoso for Cell: $TARGET_CELL (View: $VIEW_TYPE)"
echo "=========================================================================="

mkdir -p "$WS_DIR/work" "$WS_DIR/logs"

# Ensure runtime cds.lib is updated
cp -f "$WS_DIR/config/cds.lib" "$WS_DIR/work/cds.lib"

REPLAY_SCRIPT="$WS_DIR/work/open_cell.il"

if [ "$VIEW_TYPE" == "schematic" ]; then
    cat << EOF > "$REPLAY_SCRIPT"
let((cv)
    cv = dbOpenCellViewByType("$LIB_NAME" "$TARGET_CELL" "schematic" "schematic" "r")
    if(cv then
        deOpenCellView("$LIB_NAME" "$TARGET_CELL" "schematic" "schematic" nil "r")
        printf("\n[SUCCESS] Opened Schematic View for %s in %s\n" "$TARGET_CELL" "$LIB_NAME")
    else
        printf("\n[WARNING] Cellview %s schematic not found in %s\n" "$TARGET_CELL" "$LIB_NAME")
    )
)
EOF
elif [ "$VIEW_TYPE" == "layout" ]; then
    cat << EOF > "$REPLAY_SCRIPT"
let((cv)
    cv = dbOpenCellViewByType("$LIB_NAME" "$TARGET_CELL" "layout" "maskLayout" "a")
    unless(cv
        cv = dbOpenCellViewByType("$LIB_NAME" "$TARGET_CELL" "layout" "maskLayout" "w")
    )
    if(cv then
        deOpenCellView("$LIB_NAME" "$TARGET_CELL" "layout" "maskLayout" nil "a")
        printf("\n[SUCCESS] Opened Layout View for %s in %s\n" "$TARGET_CELL" "$LIB_NAME")
    else
        printf("\n[WARNING] Could not open Layout for %s in %s\n" "$TARGET_CELL" "$LIB_NAME")
    )
)
EOF
fi

cd "$WS_DIR/work"
virtuoso -log "$WS_DIR/logs/virtuoso_${TARGET_CELL}.log" -replay "$REPLAY_SCRIPT" &
echo "[SUCCESS] Virtuoso window launched for $TARGET_CELL ($VIEW_TYPE)."
