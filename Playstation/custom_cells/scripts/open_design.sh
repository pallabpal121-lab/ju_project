#!/usr/bin/env bash
# =============================================================================
# Open Specific Schematic or Layout in Cadence Virtuoso GUI
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
source "$WS_DIR/config/env_virtuoso.sh"

CELL_INPUT="${1:-booth_selector_cell}"
VIEW_TYPE="${2:-schematic}"

LIB_NAME="custom_cells_oa"

# Map folder name to actual top-level subcircuit name in custom_cells_oa
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
    *8t*|*bitcell*)
        TARGET_CELL="BITCELL_8T_2R1W"
        ;;
    *sram*|*6t*)
        TARGET_CELL="SRAM_6T_CELL"
        ;;
    *)
        TARGET_CELL=$(echo "$CELL_INPUT" | tr '[:lower:]' '[:upper:]')
        ;;
esac

echo "=========================================================================="
echo " Opening Cadence Virtuoso for Cell: $TARGET_CELL (View: $VIEW_TYPE)"
echo "=========================================================================="

mkdir -p "$WS_DIR/work" "$WS_DIR/logs"

REPLAY_SCRIPT="$WS_DIR/work/open_cell.il"

if [ "$VIEW_TYPE" == "schematic" ]; then
    cat << EOF > "$REPLAY_SCRIPT"
let((cv)
    cv = dbOpenCellViewByType("$LIB_NAME" "$TARGET_CELL" "schematic" "schematic" "r")
    if(cv then
        deOpenCellView("$LIB_NAME" "$TARGET_CELL" "schematic" "schematic" nil "r")
        printf("\n[SUCCESS] Opened Schematic View for %s\n" "$TARGET_CELL")
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
        printf("\n[SUCCESS] Opened Layout View for %s\n" "$TARGET_CELL")
    else
        printf("\n[WARNING] Could not open Layout for %s\n" "$TARGET_CELL")
    )
)
EOF
fi

cd "$WS_DIR/work"
virtuoso -log "$WS_DIR/logs/virtuoso_${TARGET_CELL}.log" -replay "$REPLAY_SCRIPT" &
echo "[SUCCESS] Virtuoso window launched for $TARGET_CELL ($VIEW_TYPE)."
