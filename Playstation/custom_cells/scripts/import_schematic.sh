#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
source "$WS_DIR/config/env_virtuoso.sh"

CELL_NAME="${1:-booth_selector_cell}"

echo "=========================================================================="
echo " Cadence SpiceIn: Importing Schematic for $CELL_NAME"
echo "=========================================================================="

SPICE_FILE=$(find "$WS_DIR" -name "schematic.spice" | grep -i "$CELL_NAME" | head -n 1 || true)

if [ -z "$SPICE_FILE" ]; then
    echo "[ERROR] schematic.spice not found for cell: $CELL_NAME"
    exit 1
fi

LIB_NAME="custom_cells_oa"
mkdir -p "$WS_DIR/work/$LIB_NAME"

cd "$WS_DIR/work"

spiceIn \
    -netlistFile "$SPICE_FILE" \
    -outputLib "$LIB_NAME" \
    -reflibList "analogLib" \
    -devmapFile "$WS_DIR/config/devmap.txt" \
    -language "SPICE"

# Ensure cds.lib in work/ defines custom_cells_oa
if ! grep -q "$LIB_NAME" "$WS_DIR/work/cds.lib" 2>/dev/null; then
    echo "DEFINE $LIB_NAME $WS_DIR/work/$LIB_NAME" >> "$WS_DIR/work/cds.lib"
fi
if ! grep -q "$LIB_NAME" "$WS_DIR/config/cds.lib" 2>/dev/null; then
    echo "DEFINE $LIB_NAME $WS_DIR/work/$LIB_NAME" >> "$WS_DIR/config/cds.lib"
fi

echo "[SUCCESS] Schematic successfully imported into OpenAccess Library: $LIB_NAME"
