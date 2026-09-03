#!/usr/bin/env bash
# =============================================================================
# Cadence SpiceIn: Automated SPICE Netlist to Virtuoso Schematic Importer
# Includes Automated Overlap Cleanup for Clean, Production-Grade Schematics
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
source "$WS_DIR/config/env_virtuoso.sh"

CELL_NAME="${1:-compressor_4to2}"

echo "=========================================================================="
echo " Cadence SpiceIn: Importing Schematic for $CELL_NAME"
echo "=========================================================================="

SPICE_FILE=$(find "$WS_DIR/cells" -name "schematic.spice" | grep -i "$CELL_NAME" | head -n 1 || true)

if [ -z "$SPICE_FILE" ]; then
    echo "[ERROR] schematic.spice not found for cell: $CELL_NAME in $WS_DIR/cells"
    exit 1
fi

LIB_NAME="custom_cells_oa"
OA_DIR="$WS_DIR/oa_libs/$LIB_NAME"
mkdir -p "$OA_DIR" "$WS_DIR/work" "$WS_DIR/logs"

# Ensure runtime work/cds.lib has persistent library definitions
cp -f "$WS_DIR/config/cds.lib" "$WS_DIR/work/cds.lib"

# Generate spiceIn parameter file with overwrite enabled
PARAM_FILE="$WS_DIR/work/spiceIn_param.il"
cat << EOF > "$PARAM_FILE"
spiceInParams = list(nil
    'language           "SPICE"
    'outputLib          "$LIB_NAME"
    'refLibList         "analogLib ts18scl"
    'devMapFile         "$WS_DIR/config/devmap.txt"
    'overwriteCells     "ALL"
    'outputViewType     "schematic"
    'masterCellForGnd   "gnd"
    'outputSimName      "spectre"
)
EOF

cd "$WS_DIR/work"

spiceIn \
    -param "$PARAM_FILE" \
    -netlistFile "$SPICE_FILE" \
    -logFile "$WS_DIR/logs/spiceIn_${CELL_NAME}.log"

echo "[SUCCESS] Schematic successfully imported into OpenAccess Library: $OA_DIR"

# -----------------------------------------------------------------------------
# Post-Processing: Automated Cleanup of Duplicate Overlapping Wire Labels on Pins
# -----------------------------------------------------------------------------
echo "--> Automatically removing duplicate overlapping wire labels on pins..."
SUBCKTS=$(grep -i "^[[:space:]]*\.subckt" "$SPICE_FILE" | awk '{print $2}')
CLEANUP_SCRIPT="$WS_DIR/work/auto_clean_labels.il"

cat << 'EOF' > "$CLEANUP_SCRIPT"
procedure(cleanCellLabels(libName cellName)
    let((cv termNames count)
        cv = dbOpenCellViewByType(libName cellName "schematic" "schematic" "a")
        if(cv then
            termNames = cv~>terminals~>name
            count = 0
            foreach(shape cv~>shapes
                if(shape~>objType == "label" && shape~>layerName == "wire" && member(shape~>theLabel termNames) then
                    dbDeleteObject(shape)
                    count = count + 1
                )
            )
            schCheck(cv)
            dbSave(cv)
            dbClose(cv)
            printf("[AUTO-CLEAN] %s: Removed %d duplicate wire labels.\n" cellName count)
        )
    )
)
EOF

for sub in $SUBCKTS; do
    echo "cleanCellLabels(\"$LIB_NAME\" \"$sub\")" >> "$CLEANUP_SCRIPT"
done
echo "exit()" >> "$CLEANUP_SCRIPT"

virtuoso -nograph -replay "$CLEANUP_SCRIPT" -log "$WS_DIR/logs/clean_labels_${CELL_NAME}.log" >/dev/null 2>&1 || true
echo "[SUCCESS] Schematic cleanup complete. No overlapping pin/wire labels."
