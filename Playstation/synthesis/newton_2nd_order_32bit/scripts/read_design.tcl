# ==============================================================================
# Reading, Analyzing, Elaborating, and Linking Design (read_design.tcl)
# ==============================================================================

echo "======================================================================"
echo " Step 1: Reading RTL Design Files (Analyze & Elaborate)"
echo "======================================================================"

# Source RTL filelist
source "${FILELIST_DIR}/rtl_syn.tcl"

# Define SystemVerilog HDL compiler settings
set hdlin_sv_packages true
set hdlin_check_no_latch true
set hdlin_enable_presto_for_vhdl true

# 1. Analyze: Parse HDL syntax and generate intermediate representation
echo "INFO: Analyzing SystemVerilog design files..."
analyze -format sverilog ${RTL_SOURCE_FILES}

# 2. Elaborate: Build generic logic hierarchy (GTECH) and resolve parameters
echo "INFO: Elaborating top-level design '${DESIGN_NAME}'..."
elaborate ${DESIGN_NAME}

# Set current working design
current_design ${DESIGN_NAME}

# 3. Link: Resolve all cell instances to target/synthetic libraries
echo "INFO: Linking design to standard cell libraries..."
link

# 4. Check Design: Detect unconnected pins, multiple drivers, latches, combinational loops
echo "INFO: Performing comprehensive design lint checks..."
check_design -multiple_designs > "${REPORTS_DIR}/check_design.rpt"

# 5. Save intermediate unmapped generic netlist
write -format ddc -hierarchy -output "${OUTPUTS_DIR}/${DESIGN_NAME}_unmapped.ddc"

echo "INFO: Reading and linking completed successfully."
echo "======================================================================"
