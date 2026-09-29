# ==============================================================================
# Script 02: Read, Analyze, Elaborate, Link & Lint RTL Design
# File   : 02_read_design.tcl
# Role   : Step 1 - RTL Ingestion and GTECH Generic Netlist Generation
# Design : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# ==============================================================================
# 
# [BEGINNER GUIDE - WHY DO WE NEED THIS FILE?]
# This script executes the first major phase of ASIC synthesis:
# 1. analyze      : Reads SystemVerilog source files, checks syntax, and stores
#                   intermediate binary structures into the WORK library.
#                   (Allows packages to be compiled before modules that import them).
# 2. elaborate    : Resolves parameters, binds module ports, infers registers
#                   (flip-flops), and creates generic, technology-independent gates (GTECH).
# 3. current_design: Tells Design Compiler which module is the top-level entity.
# 4. link         : Connects all sub-modules together and ensures all references
#                   resolve to known libraries (prevents missing black boxes).
# 5. check_design : Automated lint checker that detects unintentional latches,
#                   combinational loops, undriven pins, and multi-driven nets.
# 6. write ddc    : Saves the unmapped GTECH database as an intermediate checkpoint.
# ==============================================================================

echo "======================================================================"
echo " (02_read_design.tcl) Step 1: Reading RTL Design Files (Analyze & Elaborate)"
echo "======================================================================"

# ------------------------------------------------------------------------------
# 1. Load SystemVerilog Filelist
# ------------------------------------------------------------------------------
# Sourcing rtl_syn.tcl defines ${RTL_SOURCE_FILES} in proper hierarchical order:
# (packages first -> arithmetic units -> equation engines -> top module).
source "${FILELIST_DIR}/rtl_syn.tcl"

# ------------------------------------------------------------------------------
# 2. HDL Compiler Settings & WORK Library Binding
# ------------------------------------------------------------------------------
# Enable standard package chaining and prevent unintended latch inference
set hdlin_sv_packages chain
set hdlin_check_no_latch true

# Bind WORK library to the ./work directory on disk
define_design_lib WORK -path ${WORK_DIR}

# ------------------------------------------------------------------------------
# 3. Step 1A - Analyze (Syntax & Intermediate Compilation)
# ------------------------------------------------------------------------------
echo "INFO: Analyzing SystemVerilog design files into WORK library..."
analyze -format sverilog -work WORK ${RTL_SOURCE_FILES}

# ------------------------------------------------------------------------------
# 4. Step 1B - Elaborate (Hierarchy & GTECH Building)
# ------------------------------------------------------------------------------
echo "INFO: Elaborating top-level design '${DESIGN_NAME}'..."
elaborate ${DESIGN_NAME} -work WORK

# ------------------------------------------------------------------------------
# 5. Step 1C - Set Current Design Context
# ------------------------------------------------------------------------------
# Direct all subsequent commands and constraints to the top-level module.
current_design ${DESIGN_NAME}

# ------------------------------------------------------------------------------
# 6. Step 1D - Link Design
# ------------------------------------------------------------------------------
# Resolves all cell instances, sub-modules, and DesignWare operators against
# target_library and synthetic_library. All blocks must successfully link.
echo "INFO: Linking design hierarchy to standard cell libraries..."
link

# ------------------------------------------------------------------------------
# 7. Step 1E - Structural Lint Checks (check_design)
# ------------------------------------------------------------------------------
# Detects unconnected pins, multi-driven nets, inferred latches, and combinational loops.
echo "INFO: Performing comprehensive design lint checks..."
check_design -multiple_designs > "${REPORTS_CHECKS_DIR}/check_design.rpt"

# ------------------------------------------------------------------------------
# 8. Step 1F - Save Unmapped Generic Checkpoint Database (.ddc)
# ------------------------------------------------------------------------------
# Saves the GTECH hierarchy so you can reload the design later without re-analyzing.
write -format ddc -hierarchy -output "${OUTPUTS_DB_DIR}/${DESIGN_NAME}_unmapped.ddc"

echo "INFO: Reading, elaborating, linking, and lint checks completed successfully."
echo "======================================================================"
