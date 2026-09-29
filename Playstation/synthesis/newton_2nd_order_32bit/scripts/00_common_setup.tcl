# ==============================================================================
# Script 00: Common Synthesis Environment & Design Parameters
# File   : 00_common_setup.tcl
# Role   : Step 0A - Master Parameter Definition
# Design : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# ==============================================================================
# 
# [BEGINNER GUIDE - WHY DO WE NEED THIS FILE?]
# Before running any synthesis commands, we must declare global parameters such
# as the top module name, clock frequency, silicon technology node, and operating
# voltage/temperature corner. Defining these in one central file allows you to
# easily modify synthesis settings without editing the underlying tool scripts.
# ==============================================================================

# ------------------------------------------------------------------------------
# 1. Top-Level Design Entity Name
# ------------------------------------------------------------------------------
# This tells Design Compiler which SystemVerilog module is the top of the design.
set DESIGN_NAME "newton_2nd_order_top"

# ------------------------------------------------------------------------------
# 2. Clock Specifications & Frequency Target
# ------------------------------------------------------------------------------
# Default Target: 10.0 ns period = 100.0 MHz
# Formula: Frequency (MHz) = 1000 / Period (ns)
# If an environment variable SYNTH_CLK_PERIOD is set from the Makefile/shell,
# we use that value; otherwise, default to 10.0 ns.
if {[info exists ::env(SYNTH_CLK_PERIOD)]} {
    set CLK_PERIOD $::env(SYNTH_CLK_PERIOD)
} else {
    set CLK_PERIOD 10.0
}

# Clock Uncertainty (Jitter + Margin):
# Allocates a 200 ps (0.2 ns) timing guardband for on-chip clock jitter and skew.
set CLK_UNCERTAINTY 0.200

# Clock Transition (Slew Rate):
# Specifies the 100 ps (0.1 ns) rise and fall slew time of the incoming clock edge.
set CLK_TRANSITION  0.100

# ------------------------------------------------------------------------------
# 3. Technology Node & Operating Corner Selection
# ------------------------------------------------------------------------------
# Target Foundry Node: "SCL180" (Commercial 180nm) or "NANGATE45" (Open-source 45nm)
if {[info exists ::env(TECH_NODE)]} {
    set TECH_NODE $::env(TECH_NODE)
} else {
    set TECH_NODE "SCL180"
}

# Operating PVT Corner:
# - "typical" : Nominal silicon (1.8 V, 25 C)
# - "slow"    : Worst-case setup corner (1.62 V, 125 C)
# - "fast"    : Best-case hold corner (1.98 V, -40 C)
if {[info exists ::env(CORNER)]} {
    set CORNER $::env(CORNER)
} else {
    set CORNER "typical"
}

# ------------------------------------------------------------------------------
# 4. Project Directory Structure
# ------------------------------------------------------------------------------
# Maintain clean, organized folders for scripts, constraints, reports, and deliverables.
set WORK_DIR        "./work"
set LOGS_DIR        "./logs"
set REPORTS_DIR     "./reports"
set OUTPUTS_DIR     "./outputs"
set SCRIPTS_DIR     "./scripts"
set CONSTRAINTS_DIR "./constraints"
set FILELIST_DIR    "./filelist"

# Categorized subdirectories for detailed sign-off reports
set REPORTS_TIMING_DIR "${REPORTS_DIR}/timing"
set REPORTS_AREA_DIR   "${REPORTS_DIR}/area"
set REPORTS_POWER_DIR  "${REPORTS_DIR}/power"
set REPORTS_CHECKS_DIR "${REPORTS_DIR}/checks"

# Categorized subdirectories for downstream backend deliverables
set OUTPUTS_NETLIST_DIR     "${OUTPUTS_DIR}/netlist"
set OUTPUTS_CONSTRAINTS_DIR "${OUTPUTS_DIR}/constraints"
set OUTPUTS_DELAYS_DIR      "${OUTPUTS_DIR}/delays"
set OUTPUTS_DB_DIR          "${OUTPUTS_DIR}/db"

# Create all necessary directories on disk if they do not already exist
file mkdir ${WORK_DIR}
file mkdir ${LOGS_DIR}
file mkdir ${REPORTS_DIR}
file mkdir ${OUTPUTS_DIR}
file mkdir ${REPORTS_TIMING_DIR}
file mkdir ${REPORTS_AREA_DIR}
file mkdir ${REPORTS_POWER_DIR}
file mkdir ${REPORTS_CHECKS_DIR}
file mkdir ${OUTPUTS_NETLIST_DIR}
file mkdir ${OUTPUTS_CONSTRAINTS_DIR}
file mkdir ${OUTPUTS_DELAYS_DIR}
file mkdir ${OUTPUTS_DB_DIR}

# ------------------------------------------------------------------------------
# 5. Display Configuration Summary to User
# ------------------------------------------------------------------------------
echo "======================================================================"
echo " (00_common_setup.tcl) Design Synthesis Parameters Configured:"
echo "  Top Design       : ${DESIGN_NAME}"
echo "  Technology Node  : ${TECH_NODE}"
echo "  Operating Corner : ${CORNER}"
echo "  Target Period    : ${CLK_PERIOD} ns ([expr {1000.0 / ${CLK_PERIOD}}] MHz)"
echo "  Clock Margin     : Uncertainty = ${CLK_UNCERTAINTY} ns, Slew = ${CLK_TRANSITION} ns"
echo "======================================================================"
