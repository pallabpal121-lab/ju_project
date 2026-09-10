# ==============================================================================
# Common Design & Flow Parameters (common_setup.tcl)
# Design   : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# ==============================================================================

# Top-level Design Entity
set DESIGN_NAME "newton_2nd_order_top"

# Clock Specifications (Clock Period in nanoseconds, default: 10.0 ns = 100 MHz)
if {[info exists ::env(SYNTH_CLK_PERIOD)]} {
    set CLK_PERIOD $::env(SYNTH_CLK_PERIOD)
} else {
    set CLK_PERIOD 10.0
}

set CLK_UNCERTAINTY 0.200
set CLK_TRANSITION  0.100

# Target Technology Selection: "SCL180" or "NANGATE45"
if {[info exists ::env(TECH_NODE)]} {
    set TECH_NODE $::env(TECH_NODE)
} else {
    set TECH_NODE "SCL180"
}

# Operating Corner: "typical", "slow", "fast"
if {[info exists ::env(CORNER)]} {
    set CORNER $::env(CORNER)
} else {
    set CORNER "typical"
}

# Directory Structure
set WORK_DIR        "./work"
set LOGS_DIR        "./logs"
set REPORTS_DIR     "./reports"
set OUTPUTS_DIR     "./outputs"
set SCRIPTS_DIR     "./scripts"
set CONSTRAINTS_DIR "./constraints"
set FILELIST_DIR    "./filelist"

# Ensure runtime directories exist
file mkdir ${WORK_DIR}
file mkdir ${LOGS_DIR}
file mkdir ${REPORTS_DIR}
file mkdir ${OUTPUTS_DIR}

echo "======================================================================"
echo " Design Synthesis Configuration"
echo "  Top Design       : ${DESIGN_NAME}"
echo "  Technology Node  : ${TECH_NODE}"
echo "  Corner           : ${CORNER}"
echo "  Target Period    : ${CLK_PERIOD} ns ([expr {1000.0 / ${CLK_PERIOD}}] MHz)"
echo "======================================================================"
