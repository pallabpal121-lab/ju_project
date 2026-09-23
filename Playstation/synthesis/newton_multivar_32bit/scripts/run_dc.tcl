# ==============================================================================
# Master Synopsys Design Compiler Synthesis Script (run_dc.tcl)
# Design   : Universal Multivariable Newton BCD Accelerator
# Entity   : newton_bcd_axi_lite
# Foundry  : SCL 180nm Commercial CMOS Process (0.18 um)
# Voltage  : V_DD = 1.8 V (nominal core)
# Frequency: 50 MHz (Period = 20.0 ns)
# ==============================================================================

set start_time [clock seconds]

echo "######################################################################"
echo "#                                                                    #"
echo "#        UNIVERSAL MULTIVARIABLE NEWTON BCD ACCELERATOR              #"
echo "#                    ASIC SYNTHESIS FLOW                             #"
echo "#             Synopsys Design Compiler (dc_shell)                    #"
echo "#                                                                    #"
echo "######################################################################"

# ------------------------------------------------------------------------------
# 1. Flow Parameters & Configuration
# ------------------------------------------------------------------------------
if {[info exists ::env(DESIGN_NAME)]} {
    set DESIGN_NAME $::env(DESIGN_NAME)
} else {
    set DESIGN_NAME "newton_bcd_axi_lite"
}

if {[info exists ::env(SYNTH_CLK_PERIOD)]} {
    set CLK_PERIOD $::env(SYNTH_CLK_PERIOD)
} else {
    set CLK_PERIOD 20.0
}

if {[info exists ::env(CORNER)]} {
    set CORNER $::env(CORNER)
} else {
    set CORNER "typical"
}

if {[info exists ::env(TECH_NODE)]} {
    set TECH_NODE $::env(TECH_NODE)
} else {
    set TECH_NODE "SCL180"
}

# Directory Structure Paths
set SYNTH_DIR        "."
set WORK_DIR         "${SYNTH_DIR}/work"
set LOGS_DIR         "${SYNTH_DIR}/logs"
set REPORTS_DIR      "${SYNTH_DIR}/reports"
set OUTPUTS_DIR      "${SYNTH_DIR}/outputs"
set SCRIPTS_DIR      "${SYNTH_DIR}/scripts"
set CONSTRAINTS_DIR  "${SYNTH_DIR}/constraints"
set FILELIST_DIR     "${SYNTH_DIR}/filelist"
set RTL_DIR          "${SYNTH_DIR}/../../rtl/newton_multivar_32bit"
set PDK_LIB_DIR      "/home/user17/Synopsys/Design_Compiler/lib"

# Ensure all output and work folders exist
file mkdir ${WORK_DIR}
file mkdir ${WORK_DIR}/alib
file mkdir ${LOGS_DIR}
file mkdir ${REPORTS_DIR}
file mkdir ${OUTPUTS_DIR}

define_design_lib WORK -path ${WORK_DIR}
set alib_library_analysis_path "${WORK_DIR}/alib"

# Setup Formality SVF Recording
set_svf "${OUTPUTS_DIR}/${DESIGN_NAME}.svf"

# ------------------------------------------------------------------------------
# 2. Search Paths and Library Binding
# ------------------------------------------------------------------------------
set synopsys_root [get_unix_variable SYNOPSYS]
if {$synopsys_root == ""} {
    set synopsys_root "/home/user17/Synopsys/Design_Compiler/tool/Synthesis/syn/W-2024.09-SP1"
}
set SYNOPSYS_SYN_LIB "${synopsys_root}/libraries/syn"

set search_path [list . \
                      ${SCRIPTS_DIR} \
                      ${CONSTRAINTS_DIR} \
                      ${FILELIST_DIR} \
                      ${RTL_DIR} \
                      ${PDK_LIB_DIR} \
                      ${SYNOPSYS_SYN_LIB} \
                      ${search_path}]

# SCL 180nm Corner Selection
if {${CORNER} == "slow"} {
    set TARGET_LIB "tsl18fs120_scl_ss.db"    ;# SS: 1.62 V, 125 C (Worst-Case Setup)
} elseif {${CORNER} == "fast"} {
    set TARGET_LIB "tsl18fs120_scl_ff.db"    ;# FF: 1.98 V,   0 C (Best-Case Hold)
} else {
    set TARGET_LIB "tsl18fs120_typ.db"       ;# TT: 1.80 V,  25 C (Typical)
}

set target_library    [list ${TARGET_LIB}]
set synthetic_library [list standard.sldb]
set link_library      [list * ${target_library} ${synthetic_library}]
set symbol_library    [list generic.sdb]

echo "INFO: Target Library selected : ${target_library}"
echo "INFO: Link Library selected   : ${link_library}"

# ------------------------------------------------------------------------------
# 3. HDL Compiler Configuration
# ------------------------------------------------------------------------------
set hdlin_sv_packages true
set hdlin_check_no_latch true
set hdlin_enable_presto_for_vhdl true
set hdlin_auto_save_templates true
set bus_naming_style "%s[%d]"
set sh_enable_page_mode false

# ------------------------------------------------------------------------------
# 4. RTL Analyze & Elaborate (Ordered Filelist)
# ------------------------------------------------------------------------------
echo "======================================================================"
echo " Step 1: Reading, Analyzing & Elaborating SystemVerilog RTL"
echo "======================================================================"

set RTL_SOURCE_FILES [list \
    "${RTL_DIR}/newton_multivar_pkg.sv" \
    "${RTL_DIR}/q16_alu.sv" \
    "${RTL_DIR}/q16_divider.sv" \
    "${RTL_DIR}/dfg_multivar_engine.sv" \
    "${RTL_DIR}/newton_2var_core.sv" \
    "${RTL_DIR}/state_bram.sv" \
    "${RTL_DIR}/newton_bcd_top.sv" \
    "${RTL_DIR}/newton_bcd_axi_lite.sv" \
]

echo "INFO: Analyzing SystemVerilog files..."
analyze -format sverilog -work WORK ${RTL_SOURCE_FILES}

echo "INFO: Elaborating top-level entity '${DESIGN_NAME}'..."
elaborate ${DESIGN_NAME} -work WORK

current_design ${DESIGN_NAME}

echo "INFO: Resolving design hierarchy and linking..."
link

# ------------------------------------------------------------------------------
# 5. Pre-Synthesis Linting & Latch Verification
# ------------------------------------------------------------------------------
echo "======================================================================"
echo " Step 2: Running Design Lint & Latch Detection (check_design)"
echo "======================================================================"
check_design -multiple_designs > "${REPORTS_DIR}/check_design.rpt"

# ------------------------------------------------------------------------------
# 6. Apply SDC Timing & Optimization Constraints
# ------------------------------------------------------------------------------
echo "======================================================================"
echo " Step 3: Applying SDC Timing & Environmental Constraints"
echo "======================================================================"

set SDC_FILE "${CONSTRAINTS_DIR}/newton_multivar_scl180.sdc"
if {[file exists ${SDC_FILE}]} {
    source ${SDC_FILE}
} else {
    echo "ERROR: Constraints file not found: ${SDC_FILE}"
    exit 1
}

# Optimization directives
set_max_area 0
set_max_dynamic_power 0 mW
set_fix_multiple_port_nets -all -buffer_constants
uniquify

echo "INFO: Verifying timing constraints consistency (check_timing)..."
check_timing > "${REPORTS_DIR}/check_timing.rpt"

# ------------------------------------------------------------------------------
# 7. Logic Optimization & Synthesis Compilation
# ------------------------------------------------------------------------------
echo "======================================================================"
echo " Step 4: Compiling and Optimizing Logic (compile_ultra)"
echo "======================================================================"

# High-performance ultra compile with clock gating and register retiming
if {[catch {compile_ultra -gate_clock -retime} err_msg]} {
    echo "WARNING: compile_ultra -gate_clock -retime failed: ${err_msg}"
    echo "INFO: Retrying with compile_ultra -gate_clock..."
    if {[catch {compile_ultra -gate_clock} err_msg2]} {
        echo "WARNING: compile_ultra -gate_clock failed: ${err_msg2}"
        echo "INFO: Falling back to high-effort standard compilation..."
        compile -map_effort high -area_effort high
    }
}

echo "INFO: Compilation completed successfully."

# ------------------------------------------------------------------------------
# 8. Report Generation
# ------------------------------------------------------------------------------
echo "======================================================================"
echo " Step 5: Generating Quality of Results (QoR) & Verification Reports"
echo "======================================================================"

echo "INFO: Generating QoR report..."
report_qor > "${REPORTS_DIR}/qor.rpt"

echo "INFO: Generating Setup Timing report (Max Delay)..."
report_timing -delay max -max_paths 20 -significant_digits 3 > "${REPORTS_DIR}/timing_setup_max.rpt"

echo "INFO: Generating Hold Timing report (Min Delay)..."
report_timing -delay min -max_paths 20 -significant_digits 3 > "${REPORTS_DIR}/timing_hold_min.rpt"

echo "INFO: Generating Hierarchical Area report..."
report_area -hierarchy > "${REPORTS_DIR}/area_hierarchical.rpt"

echo "INFO: Generating Power Consumption report..."
report_power -analysis_effort medium > "${REPORTS_DIR}/power_summary.rpt"

echo "INFO: Generating Clock Gating Efficiency report..."
catch { report_clock_gating > "${REPORTS_DIR}/clock_gating.rpt" }

echo "INFO: Generating Constraint Violators report..."
report_constraint -all_violators -significant_digits 3 > "${REPORTS_DIR}/constraint_violators.rpt"

echo "INFO: Generating Standard Cell Reference report..."
report_reference > "${REPORTS_DIR}/references.rpt"

echo "INFO: Generating DesignWare Resources report..."
catch { report_resources -hierarchy > "${REPORTS_DIR}/resources.rpt" }

# ------------------------------------------------------------------------------
# 9. Deliverables Export
# ------------------------------------------------------------------------------
echo "======================================================================"
echo " Step 6: Exporting Gate-Level Deliverables"
echo "======================================================================"

echo "INFO: Writing structural Verilog gate-level netlist..."
write -format verilog -hierarchy -output "${OUTPUTS_DIR}/${DESIGN_NAME}_synth.v"

echo "INFO: Writing synthesized SDC constraints..."
write_sdc -nosplit "${OUTPUTS_DIR}/${DESIGN_NAME}_synth.sdc"

echo "INFO: Writing Standard Delay Format (SDF) file..."
write_sdf -significant_digits 3 "${OUTPUTS_DIR}/${DESIGN_NAME}_synth.sdf"

echo "INFO: Writing mapped Synopsys database (DDC)..."
write -format ddc -hierarchy -output "${OUTPUTS_DIR}/${DESIGN_NAME}_synth_mapped.ddc"

# Close Formality SVF Recording
set_svf off

set end_time [clock seconds]
set total_runtime [expr {${end_time} - ${start_time}}]

echo "######################################################################"
echo "# SYNTHESIS COMPLETED SUCCESSFULLY                                   #"
echo "# Total Elapsed Time: ${total_runtime} seconds                       #"
echo "# Deliverables in   : ${OUTPUTS_DIR}                                 #"
echo "# Reports in        : ${REPORTS_DIR}                                 #"
echo "######################################################################"

exit 0
