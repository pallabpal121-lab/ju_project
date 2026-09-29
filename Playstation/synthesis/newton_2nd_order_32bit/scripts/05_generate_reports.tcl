# ==============================================================================
# Script 05: Quality of Results (QoR) & Verification Sign-Off Reporting
# File   : 05_generate_reports.tcl
# Role   : Step 4 - Comprehensive Design Sign-Off Metric Generation
# Design : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# ==============================================================================
# 
# [BEGINNER GUIDE - WHY DO WE NEED THIS FILE?]
# After compiling the design, we must prove that all constraints are met before
# handing off the design to physical layout (Place & Route).
# This script produces structured reports in the reports/ directory:
# 1. qor.rpt                  : Executive summary (Worst Negative Slack, Total
#                               Negative Slack, cell counts, CPU runtime).
# 2. area/area_hier.rpt       : Hierarchical gate area in um^2 for each submodule.
# 3. timing/timing_setup_max.rpt : Critical timing paths (Setup max delay). Slack
#                               must be >= 0.00 ns for timing closure.
# 4. timing/timing_hold_min.rpt  : Minimum delay paths (Hold timing checks).
# 5. power/power.rpt          : Dynamic switching, internal, and leakage power.
# 6. checks/constraint_violators.rpt : Lists all timing or DRV violations.
# 7. power/clock_gating.rpt   : Efficiency of integrated clock gating.
# 8. checks/references.rpt    : Count and list of every standard cell used.
# ==============================================================================

echo "======================================================================"
echo " (05_generate_reports.tcl) Step 4: Generating Quality of Results (QoR) Reports"
echo "======================================================================"

# ------------------------------------------------------------------------------
# 1. High-Level Quality of Results (WNS, TNS, leaf cell count, runtime)
# ------------------------------------------------------------------------------
echo "INFO: Generating QoR summary report..."
report_qor > "${REPORTS_DIR}/qor.rpt"

# ------------------------------------------------------------------------------
# 2. Hierarchical Area Report (Cell area, net area, macro breakdown)
# ------------------------------------------------------------------------------
echo "INFO: Generating hierarchical area report..."
report_area -hierarchy > "${REPORTS_AREA_DIR}/area_hier.rpt"

# ------------------------------------------------------------------------------
# 3. Setup Timing Report (Max delay critical paths)
# ------------------------------------------------------------------------------
# Checks if signals arrive before the next clock edge. Slack must be >= 0.
echo "INFO: Generating setup timing report (Max Delay)..."
report_timing -delay max -max_paths 20 -significant_digits 3 > "${REPORTS_TIMING_DIR}/timing_setup_max.rpt"

# ------------------------------------------------------------------------------
# 4. Hold Timing Report (Min delay paths)
# ------------------------------------------------------------------------------
# Checks that signals do not race through too fast and corrupt the current state.
echo "INFO: Generating hold timing report (Min Delay)..."
report_timing -delay min -max_paths 20 -significant_digits 3 > "${REPORTS_TIMING_DIR}/timing_hold_min.rpt"

# ------------------------------------------------------------------------------
# 5. Power Consumption Report (Dynamic switching, internal, and leakage)
# ------------------------------------------------------------------------------
echo "INFO: Generating power dissipation report..."
report_power -analysis_effort medium > "${REPORTS_POWER_DIR}/power.rpt"

# ------------------------------------------------------------------------------
# 6. Design Rule & Constraint Violators
# ------------------------------------------------------------------------------
# Confirms whether any setup, hold, max_transition, or max_capacitance rules failed.
echo "INFO: Generating constraint violators report..."
report_constraint -all_violators -significant_digits 3 > "${REPORTS_CHECKS_DIR}/constraint_violators.rpt"

# ------------------------------------------------------------------------------
# 7. Clock Gating Efficiency Report
# ------------------------------------------------------------------------------
# Shows how many flip-flops were gated to save dynamic clock power.
echo "INFO: Generating clock gating efficiency report..."
catch { report_clock_gating > "${REPORTS_POWER_DIR}/clock_gating.rpt" }

# ------------------------------------------------------------------------------
# 8. Design Resource Allocation (DesignWare Datapath Operators)
# ------------------------------------------------------------------------------
echo "INFO: Generating DesignWare resources report..."
catch { report_resources -hierarchy > "${REPORTS_AREA_DIR}/resources.rpt" }

# ------------------------------------------------------------------------------
# 9. Standard Cell Reference Inventory
# ------------------------------------------------------------------------------
# Confirms that all gates mapped to physical standard cells with zero GTECH gates.
echo "INFO: Generating standard cell reference count report..."
report_reference > "${REPORTS_CHECKS_DIR}/references.rpt"

echo "INFO: All synthesis sign-off reports saved in: ${REPORTS_DIR}"
echo "======================================================================"
