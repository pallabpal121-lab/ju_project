# ==============================================================================
# Quality of Results (QoR) and Sign-Off Reporting (generate_reports.tcl)
# ==============================================================================

echo "======================================================================"
echo " Step 4: Generating Quality of Results (QoR) & Verification Reports   "
echo "======================================================================"

# 1. High-level Quality of Results (WNS, TNS, cell count, CPU runtime)
echo "INFO: Generating QoR report..."
report_qor > "${REPORTS_DIR}/qor.rpt"

# 2. Hierarchical Area Report (Cell area, net area, macro breakdown)
echo "INFO: Generating Area report..."
report_area -hierarchy > "${REPORTS_DIR}/area_hier.rpt"

# 3. Setup Timing (Max delay critical paths)
echo "INFO: Generating Setup Timing report (Max Delay)..."
report_timing -delay max -max_paths 20 -significant_digits 3 > "${REPORTS_DIR}/timing_setup_max.rpt"

# 4. Hold Timing (Min delay paths)
echo "INFO: Generating Hold Timing report (Min Delay)..."
report_timing -delay min -max_paths 20 -significant_digits 3 > "${REPORTS_DIR}/timing_hold_min.rpt"

# 5. Power Consumption Report (Internal, switching, and leakage power)
echo "INFO: Generating Power report..."
report_power -analysis_effort medium > "${REPORTS_DIR}/power.rpt"

# 6. Design Rule & Constraint Violators
echo "INFO: Generating Constraint Violations report..."
report_constraint -all_violators -significant_digits 3 > "${REPORTS_DIR}/constraint_violators.rpt"

# 7. Clock Gating Efficiency Report
echo "INFO: Generating Clock Gating report..."
catch { report_clock_gating > "${REPORTS_DIR}/clock_gating.rpt" }

# 8. Design Resource Allocation (ALU, adders, multipliers, dividers)
echo "INFO: Generating DesignWare resources report..."
catch { report_resources -hierarchy > "${REPORTS_DIR}/resources.rpt" }

# 9. Standard Cell Reference Count
echo "INFO: Generating standard cell reference report..."
report_reference > "${REPORTS_DIR}/references.rpt"

echo "INFO: All synthesis reports generated under: ${REPORTS_DIR}"
echo "======================================================================"
