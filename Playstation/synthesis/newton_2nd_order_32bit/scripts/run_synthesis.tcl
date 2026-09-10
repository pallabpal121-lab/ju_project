# ==============================================================================
# Master Synthesis Automation Script (run_synthesis.tcl)
# Design   : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# Standard : Industry Reference Methodology (RM)
# ==============================================================================

set start_time [clock seconds]

echo "######################################################################"
echo "#                                                                    #"
echo "#        UNIVERSAL NEWTON 2ND-ORDER ACCELERATOR SYNTHESIS FLOW       #"
echo "#                    SYNOPSYS DESIGN COMPILER                        #"
echo "#                                                                    #"
echo "######################################################################"

# Step 0: Common Setup & Technology Library Selection
source "./scripts/common_setup.tcl"
source "./scripts/dc_setup.tcl"

# Step 1: Read, Analyze, Elaborate, Link & Check RTL
source "./scripts/read_design.tcl"

# Step 2: Apply SDC Timing, Environmental & Optimization Constraints
source "./scripts/apply_constraints.tcl"

# Step 3: Compile and Optimize Logic
source "./scripts/compile_design.tcl"

# Step 4: Generate QoR, Area, Timing, Power, and Violation Reports
source "./scripts/generate_reports.tcl"

# Step 5: Export Netlist (.v), SDC, SDF, and DDC Deliverables
source "./scripts/export_outputs.tcl"

set end_time [clock seconds]
set total_runtime [expr {${end_time} - ${start_time}}]

echo "######################################################################"
echo "# SYNTHESIS COMPLETED SUCCESSFULLY                                   #"
echo "# Total Runtime: ${total_runtime} seconds                            #"
echo "# Reports Directory : ${REPORTS_DIR}                                 #"
echo "# Outputs Directory : ${OUTPUTS_DIR}                                 #"
echo "######################################################################"

exit
