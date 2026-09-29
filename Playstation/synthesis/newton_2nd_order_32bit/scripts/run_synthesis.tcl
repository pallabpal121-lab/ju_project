# ==============================================================================
# Master Synthesis Automation Script (run_synthesis.tcl)
# Design   : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# Standard : Industry Reference Methodology (RM) Sequential Execution
# ==============================================================================
# 
# [BEGINNER GUIDE - MASTER FLOW EXECUTION ORDER]
# This master script sequentially orchestrates the entire synthesis run from start
# to finish by sourcing each numbered step in order:
# 
#   Step 0A: scripts/00_common_setup.tcl    -> Declare clock, corner, and directory variables
#   Step 0B: scripts/01_dc_setup.tcl        -> Bind target (.db) and synthetic (DesignWare) libraries
#   Step 1 : scripts/02_read_design.tcl     -> Analyze RTL, elaborate GTECH, link, and check_design
#   Step 2 : scripts/03_apply_constraints.tcl-> Apply SDC clocks, I/O delays, loads, and check_timing
#   Step 3 : scripts/04_compile_design.tcl  -> Optimize logic & map to physical standard cells
#   Step 4 : scripts/05_generate_reports.tcl-> Export QoR, timing, area, and power sign-off reports
#   Step 5 : scripts/06_export_outputs.tcl   -> Export gate netlist (.v), SDC, SDF, and DDC
# ==============================================================================

set start_time [clock seconds]

echo "######################################################################"
echo "#                                                                    #"
echo "#        UNIVERSAL NEWTON 2ND-ORDER ACCELERATOR SYNTHESIS FLOW       #"
echo "#                    SYNOPSYS DESIGN COMPILER                        #"
echo "#                                                                    #"
echo "######################################################################"

# Step 0A: Common Setup (Design parameters, clock frequencies, paths)
source "./scripts/00_common_setup.tcl"

# Step 0B: DC Setup (Foundry target library & DesignWare synthetic library binding)
source "./scripts/01_dc_setup.tcl"

# Step 1: Read, Analyze, Elaborate, Link & Lint RTL
source "./scripts/02_read_design.tcl"

# Step 2: Apply SDC Timing, Environmental & Optimization Constraints
source "./scripts/03_apply_constraints.tcl"

# Step 3: Compile, Map and Optimize Logic
source "./scripts/04_compile_design.tcl"

# Step 4: Generate QoR, Area, Timing, Power, and Violation Reports
source "./scripts/05_generate_reports.tcl"

# Step 5: Export Netlist (.v), SDC, SDF, and DDC Deliverables
source "./scripts/06_export_outputs.tcl"

set end_time [clock seconds]
set total_runtime [expr {${end_time} - ${start_time}}]

echo "######################################################################"
echo "# SYNTHESIS COMPLETED SUCCESSFULLY                                   #"
echo "# Total Runtime     : ${total_runtime} seconds                       #"
echo "# Reports Directory : ${REPORTS_DIR}                                 #"
echo "# Outputs Directory : ${OUTPUTS_DIR}                                 #"
echo "######################################################################"

exit
