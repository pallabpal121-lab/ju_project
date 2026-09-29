# ==============================================================================
# Script 03: Apply SDC Timing, Environmental & Optimization Constraints
# File   : 03_apply_constraints.tcl
# Role   : Step 2 - Setting Design Goals & Operating Environment
# Design : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# ==============================================================================
# 
# [BEGINNER GUIDE - WHY DO WE NEED THIS FILE?]
# Logic synthesis is constraint-driven. Without constraints, Design Compiler does
# not know how fast the circuit needs to run or what external loads it must drive.
# In this script, we apply:
# 1. Primary SDC Constraints    : Clock definitions (100 MHz / 10.0 ns), clock jitter
#                                 uncertainty, input arrival times, and output delays.
# 2. Environmental Constraints  : Off-chip driving cells (e.g. inv0d1) and external
#                                 pin capacitive loads (e.g. 50 fF).
# 3. Timing Exceptions          : Marks non-synchronous paths (such as the asynchronous
#                                 reset 'rst_n' and static microcode inputs) as false
#                                 paths so the tool does not waste area optimizing them.
# 4. Cost Function Goals        : Directs the compiler to aggressively minimize area.
# 5. check_timing               : Verifies all flip-flops and I/O ports are constrained
#                                 before running the expensive compile step.
# ==============================================================================

echo "======================================================================"
echo " (03_apply_constraints.tcl) Step 2: Applying SDC Timing & Environmental Constraints"
echo "======================================================================"

# ------------------------------------------------------------------------------
# 1. Primary SDC Constraints (Clocks, Latency, Uncertainty, IO Delays)
# ------------------------------------------------------------------------------
echo "INFO: Sourcing primary clock and I/O constraints..."
source "${CONSTRAINTS_DIR}/${DESIGN_NAME}.sdc"

# ------------------------------------------------------------------------------
# 2. Environmental Constraints (Operating conditions, driving cells, pin loads)
# ------------------------------------------------------------------------------
echo "INFO: Sourcing design environment constraints (drivers, loads, WLM)..."
source "${CONSTRAINTS_DIR}/design_env.sdc"

# ------------------------------------------------------------------------------
# 3. Timing Exceptions (False paths on asynchronous resets & static config)
# ------------------------------------------------------------------------------
echo "INFO: Sourcing timing exceptions (false paths on reset and config)..."
source "${CONSTRAINTS_DIR}/timing_exceptions.sdc"

# ------------------------------------------------------------------------------
# 4. Optimization Goals (Cost Function Targets)
# ------------------------------------------------------------------------------
# Setting max_area to 0 instructs Design Compiler's cost function to minimize
# silicon area as much as possible, provided timing constraints are satisfied.
set_max_area 0

# ------------------------------------------------------------------------------
# 5. Sanity Check Timing Constraints Before Compilation
# ------------------------------------------------------------------------------
# check_timing ensures there are no unclocked registers, unconstrained ports,
# or combinational feedback loops before starting optimization.
echo "INFO: Checking timing constraints consistency (check_timing)..."
check_timing > "${REPORTS_TIMING_DIR}/check_timing.rpt"

echo "INFO: Constraints applied and verified successfully."
echo "======================================================================"
