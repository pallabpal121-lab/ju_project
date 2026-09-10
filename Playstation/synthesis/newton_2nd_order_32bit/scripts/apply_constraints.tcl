# ==============================================================================
# Applying Synthesis Design & Optimization Constraints (apply_constraints.tcl)
# ==============================================================================

echo "======================================================================"
echo " Step 2: Applying SDC Timing & Optimization Constraints"
echo "======================================================================"

# 1. Primary SDC Constraints (Clocks, Latency, Uncertainty, IO Delays)
source "${CONSTRAINTS_DIR}/${DESIGN_NAME}.sdc"

# 2. Environmental Constraints (Operating conditions, driving cells, loads, wireload)
source "${CONSTRAINTS_DIR}/design_env.sdc"

# 3. Timing Exceptions (False paths on asynchronous resets & static config)
source "${CONSTRAINTS_DIR}/timing_exceptions.sdc"

# 4. Optimization Goals (Cost Function Targets)
# Direct compiler to aggressively minimize area without violating timing
set_max_area 0

# Set dynamic power optimization goal
set_max_dynamic_power 0 mW

# 5. Sanity check timing constraints before optimization
echo "INFO: Checking timing constraints consistency..."
check_timing > "${REPORTS_DIR}/check_timing.rpt"

echo "INFO: Constraints applied and verified."
echo "======================================================================"
