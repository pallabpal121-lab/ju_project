# ==============================================================================
# Script 04: Logic Compilation & Technology Optimization
# File   : 04_compile_design.tcl
# Role   : Step 3 - Gate-Level Mapping & Timing/Area Optimization
# Design : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# ==============================================================================
# 
# [BEGINNER GUIDE - WHY DO WE NEED THIS FILE?]
# This script executes the core synthesis engine (the 3 Compilation Phases):
# - Phase 1: Architectural Optimization (DesignWare datapath & resource sharing)
# - Phase 2: Logic-Level Optimization (Technology-independent Boolean minimization)
# - Phase 3: Gate-Level Technology Mapping (Physical standard cells from SCL 180nm)
#
# Key commands explained:
# 1. uniquify:
#    If a module is instantiated more than once (e.g. q16_divider is instantiated
#    in both the top module and inside the derivative engine), uniquify gives each
#    instance a unique internal copy. This allows DC to optimize each instance
#    independently based on its specific timing and loading context.
#
# 2. set_fix_multiple_port_nets -all -buffer_constants:
#    Ensures that ports connected directly to each other or to constants (VSS/VDD)
#    are decoupled with physical inverter/buffer gates. This eliminates Verilog
#    "assign" statements in the netlist, which is required for clean Place & Route (P&R).
#
# 3. compile_ultra -gate_clock:
#    The highest effort optimization engine in Design Compiler. It inserts integrated
#    clock gating (ICG) cells on flip-flop enable paths to reduce dynamic power,
#    and iteratively sizes cells to satisfy setup and hold timing.
# ==============================================================================

echo "======================================================================"
echo " (04_compile_design.tcl) Step 3: Compiling, Mapping & Optimizing Logic"
echo "======================================================================"

# ------------------------------------------------------------------------------
# 1. Uniquify Multi-Instantiated Modules
# ------------------------------------------------------------------------------
echo "INFO: Uniquifying design hierarchy instances..."
uniquify

# ------------------------------------------------------------------------------
# 2. Eliminate Assign Statements for Physical Design (ICC2) Cleanliness
# ------------------------------------------------------------------------------
echo "INFO: Setting multi-port net buffering rules..."
set_fix_multiple_port_nets -all -buffer_constants

# ------------------------------------------------------------------------------
# 3. Execute Synthesis Compilation Engine
# ------------------------------------------------------------------------------
# Attempt 'compile_ultra -gate_clock' for industry-grade timing and power results.
# If ultra features are not licensed on the system, gracefully fall back to
# standard high-effort compile.
echo "INFO: Launching synthesis optimization engine..."

if {[catch {compile_ultra -gate_clock} err_msg]} {
    echo "WARNING: compile_ultra returned: ${err_msg}"
    echo "INFO: Running standard high-effort compile with area and timing optimization..."
    compile -map_effort high -area_effort high
}

echo "INFO: Logic compilation and technology mapping completed successfully."
echo "======================================================================"
