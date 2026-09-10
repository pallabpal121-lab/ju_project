# ==============================================================================
# Design Compilation and Optimization (compile_design.tcl)
# ==============================================================================

echo "======================================================================"
echo " Step 3: Compiling and Optimizing Logic"
echo "======================================================================"

# Ensure all instantiated modules have unique structural definitions
uniquify

# Eliminate assign statements and multiple port nets by inserting buffers
set_fix_multiple_port_nets -all -buffer_constants

# Compilation Strategy:
# Attempt 'compile_ultra -gate_clock' for high-performance industry-grade optimization.
# Fallback gracefully to 'compile -map_effort high' if specific ultra features are not licensed.

echo "INFO: Launching synthesis compiler engine..."

if {[catch {compile_ultra -gate_clock} err_msg]} {
    echo "WARNING: compile_ultra returned: ${err_msg}"
    echo "INFO: Running standard high-effort compile with area and timing optimization..."
    compile -map_effort high -area_effort high
}

echo "INFO: Compilation completed successfully."
echo "======================================================================"
