# ==============================================================================
# Timing Exceptions: False Paths and Multicycle Paths
# Design   : newton_2nd_order_top
# ==============================================================================

echo "Applying Timing Exceptions..."

# ------------------------------------------------------------------------------
# 1. Asynchronous Reset False Path
# ------------------------------------------------------------------------------
# Global active-low asynchronous reset path does not require synchronous setup checks
set_false_path -from [get_ports rst_n]

# ------------------------------------------------------------------------------
# 2. Static Configuration Interface False Paths
# ------------------------------------------------------------------------------
# Microcode programming signals (prog_en, prog_addr, prog_data) are pre-loaded by
# host software prior to asserting 'start'. They are quasi-static during runtime.
set_false_path -from [get_ports {prog_en prog_addr* prog_data*}]

echo "Timing Exceptions successfully applied."
