# ==============================================================================
# Synopsys Design Constraints (SDC) for Newton 2nd-Order Accelerator
# Design   : newton_2nd_order_top
# Format   : Standard Design Constraints (SDC 2.1)
# ==============================================================================

# Default clock parameters if not preset by master script
if {![info exists CLK_PERIOD]}      { set CLK_PERIOD 10.0 }
if {![info exists CLK_UNCERTAINTY]} { set CLK_UNCERTAINTY 0.2 }
if {![info exists CLK_TRANSITION]}  { set CLK_TRANSITION 0.1 }

echo "Applying SDC Timing Constraints..."
echo "  Clock Period     : ${CLK_PERIOD} ns (Target Freq: [expr {1000.0 / ${CLK_PERIOD}}] MHz)"
echo "  Clock Uncertainty: ${CLK_UNCERTAINTY} ns"
echo "  Clock Transition : ${CLK_TRANSITION} ns"

# ------------------------------------------------------------------------------
# 1. Primary Clock Definition
# ------------------------------------------------------------------------------
create_clock -name clk -period ${CLK_PERIOD} -waveform [list 0.0 [expr {${CLK_PERIOD} / 2.0}]] [get_ports clk]

# Clock Network Characteristics
set_clock_uncertainty -setup ${CLK_UNCERTAINTY} [get_clocks clk]
set_clock_uncertainty -hold 0.100 [get_clocks clk]
set_clock_transition ${CLK_TRANSITION} [get_clocks clk]
set_clock_latency -source 0.500 [get_clocks clk]
set_clock_latency 0.500 [get_clocks clk]

# Prevent DC from modifying or buffering the clock tree and reset net
set_dont_touch_network [get_ports clk]
set_dont_touch_network [get_ports rst_n]

# ------------------------------------------------------------------------------
# 2. Input / Output Timing Budgets (20% of cycle for I/O)
# ------------------------------------------------------------------------------
set INPUT_DELAY_MAX  [expr {0.20 * ${CLK_PERIOD}}]
set INPUT_DELAY_MIN  [expr {0.05 * ${CLK_PERIOD}}]
set OUTPUT_DELAY_MAX [expr {0.20 * ${CLK_PERIOD}}]
set OUTPUT_DELAY_MIN [expr {0.05 * ${CLK_PERIOD}}]

set all_data_inputs [remove_from_collection [all_inputs] [get_ports {clk rst_n}]]

# Input Arrival Constraints
set_input_delay -clock clk -max ${INPUT_DELAY_MAX} ${all_data_inputs}
set_input_delay -clock clk -min ${INPUT_DELAY_MIN} ${all_data_inputs}

# Output Required Time Constraints
set_output_delay -clock clk -max ${OUTPUT_DELAY_MAX} [all_outputs]
set_output_delay -clock clk -min ${OUTPUT_DELAY_MIN} [all_outputs]

# ------------------------------------------------------------------------------
# 3. Design Rule Constraints (DRVs)
# ------------------------------------------------------------------------------
set_max_fanout 16 [current_design]
set_max_transition 0.5 [current_design]

echo "SDC Constraints successfully applied."
