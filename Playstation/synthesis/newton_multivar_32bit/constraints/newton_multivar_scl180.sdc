# ==============================================================================
# Synopsys Design Constraints (SDC)
# Design   : Universal Multivariable Newton BCD Accelerator (newton_bcd_axi_lite)
# Target   : SCL 180nm Commercial CMOS (0.18 um)
# Voltage  : V_DD = 1.8 V (nominal core)
# Frequency: 50 MHz (Period = 20.0 ns)
# ==============================================================================

# Clock Configuration Parameters
if {![info exists CLK_PERIOD]}      { set CLK_PERIOD 20.0 }
if {![info exists CLK_UNCERTAINTY]} { set CLK_UNCERTAINTY 0.400 }
if {![info exists CLK_TRANSITION]}  { set CLK_TRANSITION 0.200 }
if {![info exists INPUT_DELAY]}     { set INPUT_DELAY 4.000 }
if {![info exists OUTPUT_DELAY]}    { set OUTPUT_DELAY 4.000 }
if {![info exists OUTPUT_LOAD]}     { set OUTPUT_LOAD 0.030 }

echo "======================================================================"
echo " Applying Timing & Environmental Constraints for newton_bcd_axi_lite"
echo "  Primary Clock     : s_axi_aclk"
echo "  Clock Period      : ${CLK_PERIOD} ns (50.0 MHz)"
echo "  Clock Uncertainty : ${CLK_UNCERTAINTY} ns (400 ps)"
echo "  Clock Transition  : ${CLK_TRANSITION} ns (200 ps)"
echo "  Input Delay Budget: ${INPUT_DELAY} ns"
echo "  Output Delay      : ${OUTPUT_DELAY} ns"
echo "  Output Pin Load   : ${OUTPUT_LOAD} pF (30 fF)"
echo "======================================================================"

# ------------------------------------------------------------------------------
# 1. Primary Clock Definition
# ------------------------------------------------------------------------------
create_clock -name s_axi_aclk -period ${CLK_PERIOD} -waveform [list 0.0 [expr {${CLK_PERIOD} / 2.0}]] [get_ports s_axi_aclk]

# Clock Network Characteristics
set_clock_uncertainty -setup ${CLK_UNCERTAINTY} [get_clocks s_axi_aclk]
set_clock_uncertainty -hold [expr {${CLK_UNCERTAINTY} / 2.0}] [get_clocks s_axi_aclk]
set_clock_transition ${CLK_TRANSITION} [get_clocks s_axi_aclk]
set_clock_latency -source 0.400 [get_clocks s_axi_aclk]
set_clock_latency 0.400 [get_clocks s_axi_aclk]

# Protect clock and reset networks from synthesis buffer insertion
set_dont_touch_network [get_ports s_axi_aclk]
set_dont_touch_network [get_ports s_axi_aresetn]

# ------------------------------------------------------------------------------
# 2. Input / Output Delay Budgets
# ------------------------------------------------------------------------------
set all_data_inputs [remove_from_collection [all_inputs] [get_ports {s_axi_aclk s_axi_aresetn}]]

# Input Delay: 4.0 ns (20% of cycle) referenced to s_axi_aclk
set_input_delay -clock s_axi_aclk -max ${INPUT_DELAY} ${all_data_inputs}
set_input_delay -clock s_axi_aclk -min 0.500 ${all_data_inputs}

# Output Delay: 4.0 ns (20% of cycle) referenced to s_axi_aclk on all outputs (including irq_done)
set_output_delay -clock s_axi_aclk -max ${OUTPUT_DELAY} [all_outputs]
set_output_delay -clock s_axi_aclk -min 0.500 [all_outputs]

# ------------------------------------------------------------------------------
# 3. Timing Exceptions
# ------------------------------------------------------------------------------
# Asynchronous active-low reset path
set_false_path -from [get_ports s_axi_aresetn]

# ------------------------------------------------------------------------------
# 4. Standard Cell Driving Model & Output Capacitance
# ------------------------------------------------------------------------------
# Input Driving Cell: BUF_X4 (or library equivalents buffd4 / bufbd4 / inv0d4)
if {[sizeof_collection [get_lib_cells -quiet */BUF_X4]] > 0} {
    set_driving_cell -lib_cell BUF_X4 ${all_data_inputs}
} elseif {[sizeof_collection [get_lib_cells -quiet */buffd4]] > 0} {
    set_driving_cell -lib_cell buffd4 ${all_data_inputs}
} elseif {[sizeof_collection [get_lib_cells -quiet */bufbd4]] > 0} {
    set_driving_cell -lib_cell bufbd4 ${all_data_inputs}
} elseif {[sizeof_collection [get_lib_cells -quiet */inv0d4]] > 0} {
    set_driving_cell -lib_cell inv0d4 ${all_data_inputs}
} else {
    set_drive 1.0 ${all_data_inputs}
}

# Output Load Capacitance: C_load = 30 fF = 0.030 pF
set_load ${OUTPUT_LOAD} [all_outputs]

# Wire Load Model for pre-layout interconnect estimation
set_wire_load_mode enclosed

# ------------------------------------------------------------------------------
# 5. Design Rule Constraints (DRVs)
# ------------------------------------------------------------------------------
set_max_fanout 16 [current_design]
set_max_transition 0.500 [current_design]

echo "INFO: SDC constraints successfully applied."
echo "======================================================================"
