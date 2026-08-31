# =============================================================================
# Synopsys Design Compiler (DC) Synthesis Script
# Universal Newton 2nd-Order Accelerator with Custom Cell Library
# =============================================================================

set target_library [list standard_cells.db custom_cells_sky130_tt.db]
set link_library   [list * standard_cells.db custom_cells_sky130_tt.db]

# Read SystemVerilog RTL and Custom Cell Wrappers
analyze -format sverilog [list     ../04_specialized_logic_cells/compressor_4to2/hdl/compressor_4to2.sv     ../04_specialized_logic_cells/booth_encoder_cell/hdl/booth_encoder.sv     ../04_specialized_logic_cells/fast_comparator_32b/hdl/fast_comparator_32b.sv     ../04_specialized_logic_cells/dynamic_overflow_detect/hdl/dynamic_overflow_detect.sv     ../02_divider_accelerators/cas_divider_slice_cell/hdl/cas_cell.sv     ../01_fast_arithmetic/kogge_stone_adder_32b/hdl/kogge_stone_adder_32b.sv     ../01_fast_arithmetic/fused_3input_adder_32b/hdl/fused_3input_adder_32b.sv     ../01_fast_arithmetic/booth_wallace_mul_32b/hdl/booth_wallace_mul_32b.sv     ../02_divider_accelerators/radix4_srt_divider_32b/hdl/radix4_srt_divider_32b.sv     /home/pallab-pal/ju_project/Playstation/rtl/newton_2nd_order_32bit/newton_types_pkg.sv     /home/pallab-pal/ju_project/Playstation/rtl/newton_2nd_order_32bit/q16_alu.sv     /home/pallab-pal/ju_project/Playstation/rtl/newton_2nd_order_32bit/q16_divider.sv     /home/pallab-pal/ju_project/Playstation/rtl/newton_2nd_order_32bit/dfg_equation_engine.sv     /home/pallab-pal/ju_project/Playstation/rtl/newton_2nd_order_32bit/derivative_engine.sv     /home/pallab-pal/ju_project/Playstation/rtl/newton_2nd_order_32bit/newton_2nd_order_top.sv ]

elaborate newton_2nd_order_top
current_design newton_2nd_order_top
link

# Clock & Timing Constraints (250 MHz Target)
create_clock -name "clk" -period 4.0 [get_ports clk]
set_clock_uncertainty 0.2 [get_clocks clk]
set_input_delay 0.5 -clock clk [all_inputs]
set_output_delay 0.5 -clock clk [all_outputs]

# High-effort Ultra synthesis
compile_ultra -timing_high_effort_script

# Generate PPA Reports
report_timing -max_paths 10 > dc_timing_report.txt
report_area -hierarchy      > dc_area_report.txt
report_power -hierarchy     > dc_power_report.txt
report_qor                  > dc_qor_report.txt

write -format verilog -hierarchy -output "newton_top_dc_netlist.v"
write_sdc "newton_top.sdc"
