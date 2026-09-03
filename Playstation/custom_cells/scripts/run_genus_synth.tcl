# =============================================================================
# Cadence Genus Synthesis Script
# Universal Newton 2nd-Order Accelerator with Custom Cell Library
# =============================================================================

set_db init_lib_search_path [list . ../lib]
set_db init_hdl_search_path [list . ../04_specialized_logic_cells ../01_fast_arithmetic ../02_divider_accelerators ../../rtl/newton_2nd_order_32bit]

set_db library [list standard_cells.lib custom_cells_sky130_tt.lib]

read_hdl -language sv [list \
    ../04_specialized_logic_cells/compressor_4to2/hdl/compressor_4to2.sv \
    ../04_specialized_logic_cells/booth_encoder_cell/hdl/booth_encoder.sv \
    ../04_specialized_logic_cells/fast_comparator_32b/hdl/fast_comparator_32b.sv \
    ../04_specialized_logic_cells/dynamic_overflow_detect/hdl/dynamic_overflow_detect.sv \
    ../02_divider_accelerators/cas_divider_slice_cell/hdl/cas_cell.sv \
    ../01_fast_arithmetic/kogge_stone_adder_32b/hdl/kogge_stone_adder_32b.sv \
    ../01_fast_arithmetic/fused_3input_adder_32b/hdl/fused_3input_adder_32b.sv \
    ../01_fast_arithmetic/booth_wallace_mul_32b/hdl/booth_wallace_mul_32b.sv \
    ../02_divider_accelerators/radix4_srt_divider_32b/hdl/radix4_srt_divider_32b.sv \
    ../../rtl/newton_2nd_order_32bit/newton_types_pkg.sv \
    ../../rtl/newton_2nd_order_32bit/q16_alu.sv \
    ../../rtl/newton_2nd_order_32bit/q16_divider.sv \
    ../../rtl/newton_2nd_order_32bit/dfg_equation_engine.sv \
    ../../rtl/newton_2nd_order_32bit/derivative_engine.sv \
    ../../rtl/newton_2nd_order_32bit/newton_2nd_order_top.sv \
]

elaborate newton_2nd_order_top
init_design

# 250 MHz Clock Constraint
create_clock -name clk -period 4.0 [get_ports clk]

syn_generic
syn_map
syn_opt

report_timing > genus_timing_report.txt
report_area   > genus_area_report.txt
report_power  > genus_power_report.txt

write_hdl > newton_top_genus_netlist.v
write_sdc > newton_top_genus.sdc
