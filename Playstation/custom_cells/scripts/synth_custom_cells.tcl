# =============================================================================
# Yosys Synthesis Script: Custom Cell Mapping Demo
# =============================================================================

# Read Custom Cell Primitives
read_verilog -sv 04_specialized_logic_cells/compressor_4to2/hdl/compressor_4to2.sv
read_verilog -sv 04_specialized_logic_cells/booth_encoder_cell/hdl/booth_encoder.sv
read_verilog -sv 04_specialized_logic_cells/fast_comparator_32b/hdl/fast_comparator_32b.sv
read_verilog -sv 04_specialized_logic_cells/dynamic_overflow_detect/hdl/dynamic_overflow_detect.sv
read_verilog -sv 02_divider_accelerators/cas_divider_slice_cell/hdl/cas_cell.sv

# Read Arithmetic Macros
read_verilog -sv 01_fast_arithmetic/kogge_stone_adder_32b/hdl/kogge_stone_adder_32b.sv
read_verilog -sv 01_fast_arithmetic/fused_3input_adder_32b/hdl/fused_3input_adder_32b.sv
read_verilog -sv 01_fast_arithmetic/booth_wallace_mul_32b/hdl/booth_wallace_mul_32b.sv
read_verilog -sv 02_divider_accelerators/radix4_srt_divider_32b/hdl/radix4_srt_divider_32b.sv

# Check Hierarchy of Multiplier
hierarchy -check -top booth_wallace_mul_32b

# High-level synthesis optimizations
synth -top booth_wallace_mul_32b -flatten

# Show cell statistics
stat
