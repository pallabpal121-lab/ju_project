# =============================================================================
# Custom Datapath Macros Manifest (HDL Source & SPICE Verification Decks)
# Project: Universal Newton 2nd-Order Accelerator Custom Datapath Units
# =============================================================================

# 1. Radix-4 Modified Booth & Wallace Tree 32x32-bit Multiplier
macros/booth_wallace_mul_32b/hdl/booth_wallace_mul_32b.sv
macros/booth_wallace_mul_32b/spice/tb_multiplier.spice

# 2. Fused 3-Input Carry-Save Adder 32-bit
macros/fused_3input_adder_32b/hdl/fused_3input_adder_32b.sv
macros/fused_3input_adder_32b/spice/fused_3input_adder.spice

# 3. Kogge-Stone 32-bit Parallel-Prefix Carry-Propagate Adder
macros/kogge_stone_adder_32b/hdl/kogge_stone_adder_32b.sv
macros/kogge_stone_adder_32b/spice/kogge_stone_cells.spice

# 4. Radix-4 SRT Restoring Divider 32-bit
macros/radix4_srt_divider_32b/hdl/radix4_srt_divider_32b.sv
macros/radix4_srt_divider_32b/spice/tb_divider.spice

# 5. Dual-Read Single-Write 16x32-bit Register File
macros/regfile_2r1w_16x32b/hdl/regfile_2r1w_16x32b.sv
macros/regfile_2r1w_16x32b/spice/bitcell_8t_2r1w.spice

# 6. SRAM Instruction/Program Memory 32x32-bit
macros/sram_prog_mem_32x32b/hdl/sram_prog_mem_32x32b.sv
macros/sram_prog_mem_32x32b/spice/sram_6t_cell.spice
