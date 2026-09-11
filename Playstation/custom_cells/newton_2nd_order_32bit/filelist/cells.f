# =============================================================================
# Custom Transistor-Level Standard Cells Manifest (SPICE Netlists & Testbenches)
# Technology Node: SCL 180nm Commercial CMOS Process (1.8V)
# =============================================================================

# --- Arithmetic Cells ---
cells/arithmetic/booth_encoder/schematic.spice
cells/arithmetic/booth_encoder/tb_transient.spice
cells/arithmetic/booth_selector/schematic.spice
cells/arithmetic/booth_selector/tb_transient.spice
cells/arithmetic/compressor_4to2/schematic.spice
cells/arithmetic/compressor_4to2/tb_transient.spice
cells/arithmetic/csa_3to2_slice/schematic.spice
cells/arithmetic/csa_3to2_slice/tb_transient.spice
cells/arithmetic/kogge_stone_cells/schematic.spice
cells/arithmetic/kogge_stone_cells/tb_transient.spice

# --- Divider Cells ---
cells/divider/cas_divider_slice/schematic.spice
cells/divider/cas_divider_slice/tb_transient.spice
cells/divider/srt_radix4_stage/schematic.spice
cells/divider/srt_radix4_stage/tb_transient.spice

# --- Memory Cells ---
cells/memory/bitcell_8t_2r1w/schematic.spice
cells/memory/bitcell_8t_2r1w/sense_amplifier.spice
cells/memory/bitcell_8t_2r1w/tb_read_write.spice
cells/memory/sram_6t_cell/schematic.spice
cells/memory/sram_6t_cell/tb_snm.spice

# --- Specialized & Datapath Acceleration Cells ---
cells/specialized/dynamic_overflow_detect/schematic.spice
cells/specialized/dynamic_overflow_detect/tb_transient.spice
cells/specialized/fast_comparator/schematic.spice
cells/specialized/fast_comparator/tb_transient.spice
cells/specialized/tg_mux/schematic.spice
cells/specialized/tg_mux/tb_transient.spice
