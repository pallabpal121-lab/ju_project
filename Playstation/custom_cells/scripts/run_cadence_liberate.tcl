# =============================================================================
# Cadence Liberate Characterization Script
# Characterizes Custom Subcircuits into Standard .lib Format
# =============================================================================

# Operating Corner: Typical (1.8V, 25C)
set_operating_condition -voltage 1.8 -temp 25 -name "TT_1P8V_25C"

# Read SPICE Netlists
read_spice -format spectre "/path/to/pdk/models/spectre/core_models.scs"
read_spice "../04_specialized_logic_cells/compressor_4to2/spice/compressor_4to2.spice"
read_spice "../04_specialized_logic_cells/booth_encoder_cell/spice/booth_encoder.spice"
read_spice "../04_specialized_logic_cells/dynamic_overflow_detect/spice/dynamic_overflow_detect.spice"
read_spice "../02_divider_accelerators/cas_divider_slice_cell/spice/cas_cell.spice"

# Slew and Load Tables
set_var table_index_template_1x1 "0.01 0.05 0.15 0.35 0.70 1.20 2.00"
set_var table_index_template_1x2 "0.001 0.01 0.03 0.07 0.15 0.30 0.60"

# Run Characterization
char_library -ext_model_list { compressor_4to2 booth_encoder dynamic_overflow_detect cas_cell }

# Output Liberty .lib
write_library -overwrite -output "custom_cells_sky130_tt.lib"
