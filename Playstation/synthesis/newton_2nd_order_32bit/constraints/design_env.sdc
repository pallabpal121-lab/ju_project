# ==============================================================================
# Design Environment Constraints (Process, Temperature, Drivers, Loads, WLM)
# Design   : newton_2nd_order_top
# ==============================================================================

echo "Applying Design Environment Constraints..."

# ------------------------------------------------------------------------------
# 1. Operating Conditions
# ------------------------------------------------------------------------------
# In SCL 180nm library, default operating conditions are specified in the .db.
# If explicit condition is specified:
if {[info exists OPERATING_CONDITION] && ${OPERATING_CONDITION} != ""} {
    set_operating_conditions ${OPERATING_CONDITION}
}

# ------------------------------------------------------------------------------
# 2. Input Driving Cell Model
# ------------------------------------------------------------------------------
# Represent off-chip drivers driving input pads/pins with an inverter cell
set all_data_inputs [remove_from_collection [all_inputs] [get_ports {clk rst_n}]]

# Drive with 1x/2x standard cell inverter (e.g. inv0d1 or inv0d2 in SCL)
if {[sizeof_collection [get_lib_cells */inv0d1 2>/dev/null]] > 0} {
    set_driving_cell -lib_cell inv0d1 ${all_data_inputs}
} elseif {[sizeof_collection [get_lib_cells */INV_X1 2>/dev/null]] > 0} {
    set_driving_cell -lib_cell INV_X1 ${all_data_inputs}
} else {
    # Fallback to drive resistance if library cell varies
    set_drive 1.0 ${all_data_inputs}
}

# ------------------------------------------------------------------------------
# 3. Output Load Capacitance
# ------------------------------------------------------------------------------
# Represent external load driven by chip outputs: 0.05 pF = 50 fF
set OUTPUT_PIN_LOAD 0.05
set_load ${OUTPUT_PIN_LOAD} [all_outputs]

# ------------------------------------------------------------------------------
# 4. Wire Load Model (Pre-Layout Interconnect Estimation)
# ------------------------------------------------------------------------------
# Use enclosed wire load mode across hierarchical blocks
set_wire_load_mode enclosed

echo "Design Environment Constraints successfully applied."
