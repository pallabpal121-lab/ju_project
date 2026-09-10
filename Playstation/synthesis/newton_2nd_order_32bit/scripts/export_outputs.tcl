# ==============================================================================
# Exporting Gate-Level Netlist and Sign-Off Deliverables (export_outputs.tcl)
# ==============================================================================

echo "======================================================================"
echo " Step 5: Exporting Gate-Level Netlists and Sign-Off Deliverables       "
echo "======================================================================"

# 1. Gate-level Structural Verilog Netlist (for Place & Route in ICC2 and STA in PrimeTime)
echo "INFO: Writing structural gate-level Verilog netlist..."
write -format verilog -hierarchy -output "${OUTPUTS_DIR}/${DESIGN_NAME}.netlist.v"

# 2. Synthesized Design Constraints (SDC for downstream physical design)
echo "INFO: Writing synthesized SDC constraints file..."
write_sdc -nosplit "${OUTPUTS_DIR}/${DESIGN_NAME}.sdc"

# 3. Standard Delay Format (SDF for gate-level dynamic timing simulation)
echo "INFO: Writing SDF back-annotation file..."
write_sdf -significant_digits 3 "${OUTPUTS_DIR}/${DESIGN_NAME}.sdf"

# 4. Mapped Synopsys Database (DDC for incremental ECOs and PrimeTime)
echo "INFO: Writing mapped DDC database..."
write -format ddc -hierarchy -output "${OUTPUTS_DIR}/${DESIGN_NAME}_mapped.ddc"

# 5. Conclude Formality SVF Recording
set_svf off

echo "INFO: All deliverables exported successfully under: ${OUTPUTS_DIR}"
echo "======================================================================"
