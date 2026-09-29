# ==============================================================================
# Script 06: Export Gate-Level Netlist & Physical Design Deliverables
# File   : 06_export_outputs.tcl
# Role   : Step 5 - Backend Handoff Deliverable Generation
# Design : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# ==============================================================================
# 
# [BEGINNER GUIDE - WHY DO WE NEED THIS FILE?]
# Logic synthesis produces deliverables required by other EDA tools in the ASIC flow:
# 1. Gate-Level Netlist (.v) : Structural Verilog containing physical standard cells
#                              (NAND, DFF, INV) connected together. Handed off to
#                              IC Compiler II (ICC2) for Place & Route (P&R).
# 2. SDC Constraints (.sdc)  : Clean, updated timing constraints exported for
#                              PrimeTime Static Timing Analysis (STA) and ICC2 APR.
# 3. Standard Delay (.sdf)   : Gate delays and pin-to-pin timing for dynamic
#                              gate-level simulation in Synopsys VCS.
# 4. Mapped Database (.ddc)  : Complete hierarchical binary database of the mapped
#                              design for fast reloading in Design Vision or PrimeTime.
# 5. Formality Record (.svf) : Closes the verification log used by Synopsys Formality
#                              for Formal Equivalence Checking (RTL vs Netlist).
# ==============================================================================

echo "======================================================================"
echo " (06_export_outputs.tcl) Step 5: Exporting Gate-Level Netlist & Deliverables"
echo "======================================================================"

# ------------------------------------------------------------------------------
# 1. Structural Gate-Level Verilog Netlist (for ICC2 Place & Route & PrimeTime)
# ------------------------------------------------------------------------------
echo "INFO: Writing mapped gate-level Verilog netlist..."
write -format verilog -hierarchy -output "${OUTPUTS_NETLIST_DIR}/${DESIGN_NAME}.netlist.v"

# ------------------------------------------------------------------------------
# 2. Synthesized Design Constraints (SDC for APR and STA)
# ------------------------------------------------------------------------------
echo "INFO: Writing post-synthesis SDC constraints..."
write_sdc -nosplit "${OUTPUTS_CONSTRAINTS_DIR}/${DESIGN_NAME}.sdc"

# ------------------------------------------------------------------------------
# 3. Standard Delay Format (SDF for VCS Gate-Level Dynamic Simulation)
# ------------------------------------------------------------------------------
echo "INFO: Writing SDF back-annotation delay model..."
write_sdf -significant_digits 3 "${OUTPUTS_DELAYS_DIR}/${DESIGN_NAME}.sdf"

# ------------------------------------------------------------------------------
# 4. Mapped Synopsys Database (DDC for Fast Reloading and ECOs)
# ------------------------------------------------------------------------------
echo "INFO: Writing mapped binary DDC database..."
write -format ddc -hierarchy -output "${OUTPUTS_DB_DIR}/${DESIGN_NAME}_mapped.ddc"

# ------------------------------------------------------------------------------
# 5. Conclude Formality SVF Recording
# ------------------------------------------------------------------------------
# Finalizes the SVF container so Formality can perform formal equivalence checking.
set_svf off

echo "INFO: All deliverables successfully exported to: ${OUTPUTS_DIR}"
echo "======================================================================"
