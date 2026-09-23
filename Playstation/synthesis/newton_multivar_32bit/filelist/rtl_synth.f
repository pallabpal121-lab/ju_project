// =============================================================================
// RTL Synthesis Filelist: Universal Multivariable Newton BCD Accelerator
// Target Design : newton_bcd_axi_lite
// Process       : SCL 180nm CMOS (0.18 um)
// =============================================================================

// Include directories for SystemVerilog header files (*.svh)
+incdir+../../rtl/newton_multivar_32bit
+incdir+/home/user17/Desktop/ju_project/Playstation/rtl/newton_multivar_32bit

// Ordered RTL Source Files
// 1. Package Definition
../../rtl/newton_multivar_32bit/newton_multivar_pkg.sv

// 2. Fixed-point Q16.16 ALU
../../rtl/newton_multivar_32bit/q16_alu.sv

// 3. 48-Cycle Non-Restoring Divider
../../rtl/newton_multivar_32bit/q16_divider.sv

// 4. Microcode Data-Flow Graph (DFG) Processor
../../rtl/newton_multivar_32bit/dfg_multivar_engine.sv

// 5. 2-Variable Hessian & Newton-Raphson Engine
../../rtl/newton_multivar_32bit/newton_2var_core.sv

// 6. State Vector Register RAM (BRAM model)
../../rtl/newton_multivar_32bit/state_bram.sv

// 7. Block Coordinate Descent (BCD) Coordinator
../../rtl/newton_multivar_32bit/newton_bcd_top.sv

// 8. Top-level AXI4-Lite Slave SoC Wrapper
../../rtl/newton_multivar_32bit/newton_bcd_axi_lite.sv
