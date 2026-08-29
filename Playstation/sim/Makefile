# =============================================================================
# Makefile for Universal Newton 2nd-Order Hardware Accelerator
# Supports:
#   1. 32-bit (Q16.16) 1D Accelerator
#   2. 64-bit (Q32.32) 1D Accelerator
#   3. Multivariable N-Dimensional (Q16.16) Accelerator with Cholesky Solver
# =============================================================================

SIM ?= iverilog
TB_DIR  = tb_sv

# 1. 32-bit (Q16.16) 1D Sources
RTL_32_DIR = ../rtl/newton_2nd_order_32bit
SRCS_32 = $(RTL_32_DIR)/newton_types_pkg.sv \
          $(RTL_32_DIR)/q16_alu.sv \
          $(RTL_32_DIR)/q16_divider.sv \
          $(RTL_32_DIR)/dfg_equation_engine.sv \
          $(RTL_32_DIR)/derivative_engine.sv \
          $(RTL_32_DIR)/newton_2nd_order_top.sv \
          $(TB_DIR)/tb_newton_2nd_order.sv

OUT_32 = sim_newton_32bit.vvp

# 2. 64-bit (Q32.32) 1D Sources
RTL_64_DIR = ../rtl/newton_2nd_order_64bit
SRCS_64 = $(RTL_64_DIR)/newton_types_64bit_pkg.sv \
          $(RTL_64_DIR)/q32_alu.sv \
          $(RTL_64_DIR)/q32_divider.sv \
          $(RTL_64_DIR)/dfg_equation_engine_64bit.sv \
          $(RTL_64_DIR)/derivative_engine_64bit.sv \
          $(RTL_64_DIR)/newton_2nd_order_64bit_top.sv \
          $(TB_DIR)/tb_newton_2nd_order_64bit.sv

OUT_64 = sim_newton_64bit.vvp

# 3. Multivariable N-Dimensional (Q16.16) Sources
RTL_MV_DIR = ../rtl/newton_multivar_32bit
SRCS_MV = $(RTL_MV_DIR)/newton_multivar_pkg.sv \
          $(RTL_MV_DIR)/q16_alu.sv \
          $(RTL_MV_DIR)/q16_divider.sv \
          $(RTL_MV_DIR)/q16_sqrt.sv \
          $(RTL_MV_DIR)/cholesky_solver_engine.sv \
          $(RTL_MV_DIR)/dfg_multivar_engine.sv \
          $(RTL_MV_DIR)/multivar_derivative_engine.sv \
          $(RTL_MV_DIR)/newton_multivar_top.sv \
          $(TB_DIR)/tb_newton_multivar.sv

OUT_MV = sim_newton_multivar.vvp

.PHONY: all clean run run_32bit run_64bit run_multivar compile_32bit compile_64bit compile_multivar

all: run_32bit run_64bit run_multivar

# 32-bit Targets
compile_32bit: $(SRCS_32)
	$(SIM) -g2012 -Wall $(SRCS_32) -o $(OUT_32)

run_32bit: compile_32bit
	vvp $(OUT_32)

# Default 'run' alias runs 32-bit
run: run_32bit

# 64-bit Targets
compile_64bit: $(SRCS_64)
	$(SIM) -g2012 -Wall $(SRCS_64) -o $(OUT_64)

run_64bit: compile_64bit
	vvp $(OUT_64)

# Multivariable Targets
compile_multivar: $(SRCS_MV)
	$(SIM) -g2012 -Wall -I$(RTL_MV_DIR) $(SRCS_MV) -o $(OUT_MV)

run_multivar: compile_multivar
	vvp $(OUT_MV)

clean:
	rm -f $(OUT_32) $(OUT_64) $(OUT_MV) *.vcd *.vvp
