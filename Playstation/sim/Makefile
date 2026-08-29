# =============================================================================
# Makefile for Universal Hardware Accelerator Suite (Solvers #1, #2, #3, #4)
# Supports:
#   1. 32-bit (Q16.16) 1D Newton Accelerator
#   2. 64-bit (Q32.32) 1D Newton Accelerator
#   3. Multivariable N-Dimensional (Q16.16) Newton Accelerator
#   4. Levenberg-Marquardt (Q16.16) Non-Linear Least Squares Accelerator
#   5. Iteratively Reweighted Least Squares (IRLS) Accelerator (Solver #3)
#   6. Quasi-Newton BFGS (Q16.16) Accelerator (Solver #4)
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

# 4. Levenberg-Marquardt (Q16.16) Sources
RTL_LM_DIR = ../rtl/levenberg_marquardt_32bit
SRCS_LM = $(RTL_LM_DIR)/lm_types_pkg.sv \
          $(RTL_LM_DIR)/q16_alu.sv \
          $(RTL_LM_DIR)/q16_divider.sv \
          $(RTL_LM_DIR)/q16_sqrt.sv \
          $(RTL_LM_DIR)/cholesky_solver_engine.sv \
          $(RTL_LM_DIR)/dfg_lm_engine.sv \
          $(RTL_LM_DIR)/lm_jacobian_engine.sv \
          $(RTL_LM_DIR)/levenberg_marquardt_top.sv \
          $(TB_DIR)/tb_levenberg_marquardt.sv

OUT_LM = sim_levenberg_marquardt.vvp

# 5. Iteratively Reweighted Least Squares (IRLS) Sources (Solver #3)
RTL_IRLS_DIR = ../rtl/irls_32bit
SRCS_IRLS = $(RTL_IRLS_DIR)/irls_types_pkg.sv \
            $(RTL_IRLS_DIR)/q16_alu.sv \
            $(RTL_IRLS_DIR)/q16_divider.sv \
            $(RTL_IRLS_DIR)/q16_sqrt.sv \
            $(RTL_IRLS_DIR)/q16_sigmoid.sv \
            $(RTL_IRLS_DIR)/cholesky_solver_engine.sv \
            $(RTL_IRLS_DIR)/irls_weight_grad_engine.sv \
            $(RTL_IRLS_DIR)/irls_top.sv \
            $(TB_DIR)/tb_irls.sv

OUT_IRLS = sim_irls.vvp

# 6. Quasi-Newton BFGS Sources (Solver #4)
RTL_BFGS_DIR = ../rtl/bfgs_32bit
SRCS_BFGS = $(RTL_BFGS_DIR)/bfgs_types_pkg.sv \
            $(RTL_BFGS_DIR)/q16_alu.sv \
            $(RTL_BFGS_DIR)/q16_divider.sv \
            $(RTL_BFGS_DIR)/dfg_bfgs_engine.sv \
            $(RTL_BFGS_DIR)/bfgs_gradient_engine.sv \
            $(RTL_BFGS_DIR)/bfgs_matrix_update_engine.sv \
            $(RTL_BFGS_DIR)/bfgs_top.sv \
            $(TB_DIR)/tb_bfgs.sv

OUT_BFGS = sim_bfgs.vvp

.PHONY: all clean run run_32bit run_64bit run_multivar run_lm run_irls run_bfgs compile_32bit compile_64bit compile_multivar compile_lm compile_irls compile_bfgs

all: run_32bit run_64bit run_multivar run_lm run_irls run_bfgs

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

# Levenberg-Marquardt Targets
compile_lm: $(SRCS_LM)
	$(SIM) -g2012 -Wall -I$(RTL_LM_DIR) $(SRCS_LM) -o $(OUT_LM)

run_lm: compile_lm
	vvp $(OUT_LM)

# IRLS Targets (Solver #3)
compile_irls: $(SRCS_IRLS)
	$(SIM) -g2012 -Wall -I$(RTL_IRLS_DIR) $(SRCS_IRLS) -o $(OUT_IRLS)

run_irls: compile_irls
	vvp $(OUT_IRLS)

# BFGS Targets (Solver #4)
compile_bfgs: $(SRCS_BFGS)
	$(SIM) -g2012 -Wall -I$(RTL_BFGS_DIR) $(SRCS_BFGS) -o $(OUT_BFGS)

run_bfgs: compile_bfgs
	vvp $(OUT_BFGS)

clean:
	rm -f $(OUT_32) $(OUT_64) $(OUT_MV) $(OUT_LM) $(OUT_IRLS) $(OUT_BFGS) *.vcd *.vvp
