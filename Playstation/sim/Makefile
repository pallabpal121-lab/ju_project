# =============================================================================
# Makefile for Universal Hardware Accelerator Suite (Solvers #1 to #12)
# Supports:
#   1. 32-bit (Q16.16) 1D Newton Accelerator
#   2. 64-bit (Q32.32) 1D Newton Accelerator
#   3. Multivariable N-Dimensional (Q16.16) Newton Accelerator
#   4. Levenberg-Marquardt (Q16.16) Non-Linear Least Squares Accelerator
#   5. Iteratively Reweighted Least Squares (IRLS) Accelerator (Solver #3)
#   6. Quasi-Newton BFGS (Q16.16) Accelerator (Solver #4)
#   7. Non-Linear Conjugate Gradient (CG) Accelerator (Solver #5)
#   8. Gauss-Newton (Q16.16) Accelerator (Solver #6)
#   9. Sequential Quadratic Programming (SQP) Constrained Accelerator (Solver #7)
#  10. Nelder-Mead Simplex Direct Search Accelerator (Solver #8)
#  11. Limited-Memory BFGS (L-BFGS) Accelerator (Solver #9)
#  12. Coordinate Descent / LASSO L1 Sparsity Accelerator (Solver #10)
#  13. Alternating Direction Method of Multipliers (ADMM) Accelerator (Solver #11)
#  14. Projected Gradient Descent (PGD) Accelerator (Solver #12)
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

# 7. Non-Linear Conjugate Gradient (CG) Sources (Solver #5)
RTL_CG_DIR = ../rtl/cg_32bit
SRCS_CG = $(RTL_CG_DIR)/cg_types_pkg.sv \
          $(RTL_CG_DIR)/q16_alu.sv \
          $(RTL_CG_DIR)/q16_divider.sv \
          $(RTL_CG_DIR)/dfg_cg_engine.sv \
          $(RTL_CG_DIR)/cg_gradient_engine.sv \
          $(RTL_CG_DIR)/cg_line_search_engine.sv \
          $(RTL_CG_DIR)/cg_top.sv \
          $(TB_DIR)/tb_cg.sv

OUT_CG = sim_cg.vvp

# 8. Gauss-Newton (GN) Sources (Solver #6)
RTL_GN_DIR = ../rtl/gauss_newton_32bit
SRCS_GN = $(RTL_GN_DIR)/gn_types_pkg.sv \
          $(RTL_GN_DIR)/q16_alu.sv \
          $(RTL_GN_DIR)/q16_divider.sv \
          $(RTL_GN_DIR)/q16_sqrt.sv \
          $(RTL_GN_DIR)/cholesky_solver_engine.sv \
          $(RTL_GN_DIR)/dfg_gn_engine.sv \
          $(RTL_GN_DIR)/gn_jacobian_engine.sv \
          $(RTL_GN_DIR)/gauss_newton_top.sv \
          $(TB_DIR)/tb_gauss_newton.sv

OUT_GN = sim_gauss_newton.vvp

# 9. Sequential Quadratic Programming (SQP) Sources (Solver #7)
RTL_SQP_DIR = ../rtl/sqp_32bit
SRCS_SQP = $(RTL_SQP_DIR)/sqp_types_pkg.sv \
           $(RTL_SQP_DIR)/q16_alu.sv \
           $(RTL_SQP_DIR)/q16_divider.sv \
           $(RTL_SQP_DIR)/q16_sqrt.sv \
           $(RTL_SQP_DIR)/cholesky_solver_engine.sv \
           $(RTL_SQP_DIR)/dfg_sqp_engine.sv \
           $(RTL_SQP_DIR)/sqp_hessian_engine.sv \
           $(RTL_SQP_DIR)/sqp_active_set_engine.sv \
           $(RTL_SQP_DIR)/sqp_top.sv \
           $(TB_DIR)/tb_sqp.sv

OUT_SQP = sim_sqp.vvp

# 10. Nelder-Mead (NM) Sources (Solver #8)
RTL_NM_DIR = ../rtl/nelder_mead_32bit
SRCS_NM = $(RTL_NM_DIR)/nm_types_pkg.sv \
          $(RTL_NM_DIR)/q16_alu.sv \
          $(RTL_NM_DIR)/q16_divider.sv \
          $(RTL_NM_DIR)/dfg_nm_engine.sv \
          $(RTL_NM_DIR)/nm_simplex_core.sv \
          $(RTL_NM_DIR)/nelder_mead_top.sv \
          $(TB_DIR)/tb_nelder_mead.sv

OUT_NM = sim_nelder_mead.vvp

# 11. Limited-Memory BFGS (L-BFGS) Sources (Solver #9)
RTL_LBFGS_DIR = ../rtl/lbfgs_32bit
SRCS_LBFGS = $(RTL_LBFGS_DIR)/lbfgs_types_pkg.sv \
             $(RTL_LBFGS_DIR)/q16_alu.sv \
             $(RTL_LBFGS_DIR)/q16_divider.sv \
             $(RTL_LBFGS_DIR)/dfg_lbfgs_engine.sv \
             $(RTL_LBFGS_DIR)/lbfgs_gradient_engine.sv \
             $(RTL_LBFGS_DIR)/lbfgs_two_loop_engine.sv \
             $(RTL_LBFGS_DIR)/lbfgs_top.sv \
             $(TB_DIR)/tb_lbfgs.sv

OUT_LBFGS = sim_lbfgs.vvp

# 12. Coordinate Descent / LASSO Sources (Solver #10)
RTL_LASSO_DIR = ../rtl/lasso_32bit
SRCS_LASSO = $(RTL_LASSO_DIR)/lasso_types_pkg.sv \
             $(RTL_LASSO_DIR)/q16_alu.sv \
             $(RTL_LASSO_DIR)/q16_divider.sv \
             $(RTL_LASSO_DIR)/q16_soft_threshold.sv \
             $(RTL_LASSO_DIR)/lasso_coordinate_engine.sv \
             $(RTL_LASSO_DIR)/lasso_top.sv \
             $(TB_DIR)/tb_lasso.sv

OUT_LASSO = sim_lasso.vvp

# 13. ADMM Sources (Solver #11)
RTL_ADMM_DIR = ../rtl/admm_32bit
SRCS_ADMM = $(RTL_ADMM_DIR)/admm_types_pkg.sv \
            $(RTL_ADMM_DIR)/q16_alu.sv \
            $(RTL_ADMM_DIR)/q16_divider.sv \
            $(RTL_ADMM_DIR)/q16_sqrt.sv \
            $(RTL_ADMM_DIR)/q16_soft_threshold.sv \
            $(RTL_ADMM_DIR)/cholesky_solver_engine.sv \
            $(RTL_ADMM_DIR)/admm_primal_x_engine.sv \
            $(RTL_ADMM_DIR)/admm_top.sv \
            $(TB_DIR)/tb_admm.sv

OUT_ADMM = sim_admm.vvp

# 14. Projected Gradient Descent (PGD) Sources (Solver #12)
RTL_PGD_DIR = ../rtl/pgd_32bit
SRCS_PGD = $(RTL_PGD_DIR)/pgd_types_pkg.sv \
           $(RTL_PGD_DIR)/q16_alu.sv \
           $(RTL_PGD_DIR)/q16_divider.sv \
           $(RTL_PGD_DIR)/q16_sqrt.sv \
           $(RTL_PGD_DIR)/dfg_pgd_engine.sv \
           $(RTL_PGD_DIR)/pgd_gradient_engine.sv \
           $(RTL_PGD_DIR)/pgd_projection_engine.sv \
           $(RTL_PGD_DIR)/pgd_top.sv \
           $(TB_DIR)/tb_pgd.sv

OUT_PGD = sim_pgd.vvp

# 15. Trust-Region Dogleg Sources (Solver #13)
RTL_DOGLEG_DIR = ../rtl/dogleg_32bit
SRCS_DOGLEG = $(RTL_DOGLEG_DIR)/dogleg_types_pkg.sv \
              $(RTL_DOGLEG_DIR)/q16_alu.sv \
              $(RTL_DOGLEG_DIR)/q16_divider.sv \
              $(RTL_DOGLEG_DIR)/q16_sqrt.sv \
              $(RTL_DOGLEG_DIR)/dfg_dogleg_engine.sv \
              $(RTL_DOGLEG_DIR)/dogleg_jacobian_engine.sv \
              $(RTL_DOGLEG_DIR)/cholesky_solver_engine.sv \
              $(RTL_DOGLEG_DIR)/dogleg_step_engine.sv \
              $(RTL_DOGLEG_DIR)/dogleg_top.sv \
              $(TB_DIR)/tb_dogleg.sv

OUT_DOGLEG = sim_dogleg.vvp

# 16. Particle Swarm Optimization (PSO) Sources (Solver #14)
RTL_PSO_DIR = ../rtl/pso_32bit
SRCS_PSO = $(RTL_PSO_DIR)/pso_types_pkg.sv \
           $(RTL_PSO_DIR)/q16_alu.sv \
           $(RTL_PSO_DIR)/q16_divider.sv \
           $(RTL_PSO_DIR)/q16_sqrt.sv \
           $(RTL_PSO_DIR)/pso_lfsr_prng.sv \
           $(RTL_PSO_DIR)/dfg_pso_engine.sv \
           $(RTL_PSO_DIR)/pso_velocity_engine.sv \
           $(RTL_PSO_DIR)/pso_top.sv \
           $(TB_DIR)/tb_pso.sv

OUT_PSO = sim_pso.vvp

.PHONY: all clean run run_32bit run_64bit run_multivar run_lm run_irls run_bfgs run_cg run_gn run_sqp run_nm run_lbfgs run_lasso run_admm run_pgd run_dogleg run_pso compile_32bit compile_64bit compile_multivar compile_lm compile_irls compile_bfgs compile_cg compile_gn compile_sqp compile_nm compile_lbfgs compile_lasso compile_admm compile_pgd compile_dogleg compile_pso

all: run_32bit run_64bit run_multivar run_lm run_irls run_bfgs run_cg run_gn run_sqp run_nm run_lbfgs run_lasso run_admm run_pgd run_dogleg run_pso

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

# CG Targets (Solver #5)
compile_cg: $(SRCS_CG)
	$(SIM) -g2012 -Wall -I$(RTL_CG_DIR) $(SRCS_CG) -o $(OUT_CG)

run_cg: compile_cg
	vvp $(OUT_CG)

# GN Targets (Solver #6)
compile_gn: $(SRCS_GN)
	$(SIM) -g2012 -Wall -I$(RTL_GN_DIR) $(SRCS_GN) -o $(OUT_GN)

run_gn: compile_gn
	vvp $(OUT_GN)

# SQP Targets (Solver #7)
compile_sqp: $(SRCS_SQP)
	$(SIM) -g2012 -Wall -I$(RTL_SQP_DIR) $(SRCS_SQP) -o $(OUT_SQP)

run_sqp: compile_sqp
	vvp $(OUT_SQP)

# NM Targets (Solver #8)
compile_nm: $(SRCS_NM)
	$(SIM) -g2012 -Wall -I$(RTL_NM_DIR) $(SRCS_NM) -o $(OUT_NM)

run_nm: compile_nm
	vvp $(OUT_NM)

# L-BFGS Targets (Solver #9)
compile_lbfgs: $(SRCS_LBFGS)
	$(SIM) -g2012 -Wall -I$(RTL_LBFGS_DIR) $(SRCS_LBFGS) -o $(OUT_LBFGS)

run_lbfgs: compile_lbfgs
	vvp $(OUT_LBFGS)

# LASSO Targets (Solver #10)
compile_lasso: $(SRCS_LASSO)
	$(SIM) -g2012 -Wall -I$(RTL_LASSO_DIR) $(SRCS_LASSO) -o $(OUT_LASSO)

run_lasso: compile_lasso
	vvp $(OUT_LASSO)

# ADMM Targets (Solver #11)
compile_admm: $(SRCS_ADMM)
	$(SIM) -g2012 -Wall -I$(RTL_ADMM_DIR) $(SRCS_ADMM) -o $(OUT_ADMM)
	
run_admm: compile_admm
	vvp $(OUT_ADMM)

# PGD Targets (Solver #12)
compile_pgd: $(SRCS_PGD)
	$(SIM) -g2012 -Wall -I$(RTL_PGD_DIR) $(SRCS_PGD) -o $(OUT_PGD)

run_pgd: compile_pgd
	vvp $(OUT_PGD)

# Dogleg Targets (Solver #13)
compile_dogleg: $(SRCS_DOGLEG)
	$(SIM) -g2012 -Wall -I$(RTL_DOGLEG_DIR) $(SRCS_DOGLEG) -o $(OUT_DOGLEG)

run_dogleg: compile_dogleg
	vvp $(OUT_DOGLEG)

# PSO Targets (Solver #14)
compile_pso: $(SRCS_PSO)
	$(SIM) -g2012 -Wall -I$(RTL_PSO_DIR) $(SRCS_PSO) -o $(OUT_PSO)

run_pso: compile_pso
	vvp $(OUT_PSO)

clean:
	rm -f $(OUT_32) $(OUT_64) $(OUT_MV) $(OUT_LM) $(OUT_IRLS) $(OUT_BFGS) $(OUT_CG) $(OUT_GN) $(OUT_SQP) $(OUT_NM) $(OUT_LBFGS) $(OUT_LASSO) $(OUT_ADMM) $(OUT_PGD) $(OUT_DOGLEG) $(OUT_PSO) *.vcd *.vvp


