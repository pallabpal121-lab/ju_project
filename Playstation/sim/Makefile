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

# 17. Fast Iterative Shrinkage-Thresholding Algorithm (FISTA) Sources (Solver #15)
RTL_FISTA_DIR = ../rtl/fista_32bit
SRCS_FISTA = $(RTL_FISTA_DIR)/fista_types_pkg.sv \
             $(RTL_FISTA_DIR)/q16_alu.sv \
             $(RTL_FISTA_DIR)/q16_divider.sv \
             $(RTL_FISTA_DIR)/q16_sqrt.sv \
             $(RTL_FISTA_DIR)/dfg_fista_engine.sv \
             $(RTL_FISTA_DIR)/fista_gradient_engine.sv \
             $(RTL_FISTA_DIR)/fista_nesterov_engine.sv \
             $(RTL_FISTA_DIR)/fista_top.sv \
             $(TB_DIR)/tb_fista.sv

OUT_FISTA = sim_fista.vvp

# 18. Primal-Dual Interior Point Method (IPM) Sources (Solver #16)
RTL_IPM_DIR = ../rtl/ipm_32bit
SRCS_IPM = $(RTL_IPM_DIR)/ipm_types_pkg.sv \
           $(RTL_IPM_DIR)/q16_alu.sv \
           $(RTL_IPM_DIR)/q16_divider.sv \
           $(RTL_IPM_DIR)/q16_sqrt.sv \
           $(RTL_IPM_DIR)/dfg_ipm_engine.sv \
           $(RTL_IPM_DIR)/cholesky_ipm_solver.sv \
           $(RTL_IPM_DIR)/ipm_kkt_engine.sv \
           $(RTL_IPM_DIR)/ipm_top.sv \
           $(TB_DIR)/tb_ipm.sv

OUT_IPM = sim_ipm.vvp

# 19. Adam / RMSProp / Momentum SGD Sources (Solver #17)
RTL_ADAM_DIR = ../rtl/adam_32bit
SRCS_ADAM = $(RTL_ADAM_DIR)/adam_types_pkg.sv \
            $(RTL_ADAM_DIR)/q16_alu.sv \
            $(RTL_ADAM_DIR)/q16_divider.sv \
            $(RTL_ADAM_DIR)/q16_sqrt.sv \
            $(RTL_ADAM_DIR)/dfg_adam_engine.sv \
            $(RTL_ADAM_DIR)/adam_gradient_engine.sv \
            $(RTL_ADAM_DIR)/adam_moment_engine.sv \
            $(RTL_ADAM_DIR)/adam_top.sv \
            $(TB_DIR)/tb_adam.sv

OUT_ADAM = sim_adam.vvp

# 20. QUBO / Simulated Annealing Ising Sources (Solver #18)
RTL_QUBO_DIR = ../rtl/qubo_32bit
SRCS_QUBO = $(RTL_QUBO_DIR)/qubo_types_pkg.sv \
            $(RTL_QUBO_DIR)/q16_alu.sv \
            $(RTL_QUBO_DIR)/q16_divider.sv \
            $(RTL_QUBO_DIR)/qubo_lfsr_prng.sv \
            $(RTL_QUBO_DIR)/qubo_exp_unit.sv \
            $(RTL_QUBO_DIR)/qubo_energy_engine.sv \
            $(RTL_QUBO_DIR)/qubo_top.sv \
            $(TB_DIR)/tb_qubo.sv

OUT_QUBO = sim_qubo.vvp

# 21. PDHG / Chambolle-Pock Sources (Solver #19)
RTL_PDHG_DIR = ../rtl/pdhg_32bit
SRCS_PDHG = $(RTL_PDHG_DIR)/pdhg_types_pkg.sv \
            $(RTL_PDHG_DIR)/q16_alu.sv \
            $(RTL_PDHG_DIR)/q16_divider.sv \
            $(RTL_PDHG_DIR)/pdhg_dual_engine.sv \
            $(RTL_PDHG_DIR)/pdhg_primal_engine.sv \
            $(RTL_PDHG_DIR)/pdhg_top.sv \
            $(TB_DIR)/tb_pdhg.sv

OUT_PDHG = sim_pdhg.vvp

# 22. Frank-Wolfe / Conditional Gradient Sources (Solver #20)
RTL_FW_DIR = ../rtl/frank_wolfe_32bit
SRCS_FW = $(RTL_FW_DIR)/fw_types_pkg.sv \
          $(RTL_FW_DIR)/q16_alu.sv \
          $(RTL_FW_DIR)/q16_divider.sv \
          $(RTL_FW_DIR)/fw_lmo_engine.sv \
          $(RTL_FW_DIR)/fw_step_engine.sv \
          $(RTL_FW_DIR)/fw_top.sv \
          $(TB_DIR)/tb_frank_wolfe.sv

OUT_FW = sim_fw.vvp

# 23. Augmented Lagrangian Method (ALM) Sources (Solver #21)
RTL_ALM_DIR = ../rtl/alm_32bit
SRCS_ALM = $(RTL_ALM_DIR)/alm_types_pkg.sv \
           $(RTL_ALM_DIR)/q16_alu.sv \
           $(RTL_ALM_DIR)/q16_divider.sv \
           $(RTL_ALM_DIR)/q16_sqrt.sv \
           $(RTL_ALM_DIR)/cholesky_alm_solver.sv \
           $(RTL_ALM_DIR)/alm_dual_engine.sv \
           $(RTL_ALM_DIR)/alm_top.sv \
           $(TB_DIR)/tb_alm.sv

OUT_ALM = sim_alm.vvp

# 24. Dual Decomposition Sources (Solver #22)
RTL_DD_DIR = ../rtl/dual_decomp_32bit
SRCS_DD = $(RTL_DD_DIR)/dd_types_pkg.sv \
          $(RTL_DD_DIR)/q16_alu.sv \
          $(RTL_DD_DIR)/q16_divider.sv \
          $(RTL_DD_DIR)/dd_agent_node.sv \
          $(RTL_DD_DIR)/dd_master_engine.sv \
          $(RTL_DD_DIR)/dd_top.sv \
          $(TB_DIR)/tb_dual_decomp.sv

OUT_DD = sim_dual_decomp.vvp

# 25. Recursive Least Squares (RLS) Sources (Solver #23)
RTL_RLS_DIR = ../rtl/rls_32bit
SRCS_RLS = $(RTL_RLS_DIR)/rls_types_pkg.sv \
           $(RTL_RLS_DIR)/q16_alu.sv \
           $(RTL_RLS_DIR)/q16_divider.sv \
           $(RTL_RLS_DIR)/rls_gain_engine.sv \
           $(RTL_RLS_DIR)/rls_update_engine.sv \
           $(RTL_RLS_DIR)/rls_top.sv \
           $(TB_DIR)/tb_rls.sv

OUT_RLS = sim_rls.vvp

# 26. Extended Kalman Filter (EKF) Sources (Solver #24)
RTL_EKF_DIR = ../rtl/ekf_32bit
SRCS_EKF = $(RTL_EKF_DIR)/ekf_types_pkg.sv \
           $(RTL_EKF_DIR)/q16_alu.sv \
           $(RTL_EKF_DIR)/q16_divider.sv \
           $(RTL_EKF_DIR)/ekf_predict_engine.sv \
           $(RTL_EKF_DIR)/ekf_correct_engine.sv \
           $(RTL_EKF_DIR)/ekf_top.sv \
           $(TB_DIR)/tb_ekf.sv

OUT_EKF = sim_ekf.vvp

# 27. Unscented Kalman Filter (UKF) Sources (Solver #25)
RTL_UKF_DIR = ../rtl/ukf_32bit
SRCS_UKF = $(RTL_UKF_DIR)/ukf_types_pkg.sv \
           $(RTL_UKF_DIR)/q16_alu.sv \
           $(RTL_UKF_DIR)/q16_divider.sv \
           $(RTL_UKF_DIR)/q16_sqrt.sv \
           $(RTL_UKF_DIR)/ukf_sigma_gen.sv \
           $(RTL_UKF_DIR)/ukf_predict_engine.sv \
           $(RTL_UKF_DIR)/ukf_correct_engine.sv \
           $(RTL_UKF_DIR)/ukf_top.sv \
           $(TB_DIR)/tb_ukf.sv

OUT_UKF = sim_ukf.vvp

# 28. Model Predictive Control (MPC) Sources (Solver #26)
RTL_MPC_DIR = ../rtl/mpc_32bit
SRCS_MPC = $(RTL_MPC_DIR)/mpc_types_pkg.sv \
           $(RTL_MPC_DIR)/q16_alu.sv \
           $(RTL_MPC_DIR)/q16_divider.sv \
           $(RTL_MPC_DIR)/mpc_condense_engine.sv \
           $(RTL_MPC_DIR)/mpc_qp_engine.sv \
           $(RTL_MPC_DIR)/mpc_top.sv \
           $(TB_DIR)/tb_mpc.sv

OUT_MPC = sim_mpc.vvp

# 29. Support Vector Machine SMO (SVM) Sources (Solver #27)
RTL_SVM_DIR = ../rtl/svm_smo_32bit
SRCS_SVM = $(RTL_SVM_DIR)/svm_types_pkg.sv \
           $(RTL_SVM_DIR)/q16_alu.sv \
           $(RTL_SVM_DIR)/q16_divider.sv \
           $(RTL_SVM_DIR)/svm_kernel_engine.sv \
           $(RTL_SVM_DIR)/svm_pair_solver.sv \
           $(RTL_SVM_DIR)/svm_top.sv \
           $(TB_DIR)/tb_svm_smo.sv

OUT_SVM = sim_svm.vvp

# 30. Principal Component Analysis (PCA) Sources (Solver #28)
RTL_PCA_DIR = ../rtl/pca_32bit
SRCS_PCA = $(RTL_PCA_DIR)/pca_types_pkg.sv \
           $(RTL_PCA_DIR)/q16_alu.sv \
           $(RTL_PCA_DIR)/q16_divider.sv \
           $(RTL_PCA_DIR)/q16_sqrt.sv \
           $(RTL_PCA_DIR)/pca_cov_engine.sv \
           $(RTL_PCA_DIR)/pca_power_iter_engine.sv \
           $(RTL_PCA_DIR)/pca_top.sv \
           $(TB_DIR)/tb_pca.sv

OUT_PCA = sim_pca.vvp

# 31. Genetic Algorithm (GA) Sources (Solver #29)
RTL_GA_DIR = ../rtl/ga_32bit
SRCS_GA = $(RTL_GA_DIR)/ga_types_pkg.sv \
          $(RTL_GA_DIR)/q16_alu.sv \
          $(RTL_GA_DIR)/xorshift32_prng.sv \
          $(RTL_GA_DIR)/ga_fitness_engine.sv \
          $(RTL_GA_DIR)/ga_reproduce_engine.sv \
          $(RTL_GA_DIR)/ga_top.sv \
          $(TB_DIR)/tb_ga.sv

OUT_GA = sim_ga.vvp

# 32. Differential Evolution (DE) Sources (Solver #30)
RTL_DE_DIR = ../rtl/de_32bit
SRCS_DE = $(RTL_DE_DIR)/de_types_pkg.sv \
          $(RTL_DE_DIR)/q16_alu.sv \
          $(RTL_DE_DIR)/xorshift32_prng.sv \
          $(RTL_DE_DIR)/de_fitness_engine.sv \
          $(RTL_DE_DIR)/de_mutate_cross_engine.sv \
          $(RTL_DE_DIR)/de_top.sv \
          $(TB_DIR)/tb_de.sv

OUT_DE = sim_de.vvp

# 33. Cross-Entropy Method (CEM) Sources (Solver #31)
RTL_CEM_DIR = ../rtl/cem_32bit
SRCS_CEM = $(RTL_CEM_DIR)/cem_types_pkg.sv \
           $(RTL_CEM_DIR)/q16_alu.sv \
           $(RTL_CEM_DIR)/q16_divider.sv \
           $(RTL_CEM_DIR)/q16_sqrt.sv \
           $(RTL_CEM_DIR)/xorshift32_prng.sv \
           $(RTL_CEM_DIR)/cem_sample_engine.sv \
           $(RTL_CEM_DIR)/cem_elite_update_engine.sv \
           $(RTL_CEM_DIR)/cem_top.sv \
           $(TB_DIR)/tb_cem.sv

OUT_CEM = sim_cem.vvp

.PHONY: all clean run run_32bit run_64bit run_multivar run_lm run_irls run_bfgs run_cg run_gn run_sqp run_nm run_lbfgs run_lasso run_admm run_pgd run_dogleg run_pso run_fista run_ipm run_adam run_qubo run_pdhg run_fw run_alm run_dd run_rls run_ekf run_ukf run_mpc run_svm run_pca run_ga run_de run_cem compile_32bit compile_64bit compile_multivar compile_lm compile_irls compile_bfgs compile_cg compile_gn compile_sqp compile_nm compile_lbfgs compile_lasso compile_admm compile_pgd compile_dogleg compile_pso compile_fista compile_ipm compile_adam compile_qubo compile_pdhg compile_fw compile_alm compile_dd compile_rls compile_ekf compile_ukf compile_mpc compile_svm compile_pca compile_ga compile_de compile_cem

all: run_32bit run_64bit run_multivar run_lm run_irls run_bfgs run_cg run_gn run_sqp run_nm run_lbfgs run_lasso run_admm run_pgd run_dogleg run_pso run_fista run_ipm run_adam run_qubo run_pdhg run_fw run_alm run_dd run_rls run_ekf run_ukf run_mpc run_svm run_pca run_ga run_de run_cem

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

# FISTA Targets (Solver #15)
compile_fista: $(SRCS_FISTA)
	$(SIM) -g2012 -Wall -I$(RTL_FISTA_DIR) $(SRCS_FISTA) -o $(OUT_FISTA)

run_fista: compile_fista
	vvp $(OUT_FISTA)

# IPM Targets (Solver #16)
compile_ipm: $(SRCS_IPM)
	$(SIM) -g2012 -Wall -I$(RTL_IPM_DIR) $(SRCS_IPM) -o $(OUT_IPM)

run_ipm: compile_ipm
	vvp $(OUT_IPM)

# Adam Targets (Solver #17)
compile_adam: $(SRCS_ADAM)
	$(SIM) -g2012 -Wall -I$(RTL_ADAM_DIR) $(SRCS_ADAM) -o $(OUT_ADAM)

run_adam: compile_adam
	vvp $(OUT_ADAM)

# QUBO Targets (Solver #18)
compile_qubo: $(SRCS_QUBO)
	$(SIM) -g2012 -Wall -I$(RTL_QUBO_DIR) $(SRCS_QUBO) -o $(OUT_QUBO)

run_qubo: compile_qubo
	vvp $(OUT_QUBO)

# PDHG Targets (Solver #19)
compile_pdhg: $(SRCS_PDHG)
	$(SIM) -g2012 -Wall -I$(RTL_PDHG_DIR) $(SRCS_PDHG) -o $(OUT_PDHG)

run_pdhg: compile_pdhg
	vvp $(OUT_PDHG)

# Frank-Wolfe Targets (Solver #20)
compile_fw: $(SRCS_FW)
	$(SIM) -g2012 -Wall -I$(RTL_FW_DIR) $(SRCS_FW) -o $(OUT_FW)

run_fw: compile_fw
	vvp $(OUT_FW)

# ALM Targets (Solver #21)
compile_alm: $(SRCS_ALM)
	$(SIM) -g2012 -Wall -I$(RTL_ALM_DIR) $(SRCS_ALM) -o $(OUT_ALM)

run_alm: compile_alm
	vvp $(OUT_ALM)

# Dual Decomposition Targets (Solver #22)
compile_dd: $(SRCS_DD)
	$(SIM) -g2012 -Wall -I$(RTL_DD_DIR) $(SRCS_DD) -o $(OUT_DD)

run_dd: compile_dd
	vvp $(OUT_DD)

# Recursive Least Squares Targets (Solver #23)
compile_rls: $(SRCS_RLS)
	$(SIM) -g2012 -Wall -I$(RTL_RLS_DIR) $(SRCS_RLS) -o $(OUT_RLS)

run_rls: compile_rls
	vvp $(OUT_RLS)

# Extended Kalman Filter Targets (Solver #24)
compile_ekf: $(SRCS_EKF)
	$(SIM) -g2012 -Wall -I$(RTL_EKF_DIR) $(SRCS_EKF) -o $(OUT_EKF)

run_ekf: compile_ekf
	vvp $(OUT_EKF)

# Unscented Kalman Filter Targets (Solver #25)
compile_ukf: $(SRCS_UKF)
	$(SIM) -g2012 -Wall -I$(RTL_UKF_DIR) $(SRCS_UKF) -o $(OUT_UKF)

run_ukf: compile_ukf
	vvp $(OUT_UKF)

# Model Predictive Control Targets (Solver #26)
compile_mpc: $(SRCS_MPC)
	$(SIM) -g2012 -Wall -I$(RTL_MPC_DIR) $(SRCS_MPC) -o $(OUT_MPC)

run_mpc: compile_mpc
	vvp $(OUT_MPC)

# Support Vector Machine Targets (Solver #27)
compile_svm: $(SRCS_SVM)
	$(SIM) -g2012 -Wall -I$(RTL_SVM_DIR) $(SRCS_SVM) -o $(OUT_SVM)

run_svm: compile_svm
	vvp $(OUT_SVM)

# Principal Component Analysis Targets (Solver #28)
compile_pca: $(SRCS_PCA)
	$(SIM) -g2012 -Wall -I$(RTL_PCA_DIR) $(SRCS_PCA) -o $(OUT_PCA)

run_pca: compile_pca
	vvp $(OUT_PCA)

# Genetic Algorithm Targets (Solver #29)
compile_ga: $(SRCS_GA)
	$(SIM) -g2012 -Wall -I$(RTL_GA_DIR) $(SRCS_GA) -o $(OUT_GA)

run_ga: compile_ga
	vvp $(OUT_GA)

# Differential Evolution Targets (Solver #30)
compile_de: $(SRCS_DE)
	$(SIM) -g2012 -Wall -I$(RTL_DE_DIR) $(SRCS_DE) -o $(OUT_DE)

run_de: compile_de
	vvp $(OUT_DE)

# Cross-Entropy Method Targets (Solver #31)
compile_cem: $(SRCS_CEM)
	$(SIM) -g2012 -Wall -I$(RTL_CEM_DIR) $(SRCS_CEM) -o $(OUT_CEM)

run_cem: compile_cem
	vvp $(OUT_CEM)

clean:
	rm -f $(OUT_32) $(OUT_64) $(OUT_MV) $(OUT_LM) $(OUT_IRLS) $(OUT_BFGS) $(OUT_CG) $(OUT_GN) $(OUT_SQP) $(OUT_NM) $(OUT_LBFGS) $(OUT_LASSO) $(OUT_ADMM) $(OUT_PGD) $(OUT_DOGLEG) $(OUT_PSO) $(OUT_FISTA) $(OUT_IPM) $(OUT_ADAM) $(OUT_QUBO) $(OUT_PDHG) $(OUT_FW) $(OUT_ALM) $(OUT_DD) $(OUT_RLS) $(OUT_EKF) $(OUT_UKF) $(OUT_MPC) $(OUT_SVM) $(OUT_PCA) $(OUT_GA) $(OUT_DE) $(OUT_CEM) *.vcd *.vvp


