// =============================================================================
// File Name   : alm_top.sv
// Module Name : alm_top
// Project     : Augmented Lagrangian Method (ALM) Accelerator (Solver #21)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Module for Augmented Lagrangian Method (ALM) Accelerator.
//   Solves equality and inequality constrained quadratic optimization:
//   min 0.5*x^T Q x - p^T x  s.t.  A x = b,  C x <= d.
//
//   Master FSM alternates:
//   1. Build augmented Hessian H_aug = Q + rho * A^T A + rho * C_act^T C_act + λ_damp I
//   2. Build augmented gradient g_aug = Q*x - p + A^T(λ + rho*r_eq) + C^T max(0, μ + rho*r_ineq)
//   3. Solve H_aug * Δx = -g_aug via Hardware Cholesky Decomposition
//   4. Update primal state x_{k+1} = x_k + Δx
//   5. Update dual multipliers λ_{k+1} and μ_{k+1} and check feasibility
// =============================================================================

`timescale 1ns / 1ps

import alm_types_pkg::*;
`include "alm_helpers.svh"

module alm_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Problem Formulation & Controls
    // -------------------------------------------------------------------------
    input  logic               start,            // 1-cycle start trigger
    input  logic [2:0]         dim_n,            // Primal variables N (1..4)
    input  logic [2:0]         num_eq,           // Equality constraints M_eq (0..4)
    input  logic [2:0]         num_ineq,         // Inequality constraints M_ineq (0..4)

    input  mat_t               q_matrix,         // Quadratic matrix Q (N x N)
    input  vec_t               p_vector,         // Linear objective vector p
    input  mat_t               a_matrix,         // Equality matrix A (M_eq x N)
    input  vec_t               b_vector,         // Equality RHS b
    input  mat_t               c_matrix,         // Inequality matrix C (M_ineq x N)
    input  vec_t               d_vector,         // Inequality RHS d
    input  vec_t               x_init,           // Initial primal guess x_0

    input  q16_t               tol_feas,         // Feasibility tolerance ε_feas
    input  q16_t               tol_opt,          // Optimality step tolerance ε_opt
    input  logic [15:0]        max_iters,        // Maximum outer iterations

    // -------------------------------------------------------------------------
    // Interface 2: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,        // Optimal primal point x*
    output vec_t               lambda_optimal,   // Optimal equality multipliers λ*
    output vec_t               mu_optimal,       // Optimal inequality multipliers μ*
    output q16_t               f_optimal,        // Minimum objective f(x*)
    output q16_t               feas_error,       // Constraint feasibility violation ||r||_inf
    output logic [15:0]        iter_count,       // Outer iterations completed
    output status_t            status,           // Convergence status code
    output logic               done,             // 1-cycle completion strobe
    output logic               busy              // High while active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        ALM_IDLE       = 4'd0,
        ALM_BUILD_AUG  = 4'd1,
        ALM_SOLVE_PRIM = 4'd2,
        ALM_WAIT_PRIM  = 4'd3,
        ALM_UPDATE_X   = 4'd4,
        ALM_UPDATE_DUAL= 4'd5,
        ALM_WAIT_DUAL  = 4'd6,
        ALM_CHECK_CONV = 4'd7,
        ALM_EVAL_F     = 4'd8
    } alm_state_t;

    alm_state_t state;

    // Registers
    vec_t        x_curr;
    vec_t        lambda_curr;
    vec_t        mu_curr;
    q16_t        rho_curr;
    q16_t        prev_viol_reg;
    q16_t        curr_viol_reg;
    q16_t        step_norm_reg;
    q16_t        f_reg;
    logic [15:0] iters_cnt;
    status_t     status_reg;

    // Latched inputs
    logic [2:0]  dim_n_reg;
    logic [2:0]  num_eq_reg;
    logic [2:0]  num_ineq_reg;
    mat_t        q_mat_reg;
    vec_t        p_vec_reg;
    mat_t        a_mat_reg;
    vec_t        b_vec_reg;
    mat_t        c_mat_reg;
    vec_t        d_vec_reg;
    q16_t        tol_feas_reg;
    q16_t        tol_opt_reg;
    logic [15:0] max_iters_reg;

    // Augmented system matrices
    mat_t h_aug;
    vec_t g_aug;

    // Sub-Engine 1: Hardware Cholesky Linear Solver
    logic start_cholesky;
    vec_t dx_out;
    logic cholesky_done, cholesky_busy, cholesky_err;

    cholesky_alm_solver u_cholesky (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (start_cholesky),
        .num_dims(dim_n_reg),
        .h_mat   (h_aug),
        .g_vec   (g_aug),
        .dx_out  (dx_out),
        .done    (cholesky_done),
        .busy    (cholesky_busy),
        .error   (cholesky_err)
    );

    // Sub-Engine 2: Hardware Dual Engine
    logic start_dual;
    vec_t lambda_next_w, mu_next_w;
    q16_t rho_next_w, curr_viol_w;
    logic dual_done, dual_busy;

    alm_dual_engine u_dual (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start_dual),
        .dim_n      (dim_n_reg),
        .num_eq     (num_eq_reg),
        .num_ineq   (num_ineq_reg),
        .a_mat      (a_mat_reg),
        .b_vec      (b_vec_reg),
        .c_mat      (c_mat_reg),
        .d_vec      (d_vec_reg),
        .x_curr     (x_curr),
        .lambda_curr(lambda_curr),
        .mu_curr    (mu_curr),
        .rho_curr   (rho_curr),
        .prev_viol  (prev_viol_reg),
        .lambda_next(lambda_next_w),
        .mu_next    (mu_next_w),
        .rho_next   (rho_next_w),
        .curr_viol  (curr_viol_w),
        .done       (dual_done),
        .busy       (dual_busy)
    );

    // Outputs
    assign x_optimal      = x_curr;
    assign lambda_optimal = lambda_curr;
    assign mu_optimal     = mu_curr;
    assign f_optimal      = f_reg;
    assign feas_error     = curr_viol_reg;
    assign iter_count     = iters_cnt;
    assign status         = status_reg;

    // Internal temporary arithmetic variables
    vec_t q_x, a_x, c_x;
    vec_t r_eq, r_ineq;
    vec_t lam_eff, mu_eff;
    vec_t at_lam, ct_mu;
    q16_t ata_elem, ctc_elem;
    q16_t a_mi, a_mj, c_mi, c_mj;
    q16_t q_elem, h_val;
    mat_t h_aug_comb;
    vec_t g_aug_comb;
    logic ineq_active[MAX_DIM];
    q16_t mu_trial;
    q16_t half_x_q_x, p_x;
    logic [2:0] inner_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ALM_IDLE;
            x_curr         <= '0;
            lambda_curr    <= '0;
            mu_curr        <= '0;
            rho_curr       <= Q16_RHO_INIT;
            prev_viol_reg  <= Q16_ZERO;
            curr_viol_reg  <= Q16_ZERO;
            step_norm_reg  <= Q16_ZERO;
            f_reg          <= Q16_ZERO;
            iters_cnt      <= 16'd0;
            inner_cnt      <= 3'd0;
            status_reg     <= STATUS_IDLE;
            dim_n_reg      <= 3'd2;
            num_eq_reg     <= 3'd0;
            num_ineq_reg   <= 3'd0;
            q_mat_reg      <= '0;
            p_vec_reg      <= '0;
            a_mat_reg      <= '0;
            b_vec_reg      <= '0;
            c_mat_reg      <= '0;
            d_vec_reg      <= '0;
            tol_feas_reg   <= Q16_FEAS_TOL;
            tol_opt_reg    <= Q16_OPT_TOL;
            max_iters_reg  <= 16'd50;
            h_aug          <= '0;
            g_aug          <= '0;
            start_cholesky <= 1'b0;
            start_dual     <= 1'b0;
            done           <= 1'b0;
            busy           <= 1'b0;
        end else begin
            start_cholesky <= 1'b0;
            start_dual     <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: ALM_IDLE - Latch User Configuration & Initialize
                // -------------------------------------------------------------
                ALM_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy          <= 1'b1;
                        dim_n_reg     <= dim_n;
                        num_eq_reg    <= num_eq;
                        num_ineq_reg  <= num_ineq;
                        q_mat_reg     <= q_matrix;
                        p_vec_reg     <= p_vector;
                        a_mat_reg     <= a_matrix;
                        b_vec_reg     <= b_vector;
                        c_mat_reg     <= c_matrix;
                        d_vec_reg     <= d_vector;
                        x_curr        <= x_init;
                        lambda_curr   <= '0;
                        mu_curr       <= '0;
                        rho_curr      <= Q16_RHO_INIT;
                        prev_viol_reg <= Q16_MAX_POS;
                        tol_feas_reg  <= (tol_feas != Q16_ZERO) ? tol_feas : Q16_FEAS_TOL;
                        tol_opt_reg   <= (tol_opt != Q16_ZERO) ? tol_opt : Q16_OPT_TOL;
                        max_iters_reg <= (max_iters != 16'd0) ? max_iters : 16'd50;
                        iters_cnt     <= 16'd0;
                        inner_cnt     <= 3'd0;
                        status_reg    <= STATUS_RUNNING;
                        state         <= ALM_BUILD_AUG;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: ALM_BUILD_AUG - Form Augmented Hessian & Gradient
                // -------------------------------------------------------------
                ALM_BUILD_AUG: begin
                    // Compute residuals
                    a_x    = mat_vec_mul(a_mat_reg, x_curr, num_eq_reg, dim_n_reg);
                    c_x    = mat_vec_mul(c_mat_reg, x_curr, num_ineq_reg, dim_n_reg);
                    r_eq   = vec_sub(a_x, b_vec_reg, num_eq_reg);
                    r_ineq = vec_sub(c_x, d_vec_reg, num_ineq_reg);

                    // Identify active inequalities: mu_i + rho * r_ineq, i > 0
                    for (int m = 0; m < MAX_DIM; m++) begin
                        if (m < num_ineq_reg) begin
                            mu_trial = mu_curr[m] + q16_mul(rho_curr, r_ineq[m]);
                            ineq_active[m] = (mu_trial > Q16_ZERO);
                            mu_eff[m]      = (mu_trial > Q16_ZERO) ? mu_trial : Q16_ZERO;
                        end else begin
                            ineq_active[m] = 1'b0;
                            mu_eff[m]      = Q16_ZERO;
                        end
                    end

                    // Effective equality multipliers: λ_eff = λ + rho * r_eq
                    for (int m = 0; m < MAX_DIM; m++) begin
                        if (m < num_eq_reg) begin
                            lam_eff[m] = lambda_curr[m] + q16_mul(rho_curr, r_eq[m]);
                        end else begin
                            lam_eff[m] = Q16_ZERO;
                        end
                    end

                    // Build H_aug = Q + rho * A^T A + rho * C_act^T C_act + λ_damp I
                    h_aug_comb = '0;
                    for (int i = 0; i < MAX_DIM; i++) begin
                        for (int j = 0; j < MAX_DIM; j++) begin
                            if (i < dim_n_reg && j < dim_n_reg) begin
                                q_elem = get_mat(q_mat_reg, 2'(i), 2'(j));

                                ata_elem = Q16_ZERO;
                                for (int m = 0; m < MAX_DIM; m++) begin
                                    if (m < num_eq_reg) begin
                                        a_mi = get_mat(a_mat_reg, 2'(m), 2'(i));
                                        a_mj = get_mat(a_mat_reg, 2'(m), 2'(j));
                                        ata_elem = ata_elem + q16_mul(a_mi, a_mj);
                                    end
                                end

                                ctc_elem = Q16_ZERO;
                                for (int m = 0; m < MAX_DIM; m++) begin
                                    if (m < num_ineq_reg && ineq_active[m]) begin
                                        c_mi = get_mat(c_mat_reg, 2'(m), 2'(i));
                                        c_mj = get_mat(c_mat_reg, 2'(m), 2'(j));
                                        ctc_elem = ctc_elem + q16_mul(c_mi, c_mj);
                                    end
                                end

                                h_val = q_elem + q16_mul(rho_curr, ata_elem + ctc_elem);
                                if (i == j) begin
                                    h_val = h_val + Q16_DAMP_DEF;
                                end
                                h_aug_comb = set_mat(h_aug_comb, 2'(i), 2'(j), h_val);
                            end else begin
                                h_aug_comb = set_mat(h_aug_comb, 2'(i), 2'(j), (i == j) ? Q16_ONE : Q16_ZERO);
                            end
                        end
                    end
                    h_aug <= h_aug_comb;

                    // Build g_aug = Q*x - p + A^T * lam_eff + C^T * mu_eff
                    q_x    = mat_vec_mul(q_mat_reg, x_curr, dim_n_reg, dim_n_reg);
                    at_lam = mat_t_vec_mul(a_mat_reg, lam_eff, num_eq_reg, dim_n_reg);
                    ct_mu  = mat_t_vec_mul(c_mat_reg, mu_eff, num_ineq_reg, dim_n_reg);

                    g_aug_comb = '0;
                    for (int i = 0; i < MAX_DIM; i++) begin
                        if (i < dim_n_reg) begin
                            g_aug_comb[i] = q_x[i] - p_vec_reg[i] + at_lam[i] + ct_mu[i];
                        end else begin
                            g_aug_comb[i] = Q16_ZERO;
                        end
                    end
                    g_aug <= g_aug_comb;

                    state <= ALM_SOLVE_PRIM;
                end

                // -------------------------------------------------------------
                // STATE 2: ALM_SOLVE_PRIM - Trigger Hardware Cholesky Solver
                // -------------------------------------------------------------
                ALM_SOLVE_PRIM: begin
                    start_cholesky <= 1'b1;
                    state          <= ALM_WAIT_PRIM;
                end

                // -------------------------------------------------------------
                // STATE 3: ALM_WAIT_PRIM - Await Cholesky Step Completion
                // -------------------------------------------------------------
                ALM_WAIT_PRIM: begin
                    if (cholesky_done) begin
                        state <= ALM_UPDATE_X;
                    end
                end

                // -------------------------------------------------------------
                // STATE 4: ALM_UPDATE_X - Apply Primal Step x = x + Δx & Inner Loop
                // -------------------------------------------------------------
                ALM_UPDATE_X: begin
                    step_norm_reg <= Q16_ZERO;
                    for (int i = 0; i < MAX_DIM; i++) begin
                        if (i < dim_n_reg) begin
                            x_curr[i] <= x_curr[i] + dx_out[i];
                            if (q16_abs(dx_out[i]) > step_norm_reg) begin
                                step_norm_reg <= q16_abs(dx_out[i]);
                            end
                        end
                    end

                    // If inner step converged or reached max inner steps, update dual multipliers
                    if (inner_cnt >= 3'd3 || (q16_abs(dx_out[0]) < tol_opt_reg && q16_abs(dx_out[1]) < tol_opt_reg)) begin
                        inner_cnt <= 3'd0;
                        state     <= ALM_UPDATE_DUAL;
                    end else begin
                        inner_cnt <= inner_cnt + 1'b1;
                        state     <= ALM_BUILD_AUG;
                    end
                end

                // -------------------------------------------------------------
                // STATE 5: ALM_UPDATE_DUAL - Trigger Dual Multiplier Engine
                // -------------------------------------------------------------
                ALM_UPDATE_DUAL: begin
                    start_dual <= 1'b1;
                    state      <= ALM_WAIT_DUAL;
                end

                // -------------------------------------------------------------
                // STATE 6: ALM_WAIT_DUAL - Latch Multiplier Updates & Residuals
                // -------------------------------------------------------------
                ALM_WAIT_DUAL: begin
                    if (dual_done) begin
                        lambda_curr   <= lambda_next_w;
                        mu_curr       <= mu_next_w;
                        rho_curr      <= rho_next_w;
                        prev_viol_reg <= curr_viol_reg;
                        curr_viol_reg <= curr_viol_w;
                        iters_cnt     <= iters_cnt + 1'b1;
                        state         <= ALM_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // STATE 7: ALM_CHECK_CONV - Convergence & Max-Iter Evaluation
                // -------------------------------------------------------------
                ALM_CHECK_CONV: begin
                    if (curr_viol_reg <= tol_feas_reg && step_norm_reg <= tol_opt_reg) begin
                        status_reg <= STATUS_CONVERGED;
                        state      <= ALM_EVAL_F;
                    end else if (iters_cnt >= max_iters_reg) begin
                        status_reg <= STATUS_MAX_ITERS;
                        state      <= ALM_EVAL_F;
                    end else begin
                        state <= ALM_BUILD_AUG;
                    end
                end

                // -------------------------------------------------------------
                // STATE 8: ALM_EVAL_F - Compute Final Objective f(x*) = 0.5*x^T Q x - p^T x
                // -------------------------------------------------------------
                ALM_EVAL_F: begin
                    q_x        = mat_vec_mul(q_mat_reg, x_curr, dim_n_reg, dim_n_reg);
                    half_x_q_x = q16_mul(Q16_HALF, dot_product(x_curr, q_x, dim_n_reg));
                    p_x        = dot_product(p_vec_reg, x_curr, dim_n_reg);
                    f_reg      <= half_x_q_x - p_x;
                    done       <= 1'b1;
                    busy       <= 1'b0;
                    state      <= ALM_IDLE;
                end

                default: state <= ALM_IDLE;
            endcase
        end
    end

endmodule
