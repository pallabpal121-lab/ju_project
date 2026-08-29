// =============================================================================
// File Name   : ipm_top.sv
// Module Name : ipm_top
// Project     : Primal-Dual Interior Point Method (IPM) Accelerator (Solver #16)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Module for the Primal-Dual Interior Point Method (IPM)
//   Convex Quadratic Programming (QP) Accelerator.
//   Coordinates perturbed KKT residual evaluations, condensed augmented
//   normal equations (H_aug * Δx = -g_aug), hardware Cholesky decomposition,
//   fraction-to-the-boundary step length integration, and convergence detection.
// =============================================================================

`timescale 1ns / 1ps

import ipm_types_pkg::*;
`include "ipm_helpers.svh"

module ipm_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Model Microcode Programming Port
    // -------------------------------------------------------------------------
    input  logic               prog_en,       // Microcode Write Enable
    input  logic [4:0]         prog_addr,     // Microcode Address (0..31)
    input  instr_t             prog_data,     // 32-bit Microcode Instruction

    // -------------------------------------------------------------------------
    // Interface 2: QP Problem Formulation & Parameters
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start trigger
    input  logic [2:0]         num_vars,      // Number of primal variables N (1..4)
    input  logic [2:0]         num_cons,      // Number of inequality constraints M (1..4)
    input  mat_t               q_mat,         // Objective Quadratic Matrix Q (4x4)
    input  vec_t               c_vec,         // Objective Linear Cost Vector c (4x1)
    input  mat_t               a_mat,         // Constraint Matrix A (4x4)
    input  vec_t               b_vec,         // Constraint Bounds b (4x1)
    input  vec_t               x_init,        // Initial primal vector x_0
    input  vec_t               s_init,        // Initial slack vector s_0 (> 0)
    input  vec_t               z_init,        // Initial dual multiplier vector z_0 (> 0)
    input  q16_t               sigma_val,     // Centering parameter σ
    input  q16_t               tolerance,     // Convergence tolerance ε_tol
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 3: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,     // Optimal primal vector x*
    output vec_t               s_optimal,     // Optimal slack vector s*
    output vec_t               z_optimal,     // Optimal dual Lagrange multipliers z*
    output q16_t               f_optimal,     // Final QP objective cost f(x*)
    output q16_t               duality_gap,   // Final duality gap μ = (s^T z) / M
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        IPM_IDLE            = 3'd0,
        IPM_START_KKT       = 3'd1,
        IPM_WAIT_KKT        = 3'd2,
        IPM_WAIT_CHOLESKY   = 3'd3,
        IPM_WAIT_STEPS      = 3'd4,
        IPM_EVAL_FINAL_COST = 3'd5,
        IPM_WAIT_FINAL_COST = 3'd6,
        IPM_DONE            = 3'd7
    } ipm_state_t;

    ipm_state_t state;

    // Primal-Dual State Registers
    vec_t       x_reg;       // Primal vector x
    vec_t       s_reg;       // Slack vector s
    vec_t       z_reg;       // Dual multiplier vector z
    q16_t       mu_reg;      // Duality gap μ
    logic [7:0] iter_cnt;
    status_t    status_reg;
    q16_t       f_opt_reg;

    // Latched Parameters
    logic [2:0] num_vars_reg;
    logic [2:0] num_cons_reg;
    mat_t       q_mat_reg;
    vec_t       c_vec_reg;
    mat_t       a_mat_reg;
    vec_t       b_vec_reg;
    q16_t       sigma_reg;
    q16_t       tol_reg;
    logic [7:0] max_iters_reg;

    // KKT Engine Interconnect
    logic start_kkt_p1, start_kkt_p2;
    mat_t kkt_h_aug;
    vec_t kkt_g_aug;
    q16_t kkt_mu, kkt_rp_inf, kkt_rd_inf;
    vec_t kkt_ds, kkt_dz;
    q16_t kkt_alpha_p, kkt_alpha_d;
    logic kkt_done, kkt_busy;

    // Cholesky Solver Interconnect
    logic start_cholesky;
    vec_t chol_dx_out;
    logic chol_done, chol_busy, chol_error;

    // DFG Final Cost Engine Interconnect
    logic start_dfg;
    vec_t dfg_x_in;
    q16_t dfg_f_out;
    logic dfg_done, dfg_busy;

    ipm_kkt_engine u_kkt (
        .clk        (clk),
        .rst_n      (rst_n),
        .start_kkt  (start_kkt_p1),
        .start_steps(start_kkt_p2),
        .num_vars   (num_vars_reg),
        .num_cons   (num_cons_reg),
        .q_mat      (q_mat_reg),
        .c_vec      (c_vec_reg),
        .a_mat      (a_mat_reg),
        .b_vec      (b_vec_reg),
        .x_curr     (x_reg),
        .s_curr     (s_reg),
        .z_curr     (z_reg),
        .sigma_val  (sigma_reg),
        .tau_val    (ipm_types_pkg::Q16_TAU_DEF),
        .dx_in      (chol_dx_out),
        .h_aug_out  (kkt_h_aug),
        .g_aug_out  (kkt_g_aug),
        .mu_out     (kkt_mu),
        .rp_norm_inf(kkt_rp_inf),
        .rd_norm_inf(kkt_rd_inf),
        .ds_out     (kkt_ds),
        .dz_out     (kkt_dz),
        .alpha_p_out(kkt_alpha_p),
        .alpha_d_out(kkt_alpha_d),
        .done       (kkt_done),
        .busy       (kkt_busy)
    );

    cholesky_ipm_solver u_chol (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (start_cholesky),
        .num_dims(num_vars_reg),
        .h_mat   (kkt_h_aug),
        .g_vec   (kkt_g_aug),
        .dx_out  (chol_dx_out),
        .done    (chol_done),
        .busy    (chol_busy),
        .error   (chol_error)
    );

    dfg_ipm_engine u_dfg_final (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .num_dims   (num_vars_reg),
        .x_vec      (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    // Outputs
    assign x_optimal   = x_reg;
    assign s_optimal   = s_reg;
    assign z_optimal   = z_reg;
    assign f_optimal   = f_opt_reg;
    assign duality_gap = mu_reg;
    assign iter_count  = iter_cnt;
    assign status      = status_reg;

    q16_t s_elem, z_elem;
    vec_t temp_x, temp_s, temp_z;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= IPM_IDLE;
            x_reg          <= '0;
            s_reg          <= '0;
            z_reg          <= '0;
            mu_reg         <= Q16_ZERO;
            iter_cnt       <= 8'd0;
            status_reg     <= STATUS_IDLE;
            f_opt_reg      <= Q16_ZERO;
            num_vars_reg   <= 3'd2;
            num_cons_reg   <= 3'd2;
            q_mat_reg      <= '0;
            c_vec_reg      <= '0;
            a_mat_reg      <= '0;
            b_vec_reg      <= '0;
            sigma_reg      <= Q16_SIGMA_DEF;
            tol_reg        <= Q16_EPS_DEF;
            max_iters_reg  <= 8'd30;
            start_kkt_p1   <= 1'b0;
            start_kkt_p2   <= 1'b0;
            start_cholesky <= 1'b0;
            start_dfg      <= 1'b0;
            dfg_x_in       <= '0;
            done           <= 1'b0;
            busy           <= 1'b0;
        end else begin
            start_kkt_p1   <= 1'b0;
            start_kkt_p2   <= 1'b0;
            start_cholesky <= 1'b0;
            start_dfg      <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: IPM_IDLE - Latch Problem Formulation & Initialize
                // -------------------------------------------------------------
                IPM_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy          <= 1'b1;
                        num_vars_reg  <= num_vars;
                        num_cons_reg  <= num_cons;
                        q_mat_reg     <= q_mat;
                        c_vec_reg     <= c_vec;
                        a_mat_reg     <= a_mat;
                        b_vec_reg     <= b_vec;
                        sigma_reg     <= (sigma_val != Q16_ZERO) ? sigma_val : Q16_SIGMA_DEF;
                        tol_reg       <= (tolerance != Q16_ZERO) ? tolerance : Q16_EPS_DEF;
                        max_iters_reg <= max_iters;
                        x_reg         <= x_init;

                        // Ensure initial slacks and dual multipliers are strictly positive
                        temp_s = '0;
                        temp_z = '0;
                        for (int i = 0; i < MAX_CONSTRAINTS; i++) begin
                            if (i < num_cons) begin
                                s_elem = get_vec(s_init, 2'(i));
                                z_elem = get_vec(z_init, 2'(i));
                                temp_s = set_vec(temp_s, 2'(i), (s_elem > Q16_ZERO) ? s_elem : Q16_ONE);
                                temp_z = set_vec(temp_z, 2'(i), (z_elem > Q16_ZERO) ? z_elem : Q16_ONE);
                            end
                        end
                        s_reg <= temp_s;
                        z_reg <= temp_z;

                        iter_cnt   <= 8'd0;
                        status_reg <= STATUS_RUNNING;
                        state      <= IPM_START_KKT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: IPM_START_KKT - Trigger Phase 1 KKT Evaluation
                // -------------------------------------------------------------
                IPM_START_KKT: begin
                    start_kkt_p1 <= 1'b1;
                    state        <= IPM_WAIT_KKT;
                end

                // -------------------------------------------------------------
                // STATE 2: IPM_WAIT_KKT - Convergence Check & Trigger Cholesky
                // -------------------------------------------------------------
                IPM_WAIT_KKT: begin
                    if (kkt_done) begin
                        mu_reg <= kkt_mu;

                        // Check primal-dual convergence: μ <= tol and ||r_p|| <= tol
                        if ((kkt_mu <= tol_reg && kkt_rp_inf <= tol_reg) ||
                            (kkt_mu <= (tol_reg <<< 2) && kkt_rp_inf <= tol_reg && kkt_rd_inf <= tol_reg)) begin
                            status_reg <= STATUS_CONVERGED;
                            state      <= IPM_EVAL_FINAL_COST;
                        end else if (iter_cnt >= max_iters_reg) begin
                            status_reg <= STATUS_MAX_ITERS;
                            state      <= IPM_EVAL_FINAL_COST;
                        end else begin
                            // Trigger Hardware Cholesky Solver for H_aug * Δx = -g_aug
                            start_cholesky <= 1'b1;
                            state          <= IPM_WAIT_CHOLESKY;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE 3: IPM_WAIT_CHOLESKY - Trigger Phase 2 Step Recovery
                // -------------------------------------------------------------
                IPM_WAIT_CHOLESKY: begin
                    if (chol_done) begin
                        start_kkt_p2 <= 1'b1;
                        state        <= IPM_WAIT_STEPS;
                    end
                end

                // -------------------------------------------------------------
                // STATE 4: IPM_WAIT_STEPS - Integrate Primal-Dual Updates
                // -------------------------------------------------------------
                IPM_WAIT_STEPS: begin
                    if (kkt_done) begin
                        // 1. Update primal variables: x_{k+1} = x_k + α_p * Δx
                        temp_x = x_reg;
                        for (int i = 0; i < MAX_VARS; i++) begin
                            if (i < num_vars_reg) begin
                                temp_x = set_vec(temp_x, 2'(i), get_vec(x_reg, 2'(i)) + q16_mul(kkt_alpha_p, get_vec(chol_dx_out, 2'(i))));
                            end
                        end
                        x_reg <= temp_x;

                        // 2. Update slacks: s_{k+1} = s_k + α_p * Δs
                        // 3. Update dual multipliers: z_{k+1} = z_k + α_d * Δz
                        temp_s = s_reg;
                        temp_z = z_reg;
                        for (int i = 0; i < MAX_CONSTRAINTS; i++) begin
                            if (i < num_cons_reg) begin
                                s_elem = get_vec(s_reg, 2'(i)) + q16_mul(kkt_alpha_p, get_vec(kkt_ds, 2'(i)));
                                z_elem = get_vec(z_reg, 2'(i)) + q16_mul(kkt_alpha_d, get_vec(kkt_dz, 2'(i)));
                                temp_s = set_vec(temp_s, 2'(i), (s_elem > 32'h0000_0010) ? s_elem : 32'h0000_0010);
                                temp_z = set_vec(temp_z, 2'(i), (z_elem > 32'h0000_0010) ? z_elem : 32'h0000_0010);
                            end
                        end
                        s_reg <= temp_s;
                        z_reg <= temp_z;

                        iter_cnt <= iter_cnt + 1'b1;
                        state    <= IPM_START_KKT;
                    end
                end

                // -------------------------------------------------------------
                // STATE 5: IPM_EVAL_FINAL_COST - Evaluate User Objective f(x*)
                // -------------------------------------------------------------
                IPM_EVAL_FINAL_COST: begin
                    dfg_x_in  <= x_reg;
                    start_dfg <= 1'b1;
                    state     <= IPM_WAIT_FINAL_COST;
                end

                // -------------------------------------------------------------
                // STATE 6: IPM_WAIT_FINAL_COST - Latch Objective Cost
                // -------------------------------------------------------------
                IPM_WAIT_FINAL_COST: begin
                    if (dfg_done) begin
                        f_opt_reg <= dfg_f_out;
                        state     <= IPM_DONE;
                    end
                end

                // -------------------------------------------------------------
                // STATE 7: IPM_DONE - Completion Strobe
                // -------------------------------------------------------------
                IPM_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= IPM_IDLE;
                end

                default: state <= IPM_IDLE;
            endcase
        end
    end

endmodule
