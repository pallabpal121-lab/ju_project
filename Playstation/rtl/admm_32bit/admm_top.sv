// =============================================================================
// File Name   : admm_top.sv
// Module Name : admm_top
// Project     : Alternating Direction Method of Multipliers (ADMM) Accelerator (Solver #11)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Top-level SoC Controller for the 3-Phase ADMM Accelerator:
//     Phase 1: Primal x-update via Hardware Cholesky Linear Solver
//     Phase 2: Primal z-update via Hardware Soft-Thresholding S_{lambda/rho}
//     Phase 3: Dual u-update via Accumulation u_{k+1} = u_k + (x_{k+1} - z_{k+1})
// =============================================================================

`timescale 1ns / 1ps

import admm_types_pkg::*;
`include "admm_helpers.svh"

module admm_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Dataset & Optimization Controls
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start pulse
    input  logic [2:0]         num_params,    // Number of features N (1..4)
    input  logic [3:0]         num_obs,       // Number of samples M (1..8)
    input  dataset_mat_t       a_matrix,      // Dataset matrix A (M x N)
    input  obs_vec_t           b_obs,         // Target observation vector b (M x 1)
    input  q16_t               lambda_reg,    // L1 regularization penalty lambda
    input  q16_t               rho_val,       // Augmented Lagrangian penalty rho
    input  q16_t               tolerance,     // Convergence threshold on residuals
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 2: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               z_optimal,     // Converged sparse primal vector z*
    output vec_t               x_primal,      // Final primal consensus vector x*
    output vec_t               u_dual,        // Final dual multiplier vector u*
    output q16_t               r_pri_norm,    // Final primal residual norm ||x - z||_inf
    output q16_t               s_dual_norm,   // Final dual residual norm ||rho*(z - z_prev)||_inf
    output logic [2:0]         sparsity_count,// Number of exact zero weights
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while solving
);

    // -------------------------------------------------------------------------
    // Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        TOP_IDLE         = 4'd0,
        TOP_DIV_TAU      = 4'd1,
        TOP_DIV_TAU_WAIT = 4'd2,
        TOP_BUILD_SYS    = 4'd3,
        TOP_WAIT_BUILD   = 4'd4,
        TOP_PHASE1_START = 4'd5,
        TOP_PHASE1_WAIT  = 4'd6,
        TOP_PHASE2_3     = 4'd7,
        TOP_CHECK_CONV   = 4'd8,
        TOP_CALC_METRICS = 4'd9,
        TOP_DONE         = 4'd10
    } top_state_t;

    top_state_t state;

    // Registers
    logic [2:0] num_params_reg;
    logic [3:0] num_obs_reg;
    q16_t       lambda_reg_val;
    q16_t       rho_val_reg;
    q16_t       tol_reg;
    logic [7:0] max_iters_reg;

    q16_t       tau_thresh;    // tau = lambda / rho
    vec_t       x_cur;
    vec_t       z_cur, z_prev;
    vec_t       u_cur;

    // Dedicated Divider for tau = lambda / rho
    logic div_start;
    q16_t div_dividend, div_divisor, div_quotient;
    logic div_done, div_by_zero, div_busy;

    q16_divider u_div_tau (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (div_start),
        .dividend   (div_dividend),
        .divisor    (div_divisor),
        .quotient   (div_quotient),
        .done       (div_done),
        .div_by_zero(div_by_zero),
        .busy       (div_busy)
    );

    // Primal X-Engine Interconnect
    logic start_engine_build;
    logic start_engine_solve;
    vec_t engine_x_out;
    mat_t engine_h_out;
    vec_t engine_atb_out;
    logic engine_build_done;
    logic engine_solve_done;
    logic engine_busy;

    admm_primal_x_engine u_primal_x (
        .clk        (clk),
        .rst_n      (rst_n),
        .start_build(start_engine_build),
        .start_solve(start_engine_solve),
        .num_params (num_params_reg),
        .num_obs    (num_obs_reg),
        .a_matrix   (a_matrix),
        .b_obs      (b_obs),
        .rho_val    (rho_val_reg),
        .z_cur      (z_cur),
        .u_cur      (u_cur),
        .x_next_out (engine_x_out),
        .h_mat_out  (engine_h_out),
        .at_b_out   (engine_atb_out),
        .build_done (engine_build_done),
        .solve_done (engine_solve_done),
        .busy       (engine_busy)
    );

    // Soft-Thresholding Unit
    q16_t thresh_v_in;
    q16_t thresh_s_out;
    logic thresh_is_zero;

    q16_soft_threshold u_thresh (
        .v_in   (thresh_v_in),
        .tau_in (tau_thresh),
        .s_out  (thresh_s_out),
        .is_zero(thresh_is_zero)
    );

    vec_t       r_pri_vec, s_dual_vec;
    q16_t       cur_v, cur_z_new, diff_z;
    logic [2:0] zero_cnt;
    vec_t       tmp_z, tmp_u;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= TOP_IDLE;
            num_params_reg     <= 3'd2;
            num_obs_reg        <= 4'd4;
            lambda_reg_val     <= Q16_ZERO;
            rho_val_reg        <= Q16_RHO_DEF;
            tol_reg            <= Q16_EPS_DEF;
            max_iters_reg      <= 8'd50;
            iter_count         <= 8'd0;
            status             <= STATUS_IDLE;
            done               <= 1'b0;
            busy               <= 1'b0;
            z_optimal          <= '0;
            x_primal           <= '0;
            u_dual             <= '0;
            r_pri_norm         <= Q16_ZERO;
            s_dual_norm        <= Q16_ZERO;
            sparsity_count     <= 3'd0;
            tau_thresh         <= Q16_ZERO;
            x_cur              <= '0;
            z_cur              <= '0;
            z_prev             <= '0;
            u_cur              <= '0;
            div_start          <= 1'b0;
            div_dividend       <= Q16_ZERO;
            div_divisor        <= Q16_ZERO;
            start_engine_build <= 1'b0;
            start_engine_solve <= 1'b0;
            thresh_v_in        <= Q16_ZERO;
            r_pri_vec          <= '0;
            s_dual_vec         <= '0;
            zero_cnt           <= 3'd0;
        end else begin
            div_start          <= 1'b0;
            start_engine_build <= 1'b0;
            start_engine_solve <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: TOP_IDLE
                // -------------------------------------------------------------
                TOP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy           <= 1'b1;
                        num_params_reg <= (num_params != 3'd0) ? num_params : 3'd2;
                        num_obs_reg    <= (num_obs != 4'd0)    ? num_obs    : 4'd4;
                        lambda_reg_val <= lambda_reg;
                        rho_val_reg    <= (rho_val != Q16_ZERO)? rho_val    : Q16_RHO_DEF;
                        tol_reg        <= (tolerance != Q16_ZERO) ? tolerance : Q16_EPS_DEF;
                        max_iters_reg  <= (max_iters != 8'd0)  ? max_iters  : 8'd50;
                        iter_count     <= 8'd0;
                        status         <= STATUS_RUNNING;
                        x_cur          <= '0;
                        z_cur          <= '0;
                        z_prev         <= '0;
                        u_cur          <= '0;

                        if (lambda_reg == Q16_ZERO) begin
                            tau_thresh <= Q16_ZERO;
                            state      <= TOP_BUILD_SYS;
                        end else begin
                            div_dividend <= lambda_reg;
                            div_divisor  <= (rho_val != Q16_ZERO) ? rho_val : Q16_RHO_DEF;
                            div_start    <= 1'b1;
                            state        <= TOP_DIV_TAU_WAIT;
                        end
                    end else begin
                        busy <= 1'b0;
                    end
                end

                TOP_DIV_TAU_WAIT: begin
                    if (div_done) begin
                        tau_thresh <= div_quotient;
                        state      <= TOP_BUILD_SYS;
                    end
                end

                // -------------------------------------------------------------
                // BUILD PRIMAL SYSTEM H = A^T*A + rho*I AND A^T*b
                // -------------------------------------------------------------
                TOP_BUILD_SYS: begin
                    start_engine_build <= 1'b1;
                    state              <= TOP_WAIT_BUILD;
                end

                TOP_WAIT_BUILD: begin
                    if (engine_build_done) begin
                        state <= TOP_PHASE1_START;
                    end
                end

                // -------------------------------------------------------------
                // PHASE 1: PRIMAL x-UPDATE VIA CHOLESKY SOLVER
                // -------------------------------------------------------------
                TOP_PHASE1_START: begin
                    start_engine_solve <= 1'b1;
                    state              <= TOP_PHASE1_WAIT;
                end

                TOP_PHASE1_WAIT: begin
                    if (engine_solve_done) begin
                        x_cur <= engine_x_out;
                        state <= TOP_PHASE2_3;
                    end
                end

                // -------------------------------------------------------------
                // PHASES 2 & 3: PRIMAL z-UPDATE & DUAL u-UPDATE
                // -------------------------------------------------------------
                TOP_PHASE2_3: begin
                    z_prev <= z_cur;
                    tmp_z  = z_cur;
                    tmp_u  = u_cur;

                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params_reg) begin
                            // v_k = x_{k+1} + u_k
                            cur_v = get_vec(x_cur, 2'(k)) + get_vec(u_cur, 2'(k));

                            // Soft-Thresholding S_tau(v_k)
                            if (tau_thresh <= Q16_ZERO) begin
                                cur_z_new = cur_v;
                            end else if (cur_v > tau_thresh) begin
                                cur_z_new = cur_v - tau_thresh;
                            end else if (cur_v < -tau_thresh) begin
                                cur_z_new = cur_v + tau_thresh;
                            end else begin
                                cur_z_new = Q16_ZERO; // Exact zero!
                            end

                            tmp_z = set_vec(tmp_z, 2'(k), cur_z_new);

                            // Dual u_{k+1} = u_k + (x_{k+1} - z_{k+1})
                            tmp_u = set_vec(tmp_u, 2'(k), get_vec(u_cur, 2'(k)) + (get_vec(x_cur, 2'(k)) - cur_z_new));
                        end
                    end

                    z_cur <= tmp_z;
                    u_cur <= tmp_u;
                    state <= TOP_CHECK_CONV;
                end

                // -------------------------------------------------------------
                // CHECK PRIMAL & DUAL RESIDUAL CONVERGENCE
                // -------------------------------------------------------------
                TOP_CHECK_CONV: begin
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params_reg) begin
                            r_pri_vec  = set_vec(r_pri_vec, 2'(k), get_vec(x_cur, 2'(k)) - get_vec(z_cur, 2'(k)));
                            diff_z     = get_vec(z_cur, 2'(k)) - get_vec(z_prev, 2'(k));
                            s_dual_vec = set_vec(s_dual_vec, 2'(k), q16_t'((64'(rho_val_reg) * 64'(diff_z)) >>> 16));
                        end
                    end

                    r_pri_norm  <= q16_norm_inf(r_pri_vec, num_params_reg);
                    s_dual_norm <= q16_norm_inf(s_dual_vec, num_params_reg);

                    // Condition 1: Converged if both primal and dual residuals <= tolerance (after at least 1 iteration)
                    if (iter_count > 8'd0 &&
                        q16_norm_inf(r_pri_vec, num_params_reg) <= tol_reg &&
                        q16_norm_inf(s_dual_vec, num_params_reg) <= tol_reg) begin
                        status <= STATUS_CONVERGED;
                        state  <= TOP_CALC_METRICS;

                    // Condition 2: Max iterations reached
                    end else if (iter_count >= max_iters_reg) begin
                        status <= STATUS_MAX_ITERS;
                        state  <= TOP_CALC_METRICS;

                    // Condition 3: Continue alternating loop
                    end else begin
                        iter_count <= iter_count + 1'b1;
                        state      <= TOP_PHASE1_START;
                    end
                end

                // -------------------------------------------------------------
                // COMPUTE FINAL METRICS
                // -------------------------------------------------------------
                TOP_CALC_METRICS: begin
                    z_optimal <= z_cur;
                    x_primal  <= x_cur;
                    u_dual    <= u_cur;

                    zero_cnt = 3'd0;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params_reg) begin
                            if (get_vec(z_cur, 2'(k)) == Q16_ZERO) begin
                                zero_cnt = zero_cnt + 1'b1;
                            end
                        end
                    end
                    sparsity_count <= zero_cnt;

                    state <= TOP_DONE;
                end

                // -------------------------------------------------------------
                // STATE: TOP_DONE
                // -------------------------------------------------------------
                TOP_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= TOP_IDLE;
                end

                default: state <= TOP_IDLE;
            endcase
        end
    end

endmodule
