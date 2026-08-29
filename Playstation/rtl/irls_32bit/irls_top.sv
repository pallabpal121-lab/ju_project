// =============================================================================
// File Name   : irls_top.sv
// Module Name : irls_top
// Project     : Iteratively Reweighted Least Squares (IRLS) Accelerator (Solver #3)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Top-level SoC Module for the Iteratively Reweighted Least Squares (IRLS)
//   Hardware Accelerator. Solves Logistic Regression & Classification problems
//   via regularized Newton updates: (XᵀWX + λI) · Δw = Xᵀ(y - p).
// =============================================================================

`timescale 1ns / 1ps

import irls_types_pkg::*;
`include "irls_helpers.svh"

module irls_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Training Feature Matrix Memory Port
    // -------------------------------------------------------------------------
    input  logic               feat_we,       // Feature Matrix Write Enable
    input  logic [2:0]         feat_s_idx,    // Sample Row Index (0..7)
    input  logic [1:0]         feat_d_idx,    // Feature Column Index (0..3)
    input  q16_t               feat_val,      // Feature Value x_mj

    // -------------------------------------------------------------------------
    // Interface 2: Training Target Label Memory Port
    // -------------------------------------------------------------------------
    input  logic               label_we,      // Label Write Enable
    input  logic [2:0]         label_s_idx,   // Sample Index (0..7)
    input  q16_t               label_val,     // Target Label y_m in {0.0, 1.0}

    // -------------------------------------------------------------------------
    // Interface 3: Optimization Controls & Parameters
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start pulse
    input  logic [2:0]         num_features,  // Number of features N (1..4)
    input  logic [3:0]         num_samples,   // Number of samples M (1..8)
    input  vec_t               w_init,        // Initial weight guess [w3, w2, w1, w0]
    input  q16_t               step_alpha,    // Learning rate / step scale α
    input  q16_t               lambda_reg,    // L2 Ridge Regularization λ
    input  q16_t               tolerance,     // Convergence threshold on gradient infinity norm
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 4: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               w_optimal,     // Converged optimal weight vector w*
    output q16_t               loss_optimal,  // Final classification loss (MSE)
    output label_vec_t         prob_pred,     // Final predicted probabilities [p7..p0]
    output q16_t               g_norm_inf,    // Final max gradient component max_j(|g_j|)
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while solving
);

    // -------------------------------------------------------------------------
    // Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        IRLS_IDLE        = 3'd0,
        IRLS_START_EVAL  = 3'd1,
        IRLS_WAIT_EVAL   = 3'd2,
        IRLS_CHECK_CONV  = 3'd3,
        IRLS_START_SOLVE = 3'd4,
        IRLS_WAIT_SOLVE  = 3'd5,
        IRLS_UPDATE_W    = 3'd6,
        IRLS_DONE        = 3'd7
    } irls_state_t;

    irls_state_t state;

    // Internal Feature Matrix and Label Vector Registers
    feat_mat_t  feat_matrix_reg;
    label_vec_t label_vector_reg;

    // Configuration & State Registers
    vec_t       w_reg;
    logic [2:0] num_features_reg;
    logic [3:0] num_samples_reg;
    q16_t       alpha_reg;
    q16_t       lambda_reg_in;
    q16_t       tol_reg;
    logic [7:0] max_iters_reg;

    // Weight/Gradient Engine Interconnect
    logic       eval_start;
    label_vec_t eval_prob_out;
    q16_t       eval_loss_out;
    mat_t       eval_mat_a;
    vec_t       eval_vec_b;
    logic       eval_done, eval_busy;

    // Cholesky Solver Interconnect
    logic       chol_start;
    vec_t       chol_vec_p;
    logic       chol_done, chol_singular, chol_busy;

    // Helper fixed-point multiplier
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Submodule 1: Weight & Gradient Generator Engine
    irls_weight_grad_engine u_eval_engine (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (eval_start),
        .num_features (num_features_reg),
        .num_samples  (num_samples_reg),
        .w_curr       (w_reg),
        .feat_matrix  (feat_matrix_reg),
        .label_vector (label_vector_reg),
        .lambda_reg   (lambda_reg_in),
        .prob_vector  (eval_prob_out),
        .loss_mse     (eval_loss_out),
        .mat_a        (eval_mat_a),
        .vec_b        (eval_vec_b),
        .done         (eval_done),
        .busy         (eval_busy)
    );

    // Submodule 2: Hardware Cholesky Linear Solver: A · Δw = b
    cholesky_solver_engine u_chol_solver (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (chol_start),
        .num_vars   (num_features_reg),
        .mat_a      (eval_mat_a),
        .vec_b      (eval_vec_b),
        .vec_p      (chol_vec_p),
        .done       (chol_done),
        .singular   (chol_singular),
        .busy       (chol_busy)
    );

    vec_t tmp_next_w;
    q16_t max_grad_calc;
    q16_t cur_b_val;

    // Master IRLS Optimization Loop FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= IRLS_IDLE;
            num_features_reg <= 3'd2;
            num_samples_reg  <= 4'd4;
            alpha_reg        <= Q16_ONE;
            lambda_reg_in    <= Q16_LAMBDA_DEF;
            tol_reg          <= Q16_EPS_DEF;
            max_iters_reg    <= 8'd50;
            iter_count       <= 8'd0;
            status           <= STATUS_IDLE;
            done             <= 1'b0;
            busy             <= 1'b0;
            loss_optimal     <= Q16_ZERO;
            g_norm_inf       <= Q16_ZERO;
            prob_pred        <= '0;
            w_optimal        <= '0;
            w_reg            <= '0;
            eval_start       <= 1'b0;
            chol_start       <= 1'b0;
            feat_matrix_reg  <= '0;
            label_vector_reg <= '0;
        end else begin
            eval_start <= 1'b0;
            chol_start <= 1'b0;

            // Feature and Label Memory Writes
            if (feat_we) begin
                feat_matrix_reg <= set_feat(feat_matrix_reg, feat_s_idx, feat_d_idx, feat_val);
            end
            if (label_we) begin
                label_vector_reg <= set_label(label_vector_reg, label_s_idx, label_val);
            end

            case (state)
                // -------------------------------------------------------------
                // STATE: IRLS_IDLE
                // -------------------------------------------------------------
                IRLS_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy             <= 1'b1;
                        num_features_reg <= (num_features != 3'd0) ? num_features : 3'd2;
                        num_samples_reg  <= (num_samples != 4'd0)  ? num_samples  : 4'd4;
                        alpha_reg        <= (step_alpha != Q16_ZERO) ? step_alpha : Q16_ONE;
                        lambda_reg_in    <= (lambda_reg != Q16_ZERO) ? lambda_reg : Q16_LAMBDA_DEF;
                        tol_reg          <= (tolerance != Q16_ZERO)  ? tolerance  : Q16_EPS_DEF;
                        max_iters_reg    <= (max_iters != 8'd0)      ? max_iters  : 8'd50;
                        iter_count       <= 8'd0;
                        status           <= STATUS_RUNNING;
                        w_reg            <= w_init;
                        state            <= IRLS_START_EVAL;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE CURRENT LOSS, PROBABILITIES, HESSIAN AND GRADIENT
                // -------------------------------------------------------------
                IRLS_START_EVAL: begin
                    eval_start <= 1'b1;
                    state      <= IRLS_WAIT_EVAL;
                end

                IRLS_WAIT_EVAL: begin
                    if (eval_done) begin
                        state <= IRLS_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // CHECK CONVERGENCE (||g||_inf <= tolerance or max iterations)
                // -------------------------------------------------------------
                IRLS_CHECK_CONV: begin
                    max_grad_calc = 32'sd0;
                    for (int j = 0; j < MAX_FEATURES; j++) begin
                        if (j < num_features_reg) begin
                            cur_b_val = get_vec(eval_vec_b, 2'(j));
                            if (cur_b_val < 32'sd0) cur_b_val = -cur_b_val;
                            if (cur_b_val > max_grad_calc) max_grad_calc = cur_b_val;
                        end
                    end

                    // Condition 1: Gradient or loss converged
                    if (max_grad_calc <= tol_reg || eval_loss_out <= tol_reg) begin
                        status       <= STATUS_CONVERGED;
                        w_optimal    <= w_reg;
                        loss_optimal <= eval_loss_out;
                        prob_pred    <= eval_prob_out;
                        g_norm_inf   <= max_grad_calc;
                        state        <= IRLS_DONE;

                    // Condition 2: Max iterations reached
                    end else if (iter_count >= max_iters_reg) begin
                        status       <= STATUS_MAX_ITERS;
                        w_optimal    <= w_reg;
                        loss_optimal <= eval_loss_out;
                        prob_pred    <= eval_prob_out;
                        g_norm_inf   <= max_grad_calc;
                        state        <= IRLS_DONE;

                    // Condition 3: Solve (XᵀWX + λI) Δw = Xᵀ(y - p) via Cholesky
                    end else begin
                        chol_start <= 1'b1;
                        state      <= IRLS_WAIT_SOLVE;
                    end
                end

                // -------------------------------------------------------------
                // WAIT FOR CHOLESKY LINEAR SOLVER
                // -------------------------------------------------------------
                IRLS_WAIT_SOLVE: begin
                    if (chol_done) begin
                        if (chol_singular) begin
                            status       <= STATUS_SINGULAR;
                            w_optimal    <= w_reg;
                            loss_optimal <= eval_loss_out;
                            prob_pred    <= eval_prob_out;
                            g_norm_inf   <= max_grad_calc;
                            state        <= IRLS_DONE;
                        end else begin
                            state <= IRLS_UPDATE_W;
                        end
                    end
                end

                // -------------------------------------------------------------
                // UPDATE WEIGHT VECTOR: w = w + α · Δw
                // -------------------------------------------------------------
                IRLS_UPDATE_W: begin
                    tmp_next_w = w_reg;
                    for (int j = 0; j < MAX_FEATURES; j++) begin
                        if (j < num_features_reg) begin
                            tmp_next_w = set_vec(tmp_next_w, 2'(j), get_vec(w_reg, 2'(j)) + q16_mul(alpha_reg, get_vec(chol_vec_p, 2'(j))));
                        end
                    end
                    w_reg      <= tmp_next_w;
                    iter_count <= iter_count + 1'b1;
                    state      <= IRLS_START_EVAL;
                end

                // -------------------------------------------------------------
                // STATE: IRLS_DONE
                // -------------------------------------------------------------
                IRLS_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= IRLS_IDLE;
                end

                default: state <= IRLS_IDLE;
            endcase
        end
    end

endmodule
