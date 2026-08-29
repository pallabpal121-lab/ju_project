// =============================================================================
// File Name   : irls_weight_grad_engine.sv
// Module Name : irls_weight_grad_engine
// Project     : Iteratively Reweighted Least Squares (IRLS) Accelerator (Solver #3)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Evaluates linear predictions η_m = x_mᵀ · w, probabilities p_m = σ(η_m),
//   dynamic weights W_mm = p_m(1 - p_m), weighted Gram matrix A = XᵀWX + λI,
//   and gradient direction vector b = Xᵀ(y - p).
// =============================================================================

`timescale 1ns / 1ps

import irls_types_pkg::*;
`include "irls_helpers.svh"

module irls_weight_grad_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Inputs
    input  logic               start,
    input  logic [2:0]         num_features, // N (1..4)
    input  logic [3:0]         num_samples,  // M (1..8)
    input  vec_t               w_curr,       // Current weight vector w
    input  feat_mat_t          feat_matrix,  // Feature matrix X (8x4)
    input  label_vec_t         label_vector, // Target labels y (8x1)
    input  q16_t               lambda_reg,   // Regularization factor λ

    // Outputs
    output label_vec_t         prob_vector,  // Predicted probabilities p (8x1)
    output q16_t               loss_mse,     // Mean squared classification error
    output mat_t               mat_a,        // Weighted Gram matrix XᵀWX + λI (4x4)
    output vec_t               vec_b,        // Gradient vector Xᵀ(y - p) (4x1)
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        W_IDLE   = 2'd0,
        W_SAMPLE = 2'd1,
        W_ACCUM  = 2'd2,
        W_DONE   = 2'd3
    } w_state_t;

    w_state_t state;

    // Internal sample storage
    q16_t p_table [0:MAX_SAMPLES-1];
    q16_t w_diag  [0:MAX_SAMPLES-1];
    q16_t err_vec [0:MAX_SAMPLES-1];

    logic [2:0] m_idx; // Sample counter 0..7

    // Sigmoid Submodule
    q16_t sig_eta, sig_prob;
    q16_sigmoid u_sigmoid (
        .eta  (sig_eta),
        .prob (sig_prob)
    );

    // Helper fixed-point multiplier
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Evaluation of eta_m = sum_j (X_mj * w_j)
    q16_t cur_eta;
    always @(*) begin
        cur_eta = 32'sd0;
        for (int j = 0; j < MAX_FEATURES; j++) begin
            if (j < num_features[1:0]) begin
                cur_eta = cur_eta + q16_mul(get_feat(feat_matrix, m_idx, 2'(j)), get_vec(w_curr, 2'(j)));
            end
        end
    end

    assign sig_eta = cur_eta;

    // Accumulation temporary variables
    q16_t sum_err_sq;
    q16_t sum_grad;
    q16_t sum_xtwx;
    vec_t out_b;
    mat_t out_a;
    label_vec_t out_p;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= W_IDLE;
            m_idx       <= '0;
            prob_vector <= '0;
            loss_mse    <= Q16_ZERO;
            mat_a       <= '0;
            vec_b       <= '0;
            done        <= 1'b0;
            busy        <= 1'b0;
            for (int m = 0; m < MAX_SAMPLES; m++) begin
                p_table[m] <= Q16_ZERO;
                w_diag[m]  <= Q16_ZERO;
                err_vec[m] <= Q16_ZERO;
            end
        end else begin
            case (state)
                // -------------------------------------------------------------
                // STATE: W_IDLE
                // -------------------------------------------------------------
                W_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        m_idx <= 3'd0;
                        state <= W_SAMPLE;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // SAMPLE EVALUATION LOOP: η_m -> p_m -> W_mm = p(1-p), e_m = y - p
                // -------------------------------------------------------------
                W_SAMPLE: begin
                    p_table[m_idx] <= sig_prob;
                    // Weight W_mm = p_m * (1 - p_m) (clamped to min 0.0005 for stability)
                    w_diag[m_idx]  <= (q16_mul(sig_prob, Q16_ONE - sig_prob) > 32'h0000_0020) ?
                                       q16_mul(sig_prob, Q16_ONE - sig_prob) : 32'h0000_0020;
                    // Residual Error e_m = y_m - p_m
                    err_vec[m_idx] <= get_label(label_vector, m_idx) - sig_prob;

                    if (m_idx + 1'b1 < num_samples[2:0]) begin
                        m_idx <= m_idx + 1'b1;
                    end else begin
                        state <= W_ACCUM;
                    end
                end

                // -------------------------------------------------------------
                // MATRIX & GRADIENT ACCUMULATION: XᵀWX + λI, Xᵀ(y - p), MSE Loss
                // -------------------------------------------------------------
                W_ACCUM: begin
                    // 1. Pack probability vector & Compute MSE loss
                    out_p = '0;
                    sum_err_sq = 32'sd0;
                    for (int m = 0; m < MAX_SAMPLES; m++) begin
                        if (m < num_samples[2:0]) begin
                            out_p = set_label(out_p, 3'(m), p_table[m]);
                            sum_err_sq = sum_err_sq + q16_mul(err_vec[m], err_vec[m]);
                        end
                    end
                    prob_vector <= out_p;
                    loss_mse    <= sum_err_sq; // Sum of squared errors

                    // 2. Gradient vector b_j = sum_{m=0}^{M-1} (X[m][j] * (y[m] - p[m]))
                    out_b = '0;
                    for (int j = 0; j < MAX_FEATURES; j++) begin
                        if (j < num_features[1:0]) begin
                            sum_grad = 32'sd0;
                            for (int m = 0; m < MAX_SAMPLES; m++) begin
                                if (m < num_samples[2:0]) begin
                                    sum_grad = sum_grad + q16_mul(get_feat(feat_matrix, 3'(m), 2'(j)), err_vec[m]);
                                end
                            end
                            out_b = set_vec(out_b, 2'(j), sum_grad);
                        end
                    end
                    vec_b <= out_b;

                    // 3. Weighted Gram Matrix A[j][k] = sum_m (X[m][j] * W[m] * X[m][k]) + (j==k ? λ : 0)
                    out_a = '0;
                    for (int j = 0; j < MAX_FEATURES; j++) begin
                        if (j < num_features[1:0]) begin
                            for (int k = 0; k < MAX_FEATURES; k++) begin
                                if (k < num_features[1:0]) begin
                                    sum_xtwx = 32'sd0;
                                    for (int m = 0; m < MAX_SAMPLES; m++) begin
                                        if (m < num_samples[2:0]) begin
                                            sum_xtwx = sum_xtwx + q16_mul(q16_mul(get_feat(feat_matrix, 3'(m), 2'(j)), w_diag[m]),
                                                                          get_feat(feat_matrix, 3'(m), 2'(k)));
                                        end
                                    end
                                    if (j == k) begin
                                        sum_xtwx = sum_xtwx + lambda_reg; // Ridge Regularization
                                    end
                                    out_a = set_mat(out_a, 2'(j), 2'(k), sum_xtwx);
                                end
                            end
                        end
                    end
                    mat_a <= out_a;

                    state <= W_DONE;
                end

                // -------------------------------------------------------------
                // STATE: W_DONE
                // -------------------------------------------------------------
                W_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= W_IDLE;
                end

                default: state <= W_IDLE;
            endcase
        end
    end

endmodule
