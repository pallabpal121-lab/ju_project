// =============================================================================
// File Name   : svm_top.sv
// Module Name : svm_top
// Project     : Support Vector Machine Sequential Minimal Optimization (SVM-SMO)
//               Accelerator (Solver #27)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Controller for Support Vector Machine (SVM) training & inference.
//   Supports complete edge machine learning workflow:
//   1. Initialization & Kernel Generation: svm_kernel_engine evaluates K_ij
//   2. Training (SMO Optimization):        svm_pair_solver updates (α_i, α_j) pairs
//                                          until KKT convergence
//   3. Real-Time Inference:                Classifies new test vectors x_test:
//                                          y_pred = sign(∑ α_j y_j (x_j^T x_test) + b)
// =============================================================================

`timescale 1ns / 1ps

import svm_types_pkg::*;
`include "svm_helpers.svh"

module svm_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Initialization & Training Configuration
    // -------------------------------------------------------------------------
    input  logic               init_svm,         // 1-cycle initialization strobe
    input  logic [3:0]         num_samples,      // Total samples M (1..8)
    input  logic [2:0]         feat_dim,         // Feature dimension D (1..4)
    input  dataset_arr_t       dataset,          // Training samples dataset [32]
    input  label_vec_t         labels,           // Training labels y_i in {-1.0, +1.0}
    input  q16_t               c_bound,          // Box bound C (Q16.16)
    input  q16_t               tol_kkt,          // KKT violation tolerance ε
    input  logic [7:0]         max_passes,       // Max SMO passes without changes

    // -------------------------------------------------------------------------
    // Interface 2: Training Execution & Support Vectors Output
    // -------------------------------------------------------------------------
    input  logic               train_valid,      // Trigger SMO training
    output alpha_vec_t         alphas_out,       // Optimal Lagrange multipliers α*
    output q16_t               b_bias_out,       // Optimal threshold bias b*
    output logic [7:0]         total_passes,     // Passes completed
    output status_t            status,           // Status code
    output logic               train_done,       // Training complete strobe

    // -------------------------------------------------------------------------
    // Interface 3: Real-Time Online Inference (Classification)
    // -------------------------------------------------------------------------
    input  logic               infer_valid,      // Trigger inference on new sample
    input  sample_vec_t        x_test,           // New test feature vector
    output q16_t               decision_value,   // Raw margin f(x)
    output q16_t               y_predicted,      // Predicted class label (-1.0 or +1.0)
    output logic               infer_done,       // Inference complete strobe
    output logic               busy              // High while engine is active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        SVM_IDLE          = 4'd0,
        SVM_START_KERN    = 4'd1,
        SVM_WAIT_KERN     = 4'd2,
        SVM_TRAIN_INIT    = 4'd3,
        SVM_CHECK_I       = 4'd4,
        SVM_PAIR_START    = 4'd5,
        SVM_PAIR_WAIT     = 4'd6,
        SVM_PASS_CHECK    = 4'd7,
        SVM_TRAIN_DONE    = 4'd8,
        SVM_INFER_CALC    = 4'd9,
        SVM_INFER_DONE    = 4'd10
    } svm_state_t;

    svm_state_t state;

    // Internal Configuration Registers
    logic [3:0]   m_samples_reg;
    logic [2:0]   d_feat_reg;
    dataset_arr_t ds_reg;
    label_vec_t   y_reg;
    q16_t         c_reg;
    q16_t         tol_reg;
    logic [7:0]   max_p_reg;
    status_t      status_reg;

    // Optimization Registers
    alpha_vec_t  alpha_reg;
    q16_t        b_reg;
    kernel_mat_t k_matrix_latched;
    logic [7:0]  passes_cnt;
    logic [7:0]  total_iter_cnt;
    logic [3:0]  num_changed;
    logic [2:0]  idx_i;
    logic [2:0]  idx_j;
    q16_t        e_i_val;
    q16_t        e_j_val;
    q16_t        r_i_kkt;

    // Sub-engine 1: svm_kernel_engine
    logic        start_kern;
    kernel_mat_t kern_out;
    logic        kern_done;
    logic        kern_busy;

    svm_kernel_engine u_kern (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start_kern),
        .num_samples(m_samples_reg),
        .feat_dim   (d_feat_reg),
        .dataset    (ds_reg),
        .k_matrix   (kern_out),
        .done       (kern_done),
        .busy       (kern_busy)
    );

    // Sub-engine 2: svm_pair_solver
    logic start_pair;
    q16_t pair_a1_in, pair_a2_in;
    q16_t pair_y1_in, pair_y2_in;
    q16_t pair_e1_in, pair_e2_in;
    q16_t pair_k11_in, pair_k12_in, pair_k22_in;
    q16_t pair_a1_out, pair_a2_out, pair_b_out;
    logic pair_changed;
    logic pair_done;
    logic pair_busy;

    svm_pair_solver u_pair (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (start_pair),
        .a1_old   (pair_a1_in),
        .a2_old   (pair_a2_in),
        .y1       (pair_y1_in),
        .y2       (pair_y2_in),
        .e1       (pair_e1_in),
        .e2       (pair_e2_in),
        .k11      (pair_k11_in),
        .k12      (pair_k12_in),
        .k22      (pair_k22_in),
        .c_bound  (c_reg),
        .b_current(b_reg),
        .a1_new   (pair_a1_out),
        .a2_new   (pair_a2_out),
        .b_new    (pair_b_out),
        .changed  (pair_changed),
        .done     (pair_done),
        .busy     (pair_busy)
    );

    // Outputs
    q16_t dec_reg;
    q16_t y_pred_reg;

    assign alphas_out     = alpha_reg;
    assign b_bias_out     = b_reg;
    assign total_passes   = passes_cnt;
    assign status         = status_reg;
    assign decision_value = dec_reg;
    assign y_predicted    = y_pred_reg;

    // Inference computation variables
    q16_t inf_dot, inf_ay, inf_prod, inf_sum;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= SVM_IDLE;
            m_samples_reg     <= 4'd6;
            d_feat_reg        <= 3'd2;
            ds_reg            <= '0;
            y_reg             <= '0;
            c_reg             <= 32'h0001_0000;
            tol_reg           <= 32'h0000_0100;
            max_p_reg         <= 8'd10;
            status_reg        <= STATUS_IDLE;
            alpha_reg         <= '0;
            b_reg             <= 32'sd0;
            k_matrix_latched  <= '0;
            passes_cnt        <= 8'd0;
            total_iter_cnt    <= 8'd0;
            num_changed       <= 4'd0;
            idx_i             <= 3'd0;
            idx_j             <= 3'd0;
            e_i_val           <= 32'sd0;
            e_j_val           <= 32'sd0;
            r_i_kkt           <= 32'sd0;
            start_kern        <= 1'b0;
            start_pair        <= 1'b0;
            train_done        <= 1'b0;
            infer_done        <= 1'b0;
            dec_reg           <= 32'sd0;
            y_pred_reg        <= 32'h0001_0000;
            busy              <= 1'b0;
        end else begin
            start_kern <= 1'b0;
            start_pair <= 1'b0;
            train_done <= 1'b0;
            infer_done <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: SVM_IDLE
                // -------------------------------------------------------------
                SVM_IDLE: begin
                    if (init_svm) begin
                        m_samples_reg    <= num_samples;
                        d_feat_reg       <= feat_dim;
                        ds_reg           <= dataset;
                        y_reg            <= labels;
                        c_reg            <= (c_bound != 32'sd0) ? c_bound : 32'h0001_0000;
                        tol_reg          <= (tol_kkt != 32'sd0) ? tol_kkt : 32'h0000_0100;
                        max_p_reg        <= (max_passes != 8'd0) ? max_passes : 8'd10;
                        alpha_reg        <= '0;
                        b_reg            <= 32'sd0;
                        status_reg       <= STATUS_IDLE;
                        busy             <= 1'b1;
                        start_kern       <= 1'b1;
                        state            <= SVM_WAIT_KERN;
                    end else if (train_valid) begin
                        busy           <= 1'b1;
                        passes_cnt     <= 8'd0;
                        total_iter_cnt <= 8'd0;
                        status_reg     <= STATUS_OPTIMIZING;
                        state          <= SVM_TRAIN_INIT;
                    end else if (infer_valid) begin
                        busy  <= 1'b1;
                        state <= SVM_INFER_CALC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: SVM_WAIT_KERN - Wait for Kernel Matrix Generation
                // -------------------------------------------------------------
                SVM_WAIT_KERN: begin
                    if (kern_done) begin
                        k_matrix_latched <= kern_out;
                        status_reg       <= STATUS_KERNEL_READY;
                        busy             <= 1'b0;
                        state            <= SVM_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // STATE 2: SVM_TRAIN_INIT - Start a new SMO pass
                // -------------------------------------------------------------
                SVM_TRAIN_INIT: begin
                    num_changed <= 4'd0;
                    idx_i       <= 3'd0;
                    state       <= SVM_CHECK_I;
                end

                // -------------------------------------------------------------
                // STATE 3: SVM_CHECK_I - Check KKT conditions on sample i
                // -------------------------------------------------------------
                SVM_CHECK_I: begin
                    e_i_val = eval_f_xi(k_matrix_latched, alpha_reg, y_reg, b_reg, idx_i, m_samples_reg) - y_reg[idx_i];
                    r_i_kkt = q16_mul(y_reg[idx_i], e_i_val);

                    // Check KKT violation: (r_i < -tol && alpha < C) || (r_i > tol && alpha > 0)
                    if ((r_i_kkt < -tol_reg && alpha_reg[idx_i] < c_reg) ||
                        (r_i_kkt >  tol_reg && alpha_reg[idx_i] > Q16_ZERO)) begin
                        // Select initial partner j != i
                        idx_j <= (idx_i + 1'b1) % m_samples_reg;
                        state <= SVM_PAIR_START;
                    end else begin
                        // Move to next i
                        if (idx_i + 1'b1 < m_samples_reg) begin
                            idx_i <= idx_i + 1'b1;
                            state <= SVM_CHECK_I;
                        end else begin
                            state <= SVM_PASS_CHECK;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE 3B: SVM_PAIR_START - Prepare inputs for partner j
                // -------------------------------------------------------------
                SVM_PAIR_START: begin
                    if (idx_j == idx_i) begin
                        idx_j <= (idx_j + 1'b1) % m_samples_reg;
                    end else begin
                        e_j_val = eval_f_xi(k_matrix_latched, alpha_reg, y_reg, b_reg, idx_j, m_samples_reg) - y_reg[idx_j];

                        pair_a1_in  <= alpha_reg[idx_i];
                        pair_a2_in  <= alpha_reg[idx_j];
                        pair_y1_in  <= y_reg[idx_i];
                        pair_y2_in  <= y_reg[idx_j];
                        pair_e1_in  <= e_i_val;
                        pair_e2_in  <= e_j_val;
                        pair_k11_in <= get_kmat(k_matrix_latched, idx_i, idx_i);
                        pair_k12_in <= get_kmat(k_matrix_latched, idx_i, idx_j);
                        pair_k22_in <= get_kmat(k_matrix_latched, idx_j, idx_j);

                        start_pair <= 1'b1;
                        state      <= SVM_PAIR_WAIT;
                    end
                end

                // -------------------------------------------------------------
                // STATE 4: SVM_PAIR_WAIT - Wait for 2-variable SMO pair solver
                // -------------------------------------------------------------
                SVM_PAIR_WAIT: begin
                    if (pair_done) begin
                        if (pair_changed) begin
                            alpha_reg[idx_i] <= pair_a1_out;
                            alpha_reg[idx_j] <= pair_a2_out;
                            b_reg            <= pair_b_out;
                            num_changed      <= num_changed + 1'b1;

                            // Advance to next i after successful step
                            if (idx_i + 1'b1 < m_samples_reg) begin
                                idx_i <= idx_i + 1'b1;
                                state <= SVM_CHECK_I;
                            end else begin
                                state <= SVM_PASS_CHECK;
                            end
                        end else begin
                            // Try next partner j
                            if ((idx_j + 1'b1) % m_samples_reg != idx_i &&
                                (idx_j + 1'b1) % m_samples_reg != (idx_i + 1'b1) % m_samples_reg) begin
                                idx_j <= (idx_j + 1'b1) % m_samples_reg;
                                state <= SVM_PAIR_START;
                            end else begin
                                // No partner could update with i, move to next i
                                if (idx_i + 1'b1 < m_samples_reg) begin
                                    idx_i <= idx_i + 1'b1;
                                    state <= SVM_CHECK_I;
                                end else begin
                                    state <= SVM_PASS_CHECK;
                                end
                            end
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE 5: SVM_PASS_CHECK - Evaluate pass completion & convergence
                // -------------------------------------------------------------
                SVM_PASS_CHECK: begin
                    total_iter_cnt <= total_iter_cnt + 1'b1;

                    if (num_changed == 4'd0) begin
                        passes_cnt <= passes_cnt + 1'b1;
                        if (passes_cnt + 1'b1 >= max_p_reg) begin
                            status_reg <= STATUS_CONVERGED;
                            state      <= SVM_TRAIN_DONE;
                        end else if (total_iter_cnt >= 8'd60) begin
                            status_reg <= STATUS_MAX_ITERS;
                            state      <= SVM_TRAIN_DONE;
                        end else begin
                            state <= SVM_TRAIN_INIT;
                        end
                    end else begin
                        passes_cnt <= 8'd0;
                        if (total_iter_cnt >= 8'd60) begin
                            status_reg <= STATUS_MAX_ITERS;
                            state      <= SVM_TRAIN_DONE;
                        end else begin
                            state <= SVM_TRAIN_INIT;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE 6: SVM_TRAIN_DONE
                // -------------------------------------------------------------
                SVM_TRAIN_DONE: begin
                    train_done <= 1'b1;
                    busy       <= 1'b0;
                    state      <= SVM_IDLE;
                end

                // -------------------------------------------------------------
                // STATE 7: SVM_INFER_CALC - Evaluate f(x_test) and sign
                // -------------------------------------------------------------
                SVM_INFER_CALC: begin
                    inf_sum = b_reg;
                    for (int j = 0; j < MAX_SAMPLES; j++) begin
                        if (j < m_samples_reg) begin
                            if (alpha_reg[j] > 32'h0000_0004) begin
                                // Kernel dot product x_j^T * x_test
                                inf_dot = 32'sd0;
                                for (int d = 0; d < MAX_FEATURES; d++) begin
                                    if (d < d_feat_reg) begin
                                        inf_dot = inf_dot + q16_mul(get_sample_feat(ds_reg, 3'(j), 2'(d)), x_test[d]);
                                    end
                                end
                                inf_ay  = q16_mul(alpha_reg[j], y_reg[j]);
                                inf_sum = inf_sum + q16_mul(inf_ay, inf_dot);
                            end
                        end
                    end

                    dec_reg    <= inf_sum;
                    y_pred_reg <= (inf_sum >= 32'sd0) ? 32'h0001_0000 : 32'hFFFF_0000;
                    state      <= SVM_INFER_DONE;
                end

                SVM_INFER_DONE: begin
                    infer_done <= 1'b1;
                    busy       <= 1'b0;
                    state      <= SVM_IDLE;
                end

                default: state <= SVM_IDLE;
            endcase
        end
    end

endmodule
