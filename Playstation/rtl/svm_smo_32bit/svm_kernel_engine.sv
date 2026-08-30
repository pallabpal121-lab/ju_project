// =============================================================================
// File Name   : svm_kernel_engine.sv
// Module Name : svm_kernel_engine
// Project     : Support Vector Machine Sequential Minimal Optimization (SVM-SMO)
//               Accelerator (Solver #27)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Kernel Matrix Generator for SVM.
//   Evaluates symmetric Linear Kernel Gram Matrix:
//     K_ij = x_i^T * x_j = ∑ (x_id * x_jd)
// =============================================================================

`timescale 1ns / 1ps

import svm_types_pkg::*;
`include "svm_helpers.svh"

module svm_kernel_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  logic [3:0]         num_samples,      // Total samples M (1..8)
    input  logic [2:0]         feat_dim,         // Feature dimension D (1..4)
    input  dataset_arr_t       dataset,          // Linearized training dataset [32]

    output kernel_mat_t        k_matrix,         // 8x8 Kernel Gram Matrix [64]
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        KERN_IDLE = 2'd0,
        KERN_CALC = 2'd1,
        KERN_DONE = 2'd2
    } kern_state_t;

    kern_state_t state;

    kernel_mat_t k_reg;
    logic [2:0]  row_i;
    logic [2:0]  col_j;

    assign k_matrix = k_reg;

    q16_t dot_prod, x_i_val, x_j_val, prod_val;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= KERN_IDLE;
            k_reg <= '0;
            row_i <= 3'd0;
            col_j <= 3'd0;
            done  <= 1'b0;
            busy  <= 1'b0;
        end else begin
            case (state)
                KERN_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        k_reg <= '0;
                        row_i <= 3'd0;
                        col_j <= 3'd0;
                        state <= KERN_CALC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                KERN_CALC: begin
                    // Compute dot product for pair (row_i, col_j)
                    dot_prod = 32'sd0;
                    for (int d = 0; d < MAX_FEATURES; d++) begin
                        if (d < feat_dim) begin
                            x_i_val  = get_sample_feat(dataset, row_i, 2'(d));
                            x_j_val  = get_sample_feat(dataset, col_j, 2'(d));
                            prod_val = q16_mul(x_i_val, x_j_val);
                            dot_prod = dot_prod + prod_val;
                        end
                    end

                    // Symmetrically write K[row_i][col_j] and K[col_j][row_i]
                    k_reg <= set_kmat(k_reg, row_i, col_j, dot_prod);
                    k_reg <= set_kmat(k_reg, col_j, row_i, dot_prod);

                    // Increment indices
                    if (col_j + 1'b1 < num_samples) begin
                        col_j <= col_j + 1'b1;
                    end else if (row_i + 1'b1 < num_samples) begin
                        row_i <= row_i + 1'b1;
                        col_j <= row_i + 1'b1;
                    end else begin
                        state <= KERN_DONE;
                    end
                end

                KERN_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= KERN_IDLE;
                end

                default: state <= KERN_IDLE;
            endcase
        end
    end

endmodule
