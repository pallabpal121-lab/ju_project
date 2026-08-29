// =============================================================================
// File Name   : cholesky_ipm_solver.sv
// Module Name : cholesky_ipm_solver
// Project     : Primal-Dual Interior Point Method (IPM) Accelerator (Solver #16)
// -----------------------------------------------------------------------------
// Description:
//   Solves the symmetric positive-definite augmented KKT system:
//   H_aug * Δx = -g_aug
//   via direct Cholesky Factorization (H_aug = L * L^T), Forward Triangular
//   Substitution (L * y = -g_aug), and Backward Substitution (L^T * Δx = y).
// =============================================================================

`timescale 1ns / 1ps

import ipm_types_pkg::*;
`include "ipm_helpers.svh"

module cholesky_ipm_solver (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Inputs
    input  logic               start,
    input  logic [2:0]         num_dims,      // Number of primal variables N (1..4)
    input  mat_t               h_mat,         // Augmented Hessian H_aug (4x4)
    input  vec_t               g_vec,         // Augmented Gradient / RHS g_aug (4x1)

    // Outputs
    output vec_t               dx_out,        // Computed step Δx
    output logic               done,
    output logic               busy,
    output logic               error
);

    typedef enum logic [3:0] {
        CH_IDLE        = 4'd0,
        CH_INIT        = 4'd1,
        CH_DIAG_START  = 4'd2,
        CH_DIAG_WAIT   = 4'd3,
        CH_OFF_START   = 4'd4,
        CH_OFF_WAIT    = 4'd5,
        CH_FWD_START   = 4'd6,
        CH_FWD_WAIT    = 4'd7,
        CH_BWD_START   = 4'd8,
        CH_BWD_WAIT    = 4'd9,
        CH_DONE        = 4'd10
    } ch_state_t;

    ch_state_t state;

    mat_t l_mat;
    vec_t y_vec;
    vec_t dx_reg;

    logic [2:0] i_idx, j_idx, k_idx;
    q16_t acc_sum;

    // Hardware Square Root Unit
    logic sqrt_start;
    q16_t sqrt_val, sqrt_root;
    logic sqrt_done, sqrt_busy;

    q16_sqrt u_sqrt (
        .clk   (clk),
        .rst_n (rst_n),
        .start (sqrt_start),
        .val   (sqrt_val),
        .root  (sqrt_root),
        .done  (sqrt_done),
        .busy  (sqrt_busy)
    );

    // Hardware Divider Unit
    logic div_start;
    q16_t div_dividend, div_divisor, div_quotient;
    logic div_done, div_by_zero, div_busy;

    q16_divider u_div (
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

    assign dx_out = dx_reg;

    q16_t term_k, h_elem, l_kk, l_ik, l_jk;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= CH_IDLE;
            l_mat        <= '0;
            y_vec        <= '0;
            dx_reg       <= '0;
            i_idx        <= 3'd0;
            j_idx        <= 3'd0;
            k_idx        <= 3'd0;
            acc_sum      <= Q16_ZERO;
            sqrt_start   <= 1'b0;
            sqrt_val     <= Q16_ZERO;
            div_start    <= 1'b0;
            div_dividend <= Q16_ZERO;
            div_divisor  <= Q16_ZERO;
            done         <= 1'b0;
            busy         <= 1'b0;
            error        <= 1'b0;
        end else begin
            sqrt_start <= 1'b0;
            div_start  <= 1'b0;

            case (state)
                CH_IDLE: begin
                    done  <= 1'b0;
                    error <= 1'b0;
                    if (start) begin
                        busy    <= 1'b1;
                        l_mat   <= '0;
                        y_vec   <= '0;
                        dx_reg  <= '0;
                        j_idx   <= 3'd0;
                        state   <= CH_INIT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                CH_INIT: begin
                    // 1. Diagonal element L_jj = sqrt(H_jj - sum_{k=0}^{j-1} L_jk^2)
                    acc_sum = Q16_ZERO;
                    for (int k = 0; k < MAX_VARS; k++) begin
                        if (k < j_idx) begin
                            l_jk    = get_mat(l_mat, 2'(j_idx), 2'(k));
                            acc_sum = acc_sum + q16_mul(l_jk, l_jk);
                        end
                    end
                    h_elem = get_mat(h_mat, 2'(j_idx), 2'(j_idx));

                    // Add small regularizing epsilon to guarantee positive definiteness
                    sqrt_val   <= (h_elem - acc_sum > Q16_REG_EPS) ? (h_elem - acc_sum) : Q16_REG_EPS;
                    sqrt_start <= 1'b1;
                    state      <= CH_DIAG_WAIT;
                end

                CH_DIAG_WAIT: begin
                    if (sqrt_done) begin
                        l_mat <= set_mat(l_mat, 2'(j_idx), 2'(j_idx), (sqrt_root > 32'h0000_0001) ? sqrt_root : 32'h0000_0001);
                        i_idx <= j_idx + 1'b1;
                        state <= CH_OFF_START;
                    end
                end

                CH_OFF_START: begin
                    if (i_idx < num_dims) begin
                        acc_sum = Q16_ZERO;
                        for (int k = 0; k < MAX_VARS; k++) begin
                            if (k < j_idx) begin
                                l_ik    = get_mat(l_mat, 2'(i_idx), 2'(k));
                                l_jk    = get_mat(l_mat, 2'(j_idx), 2'(k));
                                acc_sum = acc_sum + q16_mul(l_ik, l_jk);
                            end
                        end
                        h_elem       = get_mat(h_mat, 2'(i_idx), 2'(j_idx));
                        div_dividend <= h_elem - acc_sum;
                        div_divisor  <= get_mat(l_mat, 2'(j_idx), 2'(j_idx));
                        div_start    <= 1'b1;
                        state        <= CH_OFF_WAIT;
                    end else begin
                        if (j_idx + 1'b1 < num_dims) begin
                            j_idx <= j_idx + 1'b1;
                            state <= CH_INIT;
                        end else begin
                            // Factorization complete, start forward substitution L * y = -g_aug
                            i_idx <= 3'd0;
                            state <= CH_FWD_START;
                        end
                    end
                end

                CH_OFF_WAIT: begin
                    if (div_done) begin
                        l_mat <= set_mat(l_mat, 2'(i_idx), 2'(j_idx), div_quotient);
                        i_idx <= i_idx + 1'b1;
                        state <= CH_OFF_START;
                    end
                end

                // Forward substitution: L * y = -g_aug
                // y_i = (-g_i - sum_{k=0}^{i-1} L_ik * y_k) / L_ii
                CH_FWD_START: begin
                    if (i_idx < num_dims) begin
                        acc_sum = Q16_ZERO;
                        for (int k = 0; k < MAX_VARS; k++) begin
                            if (k < i_idx) begin
                                l_ik    = get_mat(l_mat, 2'(i_idx), 2'(k));
                                acc_sum = acc_sum + q16_mul(l_ik, get_vec(y_vec, 2'(k)));
                            end
                        end
                        div_dividend <= -get_vec(g_vec, 2'(i_idx)) - acc_sum;
                        div_divisor  <= get_mat(l_mat, 2'(i_idx), 2'(i_idx));
                        div_start    <= 1'b1;
                        state        <= CH_FWD_WAIT;
                    end else begin
                        // Forward solve complete, start backward substitution L^T * Δx = y
                        i_idx <= num_dims - 1'b1;
                        state <= CH_BWD_START;
                    end
                end

                CH_FWD_WAIT: begin
                    if (div_done) begin
                        y_vec <= set_vec(y_vec, 2'(i_idx), div_quotient);
                        i_idx <= i_idx + 1'b1;
                        state <= CH_FWD_START;
                    end
                end

                // Backward substitution: L^T * Δx = y
                // Δx_i = (y_i - sum_{k=i+1}^{N-1} L_ki * Δx_k) / L_ii
                CH_BWD_START: begin
                    acc_sum = Q16_ZERO;
                    for (int k = 0; k < MAX_VARS; k++) begin
                        if (k > i_idx && k < num_dims) begin
                            l_ik    = get_mat(l_mat, 2'(k), 2'(i_idx));
                            acc_sum = acc_sum + q16_mul(l_ik, get_vec(dx_reg, 2'(k)));
                        end
                    end
                    div_dividend <= get_vec(y_vec, 2'(i_idx)) - acc_sum;
                    div_divisor  <= get_mat(l_mat, 2'(i_idx), 2'(i_idx));
                    div_start    <= 1'b1;
                    state        <= CH_BWD_WAIT;
                end

                CH_BWD_WAIT: begin
                    if (div_done) begin
                        dx_reg <= set_vec(dx_reg, 2'(i_idx), div_quotient);
                        if (i_idx > 3'd0) begin
                            i_idx <= i_idx - 1'b1;
                            state <= CH_BWD_START;
                        end else begin
                            state <= CH_DONE;
                        end
                    end
                end

                CH_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= CH_IDLE;
                end

                default: state <= CH_IDLE;
            endcase
        end
    end

endmodule
