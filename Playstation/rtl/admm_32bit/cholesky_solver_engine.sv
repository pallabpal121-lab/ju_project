// =============================================================================
// File Name   : cholesky_solver_engine.sv
// Module Name : cholesky_solver_engine
// Project     : Alternating Direction Method of Multipliers (ADMM) Accelerator (Solver #11)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Solves the positive definite linear system H * p = b without explicit matrix
//   inversion using Cholesky Decomposition: H = L * L^T.
//   1. Factorization: L_jj = sqrt(H_jj - sum L_jk^2), L_ij = (H_ij - sum L_ik*L_jk)/L_jj
//   2. Forward Substitution : L * y = b
//   3. Back Substitution    : L^T * p = y
// =============================================================================

`timescale 1ns / 1ps

import admm_types_pkg::*;
`include "admm_helpers.svh"

module cholesky_solver_engine (
    input  logic               clk,
    input  logic               rst_n,

    input  logic               start,
    input  logic [2:0]         n_dim,        // Matrix dimension N (1..4)
    input  mat_t               h_mat,        // Input symmetric matrix H (N x N)
    input  vec_t               g_vec,        // Input RHS vector b (N x 1)
    output vec_t               p_vec,        // Solution vector p = H^-1 * b
    output logic               done,
    output logic               error_not_posdef,
    output logic               busy
);

    typedef enum logic [3:0] {
        CHOL_IDLE         = 4'd0,
        CHOL_DIAG_SUM     = 4'd1,
        CHOL_DIAG_SQRT    = 4'd2,
        CHOL_DIAG_WAIT    = 4'd3,
        CHOL_OFF_SUM      = 4'd4,
        CHOL_OFF_DIV      = 4'd5,
        CHOL_OFF_WAIT     = 4'd6,
        CHOL_NEXT_COL     = 4'd7,
        CHOL_FWD_SUM      = 4'd8,
        CHOL_FWD_DIV      = 4'd9,
        CHOL_FWD_WAIT     = 4'd10,
        CHOL_BWD_SUM      = 4'd11,
        CHOL_BWD_DIV      = 4'd12,
        CHOL_BWD_WAIT     = 4'd13,
        CHOL_DONE         = 4'd14
    } chol_state_t;

    chol_state_t state;

    logic [2:0] i_idx, j_idx, k_idx;
    mat_t       l_mat;
    vec_t       y_vec;
    vec_t       p_out_reg;
    q16_t       sum_acc;

    // Multiplier helper
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Divider Interconnect
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

    // Sqrt Interconnect
    logic sqrt_start;
    q16_t sqrt_val, sqrt_root;
    logic sqrt_done, sqrt_busy;

    q16_sqrt u_sqrt (
        .clk  (clk),
        .rst_n(rst_n),
        .start(sqrt_start),
        .val  (sqrt_val),
        .root (sqrt_root),
        .done (sqrt_done),
        .busy (sqrt_busy)
    );

    assign p_vec = p_out_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= CHOL_IDLE;
            i_idx            <= 2'd0;
            j_idx            <= 2'd0;
            k_idx            <= 2'd0;
            l_mat            <= '0;
            y_vec            <= '0;
            p_out_reg        <= '0;
            sum_acc          <= Q16_ZERO;
            div_start        <= 1'b0;
            div_dividend     <= Q16_ZERO;
            div_divisor      <= Q16_ZERO;
            sqrt_start       <= 1'b0;
            sqrt_val         <= Q16_ZERO;
            done             <= 1'b0;
            error_not_posdef <= 1'b0;
            busy             <= 1'b0;
        end else begin
            div_start  <= 1'b0;
            sqrt_start <= 1'b0;

            case (state)
                CHOL_IDLE: begin
                    done             <= 1'b0;
                    error_not_posdef <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        l_mat     <= '0;
                        y_vec     <= '0;
                        p_out_reg <= '0;
                        i_idx     <= 2'd0;
                        j_idx     <= 2'd0;
                        k_idx     <= 2'd0;
                        state     <= CHOL_DIAG_SUM;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // 1. FACTORIZATION: L_ii = sqrt(H_ii - sum_{k=0}^{i-1} L_ik^2)
                // -------------------------------------------------------------
                CHOL_DIAG_SUM: begin
                    sum_acc = 32'sd0;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (3'(k) < i_idx) begin
                            sum_acc = sum_acc + q16_mul(get_mat(l_mat, i_idx[1:0], 2'(k)), get_mat(l_mat, i_idx[1:0], 2'(k)));
                        end
                    end
                    state <= CHOL_DIAG_SQRT;
                end

                CHOL_DIAG_SQRT: begin
                    if (get_mat(h_mat, i_idx[1:0], i_idx[1:0]) - sum_acc <= 32'sd0) begin
                        sqrt_val <= Q16_EPS_DEF;
                    end else begin
                        sqrt_val <= get_mat(h_mat, i_idx[1:0], i_idx[1:0]) - sum_acc;
                    end
                    sqrt_start <= 1'b1;
                    state      <= CHOL_DIAG_WAIT;
                end

                CHOL_DIAG_WAIT: begin
                    if (sqrt_done) begin
                        l_mat <= set_mat(l_mat, i_idx[1:0], i_idx[1:0], (sqrt_root != Q16_ZERO) ? sqrt_root : Q16_EPS_DEF);
                        j_idx <= i_idx + 1'b1;
                        state <= CHOL_OFF_SUM;
                    end
                end

                // -------------------------------------------------------------
                // 2. OFF-DIAGONAL: L_ji = (H_ji - sum L_jk*L_ik) / L_ii
                // -------------------------------------------------------------
                CHOL_OFF_SUM: begin
                    if (j_idx < n_dim) begin
                        sum_acc = 32'sd0;
                        for (int k = 0; k < MAX_PARAMS; k++) begin
                            if (3'(k) < i_idx) begin
                                sum_acc = sum_acc + q16_mul(get_mat(l_mat, j_idx[1:0], 2'(k)), get_mat(l_mat, i_idx[1:0], 2'(k)));
                            end
                        end
                        state <= CHOL_OFF_DIV;
                    end else begin
                        state <= CHOL_NEXT_COL;
                    end
                end

                CHOL_OFF_DIV: begin
                    div_dividend <= get_mat(h_mat, j_idx[1:0], i_idx[1:0]) - sum_acc;
                    div_divisor  <= get_mat(l_mat, i_idx[1:0], i_idx[1:0]);
                    div_start    <= 1'b1;
                    state        <= CHOL_OFF_WAIT;
                end

                CHOL_OFF_WAIT: begin
                    if (div_done) begin
                        l_mat <= set_mat(set_mat(l_mat, j_idx[1:0], i_idx[1:0], div_quotient), i_idx[1:0], j_idx[1:0], Q16_ZERO);
                        j_idx <= j_idx + 1'b1;
                        state <= CHOL_OFF_SUM;
                    end
                end

                CHOL_NEXT_COL: begin
                    if (i_idx + 1'b1 < n_dim) begin
                        i_idx <= i_idx + 1'b1;
                        state <= CHOL_DIAG_SUM;
                    end else begin
                        i_idx <= 3'd0;
                        state <= CHOL_FWD_SUM;
                    end
                end

                // -------------------------------------------------------------
                // 3. FORWARD SUBSTITUTION: L * y = b -> y_i = (b_i - sum L_ik*y_k)/L_ii
                // -------------------------------------------------------------
                CHOL_FWD_SUM: begin
                    sum_acc = 32'sd0;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (3'(k) < i_idx) begin
                            sum_acc = sum_acc + q16_mul(get_mat(l_mat, i_idx[1:0], 2'(k)), get_vec(y_vec, 2'(k)));
                        end
                    end
                    state <= CHOL_FWD_DIV;
                end

                CHOL_FWD_DIV: begin
                    div_dividend <= get_vec(g_vec, i_idx[1:0]) - sum_acc;
                    div_divisor  <= get_mat(l_mat, i_idx[1:0], i_idx[1:0]);
                    div_start    <= 1'b1;
                    state        <= CHOL_FWD_WAIT;
                end

                CHOL_FWD_WAIT: begin
                    if (div_done) begin
                        y_vec <= set_vec(y_vec, i_idx[1:0], div_quotient);
                        if (i_idx + 1'b1 < n_dim) begin
                            i_idx <= i_idx + 1'b1;
                            state <= CHOL_FWD_SUM;
                        end else begin
                            i_idx <= n_dim - 1'b1;
                            state <= CHOL_BWD_SUM;
                        end
                    end
                end

                // -------------------------------------------------------------
                // 4. BACKWARD SUBSTITUTION: L^T * p = y -> p_i = (y_i - sum L_ki*p_k)/L_ii
                // -------------------------------------------------------------
                CHOL_BWD_SUM: begin
                    sum_acc = 32'sd0;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (3'(k) > i_idx && 3'(k) < n_dim) begin
                            sum_acc = sum_acc + q16_mul(get_mat(l_mat, 2'(k), i_idx[1:0]), get_vec(p_out_reg, 2'(k)));
                        end
                    end
                    state <= CHOL_BWD_DIV;
                end

                CHOL_BWD_DIV: begin
                    div_dividend <= get_vec(y_vec, i_idx[1:0]) - sum_acc;
                    div_divisor  <= get_mat(l_mat, i_idx[1:0], i_idx[1:0]);
                    div_start    <= 1'b1;
                    state        <= CHOL_BWD_WAIT;
                end

                CHOL_BWD_WAIT: begin
                    if (div_done) begin
                        p_out_reg <= set_vec(p_out_reg, i_idx[1:0], div_quotient);
                        if (i_idx > 3'd0) begin
                            i_idx <= i_idx - 1'b1;
                            state <= CHOL_BWD_SUM;
                        end else begin
                            state <= CHOL_DONE;
                        end
                    end
                end

                // -------------------------------------------------------------
                // 5. FINISHED
                // -------------------------------------------------------------
                CHOL_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= CHOL_IDLE;
                end

                default: state <= CHOL_IDLE;
            endcase
        end
    end

endmodule
