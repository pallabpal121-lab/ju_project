// =============================================================================
// File Name   : cholesky_solver_engine.sv
// Module Name : cholesky_solver_engine
// Project     : Sequential Quadratic Programming (SQP) Accelerator (Solver #7)
// -----------------------------------------------------------------------------
// Description: Solves A · p = -g where A is the regularized projected Hessian
//              using hardware Cholesky Factorization (A = L · Lᵀ).
// =============================================================================

`timescale 1ns / 1ps

import sqp_types_pkg::*;
`include "sqp_helpers.svh"

module cholesky_solver_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,
    input  logic [2:0]         num_vars,
    input  mat_t               mat_a,
    input  vec_t               vec_g,
    output vec_t               vec_p,
    output logic               done,
    output logic               singular,
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

    mat_t L;
    vec_t y;
    vec_t p_reg;

    logic [1:0] i_idx, j_idx, k_idx;
    logic signed [31:0] sum_acc;

    logic        sqrt_start;
    q16_t        sqrt_val, sqrt_root;
    logic        sqrt_done, sqrt_busy;

    logic        div_start;
    q16_t        div_dividend, div_divisor, div_quotient;
    logic        div_done, div_by_zero, div_busy;

    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    q16_sqrt u_sqrt (
        .clk   (clk),
        .rst_n (rst_n),
        .start (sqrt_start),
        .val   (sqrt_val),
        .root  (sqrt_root),
        .done  (sqrt_done),
        .busy  (sqrt_busy)
    );

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

    assign vec_p = p_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= CHOL_IDLE;
            done         <= 1'b0;
            singular     <= 1'b0;
            busy         <= 1'b0;
            i_idx        <= '0;
            j_idx        <= '0;
            k_idx        <= '0;
            sum_acc      <= '0;
            sqrt_start   <= 1'b0;
            sqrt_val     <= '0;
            div_start    <= 1'b0;
            div_dividend <= '0;
            div_divisor  <= '0;
            L            <= '0;
            y            <= '0;
            p_reg        <= '0;
        end else begin
            sqrt_start <= 1'b0;
            div_start  <= 1'b0;

            case (state)
                CHOL_IDLE: begin
                    done     <= 1'b0;
                    singular <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        i_idx <= 2'd0;
                        j_idx <= 2'd0;
                        k_idx <= 2'd0;
                        state <= CHOL_DIAG_SUM;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                CHOL_DIAG_SUM: begin
                    sum_acc = 32'sd0;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (2'(k) < i_idx) begin
                            sum_acc = sum_acc + q16_mul(get_mat(L, i_idx, 2'(k)), get_mat(L, i_idx, 2'(k)));
                        end
                    end
                    state <= CHOL_DIAG_SQRT;
                end

                CHOL_DIAG_SQRT: begin
                    if (get_mat(mat_a, i_idx, i_idx) - sum_acc <= 32'sd0) begin
                        sqrt_val <= Q16_EPS_DEF;
                    end else begin
                        sqrt_val <= get_mat(mat_a, i_idx, i_idx) - sum_acc;
                    end
                    sqrt_start <= 1'b1;
                    state      <= CHOL_DIAG_WAIT;
                end

                CHOL_DIAG_WAIT: begin
                    if (sqrt_done) begin
                        L     <= set_mat(L, i_idx, i_idx, (sqrt_root != Q16_ZERO) ? sqrt_root : Q16_EPS_DEF);
                        j_idx <= i_idx + 1'b1;
                        state <= CHOL_OFF_SUM;
                    end
                end

                CHOL_OFF_SUM: begin
                    if (j_idx < num_vars[1:0]) begin
                        sum_acc = 32'sd0;
                        for (int k = 0; k < MAX_PARAMS; k++) begin
                            if (2'(k) < i_idx) begin
                                sum_acc = sum_acc + q16_mul(get_mat(L, j_idx, 2'(k)), get_mat(L, i_idx, 2'(k)));
                            end
                        end
                        state <= CHOL_OFF_DIV;
                    end else begin
                        state <= CHOL_NEXT_COL;
                    end
                end

                CHOL_OFF_DIV: begin
                    div_dividend <= get_mat(mat_a, j_idx, i_idx) - sum_acc;
                    div_divisor  <= get_mat(L, i_idx, i_idx);
                    div_start    <= 1'b1;
                    state        <= CHOL_OFF_WAIT;
                end

                CHOL_OFF_WAIT: begin
                    if (div_done) begin
                        L     <= set_mat(set_mat(L, j_idx, i_idx, div_quotient), i_idx, j_idx, Q16_ZERO);
                        j_idx <= j_idx + 1'b1;
                        state <= CHOL_OFF_SUM;
                    end
                end

                CHOL_NEXT_COL: begin
                    if (i_idx + 1'b1 < num_vars[1:0]) begin
                        i_idx <= i_idx + 1'b1;
                        state <= CHOL_DIAG_SUM;
                    end else begin
                        i_idx <= 2'd0;
                        state <= CHOL_FWD_SUM;
                    end
                end

                // Solves L · y = -g
                CHOL_FWD_SUM: begin
                    sum_acc = 32'sd0;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (2'(k) < i_idx) begin
                            sum_acc = sum_acc + q16_mul(get_mat(L, i_idx, 2'(k)), get_vec(y, 2'(k)));
                        end
                    end
                    state <= CHOL_FWD_DIV;
                end

                CHOL_FWD_DIV: begin
                    div_dividend <= -get_vec(vec_g, i_idx) - sum_acc;
                    div_divisor  <= get_mat(L, i_idx, i_idx);
                    div_start    <= 1'b1;
                    state        <= CHOL_FWD_WAIT;
                end

                CHOL_FWD_WAIT: begin
                    if (div_done) begin
                        y <= set_vec(y, i_idx, div_quotient);
                        if (i_idx + 1'b1 < num_vars[1:0]) begin
                            i_idx <= i_idx + 1'b1;
                            state <= CHOL_FWD_SUM;
                        end else begin
                            i_idx <= num_vars[1:0] - 1'b1;
                            state <= CHOL_BWD_SUM;
                        end
                    end
                end

                // Solves Lᵀ · p = y
                CHOL_BWD_SUM: begin
                    sum_acc = 32'sd0;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (2'(k) > i_idx && 2'(k) < num_vars[1:0]) begin
                            sum_acc = sum_acc + q16_mul(get_mat(L, 2'(k), i_idx), get_vec(p_reg, 2'(k)));
                        end
                    end
                    state <= CHOL_BWD_DIV;
                end

                CHOL_BWD_DIV: begin
                    div_dividend <= get_vec(y, i_idx) - sum_acc;
                    div_divisor  <= get_mat(L, i_idx, i_idx);
                    div_start    <= 1'b1;
                    state        <= CHOL_BWD_WAIT;
                end

                CHOL_BWD_WAIT: begin
                    if (div_done) begin
                        p_reg <= set_vec(p_reg, i_idx, div_quotient);
                        if (i_idx > 2'd0) begin
                            i_idx <= i_idx - 1'b1;
                            state <= CHOL_BWD_SUM;
                        end else begin
                            state <= CHOL_DONE;
                        end
                    end
                end

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
