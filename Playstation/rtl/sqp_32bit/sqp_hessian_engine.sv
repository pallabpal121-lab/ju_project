// =============================================================================
// File Name   : sqp_hessian_engine.sv
// Module Name : sqp_hessian_engine
// Project     : Sequential Quadratic Programming (SQP) Accelerator (Solver #7)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Evaluates center objective f_0, numerical gradient vector g(x), and symmetric
//   Hessian matrix H(x) via finite differences using zero-cost arithmetic bit-shifts:
//     - Gradient     : g_j = (f_+j - f_-j) <<< 3
//     - Diagonal H_jj: H_jj = (f_+j - 2*f_0 + f_-j) <<< 8 + λ_ridge
//     - Off-Diag H_jk: H_jk = (f_++ - f_+- - f_-+ + f_--) <<< 6
// =============================================================================

`timescale 1ns / 1ps

import sqp_types_pkg::*;
`include "sqp_helpers.svh"

module sqp_hessian_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Programming Interface
    input  logic               prog_en,
    input  logic [4:0]         prog_addr,
    input  instr_t             prog_data,

    // Evaluation Control
    input  logic               start,
    input  logic [2:0]         num_params,
    input  vec_t               x_curr,

    // Outputs
    output q16_t               f_0_out,
    output vec_t               vec_g_out,
    output mat_t               mat_h_out,
    output logic               done,
    output logic               busy
);

    typedef enum logic [3:0] {
        H_IDLE        = 4'd0,
        H_START_F0    = 4'd1,
        H_WAIT_F0     = 4'd2,
        H_START_1D_P  = 4'd3,
        H_WAIT_1D_P   = 4'd4,
        H_START_1D_M  = 4'd5,
        H_WAIT_1D_M   = 4'd6,
        H_CALC_1D     = 4'd7,
        H_START_2D_PP = 4'd8,
        H_WAIT_2D_PP  = 4'd9,
        H_START_2D_PM = 4'd10,
        H_WAIT_2D_PM  = 4'd11,
        H_START_2D_MP = 4'd12,
        H_WAIT_2D_MP  = 4'd13,
        H_START_2D_MM = 4'd14,
        H_WAIT_2D_MM  = 4'd15
    } h_state_t;

    h_state_t state;

    // DFG Interconnect
    logic start_dfg;
    vec_t dfg_x_in;
    q16_t dfg_f_out;
    logic dfg_done, dfg_busy;

    // Registers
    logic [1:0] j_idx, k_idx;
    q16_t       f_0;
    q16_t       f_1d_plus  [0:MAX_PARAMS-1];
    q16_t       f_1d_minus [0:MAX_PARAMS-1];

    q16_t f_pp, f_pm, f_mp, f_mm;
    vec_t g_accum;
    mat_t H_accum;

    // Instantiate DFG Objective Evaluator
    dfg_sqp_engine u_dfg (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .num_params (num_params),
        .x_vec      (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    assign f_0_out   = f_0;
    assign vec_g_out = g_accum;
    assign mat_h_out = H_accum;

    q16_t cur_xj, cur_xk;
    q16_t diag_val;
    q16_t cross_val;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= H_IDLE;
            start_dfg <= 1'b0;
            dfg_x_in  <= '0;
            j_idx     <= '0;
            k_idx     <= '0;
            f_0       <= Q16_ZERO;
            f_pp      <= Q16_ZERO;
            f_pm      <= Q16_ZERO;
            f_mp      <= Q16_ZERO;
            f_mm      <= Q16_ZERO;
            g_accum   <= '0;
            H_accum   <= '0;
            done      <= 1'b0;
            busy      <= 1'b0;
            for (int i = 0; i < MAX_PARAMS; i++) begin
                f_1d_plus[i]  <= Q16_ZERO;
                f_1d_minus[i] <= Q16_ZERO;
            end
        end else begin
            start_dfg <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: H_IDLE
                // -------------------------------------------------------------
                H_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= H_START_F0;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // 1. EVALUATE CENTER POINT f_0 = f(x)
                // -------------------------------------------------------------
                H_START_F0: begin
                    dfg_x_in  <= x_curr;
                    start_dfg <= 1'b1;
                    state     <= H_WAIT_F0;
                end

                H_WAIT_F0: begin
                    if (dfg_done) begin
                        f_0   <= dfg_f_out;
                        j_idx <= 2'd0;
                        state <= H_START_1D_P;
                    end
                end

                // -------------------------------------------------------------
                // 2. EVALUATE 1D PERTURBATIONS: f(x + h*e_j) & f(x - h*e_j)
                // -------------------------------------------------------------
                H_START_1D_P: begin
                    dfg_x_in  <= set_vec(x_curr, j_idx, get_vec(x_curr, j_idx) + Q16_H_STEP);
                    start_dfg <= 1'b1;
                    state     <= H_WAIT_1D_P;
                end

                H_WAIT_1D_P: begin
                    if (dfg_done) begin
                        f_1d_plus[j_idx] <= dfg_f_out;
                        state            <= H_START_1D_M;
                    end
                end

                H_START_1D_M: begin
                    dfg_x_in  <= set_vec(x_curr, j_idx, get_vec(x_curr, j_idx) - Q16_H_STEP);
                    start_dfg <= 1'b1;
                    state     <= H_WAIT_1D_M;
                end

                H_WAIT_1D_M: begin
                    if (dfg_done) begin
                        f_1d_minus[j_idx] <= dfg_f_out;
                        if (j_idx + 1'b1 < num_params[1:0]) begin
                            j_idx <= j_idx + 1'b1;
                            state <= H_START_1D_P;
                        end else begin
                            state <= H_CALC_1D;
                        end
                    end
                end

                // -------------------------------------------------------------
                // 3. COMPUTE GRADIENTS & DIAGONAL HESSIAN CURVATURES
                // -------------------------------------------------------------
                H_CALC_1D: begin
                    for (int j = 0; j < MAX_PARAMS; j++) begin
                        if (j < num_params[1:0]) begin
                            // Gradient: (f_+ - f_-) <<< 3
                            g_accum = set_vec(g_accum, 2'(j), (f_1d_plus[j] - f_1d_minus[j]) <<< 3);

                            // Diagonal Curvature: (f_+ - 2*f_0 + f_-) <<< 8 + λ_ridge
                            diag_val = ((f_1d_plus[j] - (f_0 <<< 1) + f_1d_minus[j]) <<< 8) + Q16_RIDGE_LAMBDA;
                            if (diag_val <= 32'sd0) diag_val = Q16_RIDGE_LAMBDA; // Ensure positive-definiteness
                            H_accum  = set_mat(H_accum, 2'(j), 2'(j), diag_val);
                        end
                    end

                    j_idx <= 2'd0;
                    k_idx <= 2'd1;
                    state <= H_START_2D_PP;
                end

                // -------------------------------------------------------------
                // 4. CROSS-DERIVATIVE H_jk (j < k): 4-POINT FINITE DIFFERENCE
                // -------------------------------------------------------------
                H_START_2D_PP: begin
                    if (j_idx < num_params[1:0] && k_idx < num_params[1:0] && j_idx < k_idx) begin
                        cur_xj    = get_vec(x_curr, j_idx) + Q16_H_STEP;
                        cur_xk    = get_vec(x_curr, k_idx) + Q16_H_STEP;
                        dfg_x_in  <= set_vec(set_vec(x_curr, j_idx, cur_xj), k_idx, cur_xk);
                        start_dfg <= 1'b1;
                        state     <= H_WAIT_2D_PP;
                    end else begin
                        // Advance pair (j, k)
                        if (k_idx + 1'b1 < num_params[1:0]) begin
                            k_idx <= k_idx + 1'b1;
                            state <= H_START_2D_PP;
                        end else if (j_idx + 2'd2 < num_params[1:0]) begin
                            j_idx <= j_idx + 1'b1;
                            k_idx <= j_idx + 2'd2;
                            state <= H_START_2D_PP;
                        end else begin
                            done  <= 1'b1;
                            busy  <= 1'b0;
                            state <= H_IDLE;
                        end
                    end
                end

                H_WAIT_2D_PP: begin
                    if (dfg_done) begin
                        f_pp      <= dfg_f_out;
                        cur_xj    = get_vec(x_curr, j_idx) + Q16_H_STEP;
                        cur_xk    = get_vec(x_curr, k_idx) - Q16_H_STEP;
                        dfg_x_in  <= set_vec(set_vec(x_curr, j_idx, cur_xj), k_idx, cur_xk);
                        start_dfg <= 1'b1;
                        state     <= H_WAIT_2D_PM;
                    end
                end

                H_WAIT_2D_PM: begin
                    if (dfg_done) begin
                        f_pm      <= dfg_f_out;
                        cur_xj    = get_vec(x_curr, j_idx) - Q16_H_STEP;
                        cur_xk    = get_vec(x_curr, k_idx) + Q16_H_STEP;
                        dfg_x_in  <= set_vec(set_vec(x_curr, j_idx, cur_xj), k_idx, cur_xk);
                        start_dfg <= 1'b1;
                        state     <= H_WAIT_2D_MP;
                    end
                end

                H_WAIT_2D_MP: begin
                    if (dfg_done) begin
                        f_mp      <= dfg_f_out;
                        cur_xj    = get_vec(x_curr, j_idx) - Q16_H_STEP;
                        cur_xk    = get_vec(x_curr, k_idx) - Q16_H_STEP;
                        dfg_x_in  <= set_vec(set_vec(x_curr, j_idx, cur_xj), k_idx, cur_xk);
                        start_dfg <= 1'b1;
                        state     <= H_WAIT_2D_MM;
                    end
                end

                H_WAIT_2D_MM: begin
                    if (dfg_done) begin
                        f_mm      <= dfg_f_out;
                        // Cross-derivative: (f_++ - f_+- - f_-+ + f_--) <<< 6
                        cross_val = (f_pp - f_pm - f_mp + dfg_f_out) <<< 6;
                        H_accum   <= set_mat(set_mat(H_accum, j_idx, k_idx, cross_val), k_idx, j_idx, cross_val);

                        // Advance pair (j, k)
                        if (k_idx + 1'b1 < num_params[1:0]) begin
                            k_idx <= k_idx + 1'b1;
                            state <= H_START_2D_PP;
                        end else if (j_idx + 2'd2 < num_params[1:0]) begin
                            j_idx <= j_idx + 1'b1;
                            k_idx <= j_idx + 2'd2;
                            state <= H_START_2D_PP;
                        end else begin
                            done  <= 1'b1;
                            busy  <= 1'b0;
                            state <= H_IDLE;
                        end
                    end
                end

                default: state <= H_IDLE;
            endcase
        end
    end

endmodule
