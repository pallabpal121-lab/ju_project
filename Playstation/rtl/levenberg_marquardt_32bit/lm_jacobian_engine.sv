// =============================================================================
// File Name   : lm_jacobian_engine.sv
// Module Name : lm_jacobian_engine
// Project     : Levenberg-Marquardt (LM) Non-Linear Least Squares Accelerator
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Evaluates residual vector r (M x 1), Jacobian matrix J (M x N),
//   approximated Hessian JᵀJ (N x N), gradient vector g = Jᵀr (N x 1),
//   and total cost S = 0.5 * sum(r_m^2) with diagonal damping λ*I.
// =============================================================================

`timescale 1ns / 1ps

import lm_types_pkg::*;
`include "lm_helpers.svh"

module lm_jacobian_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Programming Interface
    input  logic               prog_en,
    input  logic [4:0]         prog_addr,
    input  instr_t             prog_data,

    // Control & Inputs
    input  logic               start,
    input  logic [2:0]         num_params,  // Number of parameters N (1..4)
    input  logic [3:0]         num_obs,     // Number of observations M (1..8)
    input  vec_t               x_curr,      // Current parameter vector x
    input  res_vec_t           obs_t_vec,   // Input data points t_m (packed 8x32-bit)
    input  res_vec_t           obs_y_vec,   // Target outputs y_m (packed 8x32-bit)
    input  q16_t               lambda_reg,  // Damping factor λ

    // Outputs
    output q16_t               cost_s,      // Total sum of squared residuals S(x)
    output vec_t               vec_g,       // Gradient vector g = Jᵀr (N x 1)
    output mat_t               mat_a,       // Augmented Hessian A = JᵀJ + λ*I (N x N)
    output logic               done,
    output logic               busy
);

    typedef enum logic [3:0] {
        J_IDLE      = 4'd0,
        J_START_F0  = 4'd1,
        J_WAIT_F0   = 4'd2,
        J_START_FP  = 4'd3,
        J_WAIT_FP   = 4'd4,
        J_START_FM  = 4'd5,
        J_WAIT_FM   = 4'd6,
        J_CALC_ELEM = 4'd7,
        J_ACCUM     = 4'd8,
        J_DONE      = 4'd9
    } j_state_t;

    j_state_t state;

    // DFG Submodule Interconnect
    logic start_dfg;
    vec_t dfg_x_in;
    q16_t dfg_t_in;
    q16_t dfg_f_out;
    logic dfg_done, dfg_busy;

    // Internal Memory for Residuals and Jacobian Elements
    q16_t r_vec [0:MAX_OBS-1];
    q16_t J_table [0:MAX_OBS-1][0:MAX_PARAMS-1];

    // Loop Counters
    logic [2:0] m_idx; // Observation index 0..7
    logic [1:0] n_idx; // Parameter index 0..3

    // Sample storage
    q16_t f_0;
    q16_t f_plus, f_minus;

    // Helper fixed-point multiplier
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Instantiate DFG Model Evaluator
    dfg_lm_engine u_dfg_lm (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .num_params (num_params),
        .x_vec      (dfg_x_in),
        .t_in       (dfg_t_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    // Accumulator temporary variables
    q16_t sum_cost;
    q16_t sum_g;
    q16_t sum_jtj;
    vec_t out_g;
    mat_t out_a;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= J_IDLE;
            start_dfg <= 1'b0;
            dfg_x_in  <= '0;
            dfg_t_in  <= Q16_ZERO;
            m_idx     <= '0;
            n_idx     <= '0;
            f_0       <= Q16_ZERO;
            f_plus    <= Q16_ZERO;
            f_minus   <= Q16_ZERO;
            cost_s    <= Q16_ZERO;
            vec_g     <= '0;
            mat_a     <= '0;
            done      <= 1'b0;
            busy      <= 1'b0;
            for (int m = 0; m < MAX_OBS; m++) begin
                r_vec[m] <= Q16_ZERO;
                for (int n = 0; n < MAX_PARAMS; n++) begin
                    J_table[m][n] <= Q16_ZERO;
                end
            end
        end else begin
            start_dfg <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: J_IDLE
                // -------------------------------------------------------------
                J_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        m_idx <= 3'd0;
                        state <= J_START_F0;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE BASE POINT: f(t_m, x) -> r_m = f(t_m, x) - y_m
                // -------------------------------------------------------------
                J_START_F0: begin
                    dfg_x_in  <= x_curr;
                    dfg_t_in  <= get_res(obs_t_vec, m_idx);
                    start_dfg <= 1'b1;
                    state     <= J_WAIT_F0;
                end

                J_WAIT_F0: begin
                    if (dfg_done) begin
                        f_0            <= dfg_f_out;
                        r_vec[m_idx]   <= dfg_f_out - get_res(obs_y_vec, m_idx); // r_m = f(t_m, x) - y_m
                        n_idx          <= 2'd0;
                        state          <= J_START_FP;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE PARTIAL DERIVATIVE: f(t_m, x + h*e_n)
                // -------------------------------------------------------------
                J_START_FP: begin
                    dfg_x_in  <= set_vec(x_curr, n_idx, get_vec(x_curr, n_idx) + Q16_H_STEP);
                    dfg_t_in  <= get_res(obs_t_vec, m_idx);
                    start_dfg <= 1'b1;
                    state     <= J_WAIT_FP;
                end

                J_WAIT_FP: begin
                    if (dfg_done) begin
                        f_plus <= dfg_f_out;
                        state  <= J_START_FM;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE PARTIAL DERIVATIVE: f(t_m, x - h*e_n)
                // -------------------------------------------------------------
                J_START_FM: begin
                    dfg_x_in  <= set_vec(x_curr, n_idx, get_vec(x_curr, n_idx) - Q16_H_STEP);
                    dfg_t_in  <= get_res(obs_t_vec, m_idx);
                    start_dfg <= 1'b1;
                    state     <= J_WAIT_FM;
                end

                J_WAIT_FM: begin
                    if (dfg_done) begin
                        f_minus <= dfg_f_out;
                        state   <= J_CALC_ELEM;
                    end
                end

                // -------------------------------------------------------------
                // CALCULATE JACOBIAN ENTRY: J[m][n] = (f_+ - f_-) / (2h)
                // -------------------------------------------------------------
                J_CALC_ELEM: begin
                    J_table[m_idx][n_idx] <= (f_plus - f_minus) <<< 3; // J_mn = diff <<< 3

                    if (n_idx + 1'b1 < num_params[1:0]) begin
                        n_idx <= n_idx + 1'b1;
                        state <= J_START_FP;
                    end else if (m_idx + 1'b1 < num_obs[2:0]) begin
                        m_idx <= m_idx + 1'b1;
                        state <= J_START_F0;
                    end else begin
                        state <= J_ACCUM;
                    end
                end

                // -------------------------------------------------------------
                // ACCUMULATE: S(x) = 0.5 * ||r||^2,  g = Jᵀr,  A = JᵀJ + λ*I
                // -------------------------------------------------------------
                J_ACCUM: begin
                    // 1. Compute Total Cost: S = 0.5 * sum(r_m^2)
                    sum_cost = 32'sd0;
                    for (int m = 0; m < MAX_OBS; m++) begin
                        if (m < num_obs[2:0]) begin
                            sum_cost = sum_cost + q16_mul(r_vec[m], r_vec[m]);
                        end
                    end
                    cost_s <= sum_cost >>> 1; // Divide by 2 (0.5 * sum(r^2))

                    // 2. Compute Gradient: g_j = sum_{m=0}^{M-1} (J[m][j] * r[m])
                    out_g = '0;
                    for (int j = 0; j < MAX_PARAMS; j++) begin
                        if (j < num_params[1:0]) begin
                            sum_g = 32'sd0;
                            for (int m = 0; m < MAX_OBS; m++) begin
                                if (m < num_obs[2:0]) begin
                                    sum_g = sum_g + q16_mul(J_table[m][j], r_vec[m]);
                                end
                            end
                            out_g = set_vec(out_g, 2'(j), sum_g);
                        end
                    end
                    vec_g <= out_g;

                    // 3. Compute Normal Matrix: A[j][k] = sum_{m=0}^{M-1} (J[m][j] * J[m][k]) + (j==k ? λ : 0)
                    out_a = '0;
                    for (int j = 0; j < MAX_PARAMS; j++) begin
                        if (j < num_params[1:0]) begin
                            for (int k = 0; k < MAX_PARAMS; k++) begin
                                if (k < num_params[1:0]) begin
                                    sum_jtj = 32'sd0;
                                    for (int m = 0; m < MAX_OBS; m++) begin
                                        if (m < num_obs[2:0]) begin
                                            sum_jtj = sum_jtj + q16_mul(J_table[m][j], J_table[m][k]);
                                        end
                                    end
                                    if (j == k) begin
                                        sum_jtj = sum_jtj + lambda_reg; // Marquardt Damping
                                    end
                                    out_a = set_mat(out_a, 2'(j), 2'(k), sum_jtj);
                                end
                            end
                        end
                    end
                    mat_a <= out_a;

                    state <= J_DONE;
                end

                // -------------------------------------------------------------
                // STATE: J_DONE
                // -------------------------------------------------------------
                J_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= J_IDLE;
                end

                default: state <= J_IDLE;
            endcase
        end
    end

endmodule
