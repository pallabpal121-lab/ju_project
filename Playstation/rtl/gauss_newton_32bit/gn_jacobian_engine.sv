// =============================================================================
// File Name   : gn_jacobian_engine.sv
// Module Name : gn_jacobian_engine
// Project     : Gauss-Newton Non-Linear Least Squares Accelerator (Solver #6)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Evaluates residual vector r (M x 1), Jacobian matrix J (M x N),
//   normal Gram matrix JᵀJ (N x N), gradient vector g = Jᵀr (N x 1),
//   and total least squares cost S = 0.5 * sum(r_m^2).
// =============================================================================

`timescale 1ns / 1ps

import gn_types_pkg::*;
`include "gn_helpers.svh"

module gn_jacobian_engine (
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

    // Outputs
    output q16_t               cost_s,      // Total sum of squared residuals S(x)
    output vec_t               vec_g,       // Gradient vector g = Jᵀr (N x 1)
    output mat_t               mat_a,       // Normal Gram Matrix A = JᵀJ + λ_eps*I (N x N)
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
    q16_t r_table  [0:MAX_OBS-1];
    q16_t J_matrix [0:MAX_OBS-1][0:MAX_PARAMS-1];

    logic [2:0] m_idx; // Observation index (0..7)
    logic [1:0] j_idx; // Parameter index (0..3)

    q16_t f_0, f_plus, f_minus;

    // Helper fixed-point multiplier
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Instantiate DFG Model Evaluator
    dfg_gn_engine u_dfg (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .num_params (num_params),
        .t_in       (dfg_t_in),
        .x_vec      (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    q16_t sum_cost;
    q16_t sum_grad;
    q16_t sum_jtj;
    vec_t out_g;
    mat_t out_a;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= J_IDLE;
            start_dfg <= 1'b0;
            dfg_t_in  <= Q16_ZERO;
            dfg_x_in  <= '0;
            m_idx     <= '0;
            j_idx     <= '0;
            f_0       <= Q16_ZERO;
            f_plus    <= Q16_ZERO;
            f_minus   <= Q16_ZERO;
            cost_s    <= Q16_ZERO;
            vec_g     <= '0;
            mat_a     <= '0;
            done      <= 1'b0;
            busy      <= 1'b0;
            for (int m = 0; m < MAX_OBS; m++) begin
                r_table[m] <= Q16_ZERO;
                for (int p = 0; p < MAX_PARAMS; p++) begin
                    J_matrix[m][p] <= Q16_ZERO;
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
                // EVALUATE RESIDUAL r_m = f(t_m, x) - y_m
                // -------------------------------------------------------------
                J_START_F0: begin
                    dfg_t_in  <= get_res(obs_t_vec, m_idx);
                    dfg_x_in  <= x_curr;
                    start_dfg <= 1'b1;
                    state     <= J_WAIT_F0;
                end

                J_WAIT_F0: begin
                    if (dfg_done) begin
                        f_0            <= dfg_f_out;
                        r_table[m_idx] <= dfg_f_out - get_res(obs_y_vec, m_idx);
                        j_idx          <= 2'd0;
                        state          <= J_START_FP;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE FORWARD PERTURBATION: f(t_m, x + h*e_j)
                // -------------------------------------------------------------
                J_START_FP: begin
                    dfg_t_in  <= get_res(obs_t_vec, m_idx);
                    dfg_x_in  <= set_vec(x_curr, j_idx, get_vec(x_curr, j_idx) + Q16_H_STEP);
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
                // EVALUATE BACKWARD PERTURBATION: f(t_m, x - h*e_j)
                // -------------------------------------------------------------
                J_START_FM: begin
                    dfg_t_in  <= get_res(obs_t_vec, m_idx);
                    dfg_x_in  <= set_vec(x_curr, j_idx, get_vec(x_curr, j_idx) - Q16_H_STEP);
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
                // CALCULATE JACOBIAN ENTRY: J_mj = (f_+ - f_-) / (2h)
                // -------------------------------------------------------------
                J_CALC_ELEM: begin
                    J_matrix[m_idx][j_idx] <= (f_plus - f_minus) <<< 3;

                    if (j_idx + 1'b1 < num_params[1:0]) begin
                        j_idx <= j_idx + 1'b1;
                        state <= J_START_FP;
                    end else if (4'(m_idx) + 1'b1 < num_obs) begin
                        m_idx <= m_idx + 1'b1;
                        state <= J_START_F0;
                    end else begin
                        state <= J_ACCUM;
                    end
                end

                // -------------------------------------------------------------
                // MATRIX & GRADIENT ACCUMULATION: JᵀJ + λ_eps*I, Jᵀr, Cost S(x)
                // -------------------------------------------------------------
                J_ACCUM: begin
                    // 1. Total cost S(x) = 0.5 * sum_{m=0}^{M-1} (r_m^2)
                    sum_cost = 32'sd0;
                    for (int m = 0; m < MAX_OBS; m++) begin
                        if (4'(m) < num_obs) begin
                            sum_cost = sum_cost + q16_mul(r_table[m], r_table[m]);
                        end
                    end
                    cost_s <= (sum_cost >>> 1);

                    // 2. Gradient vector g_j = sum_{m=0}^{M-1} (J_mj * r_m)
                    out_g = '0;
                    for (int j = 0; j < MAX_PARAMS; j++) begin
                        if (j < num_params[1:0]) begin
                            sum_grad = 32'sd0;
                            for (int m = 0; m < MAX_OBS; m++) begin
                                if (4'(m) < num_obs) begin
                                    sum_grad = sum_grad + q16_mul(J_matrix[m][j], r_table[m]);
                                end
                            end
                            out_g = set_vec(out_g, 2'(j), sum_grad);
                        end
                    end
                    vec_g <= out_g;

                    // 3. Normal matrix A_jk = sum_m (J_mj * J_mk) + (j==k ? λ_eps : 0)
                    out_a = '0;
                    for (int j = 0; j < MAX_PARAMS; j++) begin
                        if (j < num_params[1:0]) begin
                            for (int k = 0; k < MAX_PARAMS; k++) begin
                                if (k < num_params[1:0]) begin
                                    sum_jtj = 32'sd0;
                                    for (int m = 0; m < MAX_OBS; m++) begin
                                        if (4'(m) < num_obs) begin
                                            sum_jtj = sum_jtj + q16_mul(J_matrix[m][j], J_matrix[m][k]);
                                        end
                                    end
                                    if (j == k) begin
                                        sum_jtj = sum_jtj + Q16_LAMBDA_EPS; // Ridge regularizer
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
