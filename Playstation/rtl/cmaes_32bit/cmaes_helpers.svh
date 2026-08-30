// =============================================================================
// File Name   : cmaes_helpers.svh
// Project     : Covariance Matrix Adaptation Evolution Strategy (CMA-ES) (Solver #34)
// -----------------------------------------------------------------------------
// Description:
//   Packed matrix/array indexing, matrix-vector multiplication y = A*z,
//   vector norms, bounds clamping, and objective fitness evaluations.
// =============================================================================

`ifndef CMAES_HELPERS_SVH
`define CMAES_HELPERS_SVH

import cmaes_types_pkg::*;

// Fixed-point signed multiplication Q16.16
function automatic q16_t q16_mul(input q16_t a, input q16_t b);
    logic signed [63:0] prod;
    prod = 64'(a) * 64'(b);
    return prod[47:16];
endfunction

// Absolute value
function automatic q16_t q16_abs(input q16_t val);
    return (val < 0) ? -val : val;
endfunction

// Clamp parameter to bounds [lb, ub]
function automatic q16_t clamp_param(input q16_t val, input q16_t lb, input q16_t ub);
    if (val < lb) return lb;
    if (val > ub) return ub;
    return val;
endfunction

// Extract element from 4x4 matrix: [r][c] (idx = r*4 + c)
function automatic q16_t get_mat_val(input cov_mat_t mat, input logic [1:0] r, input logic [1:0] c);
    logic [3:0] idx;
    idx = {r, c};
    return mat[idx];
endfunction

// Set element in 4x4 matrix: [r][c]
function automatic cov_mat_t set_mat_val(input cov_mat_t mat, input logic [1:0] r, input logic [1:0] c, input q16_t val);
    cov_mat_t res;
    logic [3:0] idx;
    res = mat;
    idx = {r, c};
    res[idx] = val;
    return res;
endfunction

// Extract candidate parameter: [k][d] (idx = k*4 + d)
function automatic q16_t get_cand_val(input cand_arr_t carr, input logic [1:0] k, input logic [1:0] d);
    logic [3:0] idx;
    idx = {k, d};
    return carr[idx];
endfunction

// Set candidate parameter: [k][d]
function automatic cand_arr_t set_cand_val(input cand_arr_t carr, input logic [1:0] k, input logic [1:0] d, input q16_t val);
    cand_arr_t res;
    logic [3:0] idx;
    res = carr;
    idx = {k, d};
    res[idx] = val;
    return res;
endfunction

// Matrix-Vector Multiplication: y = A * z (4x4 * 4x1)
function automatic param_vec_t mat_vec_mul(
    input cov_mat_t   A_mat,
    input param_vec_t z_vec,
    input logic [2:0] dim
);
    param_vec_t y_res;
    q16_t sum_val;
    for (int r = 0; r < MAX_DIM; r++) begin
        sum_val = 32'sd0;
        for (int c = 0; c < MAX_DIM; c++) begin
            if (c < dim) begin
                sum_val = sum_val + q16_mul(get_mat_val(A_mat, 2'(r), 2'(c)), z_vec[c]);
            end
        end
        y_res[r] = (r < dim) ? sum_val : 32'sd0;
    end
    return y_res;
endfunction

// Vector L2 Norm Squared: ||x||_2^2
function automatic q16_t vec_norm_sq(
    input param_vec_t vec,
    input logic [2:0] dim
);
    q16_t sum_sq;
    sum_sq = 32'sd0;
    for (int d = 0; d < MAX_DIM; d++) begin
        if (d < dim) begin
            sum_sq = sum_sq + q16_mul(vec[d], vec[d]);
        end
    end
    return sum_sq;
endfunction

// Objective Fitness Evaluation (Minimization)
function automatic q16_t evaluate_fitness(
    input cmaes_fn_t  fn_type,
    input param_vec_t x,
    input logic [2:0] dim
);
    q16_t f_val, d0, d1, d2, d3, diff, x0_sq;
    f_val = 32'sd0;

    case (fn_type)
        FN_QUADRATIC: begin
            // f(x) = (x0 - 3)^2 + 2(x1 - 4)^2
            d0 = x[0] - 32'h0003_0000; // x0 - 3.0
            d1 = x[1] - 32'h0004_0000; // x1 - 4.0
            f_val = q16_mul(d0, d0) + q16_mul(32'h0002_0000, q16_mul(d1, d1));
        end

        FN_ROSENBROCK: begin
            // f(x) = 10*(x1 - x0^2)^2 + (1 - x0)^2
            x0_sq = q16_mul(x[0], x[0]);
            diff  = x[1] - x0_sq;
            d0    = 32'h0001_0000 - x[0];
            f_val = q16_mul(32'h000A_0000, q16_mul(diff, diff)) + q16_mul(d0, d0);
        end

        FN_RASTRIGIN: begin
            // f(x) = ∑ (x_d^2 + x_d^4)
            d0 = q16_mul(x[0], x[0]);
            d1 = q16_mul(x[1], x[1]);
            d2 = q16_mul(x[2], x[2]);
            d3 = q16_mul(x[3], x[3]);
            f_val = d0 + q16_mul(d0, d0) +
                    d1 + q16_mul(d1, d1) +
                    ((dim > 2) ? (d2 + q16_mul(d2, d2)) : 32'sd0) +
                    ((dim > 3) ? (d3 + q16_mul(d3, d3)) : 32'sd0);
        end

        default: f_val = 32'sd0;
    endcase

    return f_val;
endfunction

`endif // CMAES_HELPERS_SVH
