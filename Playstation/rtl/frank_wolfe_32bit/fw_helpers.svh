// =============================================================================
// File Name   : fw_helpers.svh
// Project     : Frank-Wolfe / Conditional Gradient Accelerator (Solver #20)
// -----------------------------------------------------------------------------
// Description:
//   Matrix-vector multiplication, dot products, convex linear combinations,
//   and fixed-point math helpers.
// =============================================================================

`ifndef FW_HELPERS_SVH
`define FW_HELPERS_SVH

import fw_types_pkg::*;

// Extract element from 4x4 matrix Q
function automatic q16_t get_mat(input mat_t mat, input logic [1:0] row, input logic [1:0] col);
    logic [3:0] idx;
    idx = {row, col};
    return mat[idx];
endfunction

// Set element in 4x4 matrix Q
function automatic mat_t set_mat(input mat_t mat, input logic [1:0] row, input logic [1:0] col, input q16_t val);
    mat_t res;
    logic [3:0] idx;
    res = mat;
    idx = {row, col};
    res[idx] = val;
    return res;
endfunction

// Absolute value for Q16.16
function automatic q16_t q16_abs(input q16_t val);
    return (val < 0) ? -val : val;
endfunction

// Fixed-point signed multiplication Q16.16
function automatic q16_t q16_mul(input q16_t a, input q16_t b);
    logic signed [63:0] prod;
    prod = 64'(a) * 64'(b);
    return prod[47:16];
endfunction

// Dot Product: a^T * b in Q16.16
function automatic q16_t dot_product(input vec_t a, input vec_t b, input logic [2:0] N);
    q16_t sum;
    sum = Q16_ZERO;
    for (int i = 0; i < MAX_DIM; i++) begin
        if (i < N) begin
            sum = sum + q16_mul(a[i], b[i]);
        end
    end
    return sum;
endfunction

// Matrix-Vector Product: u = Q * x (Q: N x N, x: N x 1, u: N x 1)
function automatic vec_t mat_vec_mul(input mat_t Q, input vec_t x, input logic [2:0] N);
    vec_t u;
    q16_t sum_row;
    q16_t q_elem, x_elem;

    u = '0;
    for (int i = 0; i < MAX_DIM; i++) begin
        if (i < N) begin
            sum_row = Q16_ZERO;
            for (int j = 0; j < MAX_DIM; j++) begin
                if (j < N) begin
                    q_elem = get_mat(Q, 2'(i), 2'(j));
                    x_elem = x[j];
                    sum_row = sum_row + q16_mul(q_elem, x_elem);
                end
            end
            u[i] = sum_row;
        end
    end
    return u;
endfunction

// Vector Difference: d = a - b
function automatic vec_t vec_sub(input vec_t a, input vec_t b, input logic [2:0] N);
    vec_t d;
    d = '0;
    for (int i = 0; i < MAX_DIM; i++) begin
        if (i < N) begin
            d[i] = a[i] - b[i];
        end
    end
    return d;
endfunction

// Convex Combination: x_next = (1 - gamma) * x + gamma * s
function automatic vec_t convex_comb(input vec_t x, input vec_t s, input q16_t gamma, input logic [2:0] N);
    vec_t res;
    q16_t one_minus_g;
    one_minus_g = Q16_ONE - gamma;
    res = '0;
    for (int i = 0; i < MAX_DIM; i++) begin
        if (i < N) begin
            res[i] = q16_mul(one_minus_g, x[i]) + q16_mul(gamma, s[i]);
        end
    end
    return res;
endfunction

`endif // FW_HELPERS_SVH
