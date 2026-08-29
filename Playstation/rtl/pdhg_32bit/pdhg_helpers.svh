// =============================================================================
// File Name   : pdhg_helpers.svh
// Project     : Primal-Dual Hybrid Gradient (PDHG / Chambolle-Pock) Accelerator (Solver #19)
// -----------------------------------------------------------------------------
// Description:
//   Matrix-vector operations (K*x and K^T*y), soft-thresholding, clamping,
//   and fixed-point math helpers.
// =============================================================================

`ifndef PDHG_HELPERS_SVH
`define PDHG_HELPERS_SVH

import pdhg_types_pkg::*;

// Extract element from 4x4 matrix K
function automatic q16_t get_mat(input mat_t mat, input logic [1:0] row, input logic [1:0] col);
    logic [3:0] idx;
    idx = {row, col};
    return mat[idx];
endfunction

// Set element in 4x4 matrix K
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

// Soft-Thresholding Operator S_gamma(z) = sign(z) * max(|z| - gamma, 0)
function automatic q16_t soft_thresh_scalar(input q16_t z, input q16_t gamma);
    q16_t abs_z;
    abs_z = q16_abs(z);
    if (abs_z <= gamma) begin
        return Q16_ZERO;
    end else if (z > 0) begin
        return abs_z - gamma;
    end else begin
        return -(abs_z - gamma);
    end
endfunction

// Box Clamping Operator clamp(z, -lambda, +lambda)
function automatic q16_t clamp_box(input q16_t z, input q16_t bound);
    if (z > bound) begin
        return bound;
    end else if (z < -bound) begin
        return -bound;
    end else begin
        return z;
    end
endfunction

// Matrix-Vector Product: u = K * x (K: M x N, x: N x 1, u: M x 1)
function automatic vec_t mat_vec_mul(input mat_t K, input vec_t x, input logic [2:0] M, input logic [2:0] N);
    vec_t u;
    q16_t sum_row;
    q16_t k_elem, x_elem;

    u = '0;
    for (int i = 0; i < MAX_DIM; i++) begin
        if (i < M) begin
            sum_row = Q16_ZERO;
            for (int j = 0; j < MAX_DIM; j++) begin
                if (j < N) begin
                    k_elem = get_mat(K, 2'(i), 2'(j));
                    x_elem = x[j];
                    sum_row = sum_row + q16_mul(k_elem, x_elem);
                end
            end
            u[i] = sum_row;
        end
    end
    return u;
endfunction

// Matrix-Transpose-Vector Product: v = K^T * y (K^T: N x M, y: M x 1, v: N x 1)
function automatic vec_t mat_t_vec_mul(input mat_t K, input vec_t y, input logic [2:0] M, input logic [2:0] N);
    vec_t v;
    q16_t sum_col;
    q16_t k_elem, y_elem;

    v = '0;
    for (int j = 0; j < MAX_DIM; j++) begin
        if (j < N) begin
            sum_col = Q16_ZERO;
            for (int i = 0; i < MAX_DIM; i++) begin
                if (i < M) begin
                    k_elem = get_mat(K, 2'(i), 2'(j));
                    y_elem = y[i];
                    sum_col = sum_col + q16_mul(k_elem, y_elem);
                end
            end
            v[j] = sum_col;
        end
    end
    return v;
endfunction

`endif // PDHG_HELPERS_SVH
