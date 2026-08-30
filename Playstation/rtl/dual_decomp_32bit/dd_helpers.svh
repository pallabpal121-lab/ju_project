// =============================================================================
// File Name   : dd_helpers.svh
// Project     : Dual Decomposition Engine (Solver #22)
// -----------------------------------------------------------------------------
// Description:
//   Matrix-vector math, transpose products, dot products, and vector operations.
// =============================================================================

`ifndef DD_HELPERS_SVH
`define DD_HELPERS_SVH

import dd_types_pkg::*;

// Extract element from 2x2 matrix
function automatic q16_t get_mat2(input logic signed [3:0][31:0] mat, input logic row, input logic col);
    logic [1:0] idx;
    idx = {row, col};
    return mat[idx];
endfunction

// Set element in 2x2 matrix
function automatic logic signed [3:0][31:0] set_mat2(input logic signed [3:0][31:0] mat, input logic row, input logic col, input q16_t val);
    logic signed [3:0][31:0] res;
    logic [1:0] idx;
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

// 2x2 Matrix-Vector Product: u = M * x (M: rows x cols, x: cols x 1)
function automatic logic signed [1:0][31:0] mat2_vec_mul(
    input logic signed [3:0][31:0] M,
    input logic signed [1:0][31:0] x,
    input logic [1:0] rows,
    input logic [1:0] cols
);
    logic signed [1:0][31:0] u;
    q16_t sum_row;
    q16_t m_elem, x_elem;

    u = '0;
    for (int i = 0; i < MAX_LOCAL_DIM; i++) begin
        if (i < rows) begin
            sum_row = Q16_ZERO;
            for (int j = 0; j < MAX_LOCAL_DIM; j++) begin
                if (j < cols) begin
                    m_elem = get_mat2(M, 1'(i), 1'(j));
                    x_elem = x[j];
                    sum_row = sum_row + q16_mul(m_elem, x_elem);
                end
            end
            u[i] = sum_row;
        end
    end
    return u;
endfunction

// Transpose 2x2 Matrix-Vector Product: u = M^T * y (M: rows x cols, y: rows x 1, u: cols x 1)
function automatic logic signed [1:0][31:0] mat2_t_vec_mul(
    input logic signed [3:0][31:0] M,
    input logic signed [1:0][31:0] y,
    input logic [1:0] rows,
    input logic [1:0] cols
);
    logic signed [1:0][31:0] u;
    q16_t sum_col;
    q16_t m_elem, y_elem;

    u = '0;
    for (int j = 0; j < MAX_LOCAL_DIM; j++) begin
        if (j < cols) begin
            sum_col = Q16_ZERO;
            for (int i = 0; i < MAX_LOCAL_DIM; i++) begin
                if (i < rows) begin
                    m_elem = get_mat2(M, 1'(i), 1'(j));
                    y_elem = y[i];
                    sum_col = sum_col + q16_mul(m_elem, y_elem);
                end
            end
            u[j] = sum_col;
        end
    end
    return u;
endfunction

// Vector Difference (2-element)
function automatic logic signed [1:0][31:0] vec2_sub(
    input logic signed [1:0][31:0] a,
    input logic signed [1:0][31:0] b,
    input logic [1:0] N
);
    logic signed [1:0][31:0] d;
    d = '0;
    for (int i = 0; i < MAX_LOCAL_DIM; i++) begin
        if (i < N) begin
            d[i] = a[i] - b[i];
        end
    end
    return d;
endfunction

// Vector Addition (2-element)
function automatic logic signed [1:0][31:0] vec2_add(
    input logic signed [1:0][31:0] a,
    input logic signed [1:0][31:0] b,
    input logic [1:0] N
);
    logic signed [1:0][31:0] s;
    s = '0;
    for (int i = 0; i < MAX_LOCAL_DIM; i++) begin
        if (i < N) begin
            s[i] = a[i] + b[i];
        end
    end
    return s;
endfunction

`endif // DD_HELPERS_SVH
