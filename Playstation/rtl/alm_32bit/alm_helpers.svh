// =============================================================================
// File Name   : alm_helpers.svh
// Project     : Augmented Lagrangian Method (ALM) Accelerator (Solver #21)
// -----------------------------------------------------------------------------
// Description:
//   Matrix-vector operations, transpose products, dot products, and fixed-point math.
// =============================================================================

`ifndef ALM_HELPERS_SVH
`define ALM_HELPERS_SVH

import alm_types_pkg::*;

// Extract element from 4x4 matrix
function automatic q16_t get_mat(input mat_t mat, input logic [1:0] row, input logic [1:0] col);
    logic [3:0] idx;
    idx = {row, col};
    return mat[idx];
endfunction

// Set element in 4x4 matrix
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

// Matrix-Vector Product: u = M * x (M: rows x cols, x: cols x 1, u: rows x 1)
function automatic vec_t mat_vec_mul(input mat_t M, input vec_t x, input logic [2:0] rows, input logic [2:0] cols);
    vec_t u;
    q16_t sum_row;
    q16_t m_elem, x_elem;

    u = '0;
    for (int i = 0; i < MAX_DIM; i++) begin
        if (i < rows) begin
            sum_row = Q16_ZERO;
            for (int j = 0; j < MAX_DIM; j++) begin
                if (j < cols) begin
                    m_elem = get_mat(M, 2'(i), 2'(j));
                    x_elem = x[j];
                    sum_row = sum_row + q16_mul(m_elem, x_elem);
                end
            end
            u[i] = sum_row;
        end
    end
    return u;
endfunction

// Transpose Matrix-Vector Product: u = M^T * y (M: rows x cols, y: rows x 1, u: cols x 1)
function automatic vec_t mat_t_vec_mul(input mat_t M, input vec_t y, input logic [2:0] rows, input logic [2:0] cols);
    vec_t u;
    q16_t sum_col;
    q16_t m_elem, y_elem;

    u = '0;
    for (int j = 0; j < MAX_DIM; j++) begin
        if (j < cols) begin
            sum_col = Q16_ZERO;
            for (int i = 0; i < MAX_DIM; i++) begin
                if (i < rows) begin
                    m_elem = get_mat(M, 2'(i), 2'(j));
                    y_elem = y[i];
                    sum_col = sum_col + q16_mul(m_elem, y_elem);
                end
            end
            u[j] = sum_col;
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

// Vector Addition: s = a + b
function automatic vec_t vec_add(input vec_t a, input vec_t b, input logic [2:0] N);
    vec_t s;
    s = '0;
    for (int i = 0; i < MAX_DIM; i++) begin
        if (i < N) begin
            s[i] = a[i] + b[i];
        end
    end
    return s;
endfunction

`endif // ALM_HELPERS_SVH
