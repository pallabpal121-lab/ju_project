// =============================================================================
// File Name   : mpc_helpers.svh
// Project     : Model Predictive Control (MPC) Accelerator (Solver #26)
// -----------------------------------------------------------------------------
// Description:
//   Matrix indexing, matrix-vector multiplication, box projection, and fixed-point math.
// =============================================================================

`ifndef MPC_HELPERS_SVH
`define MPC_HELPERS_SVH

import mpc_types_pkg::*;

// Extract element from 4x4 matrix
function automatic q16_t get_hmat(input hessian_mat_t mat, input logic [1:0] row, input logic [1:0] col);
    logic [3:0] idx;
    idx = {row, col};
    return mat[idx];
endfunction

// Set element in 4x4 matrix
function automatic hessian_mat_t set_hmat(input hessian_mat_t mat, input logic [1:0] row, input logic [1:0] col, input q16_t val);
    hessian_mat_t res;
    logic [3:0] idx;
    res = mat;
    idx = {row, col};
    res[idx] = val;
    return res;
endfunction

// Absolute value
function automatic q16_t q16_abs(input q16_t val);
    return (val < 0) ? -val : val;
endfunction

// Fixed-point signed multiplication Q16.16
function automatic q16_t q16_mul(input q16_t a, input q16_t b);
    logic signed [63:0] prod;
    prod = 64'(a) * 64'(b);
    return prod[47:16];
endfunction

// 4x4 Matrix-Vector Product: y = A * x (dim <= 4)
function automatic stacked_vec_t mat_vec_mul_4x4(
    input hessian_mat_t A,
    input stacked_vec_t x,
    input logic [2:0] dim
);
    stacked_vec_t y;
    q16_t sum_row;

    y = '0;
    for (int i = 0; i < MAX_STACKED_DIM; i++) begin
        if (i < dim) begin
            sum_row = Q16_ZERO;
            for (int j = 0; j < MAX_STACKED_DIM; j++) begin
                if (j < dim) begin
                    sum_row = sum_row + q16_mul(get_hmat(A, 2'(i), 2'(j)), x[j]);
                end
            end
            y[i] = sum_row;
        end
    end
    return y;
endfunction

// 4x4 Matrix-Vector Product for State Mapping: g = M * x (dim_r <= 4, dim_c <= 4)
function automatic stacked_vec_t map_vec_mul(
    input grad_mat_t M,
    input state_vec_t x,
    input logic [2:0] r_dim,
    input logic [2:0] c_dim
);
    stacked_vec_t g;
    q16_t sum_row;

    g = '0;
    for (int i = 0; i < MAX_STACKED_DIM; i++) begin
        if (i < r_dim) begin
            sum_row = Q16_ZERO;
            for (int j = 0; j < MAX_STATE_DIM; j++) begin
                if (j < c_dim) begin
                    sum_row = sum_row + q16_mul(get_hmat(M, 2'(i), 2'(j)), x[j]);
                end
            end
            g[i] = sum_row;
        end
    end
    return g;
endfunction

// Box Clamping Projection Operator: v_i = clamp(u_i, u_min, u_max)
function automatic stacked_vec_t vec_box_clamp(
    input stacked_vec_t u,
    input stacked_vec_t u_min,
    input stacked_vec_t u_max,
    input logic [2:0] dim
);
    stacked_vec_t v;
    v = '0;
    for (int i = 0; i < MAX_STACKED_DIM; i++) begin
        if (i < dim) begin
            if (u[i] < u_min[i]) begin
                v[i] = u_min[i];
            end else if (u[i] > u_max[i]) begin
                v[i] = u_max[i];
            end else begin
                v[i] = u[i];
            end
        end
    end
    return v;
endfunction

`endif // MPC_HELPERS_SVH
