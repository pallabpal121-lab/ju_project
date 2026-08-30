// =============================================================================
// File Name   : ukf_helpers.svh
// Project     : Unscented Kalman Filter (UKF) Accelerator (Solver #25)
// -----------------------------------------------------------------------------
// Description:
//   Matrix indexing, weighted sigma point accumulation, covariance updates,
//   and fixed-point math helpers.
// =============================================================================

`ifndef UKF_HELPERS_SVH
`define UKF_HELPERS_SVH

import ukf_types_pkg::*;

// Extract element from 4x4 matrix
function automatic q16_t get_smat(input state_mat_t mat, input logic [1:0] row, input logic [1:0] col);
    logic [3:0] idx;
    idx = {row, col};
    return mat[idx];
endfunction

// Set element in 4x4 matrix
function automatic state_mat_t set_smat(input state_mat_t mat, input logic [1:0] row, input logic [1:0] col, input q16_t val);
    state_mat_t res;
    logic [3:0] idx;
    res = mat;
    idx = {row, col};
    res[idx] = val;
    return res;
endfunction

// Extract element from 4x2 matrix (P_xz, K)
function automatic q16_t get_cmat(input cross_mat_t mat, input logic [1:0] row, input logic col);
    logic [2:0] idx;
    idx = {row, col};
    return mat[idx];
endfunction

// Set element in 4x2 matrix (P_xz, K)
function automatic cross_mat_t set_cmat(input cross_mat_t mat, input logic [1:0] row, input logic col, input q16_t val);
    cross_mat_t res;
    logic [2:0] idx;
    res = mat;
    idx = {row, col};
    res[idx] = val;
    return res;
endfunction

// Extract element from 2x2 matrix (P_zz, R)
function automatic q16_t get_imat(input innov_mat_t mat, input logic row, input logic col);
    logic [1:0] idx;
    idx = {row, col};
    return mat[idx];
endfunction

// Set element in 2x2 matrix (P_zz, R)
function automatic innov_mat_t set_imat(input innov_mat_t mat, input logic row, input logic col, input q16_t val);
    innov_mat_t res;
    logic [1:0] idx;
    res = mat;
    idx = {row, col};
    res[idx] = val;
    return res;
endfunction

// Sigma point state access: [point][dim] -> idx = point*4 + dim
function automatic q16_t get_sigma_state(input sigma_state_arr_t arr, input logic [3:0] pt, input logic [1:0] dim);
    logic [5:0] idx;
    idx = {pt, dim};
    return arr[idx];
endfunction

function automatic sigma_state_arr_t set_sigma_state(input sigma_state_arr_t arr, input logic [3:0] pt, input logic [1:0] dim, input q16_t val);
    sigma_state_arr_t res;
    logic [5:0] idx;
    res = arr;
    idx = {pt, dim};
    res[idx] = val;
    return res;
endfunction

// Sigma point measurement access: [point][dim] -> idx = point*2 + dim
function automatic q16_t get_sigma_meas(input sigma_meas_arr_t arr, input logic [3:0] pt, input logic dim);
    logic [4:0] idx;
    idx = {pt, dim};
    return arr[idx];
endfunction

function automatic sigma_meas_arr_t set_sigma_meas(input sigma_meas_arr_t arr, input logic [3:0] pt, input logic dim, input q16_t val);
    sigma_meas_arr_t res;
    logic [4:0] idx;
    res = arr;
    idx = {pt, dim};
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

// 4x2 Matrix-Vector Product: u = K * y
function automatic state_vec_t cmat_vec_mul(
    input cross_mat_t K,
    input meas_vec_t y,
    input logic [2:0] N_dim,
    input logic [1:0] M_dim
);
    state_vec_t u;
    q16_t sum_row;

    u = '0;
    for (int i = 0; i < MAX_STATE_DIM; i++) begin
        if (i < N_dim) begin
            sum_row = Q16_ZERO;
            for (int j = 0; j < MAX_MEAS_DIM; j++) begin
                if (j < M_dim) begin
                    sum_row = sum_row + q16_mul(get_cmat(K, 2'(i), 1'(j)), y[j]);
                end
            end
            u[i] = sum_row;
        end
    end
    return u;
endfunction

// Covariance Reduction: P_new = P_prior - K * P_zz * K^T (4x4)
function automatic state_mat_t mat_kpzk_sub(
    input state_mat_t P_prior,
    input cross_mat_t K,
    input innov_mat_t P_zz,
    input logic [2:0] N_dim,
    input logic [1:0] M_dim
);
    cross_mat_t temp_kpz;
    state_mat_t temp_kpzk;
    state_mat_t out_p;
    q16_t sum_m, p_elem, kpzk_elem;

    // temp_kpz = K * P_zz: 4x2
    temp_kpz = '0;
    for (int i = 0; i < MAX_STATE_DIM; i++) begin
        for (int j = 0; j < MAX_MEAS_DIM; j++) begin
            if (i < N_dim && j < M_dim) begin
                sum_m = Q16_ZERO;
                for (int m = 0; m < MAX_MEAS_DIM; m++) begin
                    if (m < M_dim) begin
                        sum_m = sum_m + q16_mul(get_cmat(K, 2'(i), 1'(m)), get_imat(P_zz, 1'(m), 1'(j)));
                    end
                end
                temp_kpz = set_cmat(temp_kpz, 2'(i), 1'(j), sum_m);
            end
        end
    end

    // temp_kpzk = (K * P_zz) * K^T: 4x4
    temp_kpzk = '0;
    for (int i = 0; i < MAX_STATE_DIM; i++) begin
        for (int j = 0; j < MAX_STATE_DIM; j++) begin
            if (i < N_dim && j < N_dim) begin
                sum_m = Q16_ZERO;
                for (int m = 0; m < MAX_MEAS_DIM; m++) begin
                    if (m < M_dim) begin
                        sum_m = sum_m + q16_mul(get_cmat(temp_kpz, 2'(i), 1'(m)), get_cmat(K, 2'(j), 1'(m)));
                    end
                end
                temp_kpzk = set_smat(temp_kpzk, 2'(i), 2'(j), sum_m);
            end
        end
    end

    // Symmetrized P_prior - KPZK
    out_p = '0;
    for (int i = 0; i < MAX_STATE_DIM; i++) begin
        for (int j = 0; j < MAX_STATE_DIM; j++) begin
            if (i < N_dim && j < N_dim) begin
                p_elem    = get_smat(P_prior, 2'(i), 2'(j));
                kpzk_elem = get_smat(temp_kpzk, 2'(i), 2'(j));
                out_p     = set_smat(out_p, 2'(i), 2'(j), p_elem - kpzk_elem);
            end else begin
                out_p = set_smat(out_p, 2'(i), 2'(j), (i == j) ? 32'h0001_0000 : 32'h0000_0000);
            end
        end
    end
    return out_p;
endfunction

`endif // UKF_HELPERS_SVH
