// =============================================================================
// File Name   : ekf_helpers.svh
// Project     : Extended Kalman Filter (EKF) Accelerator (Solver #24)
// -----------------------------------------------------------------------------
// Description:
//   Matrix indexing, matrix-vector products, matrix multiplications (FPF^T, HPH^T),
//   and fixed-point math helpers.
// =============================================================================

`ifndef EKF_HELPERS_SVH
`define EKF_HELPERS_SVH

import ekf_types_pkg::*;

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

// Extract element from 2x4 matrix H
function automatic q16_t get_mmat(input meas_mat_t mat, input logic row, input logic [1:0] col);
    logic [2:0] idx;
    idx = {row, col};
    return mat[idx];
endfunction

// Set element in 2x4 matrix H
function automatic meas_mat_t set_mmat(input meas_mat_t mat, input logic row, input logic [1:0] col, input q16_t val);
    meas_mat_t res;
    logic [2:0] idx;
    res = mat;
    idx = {row, col};
    res[idx] = val;
    return res;
endfunction

// Extract element from 4x2 matrix K
function automatic q16_t get_gmat(input gain_mat_t mat, input logic [1:0] row, input logic col);
    logic [2:0] idx;
    idx = {row, col};
    return mat[idx];
endfunction

// Set element in 4x2 matrix K
function automatic gain_mat_t set_gmat(input gain_mat_t mat, input logic [1:0] row, input logic col, input q16_t val);
    gain_mat_t res;
    logic [2:0] idx;
    res = mat;
    idx = {row, col};
    res[idx] = val;
    return res;
endfunction

// Extract element from 2x2 matrix S
function automatic q16_t get_imat(input innov_mat_t mat, input logic row, input logic col);
    logic [1:0] idx;
    idx = {row, col};
    return mat[idx];
endfunction

// Set element in 2x2 matrix S
function automatic innov_mat_t set_imat(input innov_mat_t mat, input logic row, input logic col, input q16_t val);
    innov_mat_t res;
    logic [1:0] idx;
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

// 4x4 Matrix-Vector Product: u = M * x (dim: N)
function automatic state_vec_t smat_vec_mul(
    input state_mat_t M,
    input state_vec_t x,
    input logic [2:0] N
);
    state_vec_t u;
    q16_t sum_row;

    u = '0;
    for (int i = 0; i < MAX_STATE_DIM; i++) begin
        if (i < N) begin
            sum_row = Q16_ZERO;
            for (int j = 0; j < MAX_STATE_DIM; j++) begin
                if (j < N) begin
                    sum_row = sum_row + q16_mul(get_smat(M, 2'(i), 2'(j)), x[j]);
                end
            end
            u[i] = sum_row;
        end
    end
    return u;
endfunction

// 2x4 Matrix-Vector Product: y = H * x (H: M x N, x: N x 1, y: M x 1)
function automatic meas_vec_t mmat_vec_mul(
    input meas_mat_t H,
    input state_vec_t x,
    input logic [1:0] M_dim,
    input logic [2:0] N_dim
);
    meas_vec_t y;
    q16_t sum_row;

    y = '0;
    for (int i = 0; i < MAX_MEAS_DIM; i++) begin
        if (i < M_dim) begin
            sum_row = Q16_ZERO;
            for (int j = 0; j < MAX_STATE_DIM; j++) begin
                if (j < N_dim) begin
                    sum_row = sum_row + q16_mul(get_mmat(H, 1'(i), 2'(j)), x[j]);
                end
            end
            y[i] = sum_row;
        end
    end
    return y;
endfunction

// 4x2 Matrix-Vector Product: u = K * y (K: N x M, y: M x 1, u: N x 1)
function automatic state_vec_t gmat_vec_mul(
    input gain_mat_t K,
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
                    sum_row = sum_row + q16_mul(get_gmat(K, 2'(i), 1'(j)), y[j]);
                end
            end
            u[i] = sum_row;
        end
    end
    return u;
endfunction

// Sandwich Product: Out = F * P * F^T (4x4)
function automatic state_mat_t mat4_sandwich(
    input state_mat_t F,
    input state_mat_t P,
    input logic [2:0] N
);
    state_mat_t temp_fp;
    state_mat_t out_fpf;
    q16_t sum_k;

    temp_fp = '0;
    for (int i = 0; i < MAX_STATE_DIM; i++) begin
        for (int j = 0; j < MAX_STATE_DIM; j++) begin
            if (i < N && j < N) begin
                sum_k = Q16_ZERO;
                for (int k = 0; k < MAX_STATE_DIM; k++) begin
                    if (k < N) begin
                        sum_k = sum_k + q16_mul(get_smat(F, 2'(i), 2'(k)), get_smat(P, 2'(k), 2'(j)));
                    end
                end
                temp_fp = set_smat(temp_fp, 2'(i), 2'(j), sum_k);
            end
        end
    end

    out_fpf = '0;
    for (int i = 0; i < MAX_STATE_DIM; i++) begin
        for (int j = 0; j < MAX_STATE_DIM; j++) begin
            if (i < N && j < N) begin
                sum_k = Q16_ZERO;
                for (int k = 0; k < MAX_STATE_DIM; k++) begin
                    if (k < N) begin
                        sum_k = sum_k + q16_mul(get_smat(temp_fp, 2'(i), 2'(k)), get_smat(F, 2'(j), 2'(k)));
                    end
                end
                out_fpf = set_smat(out_fpf, 2'(i), 2'(j), sum_k);
            end
        end
    end
    return out_fpf;
endfunction

// Measurement Innovation Covariance: S = H * P * H^T (2x2)
function automatic innov_mat_t mat_hph(
    input meas_mat_t H,
    input state_mat_t P,
    input logic [1:0] M_dim,
    input logic [2:0] N_dim
);
    meas_mat_t temp_hp;
    innov_mat_t out_hph;
    q16_t sum_k;

    temp_hp = '0;
    for (int i = 0; i < MAX_MEAS_DIM; i++) begin
        for (int j = 0; j < MAX_STATE_DIM; j++) begin
            if (i < M_dim && j < N_dim) begin
                sum_k = Q16_ZERO;
                for (int k = 0; k < MAX_STATE_DIM; k++) begin
                    if (k < N_dim) begin
                        sum_k = sum_k + q16_mul(get_mmat(H, 1'(i), 2'(k)), get_smat(P, 2'(k), 2'(j)));
                    end
                end
                temp_hp = set_mmat(temp_hp, 1'(i), 2'(j), sum_k);
            end
        end
    end

    out_hph = '0;
    for (int i = 0; i < MAX_MEAS_DIM; i++) begin
        for (int j = 0; j < MAX_MEAS_DIM; j++) begin
            if (i < M_dim && j < M_dim) begin
                sum_k = Q16_ZERO;
                for (int k = 0; k < MAX_STATE_DIM; k++) begin
                    if (k < N_dim) begin
                        sum_k = sum_k + q16_mul(get_mmat(temp_hp, 1'(i), 2'(k)), get_mmat(H, 1'(j), 2'(k)));
                    end
                end
                out_hph = set_imat(out_hph, 1'(i), 1'(j), sum_k);
            end
        end
    end
    return out_hph;
endfunction

// Cross Covariance Matrix: PH^T (4x2)
function automatic gain_mat_t mat_pht(
    input state_mat_t P,
    input meas_mat_t H,
    input logic [2:0] N_dim,
    input logic [1:0] M_dim
);
    gain_mat_t out_pht;
    q16_t sum_k;

    out_pht = '0;
    for (int i = 0; i < MAX_STATE_DIM; i++) begin
        for (int j = 0; j < MAX_MEAS_DIM; j++) begin
            if (i < N_dim && j < M_dim) begin
                sum_k = Q16_ZERO;
                for (int k = 0; k < MAX_STATE_DIM; k++) begin
                    if (k < N_dim) begin
                        sum_k = sum_k + q16_mul(get_smat(P, 2'(i), 2'(k)), get_mmat(H, 1'(j), 2'(k)));
                    end
                end
                out_pht = set_gmat(out_pht, 2'(i), 1'(j), sum_k);
            end
        end
    end
    return out_pht;
endfunction

// Covariance Reduction: P_new = P - K * H * P (4x4)
function automatic state_mat_t mat_khp_sub(
    input state_mat_t P,
    input gain_mat_t K,
    input meas_mat_t H,
    input logic [2:0] N_dim,
    input logic [1:0] M_dim
);
    state_mat_t temp_kh;
    state_mat_t temp_khp;
    state_mat_t out_p;
    q16_t sum_k, p_elem, khp_elem;

    // KH: 4x4
    temp_kh = '0;
    for (int i = 0; i < MAX_STATE_DIM; i++) begin
        for (int j = 0; j < MAX_STATE_DIM; j++) begin
            if (i < N_dim && j < N_dim) begin
                sum_k = Q16_ZERO;
                for (int m = 0; m < MAX_MEAS_DIM; m++) begin
                    if (m < M_dim) begin
                        sum_k = sum_k + q16_mul(get_gmat(K, 2'(i), 1'(m)), get_mmat(H, 1'(m), 2'(j)));
                    end
                end
                temp_kh = set_smat(temp_kh, 2'(i), 2'(j), sum_k);
            end
        end
    end

    // KHP: 4x4
    temp_khp = '0;
    for (int i = 0; i < MAX_STATE_DIM; i++) begin
        for (int j = 0; j < MAX_STATE_DIM; j++) begin
            if (i < N_dim && j < N_dim) begin
                sum_k = Q16_ZERO;
                for (int k = 0; k < MAX_STATE_DIM; k++) begin
                    if (k < N_dim) begin
                        sum_k = sum_k + q16_mul(get_smat(temp_kh, 2'(i), 2'(k)), get_smat(P, 2'(k), 2'(j)));
                    end
                end
                temp_khp = set_smat(temp_khp, 2'(i), 2'(j), sum_k);
            end
        end
    end

    // Symmetrized P - KHP
    out_p = '0;
    for (int i = 0; i < MAX_STATE_DIM; i++) begin
        for (int j = 0; j < MAX_STATE_DIM; j++) begin
            if (i < N_dim && j < N_dim) begin
                p_elem   = get_smat(P, 2'(i), 2'(j));
                khp_elem = get_smat(temp_khp, 2'(i), 2'(j));
                out_p    = set_smat(out_p, 2'(i), 2'(j), p_elem - khp_elem);
            end else begin
                out_p = set_smat(out_p, 2'(i), 2'(j), (i == j) ? 32'h0001_0000 : 32'h0000_0000);
            end
        end
    end
    return out_p;
endfunction

`endif // EKF_HELPERS_SVH
