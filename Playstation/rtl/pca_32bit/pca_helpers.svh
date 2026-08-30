// =============================================================================
// File Name   : pca_helpers.svh
// Project     : Principal Component Analysis (PCA) / Streaming SVD Accelerator
//               (Solver #28)
// -----------------------------------------------------------------------------
// Description:
//   Dataset indexing, covariance matrix indexing, eigenvector matrix indexing,
//   matrix-vector products, Rayleigh quotient, deflation updates, and projection.
// =============================================================================

`ifndef PCA_HELPERS_SVH
`define PCA_HELPERS_SVH

import pca_types_pkg::*;

// Extract element from dataset: [sample_idx][feat_idx] (idx = s*4 + f)
function automatic q16_t get_sample_feat(input dataset_arr_t ds, input logic [2:0] s_idx, input logic [1:0] f_idx);
    logic [4:0] idx;
    idx = {s_idx, f_idx};
    return ds[idx];
endfunction

// Set element in dataset: [sample_idx][feat_idx]
function automatic dataset_arr_t set_sample_feat(input dataset_arr_t ds, input logic [2:0] s_idx, input logic [1:0] f_idx, input q16_t val);
    dataset_arr_t res;
    logic [4:0] idx;
    res = ds;
    idx = {s_idx, f_idx};
    res[idx] = val;
    return res;
endfunction

// Extract element from 4x4 covariance matrix: [row][col] (idx = r*4 + c)
function automatic q16_t get_cov_elem(input cov_mat_t cmat, input logic [1:0] row, input logic [1:0] col);
    logic [3:0] idx;
    idx = {row, col};
    return cmat[idx];
endfunction

// Set element in 4x4 covariance matrix: [row][col]
function automatic cov_mat_t set_cov_elem(input cov_mat_t cmat, input logic [1:0] row, input logic [1:0] col, input q16_t val);
    cov_mat_t res;
    logic [3:0] idx;
    res = cmat;
    idx = {row, col};
    res[idx] = val;
    return res;
endfunction

// Extract element from 4x4 eigenvector matrix: [row][comp_k]
function automatic q16_t get_eigen_elem(input eigen_mat_t vmat, input logic [1:0] row, input logic [1:0] comp_k);
    logic [3:0] idx;
    idx = {row, comp_k};
    return vmat[idx];
endfunction

// Set element in 4x4 eigenvector matrix: [row][comp_k]
function automatic eigen_mat_t set_eigen_elem(input eigen_mat_t vmat, input logic [1:0] row, input logic [1:0] comp_k, input q16_t val);
    eigen_mat_t res;
    logic [3:0] idx;
    res = vmat;
    idx = {row, comp_k};
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

// Matrix-Vector Product: y = Σ * q (4x4 * 4x1)
function automatic feature_vec_t cov_vec_mul(
    input cov_mat_t     cmat,
    input feature_vec_t q_vec,
    input logic [2:0]   feat_dim
);
    feature_vec_t y_out;
    q16_t sum_val, c_elem, prod_val;

    for (int r = 0; r < MAX_FEATURES; r++) begin
        if (r < feat_dim) begin
            sum_val = 32'sd0;
            for (int c = 0; c < MAX_FEATURES; c++) begin
                if (c < feat_dim) begin
                    c_elem   = get_cov_elem(cmat, 2'(r), 2'(c));
                    prod_val = q16_mul(c_elem, q_vec[c]);
                    sum_val  = sum_val + prod_val;
                end
            end
            y_out[r] = sum_val;
        end else begin
            y_out[r] = 32'sd0;
        end
    end
    return y_out;
endfunction

// Rayleigh Quotient: λ = q^T * Σ * q = q^T * y
function automatic q16_t rayleigh_quotient(
    input feature_vec_t q_vec,
    input feature_vec_t y_vec,
    input logic [2:0]   feat_dim
);
    q16_t sum_val, prod_val;
    sum_val = 32'sd0;
    for (int d = 0; d < MAX_FEATURES; d++) begin
        if (d < feat_dim) begin
            prod_val = q16_mul(q_vec[d], y_vec[d]);
            sum_val  = sum_val + prod_val;
        end
    end
    return sum_val;
endfunction

// Hotelling's Deflation: Σ_new = Σ - λ * (q * q^T)
function automatic cov_mat_t deflate_cov(
    input cov_mat_t     cmat,
    input q16_t         lambda_val,
    input feature_vec_t q_vec,
    input logic [2:0]   feat_dim
);
    cov_mat_t res;
    q16_t old_val, outer_prod, sub_val;

    res = cmat;
    for (int r = 0; r < MAX_FEATURES; r++) begin
        if (r < feat_dim) begin
            for (int c = 0; c < MAX_FEATURES; c++) begin
                if (c < feat_dim) begin
                    old_val    = get_cov_elem(cmat, 2'(r), 2'(c));
                    outer_prod = q16_mul(q_vec[r], q_vec[c]);
                    sub_val    = q16_mul(lambda_val, outer_prod);
                    res        = set_cov_elem(res, 2'(r), 2'(c), old_val - sub_val);
                end
            end
        end
    end
    return res;
endfunction

// Project to Latent Space: z_k = v_k^T * (x - x_mean)
function automatic latent_vec_t project_latent(
    input eigen_mat_t   vmat,
    input feature_vec_t x_in,
    input feature_vec_t x_mean,
    input logic [2:0]   feat_dim,
    input logic [2:0]   num_comp
);
    latent_vec_t z_out;
    feature_vec_t diff_x;
    q16_t sum_val, v_elem, prod_val;

    for (int d = 0; d < MAX_FEATURES; d++) begin
        diff_x[d] = x_in[d] - x_mean[d];
    end

    for (int k = 0; k < MAX_COMPONENTS; k++) begin
        if (k < num_comp) begin
            sum_val = 32'sd0;
            for (int d = 0; d < MAX_FEATURES; d++) begin
                if (d < feat_dim) begin
                    v_elem   = get_eigen_elem(vmat, 2'(d), 2'(k));
                    prod_val = q16_mul(v_elem, diff_x[d]);
                    sum_val  = sum_val + prod_val;
                end
            end
            z_out[k] = sum_val;
        end else begin
            z_out[k] = 32'sd0;
        end
    end
    return z_out;
endfunction

// Reconstruct from Latent Space: x_hat = x_mean + ∑ (z_k * v_k)
function automatic feature_vec_t reconstruct_feature(
    input eigen_mat_t   vmat,
    input latent_vec_t  z_in,
    input feature_vec_t x_mean,
    input logic [2:0]   feat_dim,
    input logic [2:0]   num_comp
);
    feature_vec_t x_hat;
    q16_t sum_val, v_elem, prod_val;

    for (int d = 0; d < MAX_FEATURES; d++) begin
        if (d < feat_dim) begin
            sum_val = x_mean[d];
            for (int k = 0; k < MAX_COMPONENTS; k++) begin
                if (k < num_comp) begin
                    v_elem   = get_eigen_elem(vmat, 2'(d), 2'(k));
                    prod_val = q16_mul(z_in[k], v_elem);
                    sum_val  = sum_val + prod_val;
                end
            end
            x_hat[d] = sum_val;
        end else begin
            x_hat[d] = 32'sd0;
        end
    end
    return x_hat;
endfunction

`endif // PCA_HELPERS_SVH
