// =============================================================================
// File Name   : svm_helpers.svh
// Project     : Support Vector Machine Sequential Minimal Optimization (SVM-SMO)
//               Accelerator (Solver #27)
// -----------------------------------------------------------------------------
// Description:
//   Dataset indexing, kernel matrix indexing, decision function evaluation,
//   error calculation, bounding box [L, H], and fixed-point math helpers.
// =============================================================================

`ifndef SVM_HELPERS_SVH
`define SVM_HELPERS_SVH

import svm_types_pkg::*;

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

// Extract element from 8x8 kernel matrix: [row][col] (idx = r*8 + c)
function automatic q16_t get_kmat(input kernel_mat_t kmat, input logic [2:0] row, input logic [2:0] col);
    logic [5:0] idx;
    idx = {row, col};
    return kmat[idx];
endfunction

// Set element in 8x8 kernel matrix: [row][col]
function automatic kernel_mat_t set_kmat(input kernel_mat_t kmat, input logic [2:0] row, input logic [2:0] col, input q16_t val);
    kernel_mat_t res;
    logic [5:0] idx;
    res = kmat;
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

// Evaluate decision function f(x_i) = ∑ (alpha_j * y_j * K(x_j, x_i)) + b
function automatic q16_t eval_f_xi(
    input kernel_mat_t kmat,
    input alpha_vec_t  alphas,
    input label_vec_t  labels,
    input q16_t        b_bias,
    input logic [2:0]  sample_i,
    input logic [3:0]  num_samples
);
    q16_t sum_val, ay_prod, k_elem;

    sum_val = b_bias;
    for (int j = 0; j < MAX_SAMPLES; j++) begin
        if (j < num_samples) begin
            if (alphas[j] > 32'h0000_0004) begin
                ay_prod = q16_mul(alphas[j], labels[j]);
                k_elem  = get_kmat(kmat, 3'(j), sample_i);
                sum_val = sum_val + q16_mul(ay_prod, k_elem);
            end
        end
    end
    return sum_val;
endfunction

// Compute Feasible Bounds [L, H] for (alpha_1, alpha_2)
task automatic calc_L_H(
    input  q16_t a1,
    input  q16_t a2,
    input  q16_t y1,
    input  q16_t y2,
    input  q16_t C_bound,
    output q16_t L_out,
    output q16_t H_out
);
    q16_t diff_a2_a1, sum_a1_a2;

    if (y1 != y2) begin
        diff_a2_a1 = a2 - a1;
        // L = max(0, a2 - a1)
        L_out = (diff_a2_a1 > Q16_ZERO) ? diff_a2_a1 : Q16_ZERO;
        // H = min(C, C + a2 - a1)
        H_out = (C_bound + diff_a2_a1 < C_bound) ? (C_bound + diff_a2_a1) : C_bound;
        if (H_out < Q16_ZERO) H_out = Q16_ZERO;
    end else begin
        sum_a1_a2 = a1 + a2;
        // L = max(0, a1 + a2 - C)
        L_out = (sum_a1_a2 - C_bound > Q16_ZERO) ? (sum_a1_a2 - C_bound) : Q16_ZERO;
        // H = min(C, a1 + a2)
        H_out = (sum_a1_a2 < C_bound) ? sum_a1_a2 : C_bound;
    end
endtask

`endif // SVM_HELPERS_SVH
