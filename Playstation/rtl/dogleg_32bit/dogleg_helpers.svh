// =============================================================================
// File Name   : dogleg_helpers.svh
// Project     : Trust-Region Dogleg Non-Linear Optimizer Accelerator (Solver #13)
// -----------------------------------------------------------------------------
// Description:
//   Vector and matrix packing/unpacking and arithmetic helper functions.
// =============================================================================

`ifndef DOGLEG_HELPERS_SVH
`define DOGLEG_HELPERS_SVH

import dogleg_types_pkg::*;

// Extract element from 4-element vector (128-bit)
function automatic q16_t get_vec(input vec_t vec, input logic [1:0] idx);
    case (idx)
        2'd0: return vec[31:0];
        2'd1: return vec[63:32];
        2'd2: return vec[95:64];
        2'd3: return vec[127:96];
    endcase
endfunction

// Set element in 4-element vector (128-bit)
function automatic vec_t set_vec(input vec_t vec, input logic [1:0] idx, input q16_t val);
    vec_t res;
    res = vec;
    case (idx)
        2'd0: res[31:0]   = val;
        2'd1: res[63:32]  = val;
        2'd2: res[95:64]  = val;
        2'd3: res[127:96] = val;
    endcase
    return res;
endfunction

// Extract element from 8-element residual/observation vector (256-bit)
function automatic q16_t get_res(input res_vec_t rvec, input logic [2:0] idx);
    case (idx)
        3'd0: return rvec[31:0];
        3'd1: return rvec[63:32];
        3'd2: return rvec[95:64];
        3'd3: return rvec[127:96];
        3'd4: return rvec[159:128];
        3'd5: return rvec[191:160];
        3'd6: return rvec[223:192];
        3'd7: return rvec[255:224];
    endcase
endfunction

// Set element in 8-element residual/observation vector (256-bit)
function automatic res_vec_t set_res(input res_vec_t rvec, input logic [2:0] idx, input q16_t val);
    res_vec_t res;
    res = rvec;
    case (idx)
        3'd0: res[31:0]   = val;
        3'd1: res[63:32]  = val;
        3'd2: res[95:64]  = val;
        3'd3: res[127:96] = val;
        3'd4: res[159:128]= val;
        3'd5: res[191:160]= val;
        3'd6: res[223:192]= val;
        3'd7: res[255:224]= val;
    endcase
    return res;
endfunction

// Extract element from 4x4 matrix (512-bit)
function automatic q16_t get_mat(input mat_t mat, input logic [1:0] r, input logic [1:0] c);
    logic [3:0] idx;
    idx = {r, c};
    return mat[idx*32 +: 32];
endfunction

// Set element in 4x4 matrix (512-bit)
function automatic mat_t set_mat(input mat_t mat, input logic [1:0] r, input logic [1:0] c, input q16_t val);
    mat_t res;
    logic [3:0] idx;
    res = mat;
    idx = {r, c};
    res[idx*32 +: 32] = val;
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

function automatic q16_t q16_mult(input q16_t a, input q16_t b);
    logic signed [63:0] prod;
    prod = 64'(a) * 64'(b);
    return prod[47:16];
endfunction

`endif // DOGLEG_HELPERS_SVH

