// =============================================================================
// File Name   : ipm_helpers.svh
// Project     : Primal-Dual Interior Point Method (IPM) Accelerator (Solver #16)
// -----------------------------------------------------------------------------
// Description:
//   Vector and matrix packing/unpacking and arithmetic helper functions.
// =============================================================================

`ifndef IPM_HELPERS_SVH
`define IPM_HELPERS_SVH

import ipm_types_pkg::*;

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

// Extract element from 4x4 matrix (512-bit)
function automatic q16_t get_mat(input mat_t mat, input logic [1:0] row, input logic [1:0] col);
    logic [3:0] idx;
    idx = {row, col};
    case (idx)
        4'd0:  return mat[31:0];
        4'd1:  return mat[63:32];
        4'd2:  return mat[95:64];
        4'd3:  return mat[127:96];
        4'd4:  return mat[159:128];
        4'd5:  return mat[191:160];
        4'd6:  return mat[223:192];
        4'd7:  return mat[255:224];
        4'd8:  return mat[287:256];
        4'd9:  return mat[319:288];
        4'd10: return mat[351:320];
        4'd11: return mat[383:352];
        4'd12: return mat[415:384];
        4'd13: return mat[447:416];
        4'd14: return mat[479:448];
        4'd15: return mat[511:480];
    endcase
endfunction

// Set element in 4x4 matrix (512-bit)
function automatic mat_t set_mat(input mat_t mat, input logic [1:0] row, input logic [1:0] col, input q16_t val);
    mat_t res;
    logic [3:0] idx;
    res = mat;
    idx = {row, col};
    case (idx)
        4'd0:  res[31:0]    = val;
        4'd1:  res[63:32]   = val;
        4'd2:  res[95:64]   = val;
        4'd3:  res[127:96]  = val;
        4'd4:  res[159:128] = val;
        4'd5:  res[191:160] = val;
        4'd6:  res[223:192] = val;
        4'd7:  res[255:224] = val;
        4'd8:  res[287:256] = val;
        4'd9:  res[319:288] = val;
        4'd10: res[351:320] = val;
        4'd11: res[383:352] = val;
        4'd12: res[415:384] = val;
        4'd13: res[447:416] = val;
        4'd14: res[479:448] = val;
        4'd15: res[511:480] = val;
    endcase
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

// Saturated Q16.16 multiplication to prevent wrap-around
function automatic q16_t q16_mul_sat(input q16_t a, input q16_t b);
    logic signed [63:0] prod;
    prod = 64'(a) * 64'(b);
    if (prod[63:47] != 17'sd0 && prod[63:47] != 17'sh1FFFF) begin
        return prod[63] ? 32'sh8000_0000 : 32'sh7FFF_FFFF;
    end
    return prod[47:16];
endfunction

`endif // IPM_HELPERS_SVH
