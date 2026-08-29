// =============================================================================
// File Name   : adam_helpers.svh
// Project     : Adaptive Moment Estimation (Adam) Accelerator (Solver #17)
// -----------------------------------------------------------------------------
// Description:
//   Vector packing, indexing, fixed-point math, and helper functions.
// =============================================================================

`ifndef ADAM_HELPERS_SVH
`define ADAM_HELPERS_SVH

import adam_types_pkg::*;

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

`endif // ADAM_HELPERS_SVH
