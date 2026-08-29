// =============================================================================
// File Name   : pso_helpers.svh
// Project     : Particle Swarm Optimization (PSO) Accelerator (Solver #14)
// -----------------------------------------------------------------------------
// Description:
//   Vector packing, indexing, clamping, and fixed-point helper arithmetic functions.
// =============================================================================

`ifndef PSO_HELPERS_SVH
`define PSO_HELPERS_SVH

import pso_types_pkg::*;

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

// Clamping function [min_val, max_val]
function automatic q16_t q16_clamp(input q16_t val, input q16_t min_val, input q16_t max_val);
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
endfunction

`endif // PSO_HELPERS_SVH
