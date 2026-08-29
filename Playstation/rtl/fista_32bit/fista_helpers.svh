// =============================================================================
// File Name   : fista_helpers.svh
// Project     : Fast Iterative Shrinkage-Thresholding Algorithm (FISTA) Accelerator (Solver #15)
// -----------------------------------------------------------------------------
// Description:
//   Vector packing, indexing, fixed-point math, and Hardware Soft-Thresholding
//   Operator S_tau(z) = sign(z) * max(|z| - tau, 0).
// =============================================================================

`ifndef FISTA_HELPERS_SVH
`define FISTA_HELPERS_SVH

import fista_types_pkg::*;

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

// Hardware Soft-Thresholding Operator S_tau(z) = sign(z) * max(|z| - tau, 0)
function automatic q16_t q16_soft_thresh(input q16_t z, input q16_t tau);
    q16_t abs_z;
    abs_z = q16_abs(z);
    if (abs_z <= tau) begin
        return Q16_ZERO;
    end else begin
        return (z > 0) ? (abs_z - tau) : -(abs_z - tau);
    end
endfunction

`endif // FISTA_HELPERS_SVH
