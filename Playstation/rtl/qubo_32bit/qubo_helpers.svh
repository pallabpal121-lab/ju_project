// =============================================================================
// File Name   : qubo_helpers.svh
// Project     : QUBO / Simulated Annealing Ising Accelerator (Solver #18)
// -----------------------------------------------------------------------------
// Description:
//   Matrix indexing, spin accessors, fixed-point math, and helper functions.
// =============================================================================

`ifndef QUBO_HELPERS_SVH
`define QUBO_HELPERS_SVH

import qubo_types_pkg::*;

// Extract element from 8x8 QUBO matrix
function automatic q16_t get_qmat(input qubo_mat_t mat, input logic [2:0] row, input logic [2:0] col);
    logic [5:0] idx;
    idx = {row, col};
    return mat[idx];
endfunction

// Set element in 8x8 QUBO matrix
function automatic qubo_mat_t set_qmat(input qubo_mat_t mat, input logic [2:0] row, input logic [2:0] col, input q16_t val);
    qubo_mat_t res;
    logic [5:0] idx;
    res = mat;
    idx = {row, col};
    res[idx] = val;
    return res;
endfunction

// Get single binary spin q_i in {0, 1}
function automatic logic get_spin(input spin_vec_t spins, input logic [2:0] idx);
    return spins[idx];
endfunction

// Set single binary spin q_i
function automatic spin_vec_t set_spin(input spin_vec_t spins, input logic [2:0] idx, input logic val);
    spin_vec_t res;
    res = spins;
    res[idx] = val;
    return res;
endfunction

// Flip single binary spin: q_i <- 1 - q_i
function automatic spin_vec_t flip_spin(input spin_vec_t spins, input logic [2:0] idx);
    spin_vec_t res;
    res = spins;
    res[idx] = ~res[idx];
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

// Saturated multiplication for energy calculations
function automatic q16_t q16_mul_sat(input q16_t a, input q16_t b);
    logic signed [63:0] prod;
    prod = 64'(a) * 64'(b);
    if (prod[63:47] != 17'sd0 && prod[63:47] != 17'sh1FFFF) begin
        return prod[63] ? 32'sh8000_0000 : 32'sh7FFF_FFFF;
    end
    return prod[47:16];
endfunction

`endif // QUBO_HELPERS_SVH
