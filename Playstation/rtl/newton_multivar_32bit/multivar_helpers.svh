// =============================================================================
// Company / Institution : Jadavpur University (Dept. of ETCE)
// Project               : Dedicated AI Hardware Accelerator
// File Name             : multivar_helpers.svh
// Description           : Synthesizable Inline Helper Functions for Packed
//                         Vectors and Matrices.
//                         Provides clean, industry-grade bit-slice extraction
//                         and update functions for N-dimensional optimization.
// =============================================================================

`ifndef MULTIVAR_HELPERS_SVH
`define MULTIVAR_HELPERS_SVH

import newton_multivar_pkg::*;

// -----------------------------------------------------------------------------
// Function    : get_vec
// Description : Extracts a 32-bit Q16.16 scalar element from a packed vector.
// Inputs      : v   - Packed state vector of type vec_t (MAX_VARS * 32 bits)
//               idx - Variable index (0 <= idx < MAX_VARS)
// Returns     : 32-bit signed Q16.16 value at index idx.
// -----------------------------------------------------------------------------
function automatic q16_t get_vec(input vec_t v, input int idx);
    return v[idx * 32 +: 32];
endfunction

// -----------------------------------------------------------------------------
// Function    : set_vec
// Description : Updates a 32-bit Q16.16 scalar element within a packed vector.
// Inputs      : v   - Original packed vector (vec_t)
//               idx - Target variable index (0 <= idx < MAX_VARS)
//               val - New 32-bit signed Q16.16 value to insert
// Returns     : Updated packed vector (vec_t).
// -----------------------------------------------------------------------------
function automatic vec_t set_vec(input vec_t v, input int idx, input q16_t val);
    vec_t res;
    res = v;
    res[idx * 32 +: 32] = val;
    return res;
endfunction

// -----------------------------------------------------------------------------
// Function    : get_mat
// Description : Extracts a 32-bit Q16.16 matrix element from a packed matrix.
// Inputs      : m - Packed matrix of type mat_t (MAX_VARS * MAX_VARS * 32 bits)
//               r - Row index (0 <= r < MAX_VARS)
//               c - Column index (0 <= c < MAX_VARS)
// Returns     : 32-bit signed Q16.16 value at position (r, c).
// -----------------------------------------------------------------------------
function automatic q16_t get_mat(input mat_t m, input int r, input int c);
    return m[(r * MAX_VARS + c) * 32 +: 32];
endfunction

// -----------------------------------------------------------------------------
// Function    : set_mat
// Description : Updates a 32-bit Q16.16 matrix element within a packed matrix.
// Inputs      : m   - Original packed matrix (mat_t)
//               r   - Target row index (0 <= r < MAX_VARS)
//               c   - Target column index (0 <= c < MAX_VARS)
//               val - New 32-bit signed Q16.16 value to insert
// Returns     : Updated packed matrix (mat_t).
// -----------------------------------------------------------------------------
function automatic mat_t set_mat(input mat_t m, input int r, input int c, input q16_t val);
    mat_t res;
    res = m;
    res[(r * MAX_VARS + c) * 32 +: 32] = val;
    return res;
endfunction

`endif // MULTIVAR_HELPERS_SVH
