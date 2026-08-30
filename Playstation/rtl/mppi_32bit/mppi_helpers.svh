// =============================================================================
// File Name   : mppi_helpers.svh
// Project     : Model Predictive Path Integral (MPPI) Accelerator (Solver #33)
// -----------------------------------------------------------------------------
// Description:
//   Packed matrix/array indexing, inlined linear dynamics propagation,
//   quadratic stage/terminal cost evaluation, fixed-point exponential, and clamping.
// =============================================================================

`ifndef MPPI_HELPERS_SVH
`define MPPI_HELPERS_SVH

import mppi_types_pkg::*;

// Fixed-point signed multiplication Q16.16
function automatic q16_t q16_mul(input q16_t a, input q16_t b);
    logic signed [63:0] prod;
    prod = 64'(a) * 64'(b);
    return prod[47:16];
endfunction

// Absolute value
function automatic q16_t q16_abs(input q16_t val);
    return (val < 0) ? -val : val;
endfunction

// Clamp control input to actuator limits [lb, ub]
function automatic q16_t clamp_ctrl(input q16_t val, input q16_t lb, input q16_t ub);
    if (val < lb) return lb;
    if (val > ub) return ub;
    return val;
endfunction

// Extract element from 2x2 matrix: [r][c] (idx = r*2 + c)
function automatic q16_t get_mat_elem(input mat22_t mat, input logic [0:0] r, input logic [0:0] c);
    logic [1:0] idx;
    idx = {r, c};
    return mat[idx];
endfunction

// Extract element from control sequence: [t][m] (idx = t*2 + m)
function automatic q16_t get_ctrl_elem(input ctrl_seq_t seq, input logic [1:0] t, input logic [0:0] m);
    logic [2:0] idx;
    idx = {t, m};
    return seq[idx];
endfunction

// Set element in control sequence: [t][m]
function automatic ctrl_seq_t set_ctrl_elem(input ctrl_seq_t seq, input logic [1:0] t, input logic [0:0] m, input q16_t val);
    ctrl_seq_t res;
    logic [2:0] idx;
    res = seq;
    idx = {t, m};
    res[idx] = val;
    return res;
endfunction

// Extract element from noise array: [k][t][m] (idx = (k*4 + t)*2 + m)
function automatic q16_t get_noise_elem(input noise_arr_t narr, input logic [1:0] k, input logic [1:0] t, input logic [0:0] m);
    logic [4:0] idx;
    idx = {k, t, m};
    return narr[idx];
endfunction

// Set element in noise array: [k][t][m]
function automatic noise_arr_t set_noise_elem(input noise_arr_t narr, input logic [1:0] k, input logic [1:0] t, input logic [0:0] m, input q16_t val);
    noise_arr_t res;
    logic [4:0] idx;
    res = narr;
    idx = {k, t, m};
    res[idx] = val;
    return res;
endfunction

// Forward Linear Dynamics: x_{t+1} = A * x_t + B * u_t
function automatic state_vec_t step_dynamics(
    input state_vec_t x,
    input ctrl_vec_t  u,
    input mat22_t     A_mat,
    input mat22_t     B_mat
);
    state_vec_t x_next;
    x_next[0] = q16_mul(get_mat_elem(A_mat, 1'b0, 1'b0), x[0]) +
                q16_mul(get_mat_elem(A_mat, 1'b0, 1'b1), x[1]) +
                q16_mul(get_mat_elem(B_mat, 1'b0, 1'b0), u[0]) +
                q16_mul(get_mat_elem(B_mat, 1'b0, 1'b1), u[1]);

    x_next[1] = q16_mul(get_mat_elem(A_mat, 1'b1, 1'b0), x[0]) +
                q16_mul(get_mat_elem(A_mat, 1'b1, 1'b1), x[1]) +
                q16_mul(get_mat_elem(B_mat, 1'b1, 1'b0), u[0]) +
                q16_mul(get_mat_elem(B_mat, 1'b1, 1'b1), u[1]);
    return x_next;
endfunction

// Quadratic Stage Cost: c(x, u) = (x - x_ref)^T Q (x - x_ref) + u^T R u
function automatic q16_t eval_stage_cost(
    input state_vec_t x,
    input state_vec_t x_ref,
    input ctrl_vec_t  u,
    input state_vec_t Q_diag,
    input ctrl_vec_t  R_diag
);
    state_vec_t e;
    q16_t cost_x, cost_u;
    e[0] = x[0] - x_ref[0];
    e[1] = x[1] - x_ref[1];

    cost_x = q16_mul(Q_diag[0], q16_mul(e[0], e[0])) +
             q16_mul(Q_diag[1], q16_mul(e[1], e[1]));

    cost_u = q16_mul(R_diag[0], q16_mul(u[0], u[0])) +
             q16_mul(R_diag[1], q16_mul(u[1], u[1]));

    return cost_x + cost_u;
endfunction

// Quadratic Terminal Cost: phi(x) = (x - x_ref)^T Qf (x - x_ref)
function automatic q16_t eval_terminal_cost(
    input state_vec_t x,
    input state_vec_t x_ref,
    input state_vec_t Qf_diag
);
    state_vec_t e;
    e[0] = x[0] - x_ref[0];
    e[1] = x[1] - x_ref[1];

    return q16_mul(Qf_diag[0], q16_mul(e[0], e[0])) +
           q16_mul(Qf_diag[1], q16_mul(e[1], e[1]));
endfunction

// Fixed-Point Exponential Approximation for z <= 0:
// exp(z) ≈ 1 + z + z^2/2 + z^3/6
function automatic q16_t fixed_exp(input q16_t z);
    q16_t z_sq, z_cu, term2, term3, sum_val;
    if (z >= 32'sd0) return Q16_ONE;
    if (z < -32'h0004_0000) return 32'sd0; // Underflow clamp for z < -4.0

    z_sq  = q16_mul(z, z);
    z_cu  = q16_mul(z_sq, z);

    term2 = q16_mul(Q16_HALF, z_sq);               // z^2 / 2
    term3 = q16_mul(32'h0000_2AAA, z_cu);           // z^3 / 6 (1/6 ≈ 0.16666 = 0x2AAA)

    sum_val = Q16_ONE + z + term2 + term3;
    if (sum_val < 32'sd0) return 32'sd0;
    if (sum_val > Q16_ONE) return Q16_ONE;
    return sum_val;
endfunction

`endif // MPPI_HELPERS_SVH
