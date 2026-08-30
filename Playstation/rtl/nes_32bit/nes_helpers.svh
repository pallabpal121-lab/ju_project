// =============================================================================
// File Name   : nes_helpers.svh
// Project     : Natural Evolution Strategies (NES) Accelerator (Solver #32)
// -----------------------------------------------------------------------------
// Description:
//   Perturbation indexing, parameter accessors, bounds clamping, and inlined
//   objective reward evaluators (Quadratic, Rosenbrock, Sphere, Rastrigin).
// =============================================================================

`ifndef NES_HELPERS_SVH
`define NES_HELPERS_SVH

import nes_types_pkg::*;

// Extract noise component from array: [p_idx][dim_idx] (idx = p*4 + dim)
function automatic q16_t get_noise_val(input noise_arr_t narr, input logic [1:0] p_idx, input logic [1:0] dim_idx);
    logic [3:0] idx;
    idx = {p_idx, dim_idx};
    return narr[idx];
endfunction

// Set noise component in array: [p_idx][dim_idx]
function automatic noise_arr_t set_noise_val(input noise_arr_t narr, input logic [1:0] p_idx, input logic [1:0] dim_idx, input q16_t val);
    noise_arr_t res;
    logic [3:0] idx;
    res = narr;
    idx = {p_idx, dim_idx};
    res[idx] = val;
    return res;
endfunction

// Extract complete perturbation vector from noise array
function automatic policy_vec_t get_noise_vec(input noise_arr_t narr, input logic [1:0] p_idx);
    policy_vec_t res;
    for (int d = 0; d < MAX_DIM; d++) begin
        res[d] = get_noise_val(narr, p_idx, 2'(d));
    end
    return res;
endfunction

// Set complete perturbation vector in noise array
function automatic noise_arr_t set_noise_vec(input noise_arr_t narr, input logic [1:0] p_idx, input policy_vec_t pvec);
    noise_arr_t res;
    res = narr;
    for (int d = 0; d < MAX_DIM; d++) begin
        res = set_noise_val(res, p_idx, 2'(d), pvec[d]);
    end
    return res;
endfunction

// Clamp policy parameter to bounds [lb, ub]
function automatic q16_t clamp_param(input q16_t val, input q16_t lb, input q16_t ub);
    if (val < lb) return lb;
    if (val > ub) return ub;
    return val;
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

// Quadratic Reward: R(θ) = -(θ0 - 3.0)^2 - 2(θ1 - 4.0)^2
function automatic q16_t eval_fn_quadratic(input policy_vec_t theta, input logic [2:0] dim);
    q16_t diff0, diff1, term0, term1;
    diff0 = theta[0] - 32'h0003_0000; // θ0 - 3.0
    diff1 = theta[1] - 32'h0004_0000; // θ1 - 4.0
    term0 = q16_mul(diff0, diff0);
    term1 = q16_mul(diff1, diff1);
    term1 = q16_mul(32'h0002_0000, term1); // 2 * (θ1 - 4.0)^2
    return -(term0 + term1);
endfunction

// Rosenbrock Valley Reward: R(θ) = -10(θ1 - θ0^2)^2 - (1 - θ0)^2
function automatic q16_t eval_fn_rosenbrock(input policy_vec_t theta);
    q16_t th0_sq, diff_y, term_curv, diff_th0, term_lin;
    th0_sq    = q16_mul(theta[0], theta[0]);
    diff_y    = theta[1] - th0_sq;
    term_curv = q16_mul(diff_y, diff_y);
    term_curv = q16_mul(32'h000A_0000, term_curv); // 10 * (θ1 - θ0^2)^2

    diff_th0  = 32'h0001_0000 - theta[0]; // 1 - θ0
    term_lin  = q16_mul(diff_th0, diff_th0);
    return -(term_curv + term_lin);
endfunction

// Sphere Reward: R(θ) = -∑ (θ_d)^2
function automatic q16_t eval_fn_sphere(input policy_vec_t theta, input logic [2:0] dim);
    q16_t sum_val, term;
    sum_val = 32'sd0;
    for (int d = 0; d < MAX_DIM; d++) begin
        if (d < dim) begin
            term    = q16_mul(theta[d], theta[d]);
            sum_val = sum_val + term;
        end
    end
    return -sum_val;
endfunction

// Multi-Modal Reward: R(θ) = -∑ (θ_d^2 + θ_d^4)
function automatic q16_t eval_fn_rastrigin(input policy_vec_t theta, input logic [2:0] dim);
    q16_t sum_val, term_sq, term_quad;
    sum_val = 32'sd0;
    for (int d = 0; d < MAX_DIM; d++) begin
        if (d < dim) begin
            term_sq   = q16_mul(theta[d], theta[d]);
            term_quad = q16_mul(term_sq, term_sq);
            sum_val   = sum_val + term_sq + term_quad;
        end
    end
    return -sum_val;
endfunction

// General reward evaluation dispatcher
function automatic q16_t evaluate_reward(
    input reward_fn_t  fn_type,
    input policy_vec_t theta,
    input logic [2:0]  dim
);
    case (fn_type)
        FN_QUADRATIC:  return eval_fn_quadratic(theta, dim);
        FN_ROSENBROCK: return eval_fn_rosenbrock(theta);
        FN_SPHERE:     return eval_fn_sphere(theta, dim);
        FN_RASTRIGIN:  return eval_fn_rastrigin(theta, dim);
        default:       return eval_fn_quadratic(theta, dim);
    endcase
endfunction

`endif // NES_HELPERS_SVH
