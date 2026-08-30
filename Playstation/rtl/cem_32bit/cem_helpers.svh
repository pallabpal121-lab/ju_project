// =============================================================================
// File Name   : cem_helpers.svh
// Project     : Cross-Entropy Method (CEM) Accelerator (Solver #31)
// -----------------------------------------------------------------------------
// Description:
//   Sample array indexing, parameter accessors, bounds clamping, and inlined
//   objective fitness evaluators (Quadratic, Rosenbrock, Sphere, Rastrigin).
// =============================================================================

`ifndef CEM_HELPERS_SVH
`define CEM_HELPERS_SVH

import cem_types_pkg::*;

// Extract parameter from sample array: [s_idx][dim_idx] (idx = s*4 + dim)
function automatic q16_t get_sample_param(input sample_arr_t sarr, input logic [2:0] s_idx, input logic [1:0] dim_idx);
    logic [4:0] idx;
    idx = {s_idx, dim_idx};
    return sarr[idx];
endfunction

// Set parameter in sample array: [s_idx][dim_idx]
function automatic sample_arr_t set_sample_param(input sample_arr_t sarr, input logic [2:0] s_idx, input logic [1:0] dim_idx, input q16_t val);
    sample_arr_t res;
    logic [4:0] idx;
    res = sarr;
    idx = {s_idx, dim_idx};
    res[idx] = val;
    return res;
endfunction

// Extract complete parameter vector from sample array
function automatic param_vec_t get_sample_vec(input sample_arr_t sarr, input logic [2:0] s_idx);
    param_vec_t res;
    for (int d = 0; d < MAX_DIM; d++) begin
        res[d] = get_sample_param(sarr, s_idx, 2'(d));
    end
    return res;
endfunction

// Set complete parameter vector in sample array
function automatic sample_arr_t set_sample_vec(input sample_arr_t sarr, input logic [2:0] s_idx, input param_vec_t pvec);
    sample_arr_t res;
    res = sarr;
    for (int d = 0; d < MAX_DIM; d++) begin
        res = set_sample_param(res, s_idx, 2'(d), pvec[d]);
    end
    return res;
endfunction

// Clamp parameter to hyperbox bounds [lb, ub]
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

// Quadratic Fitness: f(x) = (x0 - 3.0)^2 + 2(x1 - 4.0)^2
function automatic q16_t eval_fn_quadratic(input param_vec_t x, input logic [2:0] dim);
    q16_t diff0, diff1, term0, term1;
    diff0 = x[0] - 32'h0003_0000; // x0 - 3.0
    diff1 = x[1] - 32'h0004_0000; // x1 - 4.0
    term0 = q16_mul(diff0, diff0);
    term1 = q16_mul(diff1, diff1);
    term1 = q16_mul(32'h0002_0000, term1); // 2 * (x1 - 4.0)^2
    return term0 + term1;
endfunction

// Rosenbrock Valley Fitness: f(x) = 10(x1 - x0^2)^2 + (1 - x0)^2
function automatic q16_t eval_fn_rosenbrock(input param_vec_t x);
    q16_t x0_sq, diff_y, term_curv, diff_x0, term_lin;
    x0_sq     = q16_mul(x[0], x[0]);
    diff_y    = x[1] - x0_sq;
    term_curv = q16_mul(diff_y, diff_y);
    term_curv = q16_mul(32'h000A_0000, term_curv); // 10 * (x1 - x0^2)^2

    diff_x0   = 32'h0001_0000 - x[0]; // 1 - x0
    term_lin  = q16_mul(diff_x0, diff_x0);
    return term_curv + term_lin;
endfunction

// Sphere Fitness: f(x) = ∑ (x_d)^2
function automatic q16_t eval_fn_sphere(input param_vec_t x, input logic [2:0] dim);
    q16_t sum_val, term;
    sum_val = 32'sd0;
    for (int d = 0; d < MAX_DIM; d++) begin
        if (d < dim) begin
            term    = q16_mul(x[d], x[d]);
            sum_val = sum_val + term;
        end
    end
    return sum_val;
endfunction

// Multi-Modal Fitness: f(x) = ∑ (x_d^2 + x_d^4)
function automatic q16_t eval_fn_rastrigin(input param_vec_t x, input logic [2:0] dim);
    q16_t sum_val, term_sq, term_quad;
    sum_val = 32'sd0;
    for (int d = 0; d < MAX_DIM; d++) begin
        if (d < dim) begin
            term_sq   = q16_mul(x[d], x[d]);
            term_quad = q16_mul(term_sq, term_sq);
            sum_val   = sum_val + term_sq + term_quad;
        end
    end
    return sum_val;
endfunction

// General fitness evaluation dispatcher
function automatic q16_t evaluate_fitness(
    input fitness_fn_t fn_type,
    input param_vec_t  x,
    input logic [2:0]  dim
);
    case (fn_type)
        FN_QUADRATIC:  return eval_fn_quadratic(x, dim);
        FN_ROSENBROCK: return eval_fn_rosenbrock(x);
        FN_SPHERE:     return eval_fn_sphere(x, dim);
        FN_RASTRIGIN:  return eval_fn_rastrigin(x, dim);
        default:       return eval_fn_quadratic(x, dim);
    endcase
endfunction

`endif // CEM_HELPERS_SVH
