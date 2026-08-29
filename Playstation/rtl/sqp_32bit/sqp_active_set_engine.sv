// =============================================================================
// File Name   : sqp_active_set_engine.sv
// Module Name : sqp_active_set_engine
// Project     : Sequential Quadratic Programming (SQP) Accelerator (Solver #7)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Active-Set Primal-Dual Constraint Classifier and KKT System Builder.
//   1. Identifies active boundary variables where gradient pushes into infeasible zone.
//   2. Builds projected KKT Hessian H_proj and gradient g_proj (clamping active p_i = 0).
//   3. Evaluates Lagrange multipliers μ_i (shadow prices) and free gradient norm.
// =============================================================================

`timescale 1ns / 1ps

import sqp_types_pkg::*;
`include "sqp_helpers.svh"

module sqp_active_set_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,
    input  logic [2:0]         num_params,
    input  vec_t               x_curr,
    input  vec_t               vec_g,
    input  mat_t               mat_h,
    input  box_bounds_t        bounds,

    // Outputs
    output active_mask_t       active_mask,      // Bit i=1 if variable i is on active boundary
    output mat_t               mat_h_proj,       // Projected KKT Hessian matrix
    output vec_t               vec_g_proj,       // Projected KKT gradient vector
    output vec_t               vec_mu,           // Lagrange multipliers μ_i
    output q16_t               g_free_norm_inf,  // Max free gradient component max_{i in free}(|g_i|)
    output logic               done,
    output logic               busy
);

    active_mask_t act_mask_reg;
    mat_t         h_proj_reg;
    vec_t         g_proj_reg;
    vec_t         mu_reg;
    q16_t         g_free_max_reg;

    assign active_mask     = act_mask_reg;
    assign mat_h_proj      = h_proj_reg;
    assign vec_g_proj      = g_proj_reg;
    assign vec_mu          = mu_reg;
    assign g_free_norm_inf = g_free_max_reg;

    q16_t xi, gi, li, ui;
    logic is_upper_act, is_lower_act, is_active;
    q16_t cur_abs_g;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            act_mask_reg   <= '0;
            h_proj_reg     <= '0;
            g_proj_reg     <= '0;
            mu_reg         <= '0;
            g_free_max_reg <= Q16_ZERO;
            done           <= 1'b0;
            busy           <= 1'b0;
        end else begin
            if (start) begin
                busy <= 1'b1;

                // 1. Identify Active Constraints & KKT Multipliers
                act_mask_reg   = 4'b0000;
                mu_reg         = '0;
                g_free_max_reg = 32'sd0;

                for (int i = 0; i < MAX_PARAMS; i++) begin
                    if (i < num_params) begin
                        xi = get_vec(x_curr, 2'(i));
                        gi = get_vec(vec_g, 2'(i));
                        li = get_vec(bounds.lower_bound, 2'(i));
                        ui = get_vec(bounds.upper_bound, 2'(i));

                        is_upper_act = (xi >= (ui - Q16_BOUND_TOL)) && (gi < 32'sd0); // Gradient pushes past upper bound
                        is_lower_act = (xi <= (li + Q16_BOUND_TOL)) && (gi > 32'sd0); // Gradient pushes past lower bound

                        if (is_upper_act) begin
                            act_mask_reg[i] = 1'b1;
                            mu_reg          = set_vec(mu_reg, 2'(i), -gi); // Multiplier μ_i = -g_i > 0
                        end else if (is_lower_act) begin
                            act_mask_reg[i] = 1'b1;
                            mu_reg          = set_vec(mu_reg, 2'(i), gi);  // Multiplier μ_i = g_i > 0
                        end else begin
                            act_mask_reg[i] = 1'b0;
                            cur_abs_g = (gi < 32'sd0) ? -gi : gi;
                            if (cur_abs_g > g_free_max_reg) g_free_max_reg = cur_abs_g;
                        end
                    end
                end

                // 2. Build Projected KKT Matrix H_proj and Gradient g_proj
                h_proj_reg = mat_h;
                g_proj_reg = vec_g;

                for (int i = 0; i < MAX_PARAMS; i++) begin
                    if (i < num_params) begin
                        if (act_mask_reg[i]) begin
                            // Variable i is active: Lock degree of freedom (p_i = 0)
                            g_proj_reg = set_vec(g_proj_reg, 2'(i), Q16_ZERO);

                            for (int k = 0; k < MAX_PARAMS; k++) begin
                                if (k == i) begin
                                    h_proj_reg = set_mat(h_proj_reg, 2'(i), 2'(k), Q16_ONE); // Pivot = 1.0
                                end else begin
                                    h_proj_reg = set_mat(h_proj_reg, 2'(i), 2'(k), Q16_ZERO);
                                    h_proj_reg = set_mat(h_proj_reg, 2'(k), 2'(i), Q16_ZERO);
                                end
                            end
                        end
                    end
                end

                done <= 1'b1;
                busy <= 1'b0;
            end else begin
                done <= 1'b0;
                busy <= 1'b0;
            end
        end
    end

endmodule
