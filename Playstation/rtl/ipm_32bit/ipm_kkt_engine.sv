// =============================================================================
// File Name   : ipm_kkt_engine.sv
// Module Name : ipm_kkt_engine
// Project     : Primal-Dual Interior Point Method (IPM) Accelerator (Solver #16)
// -----------------------------------------------------------------------------
// Description:
//   Evaluates KKT residuals, duality gap μ, scaling matrix Θ = Z S^-1,
//   augmented Hessian H_aug = Q + A^T Θ A, RHS g_aug, recovered slack/dual steps
//   (Δs, Δz), and fraction-to-the-boundary step lengths (α_p, α_d).
// =============================================================================

`timescale 1ns / 1ps

import ipm_types_pkg::*;
`include "ipm_helpers.svh"

module ipm_kkt_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Problem Setup
    input  logic               start_kkt,     // Phase 1: Build H_aug & g_aug
    input  logic               start_steps,   // Phase 2: Compute Δs, Δz, α_p, α_d given Δx
    input  logic [2:0]         num_vars,      // N <= 4
    input  logic [2:0]         num_cons,      // M <= 4
    input  mat_t               q_mat,         // Objective quadratic matrix Q (4x4)
    input  vec_t               c_vec,         // Objective linear cost vector c (4x1)
    input  mat_t               a_mat,         // Constraint matrix A (4x4)
    input  vec_t               b_vec,         // Constraint bounds b (4x1)
    input  vec_t               x_curr,        // Primal variable x (4x1)
    input  vec_t               s_curr,        // Slack variable s (4x1)
    input  vec_t               z_curr,        // Dual multiplier z (4x1)
    input  q16_t               sigma_val,     // Centering parameter σ
    input  q16_t               tau_val,       // Boundary factor τ
    input  vec_t               dx_in,         // Computed Δx from Cholesky solver

    // Phase 1 Outputs (to Cholesky Solver)
    output mat_t               h_aug_out,     // Augmented Hessian H_aug
    output vec_t               g_aug_out,     // Augmented RHS g_aug
    output q16_t               mu_out,        // Duality gap μ
    output q16_t               rp_norm_inf,   // ||r_p||_inf
    output q16_t               rd_norm_inf,   // ||r_d||_inf

    // Phase 2 Outputs (for Primal-Dual Step Integration)
    output vec_t               ds_out,        // Slack step Δs
    output vec_t               dz_out,        // Dual step Δz
    output q16_t               alpha_p_out,   // Primal step size α_p
    output q16_t               alpha_d_out,   // Dual step size α_d

    output logic               done,
    output logic               busy
);

    typedef enum logic [3:0] {
        KKT_IDLE          = 4'd0,
        KKT_EVAL_RESID    = 4'd1,
        KKT_CALC_THETA    = 4'd2,
        KKT_WAIT_THETA    = 4'd3,
        KKT_BUILD_HAUG    = 4'd4,
        KKT_DONE_P1       = 4'd5,
        KKT_RECOVER_DS    = 4'd6,
        KKT_CALC_DZ       = 4'd7,
        KKT_WAIT_DZ       = 4'd8,
        KKT_CALC_ALPHA_P  = 4'd9,
        KKT_WAIT_ALPHA_P  = 4'd10,
        KKT_CALC_ALPHA_D  = 4'd11,
        KKT_WAIT_ALPHA_D  = 4'd12,
        KKT_DONE_P2       = 4'd13
    } kkt_state_t;

    kkt_state_t state;

    vec_t r_p; // Primal residual b - A*x - s
    vec_t r_d; // Dual residual -(Q*x + c + A^T*z)
    q16_t mu_reg;
    q16_t rp_inf_reg;
    q16_t rd_inf_reg;

    vec_t theta_vec; // theta_i = z_i / s_i
    vec_t v_vec;     // v_i = (sigma*mu)/s_i - z_i - theta_i * r_p,i
    mat_t h_aug_reg;
    vec_t g_aug_reg;

    vec_t ds_reg;
    vec_t dz_reg;
    q16_t min_ap_reg;
    q16_t min_ad_reg;
    q16_t alpha_p_reg;
    q16_t alpha_d_reg;

    assign h_aug_out   = h_aug_reg;
    assign g_aug_out   = g_aug_reg;
    assign mu_out      = mu_reg;
    assign rp_norm_inf = rp_inf_reg;
    assign rd_norm_inf = rd_inf_reg;
    assign ds_out      = ds_reg;
    assign dz_out      = dz_reg;
    assign alpha_p_out = alpha_p_reg;
    assign alpha_d_out = alpha_d_reg;

    // Hardware Divider
    logic div_start;
    q16_t div_dividend, div_divisor, div_quotient;
    logic div_done, div_by_zero, div_busy;

    q16_divider u_div (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (div_start),
        .dividend   (div_dividend),
        .divisor    (div_divisor),
        .quotient   (div_quotient),
        .done       (div_done),
        .div_by_zero(div_by_zero),
        .busy       (div_busy)
    );

    logic [2:0] c_idx;
    q16_t ax_elem, qx_elem, atz_elem, atv_elem;
    q16_t s_i, z_i, rpi, rdi, th_i, max_rpi, max_rdi;
    q16_t sigma_mu, sz_sum;
    q16_t sum_h, a_ki, a_kj;
    q16_t dsi, dzi;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= KKT_IDLE;
            r_p          <= '0;
            r_d          <= '0;
            mu_reg       <= Q16_ZERO;
            rp_inf_reg   <= Q16_ZERO;
            rd_inf_reg   <= Q16_ZERO;
            theta_vec    <= '0;
            v_vec        <= '0;
            h_aug_reg    <= '0;
            g_aug_reg    <= '0;
            ds_reg       <= '0;
            dz_reg       <= '0;
            min_ap_reg   <= Q16_ONE;
            min_ad_reg   <= Q16_ONE;
            alpha_p_reg  <= Q16_ONE;
            alpha_d_reg  <= Q16_ONE;
            c_idx        <= 3'd0;
            div_start    <= 1'b0;
            div_dividend <= Q16_ZERO;
            div_divisor  <= Q16_ZERO;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            div_start <= 1'b0;

            case (state)
                KKT_IDLE: begin
                    done <= 1'b0;
                    if (start_kkt) begin
                        busy  <= 1'b1;
                        state <= KKT_EVAL_RESID;
                    end else if (start_steps) begin
                        busy  <= 1'b1;
                        state <= KKT_RECOVER_DS;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // PHASE 1: EVALUATE RESIDUALS & DUALITY GAP
                // -------------------------------------------------------------
                KKT_EVAL_RESID: begin
                    // 1. Primal residual: r_p = b - A*x - s
                    max_rpi = Q16_ZERO;
                    for (int i = 0; i < MAX_CONSTRAINTS; i++) begin
                        if (i < num_cons) begin
                            ax_elem = Q16_ZERO;
                            for (int j = 0; j < MAX_VARS; j++) begin
                                if (j < num_vars) begin
                                    ax_elem = ax_elem + q16_mul(get_mat(a_mat, 2'(i), 2'(j)), get_vec(x_curr, 2'(j)));
                                end
                            end
                            rpi = get_vec(b_vec, 2'(i)) - ax_elem - get_vec(s_curr, 2'(i));
                            r_p = set_vec(r_p, 2'(i), rpi);
                            if (q16_abs(rpi) > max_rpi) begin
                                max_rpi = q16_abs(rpi);
                            end
                        end
                    end
                    rp_inf_reg <= max_rpi;

                    // 2. Dual residual: r_d = -(Q*x + c + A^T*z)
                    max_rdi = Q16_ZERO;
                    for (int i = 0; i < MAX_VARS; i++) begin
                        if (i < num_vars) begin
                            qx_elem  = Q16_ZERO;
                            atz_elem = Q16_ZERO;
                            for (int j = 0; j < MAX_VARS; j++) begin
                                if (j < num_vars) begin
                                    qx_elem = qx_elem + q16_mul(get_mat(q_mat, 2'(i), 2'(j)), get_vec(x_curr, 2'(j)));
                                end
                            end
                            for (int k = 0; k < MAX_CONSTRAINTS; k++) begin
                                if (k < num_cons) begin
                                    atz_elem = atz_elem + q16_mul(get_mat(a_mat, 2'(k), 2'(i)), get_vec(z_curr, 2'(k)));
                                end
                            end
                            rdi = -(qx_elem + get_vec(c_vec, 2'(i)) + atz_elem);
                            r_d = set_vec(r_d, 2'(i), rdi);
                            if (q16_abs(rdi) > max_rdi) begin
                                max_rdi = q16_abs(rdi);
                            end
                        end
                    end
                    rd_inf_reg <= max_rdi;

                    // 3. Duality gap: mu = (s^T * z) / M
                    sz_sum = Q16_ZERO;
                    for (int k = 0; k < MAX_CONSTRAINTS; k++) begin
                        if (k < num_cons) begin
                            sz_sum = sz_sum + q16_mul(get_vec(s_curr, 2'(k)), get_vec(z_curr, 2'(k)));
                        end
                    end
                    case (num_cons)
                        3'd1: mu_reg <= sz_sum;
                        3'd2: mu_reg <= sz_sum >>> 1;
                        3'd3: mu_reg <= q16_mul(sz_sum, 32'h0000_5555); // * (1/3)
                        3'd4: mu_reg <= sz_sum >>> 2;
                        default: mu_reg <= sz_sum;
                    endcase

                    c_idx <= 3'd0;
                    state <= KKT_CALC_THETA;
                end

                // -------------------------------------------------------------
                // CALCULATE THETA_i = min(z_i / s_i, 512.0)
                // -------------------------------------------------------------
                KKT_CALC_THETA: begin
                    if (c_idx < num_cons) begin
                        div_dividend <= get_vec(z_curr, 2'(c_idx));
                        div_divisor  <= get_vec(s_curr, 2'(c_idx));
                        div_start    <= 1'b1;
                        state        <= KKT_WAIT_THETA;
                    end else begin
                        state <= KKT_BUILD_HAUG;
                    end
                end

                KKT_WAIT_THETA: begin
                    if (div_done) begin
                        // Clamp Theta to [0.001, 512.0] to prevent fixed-point matrix ill-conditioning
                        if (div_by_zero || div_quotient > 32'h0200_0000) begin
                            th_i = 32'h0200_0000; // 512.0
                        end else if (div_quotient < 32'h0000_0010) begin
                            th_i = 32'h0000_0010; // ~ 0.00024
                        end else begin
                            th_i = div_quotient;
                        end
                        theta_vec <= set_vec(theta_vec, 2'(c_idx), th_i);

                        sigma_mu  = q16_mul(sigma_val, mu_reg);
                        v_vec     <= set_vec(v_vec, 2'(c_idx), q16_mul(sigma_mu, th_i) - get_vec(z_curr, 2'(c_idx)) - q16_mul(th_i, get_vec(r_p, 2'(c_idx))));

                        c_idx <= c_idx + 1'b1;
                        state <= KKT_CALC_THETA;
                    end
                end

                // -------------------------------------------------------------
                // BUILD H_aug = Q + A^T Θ A and g_aug = -r_d + A^T v
                // -------------------------------------------------------------
                KKT_BUILD_HAUG: begin
                    // 1. H_aug = Q + A^T * Θ * A
                    for (int i = 0; i < MAX_VARS; i++) begin
                        for (int j = 0; j < MAX_VARS; j++) begin
                            sum_h = (i < num_vars && j < num_vars) ? get_mat(q_mat, 2'(i), 2'(j)) : Q16_ZERO;
                            for (int k = 0; k < MAX_CONSTRAINTS; k++) begin
                                if (k < num_cons) begin
                                    a_ki  = get_mat(a_mat, 2'(k), 2'(i));
                                    a_kj  = get_mat(a_mat, 2'(k), 2'(j));
                                    th_i  = get_vec(theta_vec, 2'(k));
                                    sum_h = sum_h + q16_mul(a_ki, q16_mul(th_i, a_kj));
                                end
                            end
                            h_aug_reg = set_mat(h_aug_reg, 2'(i), 2'(j), sum_h);
                        end
                    end

                    // 2. g_aug = -r_d + A^T * v
                    for (int i = 0; i < MAX_VARS; i++) begin
                        if (i < num_vars) begin
                            atv_elem = Q16_ZERO;
                            for (int k = 0; k < MAX_CONSTRAINTS; k++) begin
                                if (k < num_cons) begin
                                    atv_elem = atv_elem + q16_mul(get_mat(a_mat, 2'(k), 2'(i)), get_vec(v_vec, 2'(k)));
                                end
                            end
                            g_aug_reg = set_vec(g_aug_reg, 2'(i), -get_vec(r_d, 2'(i)) + atv_elem);
                        end else begin
                            g_aug_reg = set_vec(g_aug_reg, 2'(i), Q16_ZERO);
                        end
                    end

                    state <= KKT_DONE_P1;
                end

                KKT_DONE_P1: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= KKT_IDLE;
                end

                // -------------------------------------------------------------
                // PHASE 2: RECOVER Δs = r_p - A * Δx
                // -------------------------------------------------------------
                KKT_RECOVER_DS: begin
                    for (int i = 0; i < MAX_CONSTRAINTS; i++) begin
                        if (i < num_cons) begin
                            ax_elem = Q16_ZERO;
                            for (int j = 0; j < MAX_VARS; j++) begin
                                if (j < num_vars) begin
                                    ax_elem = ax_elem + q16_mul(get_mat(a_mat, 2'(i), 2'(j)), get_vec(dx_in, 2'(j)));
                                end
                            end
                            ds_reg = set_vec(ds_reg, 2'(i), get_vec(r_p, 2'(i)) - ax_elem);
                        end else begin
                            ds_reg = set_vec(ds_reg, 2'(i), Q16_ZERO);
                        end
                    end

                    c_idx <= 3'd0;
                    state <= KKT_CALC_DZ;
                end

                // -------------------------------------------------------------
                // CALCULATE Δz_i = (sigma*mu)/s_i - z_i - theta_i * Δs_i
                // -------------------------------------------------------------
                KKT_CALC_DZ: begin
                    if (c_idx < num_cons) begin
                        sigma_mu     = q16_mul(sigma_val, mu_reg);
                        div_dividend <= sigma_mu;
                        div_divisor  <= get_vec(s_curr, 2'(c_idx));
                        div_start    <= 1'b1;
                        state        <= KKT_WAIT_DZ;
                    end else begin
                        c_idx      <= 3'd0;
                        min_ap_reg <= Q16_ONE;
                        min_ad_reg <= Q16_ONE;
                        state      <= KKT_CALC_ALPHA_P;
                    end
                end

                KKT_WAIT_DZ: begin
                    if (div_done) begin
                        th_i   = get_vec(theta_vec, 2'(c_idx));
                        dzi    = div_quotient - get_vec(z_curr, 2'(c_idx)) - q16_mul(th_i, get_vec(ds_reg, 2'(c_idx)));
                        dz_reg <= set_vec(dz_reg, 2'(c_idx), dzi);

                        c_idx <= c_idx + 1'b1;
                        state <= KKT_CALC_DZ;
                    end
                end

                // -------------------------------------------------------------
                // FRACTION-TO-BOUNDARY: α_p = min(1.0, tau * min_{Δs < 0} (s_i / -Δs_i))
                // -------------------------------------------------------------
                KKT_CALC_ALPHA_P: begin
                    if (c_idx < num_cons) begin
                        dsi = get_vec(ds_reg, 2'(c_idx));
                        if (dsi < Q16_ZERO) begin
                            s_i          = get_vec(s_curr, 2'(c_idx));
                            div_dividend <= q16_mul(tau_val, s_i);
                            div_divisor  <= -dsi;
                            div_start    <= 1'b1;
                            state        <= KKT_WAIT_ALPHA_P;
                        end else begin
                            c_idx <= c_idx + 1'b1;
                            state <= KKT_CALC_ALPHA_P;
                        end
                    end else begin
                        c_idx <= 3'd0;
                        state <= KKT_CALC_ALPHA_D;
                    end
                end

                KKT_WAIT_ALPHA_P: begin
                    if (div_done) begin
                        if (div_quotient < min_ap_reg && div_quotient > Q16_ZERO) begin
                            min_ap_reg <= div_quotient;
                        end
                        c_idx <= c_idx + 1'b1;
                        state <= KKT_CALC_ALPHA_P;
                    end
                end

                // -------------------------------------------------------------
                // FRACTION-TO-BOUNDARY: α_d = min(1.0, tau * min_{Δz < 0} (z_i / -Δz_i))
                // -------------------------------------------------------------
                KKT_CALC_ALPHA_D: begin
                    if (c_idx < num_cons) begin
                        dzi = get_vec(dz_reg, 2'(c_idx));
                        if (dzi < Q16_ZERO) begin
                            z_i          = get_vec(z_curr, 2'(c_idx));
                            div_dividend <= q16_mul(tau_val, z_i);
                            div_divisor  <= -dzi;
                            div_start    <= 1'b1;
                            state        <= KKT_WAIT_ALPHA_D;
                        end else begin
                            c_idx <= c_idx + 1'b1;
                            state <= KKT_CALC_ALPHA_D;
                        end
                    end else begin
                        alpha_p_reg <= min_ap_reg;
                        alpha_d_reg <= min_ad_reg;
                        state       <= KKT_DONE_P2;
                    end
                end

                KKT_WAIT_ALPHA_D: begin
                    if (div_done) begin
                        if (div_quotient < min_ad_reg && div_quotient > Q16_ZERO) begin
                            min_ad_reg <= div_quotient;
                        end
                        c_idx <= c_idx + 1'b1;
                        state <= KKT_CALC_ALPHA_D;
                    end
                end

                KKT_DONE_P2: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= KKT_IDLE;
                end

                default: state <= KKT_IDLE;
            endcase
        end
    end

endmodule
