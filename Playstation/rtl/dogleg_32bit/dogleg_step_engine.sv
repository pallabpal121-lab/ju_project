// =============================================================================
// File Name   : dogleg_step_engine.sv
// Module Name : dogleg_step_engine
// Project     : Trust-Region Dogleg Non-Linear Optimizer Accelerator (Solver #13)
// -----------------------------------------------------------------------------
// Description:
//   Powell's Dogleg Step Computation Engine.
//   1. Computes Cauchy Point p_c = - (gᵀg / gᵀBg) * g using Scale-Invariant
//      gradient normalization (ensuring max |gnorm| <= 1.0) to completely
//      eliminate fixed-point arithmetic overflow.
//   2. Computes L2 norms ||p_gn||_2 and ||p_c||_2
//   3. Selects step along the piecewise linear Dogleg path:
//      - If ||p_gn|| <= Δ           : p_dl = p_gn (Full Gauss-Newton)
//      - Else if Δ <= ||p_c||       : p_dl = (Δ / ||p_c||) * p_c (Truncated Cauchy)
//      - Else (||p_c|| < Δ < ||p_gn||): p_dl = p_c + β * (p_gn - p_c)
//        where β is solved from ||p_c + β(p_gn - p_c)||² = Δ²
//   4. Evaluates predicted quadratic reduction:
//      Δm_pred = -gᵀp_dl - 0.5 * p_dlᵀ * B * p_dl
// =============================================================================

`timescale 1ns / 1ps

import dogleg_types_pkg::*;
`include "dogleg_helpers.svh"

module dogleg_step_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Inputs
    input  logic               start,
    input  logic [2:0]         num_params,    // Number of parameters N (1..4)
    input  vec_t               vec_g,         // Gradient vector g (N x 1)
    input  mat_t               mat_b,         // Hessian approximation B = JᵀJ (N x N)
    input  vec_t               vec_p_gn,      // Full Gauss-Newton step p_gn (N x 1)
    input  q16_t               delta_radius,  // Trust region radius Δ

    // Outputs
    output vec_t               vec_p_dl,      // Computed Dogleg step p_dl (N x 1)
    output q16_t               pred_reduct,   // Predicted model reduction Δm_pred
    output q16_t               p_norm_l2,     // L2 norm ||p_dl||_2
    output step_type_t         step_type,     // Classification of chosen step
    output logic               done,
    output logic               busy
);

    typedef enum logic [3:0] {
        DL_IDLE              = 4'd0,
        DL_CALC_G_BG         = 4'd1,
        DL_DIV_ALPHA_C       = 4'd2,
        DL_WAIT_ALPHA_C      = 4'd3,
        DL_SQRT_NORMS        = 4'd4,
        DL_WAIT_GN_SQRT      = 4'd5,
        DL_WAIT_C_SQRT       = 4'd6,
        DL_EVAL_BRANCH       = 4'd7,
        DL_DIV_CAUCHY_SCALE  = 4'd8,
        DL_WAIT_CAUCHY_SCALE = 4'd9,
        DL_SQRT_DISCRIM      = 4'd10,
        DL_WAIT_DISCRIM      = 4'd11,
        DL_WAIT_BETA         = 4'd12,
        DL_CALC_PRED_REDUCT  = 4'd13,
        DL_WAIT_FINAL_NORM   = 4'd14,
        DL_DONE              = 4'd15
    } dl_state_t;

    dl_state_t state;

    // Registers for intermediate math
    q16_t g_dot_g;
    q16_t g_dot_bg;
    q16_t alpha_c;
    vec_t p_c_vec;
    vec_t v_vec; // p_gn - p_c

    q16_t norm_sq_gn;
    q16_t norm_sq_c;
    q16_t norm_gn;
    q16_t norm_c;

    q16_t quad_a;
    q16_t quad_b;
    q16_t quad_c;
    q16_t discrim_d;
    q16_t sqrt_d_val;
    q16_t beta_val;

    vec_t p_dl_reg;
    q16_t pred_reduct_reg;
    q16_t p_norm_l2_reg;
    step_type_t step_type_reg;

    // Saturated Q16.16 multiplication to avoid wrap-around
    function automatic q16_t q16_mul_sat(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        if (prod[63:47] != 17'sd0 && prod[63:47] != 17'sh1FFFF) begin
            return prod[63] ? 32'sh8000_0000 : 32'sh7FFF_FFFF;
        end
        return prod[47:16];
    endfunction

    // Hardware Divider Submodule
    logic        div_start;
    q16_t        div_dividend, div_divisor, div_quotient;
    logic        div_done, div_by_zero, div_busy;

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

    // Hardware Sqrt Submodule
    logic        sqrt_start;
    q16_t        sqrt_val, sqrt_root;
    logic        sqrt_done, sqrt_busy;

    q16_sqrt u_sqrt (
        .clk   (clk),
        .rst_n (rst_n),
        .start (sqrt_start),
        .val   (sqrt_val),
        .root  (sqrt_root),
        .done  (sqrt_done),
        .busy  (sqrt_busy)
    );

    assign vec_p_dl    = p_dl_reg;
    assign pred_reduct = pred_reduct_reg;
    assign p_norm_l2   = p_norm_l2_reg;
    assign step_type   = step_type_reg;

    // Local variables for blocking vector construction
    q16_t  temp_acc, temp_val;
    q16_t  alpha_eff, beta_eff;
    vec_t  temp_v;
    q16_t  gnorm_vec [0:MAX_PARAMS-1];
    q16_t  bg_vec    [0:MAX_PARAMS-1];
    q16_t  bp_vec    [0:MAX_PARAMS-1];
    q16_t  max_g_elem;
    int    shift_k;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= DL_IDLE;
            done            <= 1'b0;
            busy            <= 1'b0;
            div_start       <= 1'b0;
            div_dividend    <= Q16_ZERO;
            div_divisor     <= Q16_ZERO;
            sqrt_start      <= 1'b0;
            sqrt_val        <= Q16_ZERO;
            g_dot_g         <= Q16_ZERO;
            g_dot_bg        <= Q16_ZERO;
            alpha_c         <= Q16_ZERO;
            p_c_vec         <= '0;
            v_vec           <= '0;
            norm_sq_gn      <= Q16_ZERO;
            norm_sq_c       <= Q16_ZERO;
            norm_gn         <= Q16_ZERO;
            norm_c          <= Q16_ZERO;
            quad_a          <= Q16_ZERO;
            quad_b          <= Q16_ZERO;
            quad_c          <= Q16_ZERO;
            discrim_d       <= Q16_ZERO;
            sqrt_d_val      <= Q16_ZERO;
            beta_val        <= Q16_ZERO;
            p_dl_reg        <= '0;
            pred_reduct_reg <= Q16_ZERO;
            p_norm_l2_reg   <= Q16_ZERO;
            step_type_reg   <= STEP_NONE;
        end else begin
            div_start  <= 1'b0;
            sqrt_start <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: DL_IDLE
                // -------------------------------------------------------------
                DL_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= DL_CALC_G_BG;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // 1. CALCULATE SCALE-INVARIANT gᵀg AND gᵀBg
                // -------------------------------------------------------------
                DL_CALC_G_BG: begin
                    // Find max component of g to determine shift factor
                    max_g_elem = Q16_ZERO;
                    for (int i = 0; i < MAX_PARAMS; i++) begin
                        if (i < num_params[1:0]) begin
                            temp_val = q16_abs(get_vec(vec_g, 2'(i)));
                            if (temp_val > max_g_elem) begin
                                max_g_elem = temp_val;
                            end
                        end
                    end

                    // Normalize gradient so max component <= 1.0 (32'h0001_0000)
                    if (max_g_elem > 32'h0800_0000) begin
                        shift_k = 12;
                    end else if (max_g_elem > 32'h0200_0000) begin
                        shift_k = 10;
                    end else if (max_g_elem > 32'h0080_0000) begin
                        shift_k = 8;
                    end else if (max_g_elem > 32'h0020_0000) begin
                        shift_k = 6;
                    end else if (max_g_elem > 32'h0008_0000) begin
                        shift_k = 4;
                    end else if (max_g_elem > 32'h0002_0000) begin
                        shift_k = 2;
                    end else begin
                        shift_k = 0;
                    end

                    // Create scaled normalized gradient g_norm = g >>> shift_k
                    for (int i = 0; i < MAX_PARAMS; i++) begin
                        if (i < num_params[1:0]) begin
                            gnorm_vec[i] = get_vec(vec_g, 2'(i)) >>> shift_k;
                        end else begin
                            gnorm_vec[i] = Q16_ZERO;
                        end
                    end

                    // Compute g_normᵀ * g_norm
                    temp_acc = Q16_ZERO;
                    for (int i = 0; i < MAX_PARAMS; i++) begin
                        if (i < num_params[1:0]) begin
                            temp_acc = temp_acc + q16_mul_sat(gnorm_vec[i], gnorm_vec[i]);
                        end
                    end
                    g_dot_g <= (temp_acc > Q16_ZERO) ? temp_acc : Q16_EPS_DEF;

                    // Compute B * g_norm: (Bg)_i = sum_j B_ij * gnorm_j
                    for (int i = 0; i < MAX_PARAMS; i++) begin
                        temp_val = Q16_ZERO;
                        if (i < num_params[1:0]) begin
                            for (int j = 0; j < MAX_PARAMS; j++) begin
                                if (j < num_params[1:0]) begin
                                    temp_val = temp_val + q16_mul_sat(get_mat(mat_b, 2'(i), 2'(j)), gnorm_vec[j]);
                                end
                            end
                        end
                        bg_vec[i] = temp_val;
                    end

                    // Compute g_normᵀ * (B * g_norm)
                    temp_acc = Q16_ZERO;
                    for (int i = 0; i < MAX_PARAMS; i++) begin
                        if (i < num_params[1:0]) begin
                            temp_acc = temp_acc + q16_mul_sat(gnorm_vec[i], bg_vec[i]);
                        end
                    end
                    g_dot_bg <= (temp_acc > Q16_ZERO) ? temp_acc : Q16_EPS_DEF;

                    state <= DL_DIV_ALPHA_C;
                end

                // -------------------------------------------------------------
                // 2. COMPUTE CAUCHY SCALAR α_c = (g_normᵀ g_norm) / (g_normᵀ B g_norm)
                // -------------------------------------------------------------
                DL_DIV_ALPHA_C: begin
                    if (g_dot_bg <= 32'sd0 || g_dot_g <= 32'sd0) begin
                        alpha_c <= Q16_ONE;
                        state   <= DL_WAIT_ALPHA_C;
                    end else begin
                        alpha_c      <= Q16_ZERO;
                        div_dividend <= g_dot_g;
                        div_divisor  <= g_dot_bg;
                        div_start    <= 1'b1;
                        state        <= DL_WAIT_ALPHA_C;
                    end
                end

                DL_WAIT_ALPHA_C: begin
                    if (div_done || div_by_zero || (alpha_c == Q16_ONE)) begin
                        alpha_eff = (alpha_c == Q16_ONE) ? Q16_ONE : ((div_quotient > Q16_ZERO) ? div_quotient : Q16_ONE);
                        alpha_c  <= alpha_eff;

                        // Compute Cauchy vector p_c = - α_c * g
                        temp_v   = '0;
                        temp_acc = Q16_ZERO;
                        for (int i = 0; i < MAX_PARAMS; i++) begin
                            if (i < num_params[1:0]) begin
                                temp_val = -q16_mul_sat(alpha_eff, get_vec(vec_g, 2'(i)));
                                temp_v   = set_vec(temp_v, 2'(i), temp_val);
                                temp_acc = temp_acc + q16_mul_sat(temp_val, temp_val);
                            end
                        end
                        p_c_vec   <= temp_v;
                        norm_sq_c <= temp_acc;

                        // Compute ||p_gn||²
                        temp_acc = Q16_ZERO;
                        for (int i = 0; i < MAX_PARAMS; i++) begin
                            if (i < num_params[1:0]) begin
                                temp_val = get_vec(vec_p_gn, 2'(i));
                                temp_acc = temp_acc + q16_mul_sat(temp_val, temp_val);
                            end
                        end
                        norm_sq_gn <= temp_acc;

                        state <= DL_SQRT_NORMS;
                    end
                end

                // -------------------------------------------------------------
                // 3. COMPUTE ||p_gn||_2 VIA HARDWARE SQRT
                // -------------------------------------------------------------
                DL_SQRT_NORMS: begin
                    sqrt_val   <= norm_sq_gn;
                    sqrt_start <= 1'b1;
                    state      <= DL_WAIT_GN_SQRT;
                end

                DL_WAIT_GN_SQRT: begin
                    if (sqrt_done) begin
                        norm_gn    <= sqrt_root;
                        sqrt_val   <= norm_sq_c;
                        sqrt_start <= 1'b1;
                        state      <= DL_WAIT_C_SQRT;
                    end
                end

                // -------------------------------------------------------------
                // 4. COMPUTE ||p_c||_2 VIA HARDWARE SQRT & EVALUATE BRANCH
                // -------------------------------------------------------------
                DL_WAIT_C_SQRT: begin
                    if (sqrt_done) begin
                        norm_c <= sqrt_root;
                        state  <= DL_EVAL_BRANCH;
                    end
                end

                DL_EVAL_BRANCH: begin
                    // Case 1: Gauss-Newton step within trust region (||p_gn|| <= Δ)
                    if (norm_gn <= delta_radius) begin
                        p_dl_reg      <= vec_p_gn;
                        step_type_reg <= STEP_GAUSS_NEWTON;
                        state         <= DL_CALC_PRED_REDUCT;
                    end
                    // Case 2: Cauchy point outside trust region (Δ <= ||p_c||)
                    else if (delta_radius <= norm_c) begin
                        step_type_reg <= STEP_CAUCHY_TRUNC;
                        state         <= DL_DIV_CAUCHY_SCALE;
                    end
                    // Case 3: Dogleg interpolation (||p_c|| < Δ < ||p_gn||)
                    else begin
                        step_type_reg <= STEP_DOGLEG_INTERP;

                        // Difference vector v = p_gn - p_c
                        temp_v = '0;
                        for (int i = 0; i < MAX_PARAMS; i++) begin
                            if (i < num_params[1:0]) begin
                                temp_v = set_vec(temp_v, 2'(i), get_vec(vec_p_gn, 2'(i)) - get_vec(p_c_vec, 2'(i)));
                            end
                        end
                        v_vec <= temp_v;

                        // Compute quadratic equation coefficients: ||p_c + β v||² = Δ²
                        // a = ||v||²
                        // b = 2 * (p_cᵀ v)
                        // c = ||p_c||² - Δ²
                        temp_acc = Q16_ZERO;
                        for (int i = 0; i < MAX_PARAMS; i++) begin
                            if (i < num_params[1:0]) begin
                                temp_val = get_vec(vec_p_gn, 2'(i)) - get_vec(p_c_vec, 2'(i));
                                temp_acc = temp_acc + q16_mul_sat(temp_val, temp_val);
                            end
                        end
                        quad_a <= (temp_acc > Q16_ZERO) ? temp_acc : Q16_EPS_DEF;

                        temp_acc = Q16_ZERO;
                        for (int i = 0; i < MAX_PARAMS; i++) begin
                            if (i < num_params[1:0]) begin
                                temp_acc = temp_acc + q16_mul_sat(get_vec(p_c_vec, 2'(i)), get_vec(vec_p_gn, 2'(i)) - get_vec(p_c_vec, 2'(i)));
                            end
                        end
                        quad_b <= (temp_acc <<< 1); // 2 * (p_cᵀ v)

                        quad_c <= norm_sq_c - q16_mul_sat(delta_radius, delta_radius);

                        state <= DL_SQRT_DISCRIM;
                    end
                end

                // -------------------------------------------------------------
                // CASE 2: TRUNCATED CAUCHY POINT: p_dl = (Δ / ||p_c||) * p_c
                // -------------------------------------------------------------
                DL_DIV_CAUCHY_SCALE: begin
                    div_dividend <= delta_radius;
                    div_divisor  <= (norm_c > Q16_ZERO) ? norm_c : Q16_EPS_DEF;
                    div_start    <= 1'b1;
                    state        <= DL_WAIT_CAUCHY_SCALE;
                end

                DL_WAIT_CAUCHY_SCALE: begin
                    if (div_done) begin
                        temp_v = '0;
                        for (int i = 0; i < MAX_PARAMS; i++) begin
                            if (i < num_params[1:0]) begin
                                temp_v = set_vec(temp_v, 2'(i), q16_mul_sat(div_quotient, get_vec(p_c_vec, 2'(i))));
                            end
                        end
                        p_dl_reg <= temp_v;
                        state    <= DL_CALC_PRED_REDUCT;
                    end
                end

                // -------------------------------------------------------------
                // CASE 3: QUADRATIC DOGLEG INTERPOLATION ROOT β
                // Discriminant D = b² - 4ac = b² + 4a(Δ² - ||p_c||²) > 0
                // -------------------------------------------------------------
                DL_SQRT_DISCRIM: begin
                    temp_val  = q16_mul_sat(quad_b, quad_b) - (q16_mul_sat(quad_a, quad_c) <<< 2);
                    discrim_d <= (temp_val > Q16_ZERO) ? temp_val : Q16_ZERO;

                    sqrt_val   <= (temp_val > Q16_ZERO) ? temp_val : Q16_ZERO;
                    sqrt_start <= 1'b1;
                    state      <= DL_WAIT_DISCRIM;
                end

                DL_WAIT_DISCRIM: begin
                    if (sqrt_done) begin
                        sqrt_d_val   <= sqrt_root;
                        div_dividend <= -quad_b + sqrt_root;
                        div_divisor  <= (quad_a <<< 1); // 2 * a
                        div_start    <= 1'b1;
                        state        <= DL_WAIT_BETA;
                    end
                end

                DL_WAIT_BETA: begin
                    if (div_done) begin
                        // Clamp β to [0.0, 1.0]
                        if (div_quotient < Q16_ZERO) begin
                            beta_eff = Q16_ZERO;
                        end else if (div_quotient > Q16_ONE) begin
                            beta_eff = Q16_ONE;
                        end else begin
                            beta_eff = div_quotient;
                        end
                        beta_val <= beta_eff;

                        // p_dl = p_c + β * (p_gn - p_c)
                        temp_v = '0;
                        for (int i = 0; i < MAX_PARAMS; i++) begin
                            if (i < num_params[1:0]) begin
                                temp_val = get_vec(p_c_vec, 2'(i)) + q16_mul_sat(beta_eff, get_vec(v_vec, 2'(i)));
                                temp_v   = set_vec(temp_v, 2'(i), temp_val);
                            end
                        end
                        p_dl_reg <= temp_v;

                        state <= DL_CALC_PRED_REDUCT;
                    end
                end

                // -------------------------------------------------------------
                // 5. EVALUATE PREDICTED MODEL REDUCTION:
                //    Δm_pred = -gᵀp_dl - 0.5 * p_dlᵀ * B * p_dl
                // -------------------------------------------------------------
                DL_CALC_PRED_REDUCT: begin
                    // Compute B * p_dl vector
                    for (int i = 0; i < MAX_PARAMS; i++) begin
                        temp_val = Q16_ZERO;
                        if (i < num_params[1:0]) begin
                            for (int j = 0; j < MAX_PARAMS; j++) begin
                                if (j < num_params[1:0]) begin
                                    temp_val = temp_val + q16_mul_sat(get_mat(mat_b, 2'(i), 2'(j)), get_vec(p_dl_reg, 2'(j)));
                                end
                            end
                        end
                        bp_vec[i] = temp_val;
                    end

                    // Compute p_dlᵀ * B * p_dl
                    temp_acc = Q16_ZERO;
                    for (int i = 0; i < MAX_PARAMS; i++) begin
                        if (i < num_params[1:0]) begin
                            temp_acc = temp_acc + q16_mul_sat(get_vec(p_dl_reg, 2'(i)), bp_vec[i]);
                        end
                    end

                    // Compute gᵀ * p_dl
                    temp_val = Q16_ZERO;
                    for (int i = 0; i < MAX_PARAMS; i++) begin
                        if (i < num_params[1:0]) begin
                            temp_val = temp_val + q16_mul_sat(get_vec(vec_g, 2'(i)), get_vec(p_dl_reg, 2'(i)));
                        end
                    end

                    pred_reduct_reg <= -temp_val - (temp_acc >>> 1);

                    // Compute ||p_dl||²
                    temp_acc = Q16_ZERO;
                    for (int i = 0; i < MAX_PARAMS; i++) begin
                        if (i < num_params[1:0]) begin
                            temp_acc = temp_acc + q16_mul_sat(get_vec(p_dl_reg, 2'(i)), get_vec(p_dl_reg, 2'(i)));
                        end
                    end

                    sqrt_val   <= temp_acc;
                    sqrt_start <= 1'b1;
                    state      <= DL_WAIT_FINAL_NORM;
                end

                DL_WAIT_FINAL_NORM: begin
                    if (sqrt_done) begin
                        p_norm_l2_reg <= sqrt_root;
                        state         <= DL_DONE;
                    end
                end

                // -------------------------------------------------------------
                // STATE: DL_DONE
                // -------------------------------------------------------------
                DL_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= DL_IDLE;
                end

                default: state <= DL_IDLE;
            endcase
        end
    end

endmodule
