// =============================================================================
// File Name   : adam_moment_engine.sv
// Module Name : adam_moment_engine
// Project     : Adaptive Moment Estimation (Adam) Accelerator (Solver #17)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined moment update and parameter stepping engine supporting:
//   1. Adam (m_t & v_t adaptive moment estimation with AdamW decoupled weight decay)
//   2. RMSProp (v_t second moment variance normalization)
//   3. Classical Momentum SGD (exponentially smoothed velocity m_t)
//   4. Pure SGD with L2 weight decay
// =============================================================================

`timescale 1ns / 1ps

import adam_types_pkg::*;
`include "adam_helpers.svh"

module adam_moment_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Problem Setup
    input  logic               start,
    input  opt_mode_t          opt_mode,      // Mode: ADAM, RMSPROP, MOMENTUM, SGD
    input  logic [2:0]         num_dims,      // N <= 4
    input  vec_t               g_in,          // Current gradient vector g_t
    input  vec_t               m_curr,        // Current 1st moment vector m_{t-1}
    input  vec_t               v_curr,        // Current 2nd moment vector v_{t-1}
    input  vec_t               theta_curr,    // Current parameter vector θ_{t-1}
    input  q16_t               alpha_lr,      // Learning rate α
    input  q16_t               beta1_val,     // Momentum decay β1 (e.g. 0.90)
    input  q16_t               beta2_val,     // Variance decay β2 (e.g. 0.99)
    input  q16_t               weight_decay,  // Weight decay penalty λ

    // Outputs
    output vec_t               m_next,        // Updated 1st moment vector m_t
    output vec_t               v_next,        // Updated 2nd moment vector v_t
    output vec_t               theta_next,    // Updated parameter vector θ_t
    output vec_t               dtheta_out,    // Step vector Δθ_t
    output q16_t               dtheta_norm_inf,// ||Δθ_t||_inf
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        MOM_IDLE       = 3'd0,
        MOM_CALC_ELEM  = 3'd1,
        MOM_WAIT_SQRT  = 3'd2,
        MOM_START_DIV  = 3'd3,
        MOM_WAIT_DIV   = 3'd4,
        MOM_NEXT_DIM   = 3'd5,
        MOM_DONE       = 3'd6
    } mom_state_t;

    mom_state_t state;

    logic [2:0] curr_dim;
    vec_t       m_acc, v_acc, th_acc, dth_acc;
    q16_t       max_dth_reg;

    assign m_next          = m_acc;
    assign v_next          = v_acc;
    assign theta_next      = th_acc;
    assign dtheta_out      = dth_acc;
    assign dtheta_norm_inf = max_dth_reg;

    // Hardware Square Root Unit
    logic sqrt_start;
    q16_t sqrt_val, sqrt_root;
    logic sqrt_done, sqrt_busy;

    q16_sqrt u_sqrt (
        .clk   (clk),
        .rst_n (rst_n),
        .start (sqrt_start),
        .val   (sqrt_val),
        .root  (sqrt_root),
        .done  (sqrt_done),
        .busy  (sqrt_busy)
    );

    // Hardware Divider Unit
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

    q16_t gi, mi, vi, thi;
    q16_t one_m_b1, one_m_b2;
    q16_t m_new, v_new, step_i, th_new;
    q16_t wd_term, g_reg;

    assign one_m_b1 = Q16_ONE - beta1_val;
    assign one_m_b2 = Q16_ONE - beta2_val;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= MOM_IDLE;
            curr_dim     <= 3'd0;
            m_acc        <= '0;
            v_acc        <= '0;
            th_acc       <= '0;
            dth_acc      <= '0;
            max_dth_reg  <= Q16_ZERO;
            sqrt_start   <= 1'b0;
            sqrt_val     <= Q16_ZERO;
            div_start    <= 1'b0;
            div_dividend <= Q16_ZERO;
            div_divisor  <= Q16_ZERO;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            sqrt_start <= 1'b0;
            div_start  <= 1'b0;

            case (state)
                MOM_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy        <= 1'b1;
                        curr_dim    <= 3'd0;
                        m_acc       <= m_curr;
                        v_acc       <= v_curr;
                        th_acc      <= theta_curr;
                        dth_acc     <= '0;
                        max_dth_reg <= Q16_ZERO;
                        state       <= MOM_CALC_ELEM;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                MOM_CALC_ELEM: begin
                    if (curr_dim < num_dims) begin
                        gi  = get_vec(g_in, 2'(curr_dim));
                        mi  = get_vec(m_curr, 2'(curr_dim));
                        vi  = get_vec(v_curr, 2'(curr_dim));
                        thi = get_vec(theta_curr, 2'(curr_dim));

                        case (opt_mode)
                            MODE_ADAM: begin
                                // m_t = β1 * m_{t-1} + (1 - β1) * g_t
                                m_new = q16_mul(beta1_val, mi) + q16_mul(one_m_b1, gi);
                                // v_t = β2 * v_{t-1} + (1 - β2) * g_t^2
                                v_new = q16_mul(beta2_val, vi) + q16_mul(one_m_b2, q16_mul(gi, gi));
                                m_acc <= set_vec(m_acc, 2'(curr_dim), m_new);
                                v_acc <= set_vec(v_acc, 2'(curr_dim), v_new);

                                // Trigger sqrt(v_t)
                                sqrt_val   <= v_new;
                                sqrt_start <= 1'b1;
                                state      <= MOM_WAIT_SQRT;
                            end

                            MODE_RMSPROP: begin
                                // v_t = β2 * v_{t-1} + (1 - β2) * g_t^2
                                v_new = q16_mul(beta2_val, vi) + q16_mul(one_m_b2, q16_mul(gi, gi));
                                v_acc <= set_vec(v_acc, 2'(curr_dim), v_new);

                                sqrt_val   <= v_new;
                                sqrt_start <= 1'b1;
                                state      <= MOM_WAIT_SQRT;
                            end

                            MODE_MOMENTUM: begin
                                // Standard PyTorch momentum: m_t = β1 * m_{t-1} + (1 - β1)*(g_t + λ*θ)
                                g_reg   = gi + q16_mul(weight_decay, thi);
                                m_new   = q16_mul(beta1_val, mi) + q16_mul(one_m_b1, g_reg);
                                m_acc   <= set_vec(m_acc, 2'(curr_dim), m_new);
                                step_i  = q16_mul(alpha_lr, m_new);
                                th_new  = thi - step_i;

                                th_acc  <= set_vec(th_acc, 2'(curr_dim), th_new);
                                dth_acc <= set_vec(dth_acc, 2'(curr_dim), step_i);
                                if (q16_abs(step_i) > max_dth_reg) begin
                                    max_dth_reg <= q16_abs(step_i);
                                end
                                state <= MOM_NEXT_DIM;
                            end

                            MODE_SGD: begin
                                g_reg   = gi + q16_mul(weight_decay, thi);
                                step_i  = q16_mul(alpha_lr, g_reg);
                                th_new  = thi - step_i;

                                th_acc  <= set_vec(th_acc, 2'(curr_dim), th_new);
                                dth_acc <= set_vec(dth_acc, 2'(curr_dim), step_i);
                                if (q16_abs(step_i) > max_dth_reg) begin
                                    max_dth_reg <= q16_abs(step_i);
                                end
                                state <= MOM_NEXT_DIM;
                            end

                            default: state <= MOM_NEXT_DIM;
                        endcase
                    end else begin
                        state <= MOM_DONE;
                    end
                end

                MOM_WAIT_SQRT: begin
                    if (sqrt_done) begin
                        state <= MOM_START_DIV;
                    end
                end

                MOM_START_DIV: begin
                    gi  = get_vec(g_in, 2'(curr_dim));
                    mi  = get_vec(m_acc, 2'(curr_dim));

                    if (opt_mode == MODE_ADAM) begin
                        div_dividend <= q16_mul(alpha_lr, mi);
                    end else begin
                        div_dividend <= q16_mul(alpha_lr, gi);
                    end
                    div_divisor  <= sqrt_root + Q16_NUM_EPS;
                    div_start    <= 1'b1;
                    state        <= MOM_WAIT_DIV;
                end

                MOM_WAIT_DIV: begin
                    if (div_done) begin
                        thi     = get_vec(theta_curr, 2'(curr_dim));
                        step_i  = (div_by_zero) ? Q16_ZERO : div_quotient;
                        wd_term = q16_mul(q16_mul(alpha_lr, weight_decay), thi);
                        th_new  = thi - step_i - wd_term;

                        th_acc  <= set_vec(th_acc, 2'(curr_dim), th_new);
                        dth_acc <= set_vec(dth_acc, 2'(curr_dim), step_i);
                        if (q16_abs(step_i) > max_dth_reg) begin
                            max_dth_reg <= q16_abs(step_i);
                        end
                        state <= MOM_NEXT_DIM;
                    end
                end

                MOM_NEXT_DIM: begin
                    curr_dim <= curr_dim + 1'b1;
                    state    <= MOM_CALC_ELEM;
                end

                MOM_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= MOM_IDLE;
                end

                default: state <= MOM_IDLE;
            endcase
        end
    end

endmodule
