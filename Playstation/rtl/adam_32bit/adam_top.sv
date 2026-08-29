// =============================================================================
// File Name   : adam_top.sv
// Module Name : adam_top
// Project     : Adaptive Moment Estimation (Adam) Accelerator (Solver #17)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Module for the Adam / RMSProp / Momentum SGD Neural Optimizer.
//   Coordinates gradient evaluations ∇f(θ_t), first/second moment tracking,
//   adaptive learning rate scaling, L2 weight decay, and convergence reporting.
// =============================================================================

`timescale 1ns / 1ps

import adam_types_pkg::*;
`include "adam_helpers.svh"

module adam_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Model Microcode Programming Port
    // -------------------------------------------------------------------------
    input  logic               prog_en,       // Microcode Write Enable
    input  logic [4:0]         prog_addr,     // Microcode Address (0..31)
    input  instr_t             prog_data,     // 32-bit Microcode Instruction

    // -------------------------------------------------------------------------
    // Interface 2: Algorithm Parameters & Controls
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start trigger
    input  opt_mode_t          opt_mode,      // Mode: ADAM, RMSPROP, MOMENTUM, SGD
    input  logic [2:0]         num_dims,      // Number of parameters N (1..4)
    input  q16_t               alpha_lr,      // Learning rate α
    input  q16_t               beta1_val,     // Momentum decay β1
    input  q16_t               beta2_val,     // Variance decay β2
    input  q16_t               weight_decay,  // L2 weight decay penalty λ
    input  vec_t               theta_init,    // Initial parameter vector θ_0
    input  q16_t               tolerance,     // Convergence threshold ε_tol
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 3: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               theta_optimal, // Optimized parameter vector θ*
    output q16_t               f_loss_optimal,// Final loss f(θ*)
    output q16_t               grad_norm_inf, // Final gradient norm ||g||_inf
    output q16_t               dtheta_norm_inf,// Final step norm ||Δθ||_inf
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        ADAM_IDLE            = 3'd0,
        ADAM_START_GRAD      = 3'd1,
        ADAM_WAIT_GRAD       = 3'd2,
        ADAM_WAIT_MOMENTS    = 3'd3,
        ADAM_EVAL_FINAL_LOSS = 3'd4,
        ADAM_WAIT_FINAL_LOSS = 3'd5,
        ADAM_DONE            = 3'd6
    } adam_state_t;

    adam_state_t state;

    // State Registers
    vec_t       theta_reg;   // Parameters θ_t
    vec_t       m_reg;       // First moment vector m_t
    vec_t       v_reg;       // Second moment vector v_t
    vec_t       g_prev_reg;  // Previous gradient vector g_{t-1}
    logic [7:0] iter_cnt;
    status_t    status_reg;
    q16_t       g_inf_reg;
    q16_t       dth_inf_reg;
    q16_t       f_loss_reg;

    // Latched Parameters
    opt_mode_t  mode_reg;
    logic [2:0] num_dims_reg;
    q16_t       alpha_reg;
    q16_t       b1_reg;
    q16_t       b2_reg;
    q16_t       wd_reg;
    q16_t       tol_reg;
    logic [7:0] max_iters_reg;

    // Gradient Engine Interconnect
    logic start_grad;
    vec_t grad_vec_out;
    q16_t grad_f_base;
    logic grad_done, grad_busy;

    adam_gradient_engine u_grad (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (start_grad),
        .num_dims   (num_dims_reg),
        .theta_in   (theta_reg),
        .vec_g_out  (grad_vec_out),
        .f_base_out (grad_f_base),
        .done       (grad_done),
        .busy       (grad_busy)
    );

    // Moment Engine Interconnect
    logic start_mom;
    vec_t mom_m_next;
    vec_t mom_v_next;
    vec_t mom_th_next;
    vec_t mom_dth_out;
    q16_t mom_dth_inf;
    logic mom_done, mom_busy;

    adam_moment_engine u_mom (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start_mom),
        .opt_mode       (mode_reg),
        .num_dims       (num_dims_reg),
        .g_in           (grad_vec_out),
        .m_curr         (m_reg),
        .v_curr         (v_reg),
        .theta_curr     (theta_reg),
        .alpha_lr       (alpha_reg),
        .beta1_val      (b1_reg),
        .beta2_val      (b2_reg),
        .weight_decay   (wd_reg),
        .m_next         (mom_m_next),
        .v_next         (mom_v_next),
        .theta_next     (mom_th_next),
        .dtheta_out     (mom_dth_out),
        .dtheta_norm_inf(mom_dth_inf),
        .done           (mom_done),
        .busy           (mom_busy)
    );

    // DFG Final Loss Interconnect
    logic start_dfg;
    vec_t dfg_x_in;
    q16_t dfg_f_out;
    logic dfg_done, dfg_busy;

    dfg_adam_engine u_dfg_final (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .num_dims   (num_dims_reg),
        .x_vec      (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    // Outputs
    assign theta_optimal   = theta_reg;
    assign f_loss_optimal  = f_loss_reg;
    assign grad_norm_inf   = g_inf_reg;
    assign dtheta_norm_inf = dth_inf_reg;
    assign iter_count      = iter_cnt;
    assign status          = status_reg;

    q16_t max_g, g_elem, g_dot;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ADAM_IDLE;
            theta_reg     <= '0;
            m_reg         <= '0;
            v_reg         <= '0;
            g_prev_reg    <= '0;
            iter_cnt      <= 8'd0;
            status_reg    <= STATUS_IDLE;
            g_inf_reg     <= Q16_ZERO;
            dth_inf_reg   <= Q16_ZERO;
            f_loss_reg    <= Q16_ZERO;
            mode_reg      <= MODE_ADAM;
            num_dims_reg  <= 3'd2;
            alpha_reg     <= Q16_ALPHA_DEF;
            b1_reg        <= Q16_BETA1_DEF;
            b2_reg        <= Q16_BETA2_DEF;
            wd_reg        <= Q16_ZERO;
            tol_reg       <= Q16_EPS_DEF;
            max_iters_reg <= 8'd40;
            start_grad    <= 1'b0;
            start_mom     <= 1'b0;
            start_dfg     <= 1'b0;
            dfg_x_in      <= '0;
            done          <= 1'b0;
            busy          <= 1'b0;
        end else begin
            start_grad <= 1'b0;
            start_mom  <= 1'b0;
            start_dfg  <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: ADAM_IDLE - Latch User Configuration & Initialization
                // -------------------------------------------------------------
                ADAM_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy          <= 1'b1;
                        mode_reg      <= opt_mode;
                        num_dims_reg  <= num_dims;
                        alpha_reg     <= (alpha_lr != Q16_ZERO)  ? alpha_lr  : Q16_ALPHA_DEF;
                        b1_reg        <= (beta1_val != Q16_ZERO) ? beta1_val : Q16_BETA1_DEF;
                        b2_reg        <= (beta2_val != Q16_ZERO) ? beta2_val : Q16_BETA2_DEF;
                        wd_reg        <= weight_decay;
                        tol_reg       <= (tolerance != Q16_ZERO) ? tolerance : Q16_EPS_DEF;
                        max_iters_reg <= max_iters;
                        theta_reg     <= theta_init;
                        m_reg         <= '0;
                        v_reg         <= '0;
                        g_prev_reg    <= '0;
                        iter_cnt      <= 8'd0;
                        g_inf_reg     <= Q16_ZERO;
                        dth_inf_reg   <= Q16_ZERO;
                        status_reg    <= STATUS_RUNNING;
                        state         <= ADAM_START_GRAD;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: ADAM_START_GRAD - Trigger Gradient Evaluation
                // -------------------------------------------------------------
                ADAM_START_GRAD: begin
                    start_grad <= 1'b1;
                    state      <= ADAM_WAIT_GRAD;
                end

                // -------------------------------------------------------------
                // STATE 2: ADAM_WAIT_GRAD - Check Convergence & Trigger Moments
                // -------------------------------------------------------------
                ADAM_WAIT_GRAD: begin
                    if (grad_done) begin
                        max_g = Q16_ZERO;
                        g_dot = Q16_ZERO;
                        for (int i = 0; i < MAX_PARAMS; i++) begin
                            if (i < num_dims_reg) begin
                                g_elem = q16_abs(get_vec(grad_vec_out, 2'(i)));
                                if (g_elem > max_g) begin
                                    max_g = g_elem;
                                end
                                g_dot = g_dot + q16_mul(get_vec(grad_vec_out, 2'(i)), get_vec(g_prev_reg, 2'(i)));
                            end
                        end
                        g_inf_reg <= max_g;

                        // Convergence check on gradient norm
                        if (max_g <= tol_reg) begin
                            status_reg <= STATUS_CONVERGED;
                            state      <= ADAM_EVAL_FINAL_LOSS;
                        end else if (iter_cnt >= max_iters_reg) begin
                            status_reg <= STATUS_MAX_ITERS;
                            state      <= ADAM_EVAL_FINAL_LOSS;
                        end else begin
                            // Adaptive Step Contracting on Ravine Oscillation
                            if (iter_cnt > 8'd0 && g_dot < Q16_ZERO) begin
                                alpha_reg <= alpha_reg - (alpha_reg >>> 2); // α ← 0.75 * α
                            end
                            g_prev_reg <= grad_vec_out;

                            start_mom <= 1'b1;
                            state     <= ADAM_WAIT_MOMENTS;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE 3: ADAM_WAIT_MOMENTS - Latch Updated Parameters & Step
                // -------------------------------------------------------------
                ADAM_WAIT_MOMENTS: begin
                    if (mom_done) begin
                        m_reg       <= mom_m_next;
                        v_reg       <= mom_v_next;
                        theta_reg   <= mom_th_next;
                        dth_inf_reg <= mom_dth_inf;
                        iter_cnt    <= iter_cnt + 1'b1;

                        if (mom_dth_inf <= tol_reg) begin
                            status_reg <= STATUS_CONVERGED;
                            state      <= ADAM_EVAL_FINAL_LOSS;
                        end else begin
                            state <= ADAM_START_GRAD;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE 4: ADAM_EVAL_FINAL_LOSS - Evaluate Final Training Loss
                // -------------------------------------------------------------
                ADAM_EVAL_FINAL_LOSS: begin
                    dfg_x_in  <= theta_reg;
                    start_dfg <= 1'b1;
                    state     <= ADAM_WAIT_FINAL_LOSS;
                end

                // -------------------------------------------------------------
                // STATE 5: ADAM_WAIT_FINAL_LOSS - Latch Objective Loss
                // -------------------------------------------------------------
                ADAM_WAIT_FINAL_LOSS: begin
                    if (dfg_done) begin
                        f_loss_reg <= dfg_f_out;
                        state      <= ADAM_DONE;
                    end
                end

                // -------------------------------------------------------------
                // STATE 6: ADAM_DONE - Assert Completion Strobe
                // -------------------------------------------------------------
                ADAM_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= ADAM_IDLE;
                end

                default: state <= ADAM_IDLE;
            endcase
        end
    end

endmodule
