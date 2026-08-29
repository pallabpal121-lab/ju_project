// =============================================================================
// File Name   : pgd_top.sv
// Module Name : pgd_top
// Project     : Projected Gradient Descent (PGD) Accelerator (Solver #12)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Top-level SoC Controller for the Projected Gradient Descent Accelerator:
//     1. Computes numerical gradient g_k = grad_f(x_k) via zero-cost bit-shifts
//     2. Evaluates unconstrained descent step y_{k+1} = x_k - alpha * g_k
//     3. Projects y_{k+1} into convex set C: x_{k+1} = Pi_C(y_{k+1})
//     4. Terminates when ||x_{k+1} - x_k||_inf <= tolerance
// =============================================================================

`timescale 1ns / 1ps

import pgd_types_pkg::*;
`include "pgd_helpers.svh"

module pgd_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Programming Interface (DFG microcode instructions)
    // -------------------------------------------------------------------------
    input  logic               prog_en,
    input  logic [4:0]         prog_addr,
    input  instr_t             prog_data,

    // -------------------------------------------------------------------------
    // Interface 1: Problem Parameters & Configuration
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start strobe
    input  logic [2:0]         num_params,    // Number of variables N (1..4)
    input  vec_t               x_init,        // Initial starting vector x_0
    input  proj_mode_t         proj_mode,     // Projection geometry selector
    input  vec_t               box_lower,     // Box lower bounds l (for PROJ_BOX)
    input  vec_t               box_upper,     // Box upper bounds u (for PROJ_BOX)
    input  q16_t               ball_radius,   // L2 Ball radius R (for PROJ_L2_BALL)
    input  q16_t               step_size,     // Gradient step size alpha
    input  q16_t               tolerance,     // Convergence threshold on ||delta x||_inf
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 2: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,     // Converged constrained optimal vector x*
    output q16_t               cost_optimal,  // Final objective value f(x*)
    output q16_t               g_norm_inf,    // Gradient norm ||grad_f(x*)||_inf
    output q16_t               delta_x_norm,  // Final step shift ||x_{k+1} - x_k||_inf
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while solving
);

    // -------------------------------------------------------------------------
    // Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        TOP_IDLE         = 4'd0,
        TOP_INIT_PROJ    = 4'd1,
        TOP_INIT_WAIT    = 4'd2,
        TOP_GRAD_START   = 4'd3,
        TOP_GRAD_WAIT    = 4'd4,
        TOP_STEP_PROJ    = 4'd5,
        TOP_PROJ_WAIT    = 4'd6,
        TOP_CHECK_CONV   = 4'd7,
        TOP_DONE         = 4'd8
    } top_state_t;

    top_state_t state;

    // Registers
    logic [2:0] num_params_reg;
    proj_mode_t proj_mode_reg;
    vec_t       box_low_reg, box_up_reg;
    q16_t       radius_reg;
    q16_t       alpha_reg;
    q16_t       tol_reg;
    logic [7:0] max_iters_reg;

    vec_t       x_cur, x_next;
    vec_t       g_cur;
    q16_t       f_cur;
    vec_t       unconstrained_y;
    vec_t       delta_x_vec;

    // Gradient Engine Interconnect
    logic start_grad;
    vec_t engine_grad_out;
    q16_t engine_f0_out;
    logic grad_done, grad_busy;

    pgd_gradient_engine u_grad (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (start_grad),
        .num_params (num_params_reg),
        .param_in   (x_cur),
        .grad_out   (engine_grad_out),
        .f0_out     (engine_f0_out),
        .done       (grad_done),
        .busy       (grad_busy)
    );

    // Projection Engine Interconnect
    logic start_proj;
    vec_t proj_in_vec;
    vec_t proj_out_vec;
    logic proj_done, proj_busy;

    pgd_projection_engine u_proj (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start_proj),
        .proj_mode  (proj_mode_reg),
        .num_params (num_params_reg),
        .x_in       (proj_in_vec),
        .box_lower  (box_low_reg),
        .box_upper  (box_up_reg),
        .ball_radius(radius_reg),
        .x_out      (proj_out_vec),
        .done       (proj_done),
        .busy       (proj_busy)
    );

    q16_t step_prod;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= TOP_IDLE;
            num_params_reg  <= 3'd2;
            proj_mode_reg   <= PROJ_NONE;
            box_low_reg     <= '0;
            box_up_reg      <= '0;
            radius_reg      <= Q16_ONE;
            alpha_reg       <= Q16_ALPHA_DEF;
            tol_reg         <= Q16_EPS_DEF;
            max_iters_reg   <= 8'd50;
            iter_count      <= 8'd0;
            status          <= STATUS_IDLE;
            done            <= 1'b0;
            busy            <= 1'b0;
            x_optimal       <= '0;
            cost_optimal    <= Q16_ZERO;
            g_norm_inf      <= Q16_ZERO;
            delta_x_norm    <= Q16_ZERO;
            x_cur           <= '0;
            x_next          <= '0;
            g_cur           <= '0;
            f_cur           <= Q16_ZERO;
            unconstrained_y <= '0;
            delta_x_vec     <= '0;
            start_grad      <= 1'b0;
            start_proj      <= 1'b0;
            proj_in_vec     <= '0;
        end else begin
            start_grad <= 1'b0;
            start_proj <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: TOP_IDLE
                // -------------------------------------------------------------
                TOP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy           <= 1'b1;
                        num_params_reg <= (num_params != 3'd0) ? num_params : 3'd2;
                        proj_mode_reg  <= proj_mode;
                        box_low_reg    <= box_lower;
                        box_up_reg     <= box_upper;
                        radius_reg     <= (ball_radius != Q16_ZERO) ? ball_radius : Q16_ONE;
                        alpha_reg      <= (step_size != Q16_ZERO)   ? step_size   : Q16_ALPHA_DEF;
                        tol_reg        <= (tolerance != Q16_ZERO)   ? tolerance   : Q16_EPS_DEF;
                        max_iters_reg  <= (max_iters != 8'd0)       ? max_iters   : 8'd50;
                        iter_count     <= 8'd0;
                        status         <= STATUS_RUNNING;

                        // Initial feasibility projection on x_init
                        proj_in_vec <= x_init;
                        start_proj  <= 1'b1;
                        state       <= TOP_INIT_WAIT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                TOP_INIT_WAIT: begin
                    if (proj_done) begin
                        x_cur      <= proj_out_vec;
                        start_grad <= 1'b1;
                        state      <= TOP_GRAD_WAIT;
                    end
                end

                // -------------------------------------------------------------
                // 1. GRADIENT EVALUATION: g_k = grad_f(x_k)
                // -------------------------------------------------------------
                TOP_GRAD_START: begin
                    start_grad <= 1'b1;
                    state      <= TOP_GRAD_WAIT;
                end

                TOP_GRAD_WAIT: begin
                    if (grad_done) begin
                        g_cur <= engine_grad_out;
                        f_cur <= engine_f0_out;

                        // Unconstrained gradient descent step: y = x - alpha * g
                        for (int k = 0; k < MAX_PARAMS; k++) begin
                            if (k < num_params_reg) begin
                                step_prod = q16_t'((64'(alpha_reg) * 64'(get_vec(engine_grad_out, 2'(k)))) >>> 16);
                                unconstrained_y = set_vec(unconstrained_y, 2'(k), get_vec(x_cur, 2'(k)) - step_prod);
                            end
                        end

                        // Trigger hardware projection on y
                        proj_in_vec <= unconstrained_y;
                        start_proj  <= 1'b1;
                        state       <= TOP_PROJ_WAIT;
                    end
                end

                // -------------------------------------------------------------
                // 2. HARDWARE PROJECTION: x_{k+1} = Pi_C(y)
                // -------------------------------------------------------------
                TOP_PROJ_WAIT: begin
                    if (proj_done) begin
                        x_next <= proj_out_vec;
                        state  <= TOP_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // 3. CONVERGENCE CHECK: ||x_{k+1} - x_k||_inf <= tolerance
                // -------------------------------------------------------------
                TOP_CHECK_CONV: begin
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params_reg) begin
                            delta_x_vec = set_vec(delta_x_vec, 2'(k), get_vec(x_next, 2'(k)) - get_vec(x_cur, 2'(k)));
                        end
                    end

                    delta_x_norm <= q16_norm_inf(delta_x_vec, num_params_reg);
                    g_norm_inf   <= q16_norm_inf(g_cur, num_params_reg);

                    // Condition 1: Converged if step displacement <= tolerance
                    if (q16_norm_inf(delta_x_vec, num_params_reg) <= tol_reg) begin
                        x_optimal    <= x_next;
                        cost_optimal <= f_cur;
                        status       <= STATUS_CONVERGED;
                        state        <= TOP_DONE;

                    // Condition 2: Max iterations reached
                    end else if (iter_count >= max_iters_reg) begin
                        x_optimal    <= x_next;
                        cost_optimal <= f_cur;
                        status       <= STATUS_MAX_ITERS;
                        state        <= TOP_DONE;

                    // Condition 3: Continue PGD loop
                    end else begin
                        x_cur      <= x_next;
                        iter_count <= iter_count + 1'b1;
                        state      <= TOP_GRAD_START;
                    end
                end

                // -------------------------------------------------------------
                // STATE: TOP_DONE
                // -------------------------------------------------------------
                TOP_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= TOP_IDLE;
                end

                default: state <= TOP_IDLE;
            endcase
        end
    end

endmodule
