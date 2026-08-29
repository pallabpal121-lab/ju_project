// =============================================================================
// File Name   : newton_2nd_order_64bit_top.sv
// Module Name : newton_2nd_order_64bit_top
// Project     : Universal Newton 2nd-Order Optimization Accelerator (64-Bit)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This is the TOP-LEVEL SoC module of the 64-Bit Universal Newton 2nd-Order
//   Accelerator operating in Q32.32 signed fixed-point arithmetic.
//
// Integrated Submodules:
//   1. derivative_engine_64bit : Computes 64-bit gradient & Hessian via finite differences.
//   2. q32_divider             : 96-cycle 64-bit linear step solver: Δx = -num / den.
//   3. Step Scaler (128-bit)   : Scales update step: α * Δx.
//   4. Master 64-bit Loop FSM  : Manages multi-iteration convergence loop.
// =============================================================================

`timescale 1ns / 1ps

import newton_types_64bit_pkg::*;

module newton_2nd_order_64bit_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: 64-Bit Equation Microcode Programming Port
    // -------------------------------------------------------------------------
    input  logic               prog_en,       // Microcode Write Enable
    input  logic [5:0]         prog_addr,     // Microcode Memory Address (0..63)
    input  instr64_t           prog_data,     // 64-bit Microcode Instruction Word

    // -------------------------------------------------------------------------
    // Interface 2: Optimization Parameters & Control
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle pulse to launch optimization
    input  q32_t               x_init,        // Initial guess x_0 (Q32.32 signed)
    input  q32_t               tolerance,     // Convergence threshold ε (Q32.32 signed)
    input  q32_t               step_alpha,    // Step size / learning rate α (Q32.32 signed)
    input  q32_t               lambda_reg,    // Regularization damping factor λ (Q32.32 signed)
    input  logic [7:0]         max_iters,     // Safety limit on max iterations (e.g. 50)

    // -------------------------------------------------------------------------
    // Interface 3: Results & Status Outputs
    // -------------------------------------------------------------------------
    output q32_t               x_optimal,     // Converged optimal point x* (Q32.32)
    output q32_t               f_optimal,     // Final function value f(x*) (Q32.32)
    output q32_t               g_final,       // Final gradient norm g(x*) (Q32.32)
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Status Code: CONVERGED, MAX_ITERS, SINGULAR, etc.
    output logic               done,          // 1-cycle completion pulse
    output logic               busy           // High while optimization loop is running
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        TOP_IDLE        = 3'd0, // Idle
        TOP_START_DERIV = 3'd1, // Launch derivative engine
        TOP_WAIT_DERIV  = 3'd2, // Wait for derivative sampling
        TOP_CHECK_CONV  = 3'd3, // Check |g| <= tol & max iters
        TOP_START_SOLVE = 3'd4, // Trigger 96-cycle divider
        TOP_WAIT_SOLVE  = 3'd5, // Wait for divider
        TOP_UPDATE_X    = 3'd6, // Update x = x + alpha * Δx
        TOP_DONE        = 3'd7  // Done
    } top_state_t;

    top_state_t state;

    // -------------------------------------------------------------------------
    // Configuration & State Registers
    // -------------------------------------------------------------------------
    q32_t       x_reg;
    q32_t       tol_reg;
    q32_t       alpha_reg;
    q32_t       lambda_reg_in;
    logic [7:0] max_iters_reg;

    // -------------------------------------------------------------------------
    // Derivative Engine Interconnect Signals
    // -------------------------------------------------------------------------
    logic deriv_start;
    q32_t deriv_f_val;
    q32_t deriv_grad;
    q32_t deriv_step_num;
    q32_t deriv_step_den;
    logic deriv_done;
    logic deriv_busy;

    // -------------------------------------------------------------------------
    // Solver / Divider Interconnect Signals
    // -------------------------------------------------------------------------
    logic div_start;
    q32_t div_dividend;
    q32_t div_divisor;
    q32_t div_quotient;
    logic div_done;
    logic div_by_zero;
    logic div_busy;

    // -------------------------------------------------------------------------
    // 64-Bit Fixed-Point Step Scaler: scaled_step = alpha * div_quotient
    // 128-bit product sliced at [95:32]
    // -------------------------------------------------------------------------
    logic signed [127:0] scaled_step_128;
    q32_t                scaled_step;
    assign scaled_step_128 = 128'(alpha_reg) * 128'(div_quotient);
    assign scaled_step     = scaled_step_128[95:32];

    // Absolute value of gradient for convergence testing: |g(x)|
    q32_t abs_grad;
    assign abs_grad = (deriv_grad[63]) ? -deriv_grad : deriv_grad;

    // -------------------------------------------------------------------------
    // Instantiate 64-Bit Derivative Engine
    // -------------------------------------------------------------------------
    derivative_engine_64bit u_deriv_engine (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (deriv_start),
        .x_curr     (x_reg),
        .lambda_reg (lambda_reg_in),
        .f_val      (deriv_f_val),
        .gradient   (deriv_grad),
        .step_num   (deriv_step_num),
        .step_den   (deriv_step_den),
        .done       (deriv_done),
        .busy       (deriv_busy)
    );

    // -------------------------------------------------------------------------
    // Instantiate 64-Bit Step Solver / Divider (96-cycle Restoring Divider)
    // -------------------------------------------------------------------------
    q32_divider u_solver (
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

    // -------------------------------------------------------------------------
    // Master Optimization Loop State Machine
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= TOP_IDLE;
            x_reg         <= Q32_ZERO;
            tol_reg       <= Q32_EPS_DEF;
            alpha_reg     <= Q32_ONE;
            lambda_reg_in <= Q32_LAMBDA_DEF;
            max_iters_reg <= 8'd50;
            iter_count    <= 8'd0;
            status        <= STATUS_IDLE;
            done          <= 1'b0;
            busy          <= 1'b0;
            x_optimal     <= Q32_ZERO;
            f_optimal     <= Q32_ZERO;
            g_final       <= Q32_ZERO;
            deriv_start   <= 1'b0;
            div_start     <= 1'b0;
            div_dividend  <= Q32_ZERO;
            div_divisor   <= Q32_ZERO;
        end else begin
            deriv_start <= 1'b0;
            div_start   <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: TOP_IDLE
                // -------------------------------------------------------------
                TOP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy          <= 1'b1;
                        x_reg         <= x_init;
                        tol_reg       <= (tolerance != Q32_ZERO)  ? tolerance  : Q32_EPS_DEF;
                        alpha_reg     <= (step_alpha != Q32_ZERO) ? step_alpha : Q32_ONE;
                        lambda_reg_in <= (lambda_reg != Q32_ZERO) ? lambda_reg : Q32_LAMBDA_DEF;
                        max_iters_reg <= (max_iters != 8'd0)      ? max_iters  : 8'd50;
                        iter_count    <= 8'd0;
                        status        <= STATUS_RUNNING;
                        state         <= TOP_START_DERIV;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE: TOP_START_DERIV
                // -------------------------------------------------------------
                TOP_START_DERIV: begin
                    deriv_start <= 1'b1;
                    state       <= TOP_WAIT_DERIV;
                end

                // -------------------------------------------------------------
                // STATE: TOP_WAIT_DERIV
                // -------------------------------------------------------------
                TOP_WAIT_DERIV: begin
                    if (deriv_done) begin
                        state <= TOP_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // STATE: TOP_CHECK_CONV
                // -------------------------------------------------------------
                TOP_CHECK_CONV: begin
                    // Condition 1: Converged (|g| <= tolerance)
                    if (abs_grad <= tol_reg) begin
                        status    <= STATUS_CONVERGED;
                        x_optimal <= x_reg;
                        f_optimal <= deriv_f_val;
                        g_final   <= deriv_grad;
                        state     <= TOP_DONE;

                    // Condition 2: Max iterations exceeded
                    end else if (iter_count >= max_iters_reg) begin
                        status    <= STATUS_MAX_ITERS;
                        x_optimal <= x_reg;
                        f_optimal <= deriv_f_val;
                        g_final   <= deriv_grad;
                        state     <= TOP_DONE;

                    // Condition 3: Solve Newton step Δx = -num / den
                    end else begin
                        div_dividend <= -deriv_step_num;
                        div_divisor  <= deriv_step_den;
                        div_start    <= 1'b1;
                        state        <= TOP_WAIT_SOLVE;
                    end
                end

                // -------------------------------------------------------------
                // STATE: TOP_WAIT_SOLVE
                // -------------------------------------------------------------
                TOP_WAIT_SOLVE: begin
                    if (div_done) begin
                        if (div_by_zero) begin
                            status    <= STATUS_SINGULAR;
                            x_optimal <= x_reg;
                            f_optimal <= deriv_f_val;
                            g_final   <= deriv_grad;
                            state     <= TOP_DONE;
                        end else begin
                            state <= TOP_UPDATE_X;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE: TOP_UPDATE_X
                // -------------------------------------------------------------
                TOP_UPDATE_X: begin
                    x_reg      <= x_reg + scaled_step;
                    iter_count <= iter_count + 1'b1;
                    state      <= TOP_START_DERIV;
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
