// =============================================================================
// File Name   : newton_2nd_order_top.sv
// Module Name : newton_2nd_order_top
// Project     : Universal Newton 2nd-Order Optimization Accelerator
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This is the TOP-LEVEL SoC module of the Universal Newton 2nd-Order Accelerator.
//   It integrates the entire system into a single, cohesive hardware solver:
//
// Submodules Integrated:
//   1. derivative_engine : Evaluates f(x), f(x+h), f(x-h) to compute gradient & curvature.
//   2. q16_divider       : Solves the linear Newton step: Δx = - Numerator / Denominator.
//   3. Fixed-Point Scaler: Scales Δx by the user's step rate alpha (α * Δx).
//   4. Master FSM        : Drives the multi-iteration optimization loop until convergence.
//
// The Full Optimization Journey (Step-by-Step):
//   1. Host loads equation microcode into internal memory via prog_en/prog_data.
//   2. Host pulses 'start' with initial guess x_init (e.g. 10.0), tolerance, alpha, max_iters.
//   3. State TOP_START_DERIV: Triggers derivative_engine to sample derivatives.
//   4. State TOP_CHECK_CONV : Checks if absolute gradient |g| <= tolerance.
//        - If YES -> Converged! Outputs x_optimal = x_reg, status = STATUS_CONVERGED.
//        - If iter_count >= max_iters -> Stops with status = STATUS_MAX_ITERS.
//        - If NO  -> Loads dividend = -step_num, divisor = step_den and triggers solver divider.
//   5. State TOP_WAIT_SOLVE : Waits 48 cycles for divider to compute Δx.
//   6. State TOP_UPDATE_X   : Updates x_(k+1) = x_k + (alpha * Δx), increments iter_count,
//                             and loops back to step 3 for the next iteration.
// =============================================================================

`timescale 1ns / 1ps

import newton_types_pkg::*;

module newton_2nd_order_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock (e.g. 100MHz)
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Equation Microcode Programming Port
    // -------------------------------------------------------------------------
    input  logic               prog_en,       // Microcode Write Enable
    input  logic [4:0]         prog_addr,     // Microcode Memory Address (0..31)
    input  instr_t             prog_data,     // 32-bit Micro-Instruction Word

    // -------------------------------------------------------------------------
    // Interface 2: Optimization Parameters & Control
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle pulse to launch optimization
    input  q16_t               x_init,        // Initial starting guess x_0 (Q16.16 signed)
    input  q16_t               tolerance,     // Convergence threshold ε (default ~0.001)
    input  q16_t               step_alpha,    // Step size / learning rate α (default 1.0)
    input  q16_t               lambda_reg,    // Regularization damping factor λ (default ~0.004)
    input  logic [7:0]         max_iters,     // Safety limit on max iterations (e.g. 50)

    // -------------------------------------------------------------------------
    // Interface 3: Results & Status Outputs
    // -------------------------------------------------------------------------
    output q16_t               x_optimal,     // Converged optimal solution x* (root / minimum)
    output q16_t               f_optimal,     // Final function value f(x*)
    output q16_t               g_final,       // Final gradient norm g(x*)
    output logic [7:0]         iter_count,    // Total iterations executed before stopping
    output status_t            status,        // Status Code: CONVERGED, MAX_ITERS, SINGULAR, etc.
    output logic               done,          // 1-cycle completion pulse
    output logic               busy           // High while optimization loop is running
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        TOP_IDLE        = 3'd0, // Idle: Ready for new start request
        TOP_START_DERIV = 3'd1, // Strobe start pulse to derivative_engine
        TOP_WAIT_DERIV  = 3'd2, // Wait for derivative engine to finish 3-point sampling
        TOP_CHECK_CONV  = 3'd3, // Check |g| <= tol and max iteration conditions
        TOP_START_SOLVE = 3'd4, // Trigger 48-cycle divider for Newton step Δx = -num/den
        TOP_WAIT_SOLVE  = 3'd5, // Wait for divider completion
        TOP_UPDATE_X    = 3'd6, // Update x = x + alpha * Δx and advance iteration counter
        TOP_DONE        = 3'd7  // Latch final outputs and assert 'done'
    } top_state_t;

    top_state_t state;

    // -------------------------------------------------------------------------
    // Configuration & State Registers
    // -------------------------------------------------------------------------
    q16_t       x_reg;          // Current position x_k
    q16_t       tol_reg;        // Registered convergence tolerance
    q16_t       alpha_reg;      // Registered step scale alpha
    q16_t       lambda_reg_in;  // Registered damping factor lambda
    logic [7:0] max_iters_reg;  // Registered maximum iteration limit

    // -------------------------------------------------------------------------
    // Derivative Engine Interconnect Signals
    // -------------------------------------------------------------------------
    logic deriv_start;
    q16_t deriv_f_val;
    q16_t deriv_grad;
    q16_t deriv_step_num;
    q16_t deriv_step_den;
    logic deriv_done;
    logic deriv_busy;

    // -------------------------------------------------------------------------
    // Step Solver / Divider Interconnect Signals
    // -------------------------------------------------------------------------
    logic div_start;
    q16_t div_dividend;
    q16_t div_divisor;
    q16_t div_quotient; // Raw Newton Step: Δx = -deriv_step_num / deriv_step_den
    logic div_done;
    logic div_by_zero;
    logic div_busy;

    // -------------------------------------------------------------------------
    // Fixed-Point Step Scaler: scaled_step = alpha * div_quotient
    // Multiplies two Q16.16 values to form a 64-bit product, sliced at [47:16]
    // -------------------------------------------------------------------------
    logic signed [63:0] scaled_step_64;
    q16_t               scaled_step;
    assign scaled_step_64 = 64'(alpha_reg) * 64'(div_quotient);
    assign scaled_step    = scaled_step_64[47:16];

    // Trust-Region Step Bounding (Clamp maximum step jump to ±10.0 in Q16.16)
    localparam signed [31:0] MAX_STEP_BOUND = 32'sh000A_0000; // +10.0
    localparam signed [31:0] MIN_STEP_BOUND = -MAX_STEP_BOUND; // -10.0

    logic signed [31:0] clamped_step;
    assign clamped_step = (scaled_step > MAX_STEP_BOUND) ? MAX_STEP_BOUND :
                          (scaled_step < MIN_STEP_BOUND) ? MIN_STEP_BOUND :
                          scaled_step;

    // 32-bit Saturating Position Adder (Guarantees no 2's complement wrap-around)
    logic signed [32:0] x_full_sum;
    logic signed [31:0] x_sat_sum;
    assign x_full_sum = {x_reg[31], x_reg} + {clamped_step[31], clamped_step};

    wire pos_overflow = (~x_reg[31]) & (~clamped_step[31]) & x_full_sum[31];
    wire neg_overflow = (x_reg[31])  & (clamped_step[31])  & (~x_full_sum[31]);

    assign x_sat_sum = pos_overflow ? 32'sh7FFF_FFFF :
                       neg_overflow ? 32'sh8000_0000 :
                       x_full_sum[31:0];

    // Absolute value of gradient for convergence testing: |g(x)|
    q16_t abs_grad;
    assign abs_grad = (deriv_grad[31]) ? -deriv_grad : deriv_grad;

    // -------------------------------------------------------------------------
    // Submodule 1: Universal Derivative & Curvature Engine
    // -------------------------------------------------------------------------
    derivative_engine u_deriv_engine (
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
    // Submodule 2: Newton Step Solver / Divider (48-cycle Q16.16 Restoring Divider)
    // -------------------------------------------------------------------------
    q16_divider u_solver (
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
            // Reset all state and output registers
            state         <= TOP_IDLE;
            x_reg         <= Q16_ZERO;
            tol_reg       <= Q16_EPS_DEF;
            alpha_reg     <= Q16_ONE;
            lambda_reg_in <= Q16_LAMBDA_DEF;
            max_iters_reg <= 8'd50;
            iter_count    <= 8'd0;
            status        <= STATUS_IDLE;
            done          <= 1'b0;
            busy          <= 1'b0;
            x_optimal     <= Q16_ZERO;
            f_optimal     <= Q16_ZERO;
            g_final       <= Q16_ZERO;
            deriv_start   <= 1'b0;
            div_start     <= 1'b0;
            div_dividend  <= Q16_ZERO;
            div_divisor   <= Q16_ZERO;
        end else begin
            deriv_start <= 1'b0; // Default pulse suppression
            div_start   <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: TOP_IDLE (Wait for start and register parameters)
                // -------------------------------------------------------------
                TOP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy          <= 1'b1;
                        x_reg         <= x_init;
                        // Use default fallbacks if inputs are zero
                        tol_reg       <= (tolerance != Q16_ZERO)  ? tolerance  : Q16_EPS_DEF;
                        alpha_reg     <= (step_alpha != Q16_ZERO) ? step_alpha : Q16_ONE;
                        lambda_reg_in <= (lambda_reg != Q16_ZERO) ? lambda_reg : Q16_LAMBDA_DEF;
                        max_iters_reg <= (max_iters != 8'd0)      ? max_iters  : 8'd50;
                        iter_count    <= 8'd0;
                        status        <= STATUS_RUNNING;
                        state         <= TOP_START_DERIV;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE: TOP_START_DERIV (Launch Derivative Engine)
                // -------------------------------------------------------------
                TOP_START_DERIV: begin
                    deriv_start <= 1'b1; // Trigger derivative engine
                    state       <= TOP_WAIT_DERIV;
                end

                // -------------------------------------------------------------
                // STATE: TOP_WAIT_DERIV (Wait for 3-point sampling to finish)
                // -------------------------------------------------------------
                TOP_WAIT_DERIV: begin
                    if (deriv_done) begin
                        state <= TOP_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // STATE: TOP_CHECK_CONV (Check Convergence & Iteration Limits)
                // -------------------------------------------------------------
                TOP_CHECK_CONV: begin
                    // Condition 1: Gradient is sufficiently flat -> CONVERGED!
                    if (abs_grad <= tol_reg) begin
                        status    <= STATUS_CONVERGED;
                        x_optimal <= x_reg;
                        f_optimal <= deriv_f_val;
                        g_final   <= deriv_grad;
                        state     <= TOP_DONE;

                    // Condition 2: Exceeded max allowed iterations -> STOP
                    end else if (iter_count >= max_iters_reg) begin
                        status    <= STATUS_MAX_ITERS;
                        x_optimal <= x_reg;
                        f_optimal <= deriv_f_val;
                        g_final   <= deriv_grad;
                        state     <= TOP_DONE;

                    // Condition 3: Continue -> Solve for Newton Step Δx = -num / den
                    end else begin
                        div_dividend <= -deriv_step_num; // Numerator = - [ h * Δ1 ]
                        div_divisor  <= deriv_step_den;  // Denominator = 2 * Δ2 ± λ
                        div_start    <= 1'b1;            // Trigger 48-cycle divider
                        state        <= TOP_WAIT_SOLVE;
                    end
                end

                // -------------------------------------------------------------
                // STATE: TOP_WAIT_SOLVE (Wait for Divider to Compute Step)
                // -------------------------------------------------------------
                TOP_WAIT_SOLVE: begin
                    if (div_done) begin
                        // Check if division by zero occurred (singular Hessian)
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
                // STATE: TOP_UPDATE_X (Update Position x = x + alpha * Δx)
                // -------------------------------------------------------------
                TOP_UPDATE_X: begin
                    x_reg      <= x_sat_sum;           // Apply bounded, saturated update step
                    iter_count <= iter_count + 1'b1;   // Increment iteration counter
                    state      <= TOP_START_DERIV;     // Repeat loop for next iteration!
                end

                // -------------------------------------------------------------
                // STATE: TOP_DONE (Assert Completion Strobe)
                // -------------------------------------------------------------
                TOP_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= TOP_IDLE; // Ready for next solve
                end

                default: state <= TOP_IDLE;
            endcase
        end
    end

endmodule
