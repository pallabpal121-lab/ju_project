// =============================================================================
// File Name   : derivative_engine.sv
// Module Name : derivative_engine
// Project     : Universal Newton 2nd-Order Optimization Accelerator
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This module computes the 1st Derivative (Gradient g(x)) and 2nd Derivative
//   (Curvature / Hessian H(x)) for ANY equation programmed into the DFG engine.
//
// How Does Hardware Compute Derivatives Without Symbolic Calculus?
//   Instead of algebraically deriving f'(x) and f''(x) with complex hardware,
//   we use Numerical Finite Differences by sampling f(x) at 3 nearby points:
//     1. Center Point : f_0     = f(x)
//     2. Right Point  : f_plus  = f(x + h)
//     3. Left Point   : f_minus = f(x - h)
//
// The Calculus Formulas:
//   - 1st Difference: Δ1 = f(x + h) - f(x - h)
//   - 2nd Difference: Δ2 = f(x + h) - 2*f(x) + f(x - h)
//
//   - Gradient g(x) = Δ1 / (2h)
//   - Hessian  H(x) = Δ2 / (h^2)
//
// Why the Newton Step Simplification is Brilliant in Hardware:
//   The classical Newton update step is:
//       Δx = - g(x) / H(x) = - [ Δ1 / (2h) ] / [ Δ2 / (h^2) ]
//   Multiplying numerator and denominator by h^2 gives:
//       Δx = - [ h * Δ1 ] / [ 2 * Δ2 ]
//
//   With h = 2^-4 = 0.0625:
//     - Numerator   : h * Δ1  == (diff_1st >>> 4)  [Single Right-Shift!]
//     - Denominator : 2 * Δ2  == (diff_2nd <<< 1)  [Single Left-Shift!]
//     - Gradient    : Δ1 / (2h) == (diff_1st <<< 3) [Single Left-Shift because 1/(2*0.0625) = 8!]
//
//   This eliminates multiple floating-point dividers down to zero-cost bit-shifts!
// =============================================================================

`timescale 1ns / 1ps

import newton_types_pkg::*;

module derivative_engine (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,         // System Clock
    input  logic               rst_n,       // Active-Low Reset

    // -------------------------------------------------------------------------
    // Microcode Programming Interface (Pass-through to internal DFG engine)
    // -------------------------------------------------------------------------
    input  logic               prog_en,     // Program Write Enable
    input  logic [4:0]         prog_addr,   // Program Memory Address (0..31)
    input  instr_t             prog_data,   // Microcode Instruction Word

    // -------------------------------------------------------------------------
    // Control & Optimization Parameters
    // -------------------------------------------------------------------------
    input  logic               start,       // 1-cycle strobe to start derivative computation
    input  q16_t               x_curr,      // Current optimization point x_k
    input  q16_t               lambda_reg,  // Damping factor added to denominator for stability

    // -------------------------------------------------------------------------
    // Outputs
    // -------------------------------------------------------------------------
    output q16_t               f_val,       // f(x) evaluated at center point
    output q16_t               gradient,    // g(x) = f'(x) used for convergence check |g| <= tol
    output q16_t               step_num,    // h * (f_+ - f_-) = Numerator of Newton Step
    output q16_t               step_den,    // 2 * (f_+ - 2*f_0 + f_-) ± lambda = Denominator
    output logic               done,        // 1-cycle pulse asserted when all values are ready
    output logic               busy         // High while sampling and computing derivatives
);

    // -------------------------------------------------------------------------
    // State Machine Definitions for 3-Point Sampling Sequence
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        DERIV_IDLE    = 3'd0, // Waiting for start pulse
        DERIV_EVAL_F0 = 3'd1, // Sample 1: Evaluate f(x)
        DERIV_EVAL_FP = 3'd2, // Sample 2: Evaluate f(x + h)
        DERIV_EVAL_FM = 3'd3, // Sample 3: Evaluate f(x - h)
        DERIV_CALC    = 3'd4, // Compute difference terms and bit-shifts
        DERIV_DONE    = 3'd5  // Assert done strobe and return to idle
    } deriv_state_t;

    deriv_state_t state;

    // -------------------------------------------------------------------------
    // Internal Signals to drive DFG Equation Engine
    // -------------------------------------------------------------------------
    logic start_dfg;     // Pulse to start DFG engine evaluation
    q16_t dfg_x_in;      // Value fed into DFG engine as input 'x'
    q16_t dfg_f_out;     // Evaluated result from DFG engine
    logic dfg_done;      // High when DFG engine finishes evaluation
    logic dfg_busy;

    // Registers to store the 3 sampled points
    q16_t f_0;           // Holds f(x)
    q16_t f_plus;        // Holds f(x + h)
    q16_t f_minus;       // Holds f(x - h)

    // -------------------------------------------------------------------------
    // Instantiate Internal DFG Equation Evaluator
    // -------------------------------------------------------------------------
    dfg_equation_engine u_dfg (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .x_in       (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    // Difference registers
    logic signed [31:0] diff_1st;
    logic signed [31:0] diff_2nd;

    // -------------------------------------------------------------------------
    // Sequential State Machine: 3-Point Sampling & Shift Calculus
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= DERIV_IDLE;
            start_dfg  <= 1'b0;
            dfg_x_in   <= Q16_ZERO;
            f_val      <= Q16_ZERO;
            gradient   <= Q16_ZERO;
            step_num   <= Q16_ZERO;
            step_den   <= Q16_ZERO;
            done       <= 1'b0;
            busy       <= 1'b0;
            f_0        <= Q16_ZERO;
            f_plus     <= Q16_ZERO;
            f_minus    <= Q16_ZERO;
        end else begin
            start_dfg <= 1'b0; // Default pulse suppression

            case (state)
                // -------------------------------------------------------------
                // STATE: DERIV_IDLE (Wait for start)
                // -------------------------------------------------------------
                DERIV_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy       <= 1'b1;
                        dfg_x_in   <= x_curr; // Sample 1: Target x
                        start_dfg  <= 1'b1;   // Launch DFG evaluation
                        state      <= DERIV_EVAL_F0;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE: DERIV_EVAL_F0 (Wait for f(x) and launch f(x + h))
                // -------------------------------------------------------------
                DERIV_EVAL_F0: begin
                    if (dfg_done) begin
                        f_0        <= dfg_f_out;
                        f_val      <= dfg_f_out; // Also store into output port
                        dfg_x_in   <= x_curr + Q16_H_STEP; // Sample 2: x + 0.0625
                        start_dfg  <= 1'b1;                // Launch DFG evaluation
                        state      <= DERIV_EVAL_FP;
                    end
                end

                // -------------------------------------------------------------
                // STATE: DERIV_EVAL_FP (Wait for f(x + h) and launch f(x - h))
                // -------------------------------------------------------------
                DERIV_EVAL_FP: begin
                    if (dfg_done) begin
                        f_plus     <= dfg_f_out;
                        dfg_x_in   <= x_curr - Q16_H_STEP; // Sample 3: x - 0.0625
                        start_dfg  <= 1'b1;                // Launch DFG evaluation
                        state      <= DERIV_EVAL_FM;
                    end
                end

                // -------------------------------------------------------------
                // STATE: DERIV_EVAL_FM (Wait for f(x - h))
                // -------------------------------------------------------------
                DERIV_EVAL_FM: begin
                    if (dfg_done) begin
                        f_minus <= dfg_f_out;
                        state   <= DERIV_CALC; // All 3 samples ready!
                    end
                end

                // -------------------------------------------------------------
                // STATE: DERIV_CALC (Execute Zero-Cost Shift Operations)
                // -------------------------------------------------------------
                DERIV_CALC: begin
                    // 1. Calculate Fundamental Differences:
                    diff_1st = (f_plus - f_minus);
                    diff_2nd = (f_plus - (f_0 <<< 1) + f_minus); // f_+ - 2*f_0 + f_-

                    // 2. Full Gradient g(x) = diff_1st / (2h):
                    // Since 2h = 0.125 = 1/8, dividing by 1/8 is multiplying by 8 (<<< 3)
                    gradient <= diff_1st <<< 3;

                    // 3. Newton Step Numerator = h * diff_1st:
                    // Since h = 2^-4, multiplying by 2^-4 is right shift by 4 (>>> 4)
                    step_num <= diff_1st >>> 4;

                    // 4. Newton Step Denominator = 2 * diff_2nd ± lambda_reg:
                    // Multiplying by 2 is left shift by 1 (<<< 1)
                    // We add lambda_reg with the same sign as diff_2nd to prevent zero denominator
                    if (diff_2nd >= 0) begin
                        step_den <= (diff_2nd <<< 1) + lambda_reg;
                    end else begin
                        step_den <= (diff_2nd <<< 1) - lambda_reg;
                    end

                    state <= DERIV_DONE;
                end

                // -------------------------------------------------------------
                // STATE: DERIV_DONE (Assert Done Pulse)
                // -------------------------------------------------------------
                DERIV_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= DERIV_IDLE;
                end

                default: state <= DERIV_IDLE;
            endcase
        end
    end

endmodule
