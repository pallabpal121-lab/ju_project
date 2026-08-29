// =============================================================================
// File Name   : derivative_engine_64bit.sv
// Module Name : derivative_engine_64bit
// Project     : Universal Newton 2nd-Order Optimization Accelerator (64-Bit)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This module computes 64-bit high-precision derivatives (Gradient g(x) and
//   Curvature/Hessian H(x)) using 3-point numerical finite differences with
//   perturbation step h = 2^-8 = 0.00390625 (Q32_H_STEP).
//
// Zero-Cost Bit-Shift Formulas (Q32.32):
//   - diff_1st = f(x + h) - f(x - h)
//   - diff_2nd = f(x + h) - 2*f(x) + f(x - h)
//   - gradient = diff_1st / (2h) == diff_1st <<< 7 (divide by 2^-7 is multiply by 128!)
//   - step_num = h * diff_1st    == diff_1st >>> 8 (multiply by 2^-8 is shift right by 8!)
//   - step_den = 2 * diff_2nd ± λ== (diff_2nd <<< 1) ± lambda_reg
// =============================================================================

`timescale 1ns / 1ps

import newton_types_64bit_pkg::*;

module derivative_engine_64bit (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,         // System Clock
    input  logic               rst_n,       // Active-Low Reset

    // -------------------------------------------------------------------------
    // Microcode Programming Interface (Pass-through to DFG engine)
    // -------------------------------------------------------------------------
    input  logic               prog_en,     // Program Write Enable
    input  logic [5:0]         prog_addr,   // Program Memory Address (0..63)
    input  instr64_t           prog_data,   // 64-bit Microcode Instruction Word

    // -------------------------------------------------------------------------
    // Control & Optimization Parameters
    // -------------------------------------------------------------------------
    input  logic               start,       // 1-cycle strobe to start derivative computation
    input  q32_t               x_curr,      // Current optimization point x_k (Q32.32)
    input  q32_t               lambda_reg,  // Damping factor added to denominator

    // -------------------------------------------------------------------------
    // Outputs
    // -------------------------------------------------------------------------
    output q32_t               f_val,       // f(x) evaluated at center point
    output q32_t               gradient,    // g(x) = f'(x) used for convergence check |g| <= tol
    output q32_t               step_num,    // h * (f_+ - f_-) = Numerator of Newton Step
    output q32_t               step_den,    // 2 * (f_+ - 2*f_0 + f_-) ± lambda = Denominator
    output logic               done,        // 1-cycle completion pulse
    output logic               busy         // High while sampling and computing derivatives
);

    // -------------------------------------------------------------------------
    // State Machine Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        DERIV_IDLE    = 3'd0, // Waiting for start pulse
        DERIV_EVAL_F0 = 3'd1, // Sample 1: Evaluate f(x)
        DERIV_EVAL_FP = 3'd2, // Sample 2: Evaluate f(x + h)
        DERIV_EVAL_FM = 3'd3, // Sample 3: Evaluate f(x - h)
        DERIV_CALC    = 3'd4, // Compute difference terms and bit-shifts
        DERIV_DONE    = 3'd5  // Assert done strobe
    } deriv_state_t;

    deriv_state_t state;

    // -------------------------------------------------------------------------
    // DFG Equation Engine Interconnect Signals
    // -------------------------------------------------------------------------
    logic start_dfg;
    q32_t dfg_x_in;
    q32_t dfg_f_out;
    logic dfg_done;
    logic dfg_busy;

    // Registers to store the 3 sampled points
    q32_t f_0;
    q32_t f_plus;
    q32_t f_minus;

    // -------------------------------------------------------------------------
    // Instantiate 64-Bit DFG Equation Engine
    // -------------------------------------------------------------------------
    dfg_equation_engine_64bit u_dfg (
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
    logic signed [63:0] diff_1st;
    logic signed [63:0] diff_2nd;

    // -------------------------------------------------------------------------
    // Sequential State Machine
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= DERIV_IDLE;
            start_dfg  <= 1'b0;
            dfg_x_in   <= Q32_ZERO;
            f_val      <= Q32_ZERO;
            gradient   <= Q32_ZERO;
            step_num   <= Q32_ZERO;
            step_den   <= Q32_ZERO;
            done       <= 1'b0;
            busy       <= 1'b0;
            f_0        <= Q32_ZERO;
            f_plus     <= Q32_ZERO;
            f_minus    <= Q32_ZERO;
        end else begin
            start_dfg <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: DERIV_IDLE
                // -------------------------------------------------------------
                DERIV_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy       <= 1'b1;
                        dfg_x_in   <= x_curr; // Sample 1: Target x
                        start_dfg  <= 1'b1;
                        state      <= DERIV_EVAL_F0;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE: DERIV_EVAL_F0 (Sample at x)
                // -------------------------------------------------------------
                DERIV_EVAL_F0: begin
                    if (dfg_done) begin
                        f_0        <= dfg_f_out;
                        f_val      <= dfg_f_out;
                        dfg_x_in   <= x_curr + Q32_H_STEP; // Sample 2: x + h
                        start_dfg  <= 1'b1;
                        state      <= DERIV_EVAL_FP;
                    end
                end

                // -------------------------------------------------------------
                // STATE: DERIV_EVAL_FP (Sample at x + h)
                // -------------------------------------------------------------
                DERIV_EVAL_FP: begin
                    if (dfg_done) begin
                        f_plus     <= dfg_f_out;
                        dfg_x_in   <= x_curr - Q32_H_STEP; // Sample 3: x - h
                        start_dfg  <= 1'b1;
                        state      <= DERIV_EVAL_FM;
                    end
                end

                // -------------------------------------------------------------
                // STATE: DERIV_EVAL_FM (Sample at x - h)
                // -------------------------------------------------------------
                DERIV_EVAL_FM: begin
                    if (dfg_done) begin
                        f_minus <= dfg_f_out;
                        state   <= DERIV_CALC;
                    end
                end

                // -------------------------------------------------------------
                // STATE: DERIV_CALC (Zero-Cost 64-Bit Shift Calculus)
                // -------------------------------------------------------------
                DERIV_CALC: begin
                    // 1. Calculate Differences:
                    diff_1st = (f_plus - f_minus);
                    diff_2nd = (f_plus - (f_0 <<< 1) + f_minus);

                    // 2. Full Gradient g(x) = diff_1st / (2h):
                    // With 2h = 2^-7, dividing by 2^-7 is multiplying by 128 (<<< 7)
                    gradient <= diff_1st <<< 7;

                    // 3. Newton Step Numerator = h * diff_1st:
                    // With h = 2^-8, multiplying by 2^-8 is right shift by 8 (>>> 8)
                    step_num <= diff_1st >>> 8;

                    // 4. Newton Step Denominator = 2 * diff_2nd ± lambda_reg:
                    // Multiplying by 2 is left shift by 1 (<<< 1)
                    if (diff_2nd >= 0) begin
                        step_den <= (diff_2nd <<< 1) + lambda_reg;
                    end else begin
                        step_den <= (diff_2nd <<< 1) - lambda_reg;
                    end

                    state <= DERIV_DONE;
                end

                // -------------------------------------------------------------
                // STATE: DERIV_DONE
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
