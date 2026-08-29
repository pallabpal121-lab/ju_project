// =============================================================================
// File Name   : q16_divider.sv
// Module Name : q16_divider
// Project     : Universal Newton 2nd-Order Optimization Accelerator
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This module implements a 48-cycle signed fixed-point Radix-2 Restoring Divider.
//   It computes:
//       Quotient = (Dividend << 16) / Divisor
//
// Why is Division Special in Fixed-Point Hardware?
//   - Unlike addition (1 cycle) or multiplication (1 cycle), hardware division
//     is inherently an iterative process (like manual long division done on paper).
//   - In Q16.16 format, dividing (A * 2^16) by (B * 2^16) gives (A / B), which is
//     an unscaled integer.
//   - To get a Q16.16 result with 16 fractional bits, we must compute:
//         Quotient_Q16 = ((Dividend * 2^16) << 16) / (Divisor * 2^16)
//     This requires generating 32 integer quotient bits + 16 fractional quotient
//     bits = 48 total bits!
//   - Hence, this module runs for exactly 48 clock cycles in the DIV_CALC state.
//
// Algorithm (Radix-2 Restoring Division):
//   1. Take absolute values of Dividend and Divisor; record sign = sign(A) ^ sign(B).
//   2. Initialize 64-bit Remainder/Quotient register: rem_acc = {32'd0, |Dividend|}.
//   3. For each of 48 cycles:
//      a. Shift rem_acc left by 1 bit.
//      b. Subtract |Divisor| from the upper 32 bits (shifted[63:32]).
//      c. If the result is >= 0 (subtraction succeeded):
//           - Store difference in upper 32 bits, set quotient bit (LSB) to 1.
//         Else (subtraction failed):
//           - Restore original value (keep shifted), set quotient bit (LSB) to 0.
//   4. In the final cycle, negate the 32-bit quotient if sign_res is negative.
// =============================================================================

`timescale 1ns / 1ps

import newton_types_pkg::*;

module q16_divider (
    // -------------------------------------------------------------------------
    // Clock and Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,         // System Clock (e.g. 100MHz)
    input  logic               rst_n,       // Active-Low Asynchronous Reset

    // -------------------------------------------------------------------------
    // Control and Data Inputs
    // -------------------------------------------------------------------------
    input  logic               start,       // 1-cycle start pulse to launch division
    input  logic signed [31:0] dividend,    // Numerator (Q16.16 signed)
    input  logic signed [31:0] divisor,     // Denominator (Q16.16 signed)

    // -------------------------------------------------------------------------
    // Outputs
    // -------------------------------------------------------------------------
    output logic signed [31:0] quotient,    // Computed Quotient = (Dividend << 16) / Divisor
    output logic               done,        // 1-cycle pulse asserted when quotient is ready
    output logic               div_by_zero, // High if divisor == 0 was detected
    output logic               busy         // High while calculation is in progress (48 cycles)
);

    // -------------------------------------------------------------------------
    // Divider FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        DIV_IDLE = 2'd0, // Waiting for 'start' pulse
        DIV_CALC = 2'd1, // Iterating 48 cycles of shift-subtract division
        DIV_DONE = 2'd2  // Applying sign and asserting 'done' pulse
    } div_state_t;

    div_state_t state;

    // -------------------------------------------------------------------------
    // Internal Registers & Datapath Signals
    // -------------------------------------------------------------------------
    logic [5:0]  count;       // Down-counter: counts 48 iterations (48 down to 0)
    logic        sign_res;    // Expected sign of result: 1 if negative, 0 if positive
    logic [63:0] rem_acc;     // 64-bit Combined Remainder (upper 32) & Quotient (lower 32)
    logic [31:0] abs_div;     // Absolute (magnitude) value of divisor

    // Combinational 1-bit left-shifted version of the accumulator
    logic [63:0] shifted;
    assign shifted = {rem_acc[62:0], 1'b0};

    // -------------------------------------------------------------------------
    // Sequential State Machine & Datapath
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset all internal registers to default safe state
            state       <= DIV_IDLE;
            quotient    <= 32'sd0;
            done        <= 1'b0;
            div_by_zero <= 1'b0;
            busy        <= 1'b0;
            count       <= '0;
            sign_res    <= 1'b0;
            rem_acc     <= '0;
            abs_div     <= '0;
        end else begin
            case (state)
                // -------------------------------------------------------------
                // STATE: DIV_IDLE (Wait for request)
                // -------------------------------------------------------------
                DIV_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;

                        // Check for Divide-by-Zero error immediately:
                        if (divisor == 32'sd0) begin
                            div_by_zero <= 1'b1;
                            quotient    <= 32'sd0;
                            done        <= 1'b1;
                            busy        <= 1'b0;
                            state       <= DIV_IDLE; // Exit immediately
                        end else begin
                            div_by_zero <= 1'b0;
                            
                            // Result is negative if signs of dividend and divisor differ (XOR)
                            sign_res    <= (dividend[31] ^ divisor[31]);
                            
                            // Take absolute values of both operands
                            abs_div     <= divisor[31]  ? 32'(-divisor)  : divisor;
                            rem_acc     <= {32'd0, (dividend[31] ? 32'(-dividend) : dividend)};
                            
                            // Set iteration count to 48 cycles (for Q16.16 resolution)
                            count       <= 6'd48;
                            state       <= DIV_CALC;
                        end
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE: DIV_CALC (Shift-and-Subtract Loop)
                // -------------------------------------------------------------
                DIV_CALC: begin
                    if (count > 0) begin
                        // Trial subtraction: Check if upper 32 bits >= abs_div
                        if (shifted[63:32] >= abs_div) begin
                            // Subtraction succeeds: Subtract divisor and insert '1' into quotient LSB
                            rem_acc <= {(shifted[63:32] - abs_div), shifted[31:1], 1'b1};
                        end else begin
                            // Subtraction fails: Keep shifted value and insert '0' into quotient LSB
                            rem_acc <= shifted;
                        end
                        count <= count - 1'b1;
                    end else begin
                        // All 48 bits generated! Move to output formatting state
                        state <= DIV_DONE;
                    end
                end

                // -------------------------------------------------------------
                // STATE: DIV_DONE (Format Final Quotient & Assert Done)
                // -------------------------------------------------------------
                DIV_DONE: begin
                    // Apply sign to the 32-bit quotient stored in the lower half of rem_acc
                    if (sign_res) begin
                        quotient <= -32'(rem_acc[31:0]);
                    end else begin
                        quotient <= 32'(rem_acc[31:0]);
                    end

                    done  <= 1'b1; // 1-cycle completion strobe
                    busy  <= 1'b0;
                    state <= DIV_IDLE;
                end

                default: state <= DIV_IDLE;
            endcase
        end
    end

endmodule
