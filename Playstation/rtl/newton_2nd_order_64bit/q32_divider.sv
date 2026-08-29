// =============================================================================
// File Name   : q32_divider.sv
// Module Name : q32_divider
// Project     : Universal Newton 2nd-Order Optimization Accelerator (64-Bit)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   This module implements a 96-cycle signed fixed-point Radix-2 Restoring Divider.
//   It computes:
//       Quotient = (Dividend << 32) / Divisor
//
// Why 96 Cycles in Q32.32 Format?
//   - To produce a Q32.32 quotient with 32 integer bits and 32 fractional bits,
//     the restoring divider must generate 64 integer quotient bits + 32 fractional
//     quotient bits = 96 total bits.
//   - Uses a 128-bit shift-and-subtract accumulator (rem_acc).
// =============================================================================

`timescale 1ns / 1ps

import newton_types_64bit_pkg::*;

module q32_divider (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,         // System Clock
    input  logic               rst_n,       // Active-Low Reset

    // -------------------------------------------------------------------------
    // Control & Data Inputs
    // -------------------------------------------------------------------------
    input  logic               start,       // 1-cycle start pulse
    input  logic signed [63:0] dividend,    // 64-bit Numerator (Q32.32)
    input  logic signed [63:0] divisor,     // 64-bit Denominator (Q32.32)

    // -------------------------------------------------------------------------
    // Outputs
    // -------------------------------------------------------------------------
    output logic signed [63:0] quotient,    // Computed Quotient = (Dividend << 32) / Divisor
    output logic               done,        // 1-cycle completion pulse
    output logic               div_by_zero, // High if divisor == 0 was detected
    output logic               busy         // High while calculation is in progress (96 cycles)
);

    // -------------------------------------------------------------------------
    // Divider FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        DIV_IDLE = 2'd0, // Waiting for start pulse
        DIV_CALC = 2'd1, // Iterating 96 cycles of shift-subtract division
        DIV_DONE = 2'd2  // Applying sign and asserting done pulse
    } div_state_t;

    div_state_t state;

    // -------------------------------------------------------------------------
    // Internal Registers & Datapath Signals
    // -------------------------------------------------------------------------
    logic [6:0]   count;       // 7-bit down-counter: counts 96 iterations (96 down to 0)
    logic         sign_res;    // Result sign: 1 if negative, 0 if positive
    logic [127:0] rem_acc;     // 128-bit Combined Remainder (upper 64) & Quotient (lower 64)
    logic [63:0]  abs_div;     // Absolute value of divisor

    // Combinational 1-bit left-shifted accumulator
    logic [127:0] shifted;
    assign shifted = {rem_acc[126:0], 1'b0};

    // -------------------------------------------------------------------------
    // Sequential State Machine & Datapath
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= DIV_IDLE;
            quotient    <= 64'sd0;
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
                // STATE: DIV_IDLE (Wait for start)
                // -------------------------------------------------------------
                DIV_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;

                        if (divisor == 64'sd0) begin
                            div_by_zero <= 1'b1;
                            quotient    <= 64'sd0;
                            done        <= 1'b1;
                            busy        <= 1'b0;
                            state       <= DIV_IDLE;
                        end else begin
                            div_by_zero <= 1'b0;
                            sign_res    <= (dividend[63] ^ divisor[63]);
                            abs_div     <= divisor[63] ? 64'(-divisor) : divisor;
                            rem_acc     <= {64'd0, (dividend[63] ? 64'(-dividend) : dividend)};
                            count       <= 7'd96; // 96 cycles for Q32.32 resolution
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
                        if (shifted[127:64] >= abs_div) begin
                            rem_acc <= {(shifted[127:64] - abs_div), shifted[63:1], 1'b1};
                        end else begin
                            rem_acc <= shifted;
                        end
                        count <= count - 1'b1;
                    end else begin
                        state <= DIV_DONE;
                    end
                end

                // -------------------------------------------------------------
                // STATE: DIV_DONE (Format Final Quotient & Assert Done)
                // -------------------------------------------------------------
                DIV_DONE: begin
                    if (sign_res) begin
                        quotient <= -64'(rem_acc[63:0]);
                    end else begin
                        quotient <= 64'(rem_acc[63:0]);
                    end

                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= DIV_IDLE;
                end

                default: state <= DIV_IDLE;
            endcase
        end
    end

endmodule
