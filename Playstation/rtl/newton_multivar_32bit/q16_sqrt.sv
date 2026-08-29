// =============================================================================
// File Name   : q16_sqrt.sv
// Module Name : q16_sqrt
// Project     : Universal Multivariable Newton 2nd-Order Accelerator
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   24-cycle restoring fixed-point Square Root Engine for Q16.16 numbers.
//   Used by the hardware Cholesky Factorization engine to compute diagonal terms:
//       L_ii = sqrt(A_ii - sum(L_ik^2))
//
// Mathematical Theory:
//   For a number X in Q16.16 format, its integer value is X_int = X * 2^16.
//   The true square root in Q16.16 format is sqrt(X) * 2^16 = sqrt(X_int * 2^16).
//   We compute the standard integer square root of the 48-bit value (X_int << 16).
//   Takes exactly 24 clock cycles (processing 1 pair of bits per cycle).
// =============================================================================

`timescale 1ns / 1ps

import newton_multivar_pkg::*;

module q16_sqrt (
    input  logic               clk,         // System Clock
    input  logic               rst_n,       // Active-Low Reset
    input  logic               start,       // 1-cycle start strobe
    input  logic signed [31:0] val,         // Input value (Q16.16 signed)
    output logic signed [31:0] root,        // Square root result (Q16.16 signed)
    output logic               done,        // 1-cycle completion strobe
    output logic               busy         // High while calculating (24 cycles)
);

    typedef enum logic [1:0] {
        SQRT_IDLE = 2'd0,
        SQRT_CALC = 2'd1,
        SQRT_DONE = 2'd2
    } sqrt_state_t;

    sqrt_state_t state;

    logic [4:0]  count;     // Counts 24 bit-pair iterations
    logic [47:0] rad_reg;   // 48-bit Radicand shift register
    logic [47:0] rem_acc;   // 48-bit Remainder accumulator
    logic [23:0] root_acc;  // 24-bit Root accumulator

    logic [47:0] next_rem;
    assign next_rem = {rem_acc[45:0], rad_reg[47:46]};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= SQRT_IDLE;
            root     <= 32'sd0;
            done     <= 1'b0;
            busy     <= 1'b0;
            count    <= '0;
            rad_reg  <= '0;
            rem_acc  <= '0;
            root_acc <= '0;
        end else begin
            case (state)
                // -------------------------------------------------------------
                // STATE: SQRT_IDLE
                // -------------------------------------------------------------
                SQRT_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        if (val <= 32'sd0) begin
                            // Non-positive input -> return zero
                            root  <= 32'sd0;
                            done  <= 1'b1;
                            busy  <= 1'b0;
                            state <= SQRT_IDLE;
                        end else begin
                            // Radicand in Q16.16 format shifted left by 16 bits = 48 bits total
                            rad_reg  <= {val, 16'd0};
                            rem_acc  <= 48'd0;
                            root_acc <= 24'd0;
                            count    <= 5'd24;
                            state    <= SQRT_CALC;
                        end
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE: SQRT_CALC (24 Digit-Pair Recurrence Iterations)
                // -------------------------------------------------------------
                SQRT_CALC: begin
                    if (count > 0) begin
                        if (next_rem >= {22'd0, root_acc, 2'b01}) begin
                            rem_acc  <= next_rem - {22'd0, root_acc, 2'b01};
                            root_acc <= {root_acc[22:0], 1'b1};
                        end else begin
                            rem_acc  <= next_rem;
                            root_acc <= {root_acc[22:0], 1'b0};
                        end
                        rad_reg <= {rad_reg[45:0], 2'b00};
                        count   <= count - 1'b1;
                    end else begin
                        state <= SQRT_DONE;
                    end
                end

                // -------------------------------------------------------------
                // STATE: SQRT_DONE
                // -------------------------------------------------------------
                SQRT_DONE: begin
                    root  <= {8'd0, root_acc}; // Place 24-bit root into 32-bit Q16.16 format
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= SQRT_IDLE;
                end

                default: state <= SQRT_IDLE;
            endcase
        end
    end

endmodule
