// =============================================================================
// File Name   : q16_sqrt.sv
// Module Name : q16_sqrt
// Project     : Covariance Matrix Adaptation Evolution Strategy (CMA-ES) (Solver #34)
// -----------------------------------------------------------------------------
// Description: 24-cycle signed fixed-point Radix-2 Restoring Square Root.
//              Computes y = sqrt(x) in Q16.16 format.
// =============================================================================

`timescale 1ns / 1ps

import cmaes_types_pkg::*;

module q16_sqrt (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,
    input  logic signed [31:0] rad_in,
    output logic signed [31:0] sqrt_out,
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        SQRT_IDLE = 2'd0,
        SQRT_CALC = 2'd1,
        SQRT_DONE = 2'd2
    } sqrt_state_t;

    sqrt_state_t state;

    logic [47:0] rem;
    logic [47:0] root_acc;
    logic [5:0]  iter;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= SQRT_IDLE;
            sqrt_out <= 32'sd0;
            done     <= 1'b0;
            busy     <= 1'b0;
            rem      <= '0;
            root_acc <= '0;
            iter     <= '0;
        end else begin
            case (state)
                SQRT_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        if (rad_in <= 32'sd0) begin
                            sqrt_out <= 32'sd0;
                            done     <= 1'b1;
                            busy     <= 1'b0;
                            state    <= SQRT_IDLE;
                        end else begin
                            rem      <= {16'd0, rad_in[31:0], 16'd0}; // Shift by 16 bits for Q16.16
                            root_acc <= '0;
                            iter     <= 6'd24;
                            state    <= SQRT_CALC;
                        end
                    end else begin
                        busy <= 1'b0;
                    end
                end

                SQRT_CALC: begin
                    if (iter > 0) begin
                        if (rem >= (((root_acc << 2) | 48'd1) << (2 * (iter - 1)))) begin
                            rem      <= rem - (((root_acc << 2) | 48'd1) << (2 * (iter - 1)));
                            root_acc <= (root_acc << 1) | 48'd1;
                        end else begin
                            root_acc <= root_acc << 1;
                        end
                        iter <= iter - 1'b1;
                    end else begin
                        state <= SQRT_DONE;
                    end
                end

                SQRT_DONE: begin
                    sqrt_out <= root_acc[31:0];
                    done     <= 1'b1;
                    busy     <= 1'b0;
                    state    <= SQRT_IDLE;
                end

                default: state <= SQRT_IDLE;
            endcase
        end
    end

endmodule
