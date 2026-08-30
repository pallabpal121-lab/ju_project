// =============================================================================
// File Name   : q16_divider.sv
// Module Name : q16_divider
// Project     : Principal Component Analysis (PCA) / Streaming SVD Accelerator
//               (Solver #28)
// -----------------------------------------------------------------------------
// Description: 48-cycle signed fixed-point Radix-2 Restoring Divider for Q16.16.
// =============================================================================

`timescale 1ns / 1ps

import pca_types_pkg::*;

module q16_divider (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,
    input  logic signed [31:0] dividend,
    input  logic signed [31:0] divisor,
    output logic signed [31:0] quotient,
    output logic               done,
    output logic               div_by_zero,
    output logic               busy
);

    typedef enum logic [1:0] {
        DIV_IDLE = 2'd0,
        DIV_CALC = 2'd1,
        DIV_DONE = 2'd2
    } div_state_t;

    div_state_t state;

    logic [5:0]  count;
    logic        sign_res;
    logic [63:0] rem_acc;
    logic [31:0] abs_div;

    logic [63:0] shifted;
    assign shifted = {rem_acc[62:0], 1'b0};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
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
                DIV_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        if (divisor == 32'sd0) begin
                            div_by_zero <= 1'b1;
                            quotient    <= 32'sd0;
                            done        <= 1'b1;
                            busy        <= 1'b0;
                            state       <= DIV_IDLE;
                        end else begin
                            div_by_zero <= 1'b0;
                            sign_res    <= (dividend[31] ^ divisor[31]);
                            abs_div     <= divisor[31] ? 32'(-divisor) : divisor;
                            rem_acc     <= {32'd0, (dividend[31] ? 32'(-dividend) : dividend)};
                            count       <= 6'd48;
                            state       <= DIV_CALC;
                        end
                    end else begin
                        busy <= 1'b0;
                    end
                end

                DIV_CALC: begin
                    if (count > 0) begin
                        if (shifted[63:32] >= abs_div) begin
                            rem_acc <= {(shifted[63:32] - abs_div), shifted[31:1], 1'b1};
                        end else begin
                            rem_acc <= shifted;
                        end
                        count <= count - 1'b1;
                    end else begin
                        state <= DIV_DONE;
                    end
                end

                DIV_DONE: begin
                    if (sign_res) begin
                        quotient <= -32'(rem_acc[31:0]);
                    end else begin
                        quotient <= 32'(rem_acc[31:0]);
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
