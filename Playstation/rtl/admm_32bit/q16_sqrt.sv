// =============================================================================
// File Name   : q16_sqrt.sv
// Module Name : q16_sqrt
// Project     : Alternating Direction Method of Multipliers (ADMM) Accelerator (Solver #11)
// -----------------------------------------------------------------------------
// Description: 24-cycle restoring fixed-point Square Root Engine for Q16.16 numbers.
// =============================================================================

`timescale 1ns / 1ps

import admm_types_pkg::*;

module q16_sqrt (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,
    input  logic signed [31:0] val,
    output logic signed [31:0] root,
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        SQRT_IDLE = 2'd0,
        SQRT_CALC = 2'd1,
        SQRT_DONE = 2'd2
    } sqrt_state_t;

    sqrt_state_t state;

    logic [4:0]  count;
    logic [47:0] rad_reg;
    logic [47:0] rem_acc;
    logic [23:0] root_acc;

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
                SQRT_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        if (val <= 32'sd0) begin
                            root  <= 32'sd0;
                            done  <= 1'b1;
                            busy  <= 1'b0;
                            state <= SQRT_IDLE;
                        end else begin
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

                SQRT_DONE: begin
                    root  <= {8'd0, root_acc};
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= SQRT_IDLE;
                end

                default: state <= SQRT_IDLE;
            endcase
        end
    end

endmodule
