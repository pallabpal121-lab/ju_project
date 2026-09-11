// =============================================================================
// File Name   : tg_mux.sv
// Module Name : tg_mux2_32b, tg_mux4_32b, tg_mux16_32b
// Project     : Custom Cell Optimization Library
// -----------------------------------------------------------------------------
// Description : Transmission-Gate Multiplexers with minimal propagation delay
// =============================================================================

`timescale 1ns / 1ps

// 2:1 32-bit Transmission Gate Multiplexer
module tg_mux2_32b (
    input  logic [31:0] in0,
    input  logic [31:0] in1,
    input  logic        sel,
    output logic [31:0] out
);
    assign out = sel ? in1 : in0;
endmodule

// 4:1 32-bit Transmission Gate Multiplexer
module tg_mux4_32b (
    input  logic [31:0] in0,
    input  logic [31:0] in1,
    input  logic [31:0] in2,
    input  logic [31:0] in3,
    input  logic [1:0]  sel,
    output logic [31:0] out
);
    always_comb begin
        case (sel)
            2'b00:   out = in0;
            2'b01:   out = in1;
            2'b10:   out = in2;
            default: out = in3;
        endcase
    end
endmodule

// 16:1 32-bit Hierarchical Multiplexer (2-stage TG 4:1 Tree)
module tg_mux16_32b (
    input  logic [31:0] in [0:15],
    input  logic [3:0]  sel,
    output logic [31:0] out
);
    logic [31:0] stage1_out [0:3];

    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : gen_stage1
            tg_mux4_32b u_m4 (
                .in0(in[4*i + 0]),
                .in1(in[4*i + 1]),
                .in2(in[4*i + 2]),
                .in3(in[4*i + 3]),
                .sel(sel[1:0]),
                .out(stage1_out[i])
            );
        end
    endgenerate

    tg_mux4_32b u_final (
        .in0(stage1_out[0]),
        .in1(stage1_out[1]),
        .in2(stage1_out[2]),
        .in3(stage1_out[3]),
        .sel(sel[3:2]),
        .out(out)
    );

endmodule
