// =============================================================================
// File Name   : fused_3input_adder_32b.sv
// Module Name : fused_3input_adder_32b
// Project     : Custom Cell Optimization Library
// -----------------------------------------------------------------------------
// Description : Fused 3-Operand Finite-Difference Adder / Subtractor
//
// Application in Newton 2nd-Order Optimization:
//   Directly computes the 2nd Difference / Hessian Curvature:
//       diff_2nd = f(x + h) - 2*f(x) + f(x - h)
//
// Architecture:
//   - 3:2 Carry Save Adder (CSA) front-end reduces 3 operands into (Sum, Carry) in 1 gate delay.
//   - Kogge-Stone Parallel-Prefix Adder final stage eliminates dual-ripple carry stages.
// =============================================================================

`timescale 1ns / 1ps

module fused_3input_adder_32b (
    input  logic signed [31:0] f_plus,   // f(x + h)
    input  logic signed [31:0] f_0,      // f(x) (to be multiplied by -2)
    input  logic signed [31:0] f_minus,  // f(x - h)
    output logic signed [31:0] diff_2nd  // Result: f_+ - 2*f_0 + f_-
);

    // Negate 2*f_0 using two's complement: ~ (f_0 << 1) + 1
    logic [31:0] neg_2f0;
    assign neg_2f0 = ~(f_0 << 1);

    // 3:2 Carry Save Reduction
    logic [31:0] csa_sum;
    logic [31:0] csa_carry;

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gen_csa
            assign csa_sum[i]   = f_plus[i] ^ f_minus[i] ^ neg_2f0[i];
            assign csa_carry[i] = (f_plus[i] & f_minus[i]) | 
                                  (f_minus[i] & neg_2f0[i]) | 
                                  (f_plus[i] & neg_2f0[i]);
        end
    endgenerate

    // Final Addition with +1 correction for two's complement:
    // diff_2nd = csa_sum + (csa_carry << 1) + 1
    kogge_stone_adder_32b u_final_adder (
        .a   (csa_sum),
        .b   ({csa_carry[30:0], 1'b1}), // incorporates +1 LSB
        .cin (1'b0),
        .sum (diff_2nd),
        .cout()
    );

endmodule
