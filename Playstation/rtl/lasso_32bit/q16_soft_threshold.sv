// =============================================================================
// File Name   : q16_soft_threshold.sv
// Module Name : q16_soft_threshold
// Project     : Coordinate Descent / LASSO L1 Sparsity Accelerator (Solver #10)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Single-cycle combinational Hardware Soft-Thresholding Operator S_lambda(z):
//     S_lambda(z) = sign(z) * max(|z| - lambda, 0)
//     - If z >  lambda -> return z - lambda
//     - If z < -lambda -> return z + lambda
//     - If |z| <= lambda -> return 0.0 (Exact Silicon Zero!)
// =============================================================================

`timescale 1ns / 1ps

import lasso_types_pkg::*;

module q16_soft_threshold (
    input  q16_t z_in,         // Unregularized correlation / step
    input  q16_t lambda_in,    // Regularization parameter (lambda >= 0)
    output q16_t s_out,        // Soft-thresholded result
    output logic is_zero       // High when exact zero is selected (sparsity indicator)
);

    always_comb begin
        if (lambda_in <= Q16_ZERO) begin
            // Unregularized OLS bypass
            s_out   = z_in;
            is_zero = (z_in == Q16_ZERO);
        end else if (z_in > lambda_in) begin
            s_out   = z_in - lambda_in;
            is_zero = 1'b0;
        end else if (z_in < -lambda_in) begin
            s_out   = z_in + lambda_in;
            is_zero = 1'b0;
        end else begin
            // Shrinkage to exact zero
            s_out   = Q16_ZERO;
            is_zero = 1'b1;
        end
    end

endmodule
