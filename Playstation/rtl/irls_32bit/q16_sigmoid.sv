// =============================================================================
// File Name   : q16_sigmoid.sv
// Module Name : q16_sigmoid
// Project     : Iteratively Reweighted Least Squares (IRLS) Accelerator (Solver #3)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Combinational High-Precision Q16.16 Sigmoid Activation Engine:
//     p = σ(η) = 1 / (1 + e^-η)
//   Uses symmetric piecewise linear spline interpolation for fast, accurate
//   probability evaluation in physical machine learning & logistic regression.
// =============================================================================

`timescale 1ns / 1ps

import irls_types_pkg::*;

module q16_sigmoid (
    input  logic signed [31:0] eta,  // Input linear predictor η = xᵀ · w
    output logic signed [31:0] prob  // Output probability p in [0.0, 1.0]
);

    logic signed [31:0] abs_eta;
    logic signed [31:0] p_mag;
    logic               is_neg;

    assign is_neg  = eta[31];
    assign abs_eta = is_neg ? (-eta) : eta;

    // Helper fixed-point multiplier (Q16.16)
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Spline Segments (Symmetric about η = 0)
    always @(*) begin
        if (abs_eta >= 32'h0004_0000) begin // |η| >= 4.0
            p_mag = Q16_ONE; // 1.0
        end else if (abs_eta >= 32'h0002_6000) begin // 2.375 <= |η| < 4.0
            // p = 0.9304 + 0.0350 * (|η| - 2.375)
            p_mag = 32'sd60975 + q16_mul(32'sd2294, (abs_eta - 32'h0002_6000));
        end else if (abs_eta >= 32'h0001_0000) begin // 1.0 <= |η| < 2.375
            // p = 0.7311 + 0.1450 * (|η| - 1.0)
            p_mag = 32'sd47913 + q16_mul(32'sd9503, (abs_eta - 32'h0001_0000));
        end else begin // 0.0 <= |η| < 1.0
            // p = 0.5000 + 0.2311 * |η|
            p_mag = Q16_HALF + q16_mul(32'sd15145, abs_eta);
        end

        // Symmetry: σ(-η) = 1 - σ(η)
        if (is_neg) begin
            prob = Q16_ONE - p_mag;
        end else begin
            prob = p_mag;
        end
    end

endmodule
