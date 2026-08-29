// =============================================================================
// File Name   : q16_soft_threshold.sv
// Module Name : q16_soft_threshold
// Project     : Alternating Direction Method of Multipliers (ADMM) Accelerator (Solver #11)
// -----------------------------------------------------------------------------
// Description: Combinational Single-Cycle Soft-Thresholding Unit:
//   S_tau(v) = sign(v) * max(|v| - tau, 0)
// =============================================================================

`timescale 1ns / 1ps

import admm_types_pkg::*;

module q16_soft_threshold (
    input  q16_t v_in,         // Input argument (x_{k+1} + u_k)
    input  q16_t tau_in,       // Threshold tau = lambda / rho
    output q16_t s_out,        // Soft-thresholded result z_{k+1}
    output logic is_zero       // Zero indicator
);

    always_comb begin
        if (tau_in <= Q16_ZERO) begin
            s_out   = v_in;
            is_zero = (v_in == Q16_ZERO);
        end else if (v_in > tau_in) begin
            s_out   = v_in - tau_in;
            is_zero = 1'b0;
        end else if (v_in < -tau_in) begin
            s_out   = v_in + tau_in;
            is_zero = 1'b0;
        end else begin
            s_out   = Q16_ZERO;
            is_zero = 1'b1;
        end
    end

endmodule
