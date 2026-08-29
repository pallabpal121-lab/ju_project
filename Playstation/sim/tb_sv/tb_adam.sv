// =============================================================================
// File Name   : tb_adam.sv
// Module Name : tb_adam
// Project     : Adaptive Moment Estimation (Adam) Accelerator (Solver #17)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for the Adam Neural Accelerator.
//   Verifies:
//   1. Adam Mode: Ill-Conditioned Anisotropic Valley Optimization
//   2. RMSProp Mode: Non-Stationary Ridge Tracking
//   3. Momentum SGD Mode: 3D Weight Regularization with L2 Decay
// =============================================================================

`timescale 1ns / 1ps

import adam_types_pkg::*;
`include "adam_helpers.svh"

module tb_adam;

    logic        clk;
    logic        rst_n;

    // Programming Interface
    logic        prog_en;
    logic [4:0]  prog_addr;
    instr_t      prog_data;

    // Algorithm Controls & Inputs
    logic        start;
    opt_mode_t   opt_mode;
    logic [2:0]  num_dims;
    q16_t        alpha_lr;
    q16_t        beta1_val;
    q16_t        beta2_val;
    q16_t        weight_decay;
    vec_t        theta_init;
    q16_t        tolerance;
    logic [7:0]  max_iters;

    // Results & Outputs
    vec_t        theta_optimal;
    q16_t        f_loss_optimal;
    q16_t        grad_norm_inf;
    q16_t        dtheta_norm_inf;
    logic [7:0]  iter_count;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate Top Module
    adam_top dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .prog_en        (prog_en),
        .prog_addr      (prog_addr),
        .prog_data      (prog_data),
        .start          (start),
        .opt_mode       (opt_mode),
        .num_dims       (num_dims),
        .alpha_lr       (alpha_lr),
        .beta1_val      (beta1_val),
        .beta2_val      (beta2_val),
        .weight_decay   (weight_decay),
        .theta_init     (theta_init),
        .tolerance      (tolerance),
        .max_iters      (max_iters),
        .theta_optimal  (theta_optimal),
        .f_loss_optimal (f_loss_optimal),
        .grad_norm_inf  (grad_norm_inf),
        .dtheta_norm_inf(dtheta_norm_inf),
        .iter_count     (iter_count),
        .status         (status),
        .done           (done),
        .busy           (busy)
    );

    // 100MHz Clock Generator
    initial clk = 0;
    always #5 clk = ~clk;

    // Fixed-point Real Conversion Helpers
    function real q16_to_real(input q16_t val);
        q16_to_real = real'(val) / 65536.0;
    endfunction

    function q16_t real_to_q16(input real val);
        real_to_q16 = q16_t'(int'(val * 65536.0));
    endfunction

    // Task to program microcode instruction
    task write_instr(input logic [4:0] addr, input opcode_t op, input logic [3:0] dst, input logic [3:0] src_a, input logic [3:0] src_b, input logic signed [15:0] imm);
        begin
            @(posedge clk);
            prog_en         <= 1'b1;
            prog_addr       <= addr;
            prog_data.op    <= op;
            prog_data.dst   <= dst;
            prog_data.src_a <= src_a;
            prog_data.src_b <= src_b;
            prog_data.imm   <= imm;
            @(posedge clk);
            prog_en         <= 1'b0;
        end
    endtask

    initial begin
        $display("==================================================================");
        $display(" Adam / RMSProp / Momentum Neural Accelerator TB (Solver #17)");
        $display("==================================================================");

        // Reset
        rst_n        = 0;
        prog_en      = 0;
        prog_addr    = '0;
        prog_data    = '0;
        start        = 0;
        opt_mode     = MODE_ADAM;
        num_dims     = 3'd2;
        alpha_lr     = 0;
        beta1_val    = 0;
        beta2_val    = 0;
        weight_decay = 0;
        theta_init   = '0;
        tolerance    = 0;
        max_iters    = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: Adam Mode (MODE_ADAM) - Ill-Conditioned Anisotropic Valley
        // Loss: f(θ0, θ1) = 0.5*(θ0 - 2)^2 + 4*(θ1 - 3)^2
        // Target Minimum: θ0* = 2.000000, θ1* = 3.000000, f(θ*) = 0.000000
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Adam Mode (MODE_ADAM) - Ill-Conditioned Anisotropic Valley");
        $display("Loss: f(θ0, θ1) = 0.5*(θ0 - 2)^2 + 4*(θ1 - 3)^2");
        $display("Target Minimum: θ0* = 2.000000, θ1* = 3.000000, f(θ*) = 0.000000");

        // Microcode Program:
        // r0 = θ0, r1 = θ1
        // r4 = 2.0, r5 = 3.0, r6 = 4.0
        // r7 = θ0 - 2.0
        // r8 = (θ0 - 2.0)^2
        // r8 = 0.5 * (θ0 - 2.0)^2 (via div by 2)
        // r9 = θ1 - 3.0
        // r10 = (θ1 - 3.0)^2
        // r11 = 4 * (θ1 - 3.0)^2
        // r15 = r8 + r11
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd2);  // r4  = 2.0
        write_instr(5'd1, OP_LOADC, 4'd5,  4'd0, 4'd0, 16'sd3);  // r5  = 3.0
        write_instr(5'd2, OP_LOADC, 4'd6,  4'd0, 4'd0, 16'sd4);  // r6  = 4.0
        write_instr(5'd3, OP_SUB,   4'd7,  4'd0, 4'd4, 16'sd0);  // r7  = θ0 - 2.0
        write_instr(5'd4, OP_MUL,   4'd8,  4'd7, 4'd7, 16'sd0);  // r8  = (θ0 - 2.0)^2
        write_instr(5'd5, OP_DIV,   4'd8,  4'd8, 4'd4, 16'sd0);  // r8  = 0.5 * (θ0 - 2.0)^2
        write_instr(5'd6, OP_SUB,   4'd9,  4'd1, 4'd5, 16'sd0);  // r9  = θ1 - 3.0
        write_instr(5'd7, OP_MUL,   4'd10, 4'd9, 4'd9, 16'sd0);  // r10 = (θ1 - 3.0)^2
        write_instr(5'd8, OP_MUL,   4'd11, 4'd6, 4'd10, 16'sd0); // r11 = 4 * (θ1 - 3.0)^2
        write_instr(5'd9, OP_ADD,   4'd15, 4'd8, 4'd11, 16'sd0); // r15 = f(θ)
        write_instr(5'd10,OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        opt_mode     = MODE_ADAM;
        num_dims     = 3'd2;
        alpha_lr     = real_to_q16(0.35);
        beta1_val    = real_to_q16(0.90);
        beta2_val    = real_to_q16(0.99);
        weight_decay = Q16_ZERO;
        theta_init   = set_vec(set_vec('0, 2'd0, real_to_q16(0.0)), 2'd1, real_to_q16(0.0));
        tolerance    = 32'h0000_0080; // tol ≈ 0.00195
        max_iters    = 8'd80;
        start        = 1'b1;
        @(posedge clk);
        start        = 1'b0;

        $display("Starting Adam Neural Optimizer from initial guess (0.0, 0.0)...");
        @(posedge done);
        #1;

        $display("--> ADAM SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Epochs:         %0d", iter_count);
        $display("    θ0_optimal:     %f (Expected: ~ 2.000000)", q16_to_real(get_vec(theta_optimal, 2'd0)));
        $display("    θ1_optimal:     %f (Expected: ~ 3.000000)", q16_to_real(get_vec(theta_optimal, 2'd1)));
        $display("    f_loss_optimal: %f (Expected: ~ 0.000000)", q16_to_real(f_loss_optimal));
        $display("    grad_norm_inf:  %f", q16_to_real(grad_norm_inf));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(theta_optimal, 2'd0)) > 1.95 && q16_to_real(get_vec(theta_optimal, 2'd0)) < 2.05 &&
            q16_to_real(get_vec(theta_optimal, 2'd1)) > 2.95 && q16_to_real(get_vec(theta_optimal, 2'd1)) < 3.05) begin
            $display("[TEST 1 PASSED] Successfully optimized anisotropic valley via Adam in hardware!");
        end else begin
            $display("[TEST 1 FAILED] Adam did not converge to target minimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: RMSProp Mode (MODE_RMSPROP) - Non-Stationary Ridge Tracking
        // Loss: f(θ0, θ1) = (θ0 - 4)^2 + 2*(θ1 + 1)^2
        // Target Minimum: θ0* = 4.000000, θ1* = -1.000000, f(θ*) = 0.000000
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] RMSProp Mode (MODE_RMSPROP) - Ridge Tracking");
        $display("Loss: f(θ0, θ1) = (θ0 - 4)^2 + 2*(θ1 + 1)^2");
        $display("Target Minimum: θ0* = 4.000000, θ1* = -1.000000, f(θ*) = 0.000000");

        // Microcode Program:
        // r0 = θ0, r1 = θ1
        // r4 = 4.0, r5 = 1.0, r6 = 2.0
        // r7 = θ0 - 4.0
        // r8 = (θ0 - 4.0)^2
        // r9 = θ1 + 1.0
        // r10 = (θ1 + 1.0)^2
        // r11 = 2 * (θ1 + 1.0)^2
        // r15 = r8 + r11
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd4);  // r4  = 4.0
        write_instr(5'd1, OP_LOADC, 4'd5,  4'd0, 4'd0, 16'sd1);  // r5  = 1.0
        write_instr(5'd2, OP_LOADC, 4'd6,  4'd0, 4'd0, 16'sd2);  // r6  = 2.0
        write_instr(5'd3, OP_SUB,   4'd7,  4'd0, 4'd4, 16'sd0);  // r7  = θ0 - 4.0
        write_instr(5'd4, OP_MUL,   4'd8,  4'd7, 4'd7, 16'sd0);  // r8  = (θ0 - 4.0)^2
        write_instr(5'd5, OP_ADD,   4'd9,  4'd1, 4'd5, 16'sd0);  // r9  = θ1 + 1.0
        write_instr(5'd6, OP_MUL,   4'd10, 4'd9, 4'd9, 16'sd0);  // r10 = (θ1 + 1.0)^2
        write_instr(5'd7, OP_MUL,   4'd11, 4'd6, 4'd10, 16'sd0); // r11 = 2 * (θ1 + 1.0)^2
        write_instr(5'd8, OP_ADD,   4'd15, 4'd8, 4'd11, 16'sd0); // r15 = f(θ)
        write_instr(5'd9, OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        opt_mode     = MODE_RMSPROP;
        num_dims     = 3'd2;
        alpha_lr     = real_to_q16(0.20);
        beta2_val    = real_to_q16(0.90);
        weight_decay = Q16_ZERO;
        theta_init   = set_vec(set_vec('0, 2'd0, real_to_q16(1.0)), 2'd1, real_to_q16(2.0));
        tolerance    = 32'h0000_0080;
        max_iters    = 8'd40;
        start        = 1'b1;
        @(posedge clk);
        start        = 1'b0;

        $display("Starting RMSProp Optimizer...");
        @(posedge done);
        #1;

        $display("--> RMSPROP SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Epochs:         %0d", iter_count);
        $display("    θ0_optimal:     %f (Expected: ~ 4.000000)", q16_to_real(get_vec(theta_optimal, 2'd0)));
        $display("    θ1_optimal:     %f (Expected: ~ -1.000000)", q16_to_real(get_vec(theta_optimal, 2'd1)));
        $display("    f_loss_optimal: %f (Expected: ~ 0.000000)", q16_to_real(f_loss_optimal));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(theta_optimal, 2'd0)) > 3.95 && q16_to_real(get_vec(theta_optimal, 2'd0)) < 4.05 &&
            q16_to_real(get_vec(theta_optimal, 2'd1)) > -1.05 && q16_to_real(get_vec(theta_optimal, 2'd1)) < -0.95) begin
            $display("[TEST 2 PASSED] Successfully trained via RMSProp in hardware!");
        end else begin
            $display("[TEST 2 FAILED] RMSProp did not converge.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: Momentum SGD Mode (MODE_MOMENTUM) with Weight Decay
        // Loss: f(θ0, θ1, θ2) = (θ0 - 1)^2 + (θ1 - 2)^2 + (θ2 - 3)^2
        // With weight decay λ = 0.1, target is θ* ≈ [0.952381, 1.904762, 2.857143]
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Momentum SGD Mode with L2 Weight Decay (λ = 0.1)");
        $display("Loss: f(θ0, θ1, θ2) = (θ0 - 1)^2 + (θ1 - 2)^2 + (θ2 - 3)^2 + 0.1*||θ||^2");
        $display("Target Regularized Weights: θ* ≈ [0.952381, 1.904762, 2.857143]");

        // Microcode Program:
        // r0 = θ0, r1 = θ1, r2 = θ2
        // r4 = 1.0, r5 = 2.0, r6 = 3.0
        // r7 = (θ0 - 1)^2, r8 = (θ1 - 2)^2, r9 = (θ2 - 3)^2
        // r15 = r7 + r8 + r9
        write_instr(5'd0, OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd1);  // r4 = 1.0
        write_instr(5'd1, OP_LOADC, 4'd5,  4'd0, 4'd0, 16'sd2);  // r5 = 2.0
        write_instr(5'd2, OP_LOADC, 4'd6,  4'd0, 4'd0, 16'sd3);  // r6 = 3.0
        write_instr(5'd3, OP_SUB,   4'd7,  4'd0, 4'd4, 16'sd0);  // r7 = θ0 - 1.0
        write_instr(5'd4, OP_MUL,   4'd7,  4'd7, 4'd7, 16'sd0);  // r7 = (θ0 - 1.0)^2
        write_instr(5'd5, OP_SUB,   4'd8,  4'd1, 4'd5, 16'sd0);  // r8 = θ1 - 2.0
        write_instr(5'd6, OP_MUL,   4'd8,  4'd8, 4'd8, 16'sd0);  // r8 = (θ1 - 2.0)^2
        write_instr(5'd7, OP_SUB,   4'd9,  4'd2, 4'd6, 16'sd0);  // r9 = θ2 - 3.0
        write_instr(5'd8, OP_MUL,   4'd9,  4'd9, 4'd9, 16'sd0);  // r9 = (θ2 - 3.0)^2
        write_instr(5'd9, OP_ADD,   4'd10, 4'd7, 4'd8, 16'sd0);  // r10 = r7 + r8
        write_instr(5'd10,OP_ADD,   4'd15, 4'd10,4'd9, 16'sd0);  // r15 = r7 + r8 + r9
        write_instr(5'd11,OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        opt_mode     = MODE_MOMENTUM;
        num_dims     = 3'd3;
        alpha_lr     = real_to_q16(0.20);
        beta1_val    = real_to_q16(0.80);
        weight_decay = real_to_q16(0.10);
        theta_init   = '0;
        tolerance    = 32'h0000_0080;
        max_iters    = 8'd60;
        start        = 1'b1;
        @(posedge clk);
        start        = 1'b0;

        $display("Starting Momentum SGD with Weight Decay...");
        @(posedge done);
        #1;

        $display("--> MOMENTUM SGD FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Epochs:         %0d", iter_count);
        $display("    θ0_optimal:     %f (Expected: ~ 0.952381)", q16_to_real(get_vec(theta_optimal, 2'd0)));
        $display("    θ1_optimal:     %f (Expected: ~ 1.904762)", q16_to_real(get_vec(theta_optimal, 2'd1)));
        $display("    θ2_optimal:     %f (Expected: ~ 2.857143)", q16_to_real(get_vec(theta_optimal, 2'd2)));
        $display("    f_loss_optimal: %f", q16_to_real(f_loss_optimal));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(theta_optimal, 2'd0)) > 0.93 && q16_to_real(get_vec(theta_optimal, 2'd0)) < 0.98 &&
            q16_to_real(get_vec(theta_optimal, 2'd1)) > 1.88 && q16_to_real(get_vec(theta_optimal, 2'd1)) < 1.96 &&
            q16_to_real(get_vec(theta_optimal, 2'd2)) > 2.82 && q16_to_real(get_vec(theta_optimal, 2'd2)) < 2.94) begin
            $display("[TEST 3 PASSED] Successfully trained multi-layer weights with L2 regularization!");
        end else begin
            $display("[TEST 3 FAILED] Momentum SGD did not converge.");
        end

        $display("\n==================================================================");
        $display(" ALL ADAM NEURAL ACCELERATOR HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
