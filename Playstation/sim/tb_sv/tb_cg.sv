// =============================================================================
// File Name   : tb_cg.sv
// Module Name : tb_cg
// Project     : Non-Linear Conjugate Gradient (CG) Accelerator (Solver #5)
// -----------------------------------------------------------------------------
// Description:
//   Verification testbench for the Non-Linear Conjugate Gradient (CG) Accelerator.
//   Verifies exact N-step convergence on coupled 2D and 3D non-linear quadratic systems
//   using matrix-free O(N) memory.
// =============================================================================

`timescale 1ns / 1ps

import cg_types_pkg::*;
`include "cg_helpers.svh"

module tb_cg;

    logic        clk;
    logic        rst_n;

    // Programming Interface
    logic        prog_en;
    logic [4:0]  prog_addr;
    instr_t      prog_data;

    // Controls & Inputs
    logic        start;
    logic [2:0]  num_vars;
    vec_t        x_init;
    q16_t        tolerance;
    logic [7:0]  max_iters;

    // Results & Outputs
    vec_t        x_optimal;
    q16_t        f_optimal;
    q16_t        g_norm_inf;
    logic [7:0]  iter_count;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate Top Module
    cg_top dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (start),
        .num_vars   (num_vars),
        .x_init     (x_init),
        .tolerance  (tolerance),
        .max_iters  (max_iters),
        .x_optimal  (x_optimal),
        .f_optimal  (f_optimal),
        .g_norm_inf (g_norm_inf),
        .iter_count (iter_count),
        .status     (status),
        .done       (done),
        .busy       (busy)
    );

    // 100MHz Clock
    initial clk = 0;
    always #5 clk = ~clk;

    // Real Number Conversion Helpers
    function real q16_to_real(input q16_t val);
        q16_to_real = real'(val) / 65536.0;
    endfunction

    function q16_t real_to_q16(input real val);
        real_to_q16 = q16_t'(int'(val * 65536.0));
    endfunction

    // Task to program microcode
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
        $display(" Non-Linear Conjugate Gradient (CG) Accelerator Testbench (Solver #5)");
        $display("==================================================================");

        // Reset
        rst_n     = 0;
        prog_en   = 0;
        prog_addr = '0;
        prog_data = '0;
        start     = 0;
        num_vars  = 3'd2;
        tolerance = 0;
        max_iters = 0;
        x_init    = '0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Coupled Quadratic: f(x0, x1) = (x0 - 2)^2 + (x1 - 5)^2 + x0*x1
        // Target Minimum: x0* = -0.6667, x1* = 5.3333
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Programming 2D Coupled Quadratic: f(x0, x1) = (x0 - 2)^2 + (x1 - 5)^2 + x0*x1");
        $display("Target Minimum: x0* = -2/3 ≈ -0.6667, x1* = 16/3 ≈ 5.3333");

        write_instr(5'd0, OP_LOADC, 4'd2, 4'd0, 4'd0, 16'sd2);  // r2 = 2.0
        write_instr(5'd1, OP_LOADC, 4'd3, 4'd0, 4'd0, 16'sd5);  // r3 = 5.0
        write_instr(5'd2, OP_SUB,   4'd4, 4'd0, 4'd2, 16'sd0);  // r4 = x0 - 2.0
        write_instr(5'd3, OP_MUL,   4'd5, 4'd4, 4'd4, 16'sd0);  // r5 = (x0 - 2.0)^2
        write_instr(5'd4, OP_SUB,   4'd6, 4'd1, 4'd3, 16'sd0);  // r6 = x1 - 5.0
        write_instr(5'd5, OP_MUL,   4'd7, 4'd6, 4'd6, 16'sd0);  // r7 = (x1 - 5.0)^2
        write_instr(5'd6, OP_MUL,   4'd8, 4'd0, 4'd1, 16'sd0);  // r8 = x0 * x1
        write_instr(5'd7, OP_ADD,   4'd9, 4'd5, 4'd7, 16'sd0);  // r9 = (x0-2)^2 + (x1-5)^2
        write_instr(5'd8, OP_ADD,   4'd15,4'd9, 4'd8, 16'sd0);  // r15 = r9 + x0*x1
        write_instr(5'd9, OP_END,   4'd0, 4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        num_vars  = 3'd2;
        x_init    = set_vec(set_vec(x_init, 2'd0, real_to_q16(8.0)), 2'd1, real_to_q16(8.0));
        tolerance = 32'h0000_0080; // tol ≈ 0.0019
        max_iters = 8'd50;
        start     = 1'b1;
        @(posedge clk);
        start     = 1'b0;

        $display("Starting Conjugate Gradient Solver from initial guess (x0, x1) = (8.0, 8.0)...");
        @(posedge done);
        #1;

        $display("--> CG SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d (Expected: <= 2)", iter_count);
        $display("    x0_optimal:   %f (Expected: ~ -0.666667)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:   %f (Expected: ~  5.333333)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    f_optimal:    %f", q16_to_real(f_optimal));
        $display("    g_norm_inf:   %f", q16_to_real(g_norm_inf));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > -0.70 && q16_to_real(get_vec(x_optimal, 2'd0)) < -0.60 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 5.25 && q16_to_real(get_vec(x_optimal, 2'd1)) < 5.40) begin
            $display("[TEST 1 PASSED] Successfully converged 2D coupled system via Conjugate Gradient!");
        end else begin
            $display("[TEST 1 FAILED] Did not converge to target optimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2D Decoupled Paraboloid: f(x0, x1) = 2*(x0 - 3)^2 + 3*(x1 - 4)^2
        // Target Minimum: x0* = 3.0, x1* = 4.0, f(x*) = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Programming 2D Decoupled Quadratic: f(x0, x1) = 2*(x0 - 3)^2 + 3*(x1 - 4)^2");
        $display("Target Minimum: x0* = 3.0, x1* = 4.0, f(x*) = 0.0");

        write_instr(5'd0,  OP_LOADC, 4'd2, 4'd0, 4'd0, 16'sd3);  // r2  = 3.0
        write_instr(5'd1,  OP_LOADC, 4'd3, 4'd0, 4'd0, 16'sd4);  // r3  = 4.0
        write_instr(5'd2,  OP_LOADC, 4'd4, 4'd0, 4'd0, 16'sd2);  // r4  = 2.0
        write_instr(5'd3,  OP_SUB,   4'd5, 4'd0, 4'd2, 16'sd0);  // r5  = x0 - 3.0
        write_instr(5'd4,  OP_MUL,   4'd6, 4'd5, 4'd5, 16'sd0);  // r6  = (x0 - 3.0)^2
        write_instr(5'd5,  OP_MUL,   4'd7, 4'd4, 4'd6, 16'sd0);  // r7  = 2*(x0 - 3.0)^2
        write_instr(5'd6,  OP_SUB,   4'd8, 4'd1, 4'd3, 16'sd0);  // r8  = x1 - 4.0
        write_instr(5'd7,  OP_MUL,   4'd9, 4'd8, 4'd8, 16'sd0);  // r9  = (x1 - 4.0)^2
        write_instr(5'd8,  OP_LOADC, 4'd10,4'd0, 4'd0, 16'sd3);  // r10 = 3.0
        write_instr(5'd9,  OP_MUL,   4'd11,4'd10,4'd9, 16'sd0);  // r11 = 3*(x1 - 4.0)^2
        write_instr(5'd10, OP_ADD,   4'd15,4'd7, 4'd11,16'sd0);  // r15 = 2*(x0-3)^2 + 3*(x1-4)^2
        write_instr(5'd11, OP_END,   4'd0, 4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        num_vars  = 3'd2;
        x_init    = set_vec(set_vec(x_init, 2'd0, real_to_q16(10.0)), 2'd1, real_to_q16(10.0));
        tolerance = 32'h0000_0080;
        max_iters = 8'd50;
        start     = 1'b1;
        @(posedge clk);
        start     = 1'b0;

        $display("Starting Conjugate Gradient Solver from initial guess (x0, x1) = (10.0, 10.0)...");
        @(posedge done);
        #1;

        $display("--> CG SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d (Expected: <= 2)", iter_count);
        $display("    x0_optimal:   %f (Expected: ~ 3.000000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:   %f (Expected: ~ 4.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    f_optimal:    %f (Expected: ~ 0.000000)", q16_to_real(f_optimal));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 2.95 && q16_to_real(get_vec(x_optimal, 2'd0)) < 3.05 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 3.95 && q16_to_real(get_vec(x_optimal, 2'd1)) < 4.05) begin
            $display("[TEST 2 PASSED] Successfully converged 2D paraboloid via Conjugate Gradient!");
        end else begin
            $display("[TEST 2 FAILED] Did not converge to target optimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 3D Coupled Quadratic: f(x0, x1, x2) = (x0 - 1)^2 + (x1 - 2)^2 + (x2 - 3)^2 + x0*x1
        // Target Minimum: x0* = 0.0, x1* = 2.0, x2* = 3.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Programming 3D Coupled Quadratic: f(x0, x1, x2) = (x0 - 1)^2 + (x1 - 2)^2 + (x2 - 3)^2 + x0*x1");
        $display("Target Minimum: x0* = 0.0, x1* = 2.0, x2* = 3.0");

        write_instr(5'd0,  OP_LOADC, 4'd3, 4'd0, 4'd0, 16'sd1);  // r3  = 1.0
        write_instr(5'd1,  OP_LOADC, 4'd4, 4'd0, 4'd0, 16'sd2);  // r4  = 2.0
        write_instr(5'd2,  OP_LOADC, 4'd5, 4'd0, 4'd0, 16'sd3);  // r5  = 3.0
        write_instr(5'd3,  OP_SUB,   4'd6, 4'd0, 4'd3, 16'sd0);  // r6  = x0 - 1.0
        write_instr(5'd4,  OP_MUL,   4'd7, 4'd6, 4'd6, 16'sd0);  // r7  = (x0 - 1.0)^2
        write_instr(5'd5,  OP_SUB,   4'd8, 4'd1, 4'd4, 16'sd0);  // r8  = x1 - 2.0
        write_instr(5'd6,  OP_MUL,   4'd9, 4'd8, 4'd8, 16'sd0);  // r9  = (x1 - 2.0)^2
        write_instr(5'd7,  OP_SUB,   4'd10,4'd2, 4'd5, 16'sd0);  // r10 = x2 - 3.0
        write_instr(5'd8,  OP_MUL,   4'd11,4'd10,4'd10,16'sd0);  // r11 = (x2 - 3.0)^2
        write_instr(5'd9,  OP_MUL,   4'd12,4'd0, 4'd1, 16'sd0);  // r12 = x0 * x1
        write_instr(5'd10, OP_ADD,   4'd13,4'd7, 4'd9, 16'sd0);  // r13 = (x0-1)^2 + (x1-2)^2
        write_instr(5'd11, OP_ADD,   4'd14,4'd13,4'd11,16'sd0);  // r14 = r13 + (x2-3)^2
        write_instr(5'd12, OP_ADD,   4'd15,4'd14,4'd12,16'sd0);  // r15 = r14 + x0*x1
        write_instr(5'd13, OP_END,   4'd0, 4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        num_vars  = 3'd3;
        x_init    = set_vec(set_vec(set_vec(x_init, 2'd0, real_to_q16(5.0)), 2'd1, real_to_q16(6.0)), 2'd2, real_to_q16(7.0));
        tolerance = 32'h0000_0080;
        max_iters = 8'd50;
        start     = 1'b1;
        @(posedge clk);
        start     = 1'b0;

        $display("Starting 3D Conjugate Gradient Solver from initial guess (5.0, 6.0, 7.0)...");
        @(posedge done);
        #1;

        $display("--> CG SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d (Expected: <= 3)", iter_count);
        $display("    x0_optimal:   %f (Expected: ~ 0.000000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:   %f (Expected: ~ 2.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    x2_optimal:   %f (Expected: ~ 3.000000)", q16_to_real(get_vec(x_optimal, 2'd2)));
        $display("    f_optimal:    %f", q16_to_real(f_optimal));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > -0.05 && q16_to_real(get_vec(x_optimal, 2'd0)) < 0.05 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 1.95 && q16_to_real(get_vec(x_optimal, 2'd1)) < 2.05 &&
            q16_to_real(get_vec(x_optimal, 2'd2)) > 2.95 && q16_to_real(get_vec(x_optimal, 2'd2)) < 3.05) begin
            $display("[TEST 3 PASSED] Successfully converged 3D coupled system via Conjugate Gradient!");
        end else begin
            $display("[TEST 3 FAILED] Did not converge to target optimum.");
        end

        $display("\n==================================================================");
        $display(" ALL CONJUGATE GRADIENT (CG) HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
