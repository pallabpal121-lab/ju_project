// =============================================================================
// File Name   : tb_nelder_mead.sv
// Module Name : tb_nelder_mead
// Project     : Nelder-Mead Simplex Direct Search Accelerator (Solver #8)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for the Nelder-Mead Accelerator.
//   Verifies 2D Paraboloid, 2D Coupled Quadratic, and Non-Differentiable Piecewise
//   Absolute Value optimization without using any derivatives.
// =============================================================================

`timescale 1ns / 1ps

import nm_types_pkg::*;
`include "nm_helpers.svh"

module tb_nelder_mead;

    logic        clk;
    logic        rst_n;

    // Programming Interface
    logic        prog_en;
    logic [4:0]  prog_addr;
    instr_t      prog_data;

    // Controls & Inputs
    logic        start;
    logic [2:0]  num_params;
    vec_t        x_init;
    q16_t        init_step;
    q16_t        tolerance;
    logic [7:0]  max_iters;

    // Results & Outputs
    vec_t        x_optimal;
    q16_t        f_optimal;
    q16_t        simplex_radius;
    logic [7:0]  iter_count;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate Top Module
    nelder_mead_top dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .prog_en        (prog_en),
        .prog_addr      (prog_addr),
        .prog_data      (prog_data),
        .start          (start),
        .num_params     (num_params),
        .x_init         (x_init),
        .init_step      (init_step),
        .tolerance      (tolerance),
        .max_iters      (max_iters),
        .x_optimal      (x_optimal),
        .f_optimal      (f_optimal),
        .simplex_radius (simplex_radius),
        .iter_count     (iter_count),
        .status         (status),
        .done           (done),
        .busy           (busy)
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
        $display(" Nelder-Mead Simplex Direct Search Accelerator TB (Solver #8)");
        $display("==================================================================");

        // Reset
        rst_n      = 0;
        prog_en    = 0;
        prog_addr  = '0;
        prog_data  = '0;
        start      = 0;
        num_params = 3'd2;
        x_init     = '0;
        init_step  = 0;
        tolerance  = 0;
        max_iters  = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Decoupled Quadratic Paraboloid
        // Objective: f(x0, x1) = 2*(x0 - 3)^2 + 3*(x1 - 4)^2
        // Target Optimum: x0* = 3.000, x1* = 4.000, f(x*) = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2D Paraboloid Direct Search");
        $display("Objective: f(x0, x1) = 2*(x0 - 3)^2 + 3*(x1 - 4)^2");
        $display("Target Minimum: x0* = 3.000000, x1* = 4.000000, f(x*) = 0.000000");

        // Microcode Program:
        // r0 = x0, r1 = x1
        // r4 = 3.0, r5 = x0 - 3.0, r6 = (x0 - 3.0)^2, r7 = 2.0, r8 = 2*(x0-3)^2
        // r9 = 4.0, r10 = x1 - 4.0, r11 = (x1 - 4.0)^2, r12 = 3.0, r13 = 3*(x1-4)^2
        // r15 = r8 + r13
        write_instr(5'd0,  OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd3);       // r4  = 3.0
        write_instr(5'd1,  OP_SUB,   4'd5,  4'd0, 4'd4, 16'sd0);       // r5  = x0 - 3.0
        write_instr(5'd2,  OP_MUL,   4'd6,  4'd5, 4'd5, 16'sd0);       // r6  = (x0 - 3.0)^2
        write_instr(5'd3,  OP_LOADC, 4'd7,  4'd0, 4'd0, 16'sd2);       // r7  = 2.0
        write_instr(5'd4,  OP_MUL,   4'd8,  4'd6, 4'd7, 16'sd0);       // r8  = 2 * (x0 - 3)^2
        write_instr(5'd5,  OP_LOADC, 4'd9,  4'd0, 4'd0, 16'sd4);       // r9  = 4.0
        write_instr(5'd6,  OP_SUB,   4'd10, 4'd1, 4'd9, 16'sd0);       // r10 = x1 - 4.0
        write_instr(5'd7,  OP_MUL,   4'd11, 4'd10, 4'd10, 16'sd0);     // r11 = (x1 - 4.0)^2
        write_instr(5'd8,  OP_LOADC, 4'd12, 4'd0, 4'd0, 16'sd3);       // r12 = 3.0
        write_instr(5'd9,  OP_MUL,   4'd13, 4'd11, 4'd12, 16'sd0);     // r13 = 3 * (x1 - 4)^2
        write_instr(5'd10, OP_ADD,   4'd15, 4'd8, 4'd13, 16'sd0);      // r15 = r8 + r13
        write_instr(5'd11, OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);       // END

        @(posedge clk);
        num_params = 3'd2;
        x_init     = set_vec(set_vec(x_init, 2'd0, real_to_q16(8.0)), 2'd1, real_to_q16(8.0));
        init_step  = real_to_q16(2.0);
        tolerance  = 32'h0000_0100;
        max_iters  = 8'd100;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting Nelder-Mead Simplex Solver from initial guess (8.0, 8.0)...");
        @(posedge done);
        #1;

        $display("--> NELDER-MEAD SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Iterations:     %0d", iter_count);
        $display("    x0_optimal:     %f (Expected: ~ 3.000000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:     %f (Expected: ~ 4.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    f_optimal:      %f (Expected: ~ 0.000000)", q16_to_real(f_optimal));
        $display("    simplex_radius: %f", q16_to_real(simplex_radius));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 2.90 && q16_to_real(get_vec(x_optimal, 2'd0)) < 3.10 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 3.90 && q16_to_real(get_vec(x_optimal, 2'd1)) < 4.10) begin
            $display("[TEST 1 PASSED] Successfully converged 2D paraboloid via Nelder-Mead!");
        end else begin
            $display("[TEST 1 FAILED] Did not converge to target minimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2D Coupled Quadratic Function
        // Objective: f(x0, x1) = (x0 - 2)^2 + (x1 - 5)^2 + x0*x1
        // Target Optimum: x0* = -2/3 ≈ -0.6667, x1* = 16/3 ≈ 5.3333, f(x*) ≈ 3.6667
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 2D Coupled Quadratic Optimization");
        $display("Objective: f(x0, x1) = (x0 - 2)^2 + (x1 - 5)^2 + x0*x1");
        $display("Target Minimum: x0* = -0.666667, x1* = 5.333333, f(x*) = 3.666667");

        // Microcode Program:
        // r0 = x0, r1 = x1
        // r4 = 2.0, r5 = x0 - 2.0, r6 = (x0 - 2.0)^2
        // r7 = 5.0, r8 = x1 - 5.0, r9 = (x1 - 5.0)^2
        // r10 = x0 * x1
        // r11 = r6 + r9
        // r15 = r11 + r10
        write_instr(5'd0,  OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd2);  // r4  = 2.0
        write_instr(5'd1,  OP_SUB,   4'd5,  4'd0, 4'd4, 16'sd0);  // r5  = x0 - 2.0
        write_instr(5'd2,  OP_MUL,   4'd6,  4'd5, 4'd5, 16'sd0);  // r6  = (x0 - 2)^2
        write_instr(5'd3,  OP_LOADC, 4'd7,  4'd0, 4'd0, 16'sd5);  // r7  = 5.0
        write_instr(5'd4,  OP_SUB,   4'd8,  4'd1, 4'd7, 16'sd0);  // r8  = x1 - 5.0
        write_instr(5'd5,  OP_MUL,   4'd9,  4'd8, 4'd8, 16'sd0);  // r9  = (x1 - 5)^2
        write_instr(5'd6,  OP_MUL,   4'd10, 4'd0, 4'd1, 16'sd0);  // r10 = x0 * x1
        write_instr(5'd7,  OP_ADD,   4'd11, 4'd6, 4'd9, 16'sd0);  // r11 = (x0-2)^2 + (x1-5)^2
        write_instr(5'd8,  OP_ADD,   4'd15, 4'd11, 4'd10, 16'sd0); // r15 = r11 + x0*x1
        write_instr(5'd9,  OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        num_params = 3'd2;
        x_init     = set_vec(set_vec(x_init, 2'd0, real_to_q16(5.0)), 2'd1, real_to_q16(8.0));
        init_step  = real_to_q16(2.0);
        tolerance  = 32'h0000_0100;
        max_iters  = 8'd100;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting Nelder-Mead Simplex Solver from initial guess (5.0, 8.0)...");
        @(posedge done);
        #1;

        $display("--> NELDER-MEAD SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Iterations:     %0d", iter_count);
        $display("    x0_optimal:     %f (Expected: ~ -0.666667)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:     %f (Expected: ~ 5.333333)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    f_optimal:      %f (Expected: ~ 3.666667)", q16_to_real(f_optimal));
        $display("    simplex_radius: %f", q16_to_real(simplex_radius));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > -0.75 && q16_to_real(get_vec(x_optimal, 2'd0)) < -0.55 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 5.20 && q16_to_real(get_vec(x_optimal, 2'd1)) < 5.45) begin
            $display("[TEST 2 PASSED] Successfully converged coupled system via Nelder-Mead!");
        end else begin
            $display("[TEST 2 FAILED] Did not converge to target minimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: Non-Smooth / Piecewise Absolute Value Objective
        // Objective: f(x0, x1) = |x0 - 2| + 2*|x1 - 3|
        // Target Optimum: x0* = 2.000, x1* = 3.000, f(x*) = 0.0
        // (Derivative does not exist at minimum, but Nelder-Mead handles it natively!)
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Non-Smooth / Piecewise Absolute Value Function");
        $display("Objective: f(x0, x1) = |x0 - 2| + 2*|x1 - 3|");
        $display("Target Minimum: x0* = 2.000000, x1* = 3.000000, f(x*) = 0.000000");

        // Microcode Program:
        // r0 = x0, r1 = x1
        // r4 = 2.0, r5 = x0 - 2.0, r6 = |x0 - 2.0|
        // r7 = 3.0, r8 = x1 - 3.0, r9 = |x1 - 3.0|, r10 = 2.0, r11 = 2*|x1 - 3|
        // r15 = r6 + r11
        write_instr(5'd0,  OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd2);  // r4  = 2.0
        write_instr(5'd1,  OP_SUB,   4'd5,  4'd0, 4'd4, 16'sd0);  // r5  = x0 - 2.0
        write_instr(5'd2,  OP_ABS,   4'd6,  4'd5, 4'd0, 16'sd0);  // r6  = |x0 - 2.0|
        write_instr(5'd3,  OP_LOADC, 4'd7,  4'd0, 4'd0, 16'sd3);  // r7  = 3.0
        write_instr(5'd4,  OP_SUB,   4'd8,  4'd1, 4'd7, 16'sd0);  // r8  = x1 - 3.0
        write_instr(5'd5,  OP_ABS,   4'd9,  4'd8, 4'd0, 16'sd0);  // r9  = |x1 - 3.0|
        write_instr(5'd6,  OP_LOADC, 4'd10, 4'd0, 4'd0, 16'sd2);  // r10 = 2.0
        write_instr(5'd7,  OP_MUL,   4'd11, 4'd9, 4'd10, 16'sd0); // r11 = 2 * |x1 - 3.0|
        write_instr(5'd8,  OP_ADD,   4'd15, 4'd6, 4'd11, 16'sd0); // r15 = |x0-2| + 2*|x1-3|
        write_instr(5'd9,  OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        num_params = 3'd2;
        x_init     = set_vec(set_vec(x_init, 2'd0, real_to_q16(6.0)), 2'd1, real_to_q16(7.0));
        init_step  = real_to_q16(1.5);
        tolerance  = 32'h0000_0100;
        max_iters  = 8'd100;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting Nelder-Mead on Non-Smooth function from guess (6.0, 7.0)...");
        @(posedge done);
        #1;

        $display("--> NELDER-MEAD SOLVER FINISHED!");
        $display("    Status:         %0d (2 = CONVERGED)", status);
        $display("    Iterations:     %0d", iter_count);
        $display("    x0_optimal:     %f (Expected: ~ 2.000000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:     %f (Expected: ~ 3.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    f_optimal:      %f (Expected: ~ 0.000000)", q16_to_real(f_optimal));
        $display("    simplex_radius: %f", q16_to_real(simplex_radius));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 1.90 && q16_to_real(get_vec(x_optimal, 2'd0)) < 2.10 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 2.90 && q16_to_real(get_vec(x_optimal, 2'd1)) < 3.10) begin
            $display("[TEST 3 PASSED] Successfully converged non-smooth objective via Nelder-Mead!");
        end else begin
            $display("[TEST 3 FAILED] Did not converge to non-smooth minimum.");
        end

        $display("\n==================================================================");
        $display(" ALL NELDER-MEAD HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
