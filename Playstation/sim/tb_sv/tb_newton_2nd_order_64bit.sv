// =============================================================================
// File Name   : tb_newton_2nd_order_64bit.sv
// Module Name : tb_newton_2nd_order_64bit
// Project     : Universal Newton 2nd-Order Optimization Accelerator (64-Bit)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Testbench for the 64-bit Universal Newton 2nd-Order Accelerator.
//   Programs non-linear mathematical equations into 64-bit microcode memory
//   and verifies high-precision mathematical convergence in simulation.
// =============================================================================

`timescale 1ns / 1ps

import newton_types_64bit_pkg::*;

module tb_newton_2nd_order_64bit;

    logic        clk;
    logic        rst_n;

    // Programming Interface
    logic        prog_en;
    logic [5:0]  prog_addr;
    instr64_t    prog_data;

    // Solver Controls
    logic        start;
    q32_t        x_init;
    q32_t        tolerance;
    q32_t        step_alpha;
    q32_t        lambda_reg;
    logic [7:0]  max_iters;

    // Results
    q32_t        x_optimal;
    q32_t        f_optimal;
    q32_t        g_final;
    logic [7:0]  iter_count;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate 64-Bit Accelerator Top
    newton_2nd_order_64bit_top dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (start),
        .x_init     (x_init),
        .tolerance  (tolerance),
        .step_alpha (step_alpha),
        .lambda_reg (lambda_reg),
        .max_iters  (max_iters),
        .x_optimal  (x_optimal),
        .f_optimal  (f_optimal),
        .g_final    (g_final),
        .iter_count (iter_count),
        .status     (status),
        .done       (done),
        .busy       (busy)
    );

    // 100MHz Clock Generation
    initial clk = 0;
    always #5 clk = ~clk;

    // Q32.32 Real Number Helper Functions
    function real q32_to_real(input q32_t val);
        q32_to_real = real'(val) / 4294967296.0;
    endfunction

    function q32_t real_to_q32(input real val);
        real_to_q32 = q32_t'(longint'(val * 4294967296.0));
    endfunction

    // Task to write a single 64-bit instruction
    task write_instr(input logic [5:0] addr, input opcode_t op, input logic [5:0] dst, input logic [5:0] src_a, input logic [5:0] src_b, input logic signed [31:0] imm);
        begin
            @(posedge clk);
            prog_en          <= 1'b1;
            prog_addr        <= addr;
            prog_data.op     <= op;
            prog_data.dst    <= dst;
            prog_data.src_a  <= src_a;
            prog_data.src_b  <= src_b;
            prog_data.reserved <= '0;
            prog_data.imm    <= imm;
            @(posedge clk);
            prog_en          <= 1'b0;
        end
    endtask

    initial begin
        $display("==================================================================");
        $display(" 64-Bit Universal Newton 2nd-Order Accelerator Testbench (Q32.32)");
        $display("==================================================================");

        // Reset
        rst_n      = 0;
        prog_en    = 0;
        prog_addr  = '0;
        prog_data  = '0;
        start      = 0;
        x_init     = 0;
        tolerance  = 0;
        step_alpha = 0;
        lambda_reg = 0;
        max_iters  = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: Quadratic Equation f(x) = (x - 3)^2 = x^2 - 6x + 9
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Programming 64-bit Equation: f(x) = (x - 3)^2");
        $display("Target Minimum: x* = 3.0, f(x*) = 0.0");

        write_instr(6'd0, OP_LOADC, 6'd1, 6'd0, 6'd0, 32'sd3);  // r1  = 3.0
        write_instr(6'd1, OP_SUB,   6'd2, 6'd0, 6'd1, 32'sd0);  // r2  = x - 3.0
        write_instr(6'd2, OP_MUL,   6'd63,6'd2, 6'd2, 32'sd0);  // r63 = (x - 3.0)^2 (REG_RESULT)
        write_instr(6'd3, OP_END,   6'd0, 6'd0, 6'd0, 32'sd0);  // END

        @(posedge clk);
        x_init     = real_to_q32(10.0);
        tolerance  = 64'h0000_0000_0010_0000; // tol ≈ 0.00024
        step_alpha = Q32_ONE;
        lambda_reg = 64'h0000_0000_0000_0100; // damping ≈ 6e-8
        max_iters  = 8'd50;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting 64-bit Newton Hardware from initial guess x0 = 10.000...");
        @(posedge done);
        #1;

        $display("--> SOLVER FINISHED!");
        $display("    Status:     %0d (2 = CONVERGED)", status);
        $display("    Iterations: %0d", iter_count);
        $display("    x_optimal:  %f (Expected: ~3.000000)", q32_to_real(x_optimal));
        $display("    f_optimal:  %f (Expected: ~0.000000)", q32_to_real(f_optimal));
        $display("    g_final:    %f (Expected: ~0.000000)", q32_to_real(g_final));

        if (status == STATUS_CONVERGED && q32_to_real(x_optimal) > 2.99 && q32_to_real(x_optimal) < 3.01) begin
            $display("[TEST 1 PASSED] Successfully converged in 64-bit hardware!");
        end else begin
            $display("[TEST 1 FAILED] Did not converge to target optimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: Quartic Polynomial f(x) = x^4 - 4x^2 + 5
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Programming 64-bit Equation: f(x) = x^4 - 4x^2 + 5");
        $display("Target Minimum: x* = sqrt(2) ≈ 1.41421356, f(x*) = 1.0");

        write_instr(6'd0, OP_LOADC, 6'd1, 6'd0, 6'd0, 32'sd4);  // r1  = 4.0
        write_instr(6'd1, OP_LOADC, 6'd2, 6'd0, 6'd0, 32'sd5);  // r2  = 5.0
        write_instr(6'd2, OP_MUL,   6'd3, 6'd0, 6'd0, 32'sd0);  // r3  = x^2
        write_instr(6'd3, OP_MUL,   6'd4, 6'd3, 6'd3, 32'sd0);  // r4  = x^4
        write_instr(6'd4, OP_MUL,   6'd5, 6'd1, 6'd3, 32'sd0);  // r5  = 4*x^2
        write_instr(6'd5, OP_SUB,   6'd6, 6'd4, 6'd5, 32'sd0);  // r6  = x^4 - 4*x^2
        write_instr(6'd6, OP_ADD,   6'd63,6'd6, 6'd2, 32'sd0);  // r63 = x^4 - 4*x^2 + 5
        write_instr(6'd7, OP_END,   6'd0, 6'd0, 6'd0, 32'sd0);  // END

        @(posedge clk);
        x_init     = real_to_q32(3.0);
        tolerance  = 64'h0000_0000_0010_0000;
        step_alpha = Q32_ONE;
        lambda_reg = 64'h0000_0000_0000_0100;
        max_iters  = 8'd50;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting 64-bit Newton Hardware from initial guess x0 = 3.000...");
        @(posedge done);
        #1;

        $display("--> SOLVER FINISHED!");
        $display("    Status:     %0d (2 = CONVERGED)", status);
        $display("    Iterations: %0d", iter_count);
        $display("    x_optimal:  %f (Expected: ~1.414214)", q32_to_real(x_optimal));
        $display("    f_optimal:  %f (Expected: ~1.000000)", q32_to_real(f_optimal));
        $display("    g_final:    %f (Expected: ~0.000000)", q32_to_real(g_final));

        if (status == STATUS_CONVERGED && q32_to_real(x_optimal) > 1.40 && q32_to_real(x_optimal) < 1.43) begin
            $display("[TEST 2 PASSED] Successfully converged to x* = 1.414 in 64-bit hardware!");
        end else begin
            $display("[TEST 2 FAILED] Did not converge to target optimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: High-Degree Equation f(x) = x^8 - 4x^4 + 3
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Programming 64-bit High-Degree Equation: f(x) = x^8 - 4x^4 + 3");
        $display("Target Minimum: x* = 2^(0.25) ≈ 1.189207, f(x*) = -1.000000");

        write_instr(6'd0, OP_LOADC, 6'd1, 6'd0, 6'd0, 32'sd4);  // r1  = 4.0
        write_instr(6'd1, OP_LOADC, 6'd2, 6'd0, 6'd0, 32'sd3);  // r2  = 3.0
        write_instr(6'd2, OP_MUL,   6'd3, 6'd0, 6'd0, 32'sd0);  // r3  = x^2
        write_instr(6'd3, OP_MUL,   6'd4, 6'd3, 6'd3, 32'sd0);  // r4  = x^4
        write_instr(6'd4, OP_MUL,   6'd5, 6'd4, 6'd4, 32'sd0);  // r5  = x^8
        write_instr(6'd5, OP_MUL,   6'd6, 6'd1, 6'd4, 32'sd0);  // r6  = 4*x^4
        write_instr(6'd6, OP_SUB,   6'd7, 6'd5, 6'd6, 32'sd0);  // r7  = x^8 - 4*x^4
        write_instr(6'd7, OP_ADD,   6'd63,6'd7, 6'd2, 32'sd0);  // r63 = x^8 - 4*x^4 + 3
        write_instr(6'd8, OP_END,   6'd0, 6'd0, 6'd0, 32'sd0);  // END

        @(posedge clk);
        x_init     = real_to_q32(2.0);
        tolerance  = 64'h0000_0000_0010_0000;
        step_alpha = Q32_ONE;
        lambda_reg = 64'h0000_0000_0000_0100;
        max_iters  = 8'd50;
        start      = 1'b1;
        @(posedge clk);
        start      = 1'b0;

        $display("Starting 64-bit Newton Hardware from initial guess x0 = 2.000...");
        @(posedge done);
        #1;

        $display("--> SOLVER FINISHED!");
        $display("    Status:     %0d (2 = CONVERGED)", status);
        $display("    Iterations: %0d", iter_count);
        $display("    x_optimal:  %f (Expected: ~1.189207)", q32_to_real(x_optimal));
        $display("    f_optimal:  %f (Expected: ~ -1.000000)", q32_to_real(f_optimal));
        $display("    g_final:    %f (Expected: ~0.000000)", q32_to_real(g_final));

        if (status == STATUS_CONVERGED && q32_to_real(x_optimal) > 1.17 && q32_to_real(x_optimal) < 1.20) begin
            $display("[TEST 3 PASSED] Successfully converged to x* = 1.189 in 64-bit hardware!");
        end else begin
            $display("[TEST 3 FAILED] Did not converge to target optimum.");
        end

        $display("\n==================================================================");
        $display(" ALL 64-BIT HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
