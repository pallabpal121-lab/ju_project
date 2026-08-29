// =============================================================================
// File: tb_newton_2nd_order.sv
// Module: Testbench for Universal Newton 2nd-Order Accelerator
// Description: Programs an arbitrary non-linear mathematical equation into
//              the hardware micro-op memory and verifies that Newton's 2nd-order
//              hardware FSM converges to the exact mathematical optimum x*.
// =============================================================================

`timescale 1ns / 1ps

import newton_types_pkg::*;

module tb_newton_2nd_order;

    logic        clk;
    logic        rst_n;

    // Programming Interface
    logic        prog_en;
    logic [4:0]  prog_addr;
    instr_t      prog_data;

    // Solver Controls
    logic        start;
    q16_t        x_init;
    q16_t        tolerance;
    q16_t        step_alpha;
    q16_t        lambda_reg;
    logic [7:0]  max_iters;

    // Results
    q16_t        x_optimal;
    q16_t        f_optimal;
    q16_t        g_final;
    logic [7:0]  iter_count;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate Universal Newton 2nd-Order Accelerator
    newton_2nd_order_top dut (
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

    // Real-number helper function for display
    function real q16_to_real(input q16_t val);
        q16_to_real = real'(val) / 65536.0;
    endfunction

    function q16_t real_to_q16(input real val);
        real_to_q16 = q16_t'(int'(val * 65536.0));
    endfunction

    // Task to program a single instruction
    task write_instr(input logic [4:0] addr, input opcode_t op, input logic [3:0] dst, input logic [3:0] src_a, input logic [3:0] src_b, input logic signed [15:0] imm);
        begin
            @(posedge clk);
            prog_en   <= 1'b1;
            prog_addr <= addr;
            prog_data.op    <= op;
            prog_data.dst   <= dst;
            prog_data.src_a <= src_a;
            prog_data.src_b <= src_b;
            prog_data.imm   <= imm;
            @(posedge clk);
            prog_en   <= 1'b0;
        end
    endtask

    initial begin
        $display("==================================================================");
        $display(" Universal Newton 2nd-Order Hardware Accelerator Testbench");
        $display("==================================================================");

        // Reset
        rst_n      = 0;
        prog_en    = 0;
        prog_addr  = '0;
        prog_data  = '0;
        start      = 0;
        x_init     = Q16_ZERO;
        tolerance  = real_to_q16(0.005);
        step_alpha = real_to_q16(1.0);
        lambda_reg = real_to_q16(0.001);
        max_iters  = 8'd20;

        #20;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: Optimize Quadratic Equation: f(x) = (x - 3)^2 = x^2 - 6x + 9
        // True Global Minimum: x* = 3.0, f(x*) = 0.0, f'(x*) = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Programming Equation: f(x) = x^2 - 6x + 9");
        $display("Target Minimum: x* = 3.0");

        // Micro-Op 0: r1 = r0 * r0  (x^2)
        write_instr(5'd0, OP_MUL, 4'd1, 4'd0, 4'd0, 16'd0);
        // Micro-Op 1: r2 = -6.0 (constant)
        write_instr(5'd1, OP_LOADC, 4'd2, 4'd0, 4'd0, -16'sd6);
        // Micro-Op 2: r3 = r0 * r2  (-6 * x)
        write_instr(5'd2, OP_MUL, 4'd3, 4'd0, 4'd2, 16'd0);
        // Micro-Op 3: r4 = r1 + r3  (x^2 - 6x)
        write_instr(5'd3, OP_ADD, 4'd4, 4'd1, 4'd3, 16'd0);
        // Micro-Op 4: r5 = +9.0 (constant)
        write_instr(5'd4, OP_LOADC, 4'd5, 4'd0, 4'd0, 16'sd9);
        // Micro-Op 5: r15 = r4 + r5 (f(x) = x^2 - 6x + 9)
        write_instr(5'd5, OP_ADD, 4'd15, 4'd4, 4'd5, 16'd0);
        // Micro-Op 6: END
        write_instr(5'd6, OP_END, 4'd0, 4'd0, 4'd0, 16'd0);

        // Start Newton Optimizer from initial guess x0 = 10.0
        $display("Starting Newton Hardware from initial guess x0 = 10.000...");
        x_init = real_to_q16(10.0);
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        // Wait for Done
        wait(done == 1'b1);
        @(posedge clk);

        $display("--> SOLVER FINISHED!");
        $display("    Status:     %0d (2 = CONVERGED)", status);
        $display("    Iterations: %0d", iter_count);
        $display("    x_optimal:  %f (Expected: ~3.000)", q16_to_real(x_optimal));
        $display("    f_optimal:  %f (Expected: ~0.000)", q16_to_real(f_optimal));
        $display("    g_final:    %f (Expected: ~0.000)", q16_to_real(g_final));

        if (status == STATUS_CONVERGED && q16_to_real(x_optimal) > 2.95 && q16_to_real(x_optimal) < 3.05) begin
            $display("[TEST 1 PASSED] Successfully converged to x* = 3.0 in hardware!");
        end else begin
            $display("[TEST 1 FAILED] Did not reach expected result.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: Optimize Quartic Equation: f(x) = x^4 - 4x^2 + 5
        // Local minimum at x* = sqrt(2) ≈ 1.4142
        // f'(x) = 4x^3 - 8x = 4x(x^2 - 2) = 0
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Programming Equation: f(x) = x^4 - 4x^2 + 5");
        $display("Target Minimum: x* = sqrt(2) ≈ 1.4142");

        // Micro-Op 0: r1 = x * x (x^2)
        write_instr(5'd0, OP_MUL, 4'd1, 4'd0, 4'd0, 16'd0);
        // Micro-Op 1: r2 = r1 * r1 (x^4)
        write_instr(5'd1, OP_MUL, 4'd2, 4'd1, 4'd1, 16'd0);
        // Micro-Op 2: r3 = -4.0
        write_instr(5'd2, OP_LOADC, 4'd3, 4'd0, 4'd0, -16'sd4);
        // Micro-Op 3: r4 = r1 * r3 (-4 * x^2)
        write_instr(5'd3, OP_MUL, 4'd4, 4'd1, 4'd3, 16'd0);
        // Micro-Op 4: r5 = r2 + r4 (x^4 - 4x^2)
        write_instr(5'd4, OP_ADD, 4'd5, 4'd2, 4'd4, 16'd0);
        // Micro-Op 5: r6 = +5.0
        write_instr(5'd5, OP_LOADC, 4'd6, 4'd0, 4'd0, 16'sd5);
        // Micro-Op 6: r15 = r5 + r6 (f(x))
        write_instr(5'd6, OP_ADD, 4'd15, 4'd5, 4'd6, 16'd0);
        // Micro-Op 7: END
        write_instr(5'd7, OP_END, 4'd0, 4'd0, 4'd0, 16'd0);

        // Start from x0 = 3.0
        $display("Starting Newton Hardware from initial guess x0 = 3.000...");
        x_init = real_to_q16(3.0);
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        // Wait for Done
        wait(done == 1'b1);
        @(posedge clk);

        $display("--> SOLVER FINISHED!");
        $display("    Status:     %0d (2 = CONVERGED)", status);
        $display("    Iterations: %0d", iter_count);
        $display("    x_optimal:  %f (Expected: ~1.414)", q16_to_real(x_optimal));
        $display("    f_optimal:  %f (Expected: ~1.000)", q16_to_real(f_optimal));
        $display("    g_final:    %f (Expected: ~0.000)", q16_to_real(g_final));

        if (status == STATUS_CONVERGED && q16_to_real(x_optimal) > 1.39 && q16_to_real(x_optimal) < 1.44) begin
            $display("[TEST 2 PASSED] Successfully converged to x* = 1.414 in hardware!");
        end else begin
            $display("[TEST 2 FAILED] Did not reach expected result.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: High-Degree 8th-Order Polynomial: f(x) = x^8 - 4x^4 + 3
        // Minimum at x* = (2)^(1/4) ≈ 1.1892, f(x*) = -1.0000
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Programming High-Degree Equation: f(x) = x^8 - 4x^4 + 3");
        $display("Target Minimum: x* = 2^(0.25) ≈ 1.1892, f(x*) = -1.0000");

        // Micro-Op 0: r1 = x * x (x^2)
        write_instr(5'd0, OP_MUL, 4'd1, 4'd0, 4'd0, 16'd0);
        // Micro-Op 1: r2 = r1 * r1 (x^4)
        write_instr(5'd1, OP_MUL, 4'd2, 4'd1, 4'd1, 16'd0);
        // Micro-Op 2: r3 = r2 * r2 (x^8)
        write_instr(5'd2, OP_MUL, 4'd3, 4'd2, 4'd2, 16'd0);
        // Micro-Op 3: r4 = -4.0 (constant)
        write_instr(5'd3, OP_LOADC, 4'd4, 4'd0, 4'd0, -16'sd4);
        // Micro-Op 4: r5 = r2 * r4 (-4 * x^4)
        write_instr(5'd4, OP_MUL, 4'd5, 4'd2, 4'd4, 16'd0);
        // Micro-Op 5: r6 = r3 + r5 (x^8 - 4x^4)
        write_instr(5'd5, OP_ADD, 4'd6, 4'd3, 4'd5, 16'd0);
        // Micro-Op 6: r7 = +3.0 (constant)
        write_instr(5'd6, OP_LOADC, 4'd7, 4'd0, 4'd0, 16'sd3);
        // Micro-Op 7: r15 = r6 + r7 (f(x) = x^8 - 4x^4 + 3)
        write_instr(5'd7, OP_ADD, 4'd15, 4'd6, 4'd7, 16'd0);
        // Micro-Op 8: END
        write_instr(5'd8, OP_END, 4'd0, 4'd0, 4'd0, 16'd0);

        // Start from x0 = 2.0
        $display("Starting Newton Hardware from initial guess x0 = 2.000...");
        x_init     = real_to_q16(2.0);
        tolerance  = real_to_q16(0.008);
        lambda_reg = real_to_q16(0.005);
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;

        // Wait for Done
        wait(done == 1'b1);
        @(posedge clk);

        $display("--> SOLVER FINISHED!");
        $display("    Status:     %0d (2 = CONVERGED)", status);
        $display("    Iterations: %0d", iter_count);
        $display("    x_optimal:  %f (Expected: ~1.189)", q16_to_real(x_optimal));
        $display("    f_optimal:  %f (Expected: ~ -1.000)", q16_to_real(f_optimal));
        $display("    g_final:    %f (Expected: ~0.000)", q16_to_real(g_final));

        if (status == STATUS_CONVERGED && q16_to_real(x_optimal) > 1.15 && q16_to_real(x_optimal) < 1.22) begin
            $display("[TEST 3 PASSED] Successfully converged to x* = 1.189 in hardware!");
        end else begin
            $display("[TEST 3 FAILED] Did not reach expected result.");
        end

        $display("\n==================================================================");
        $display(" ALL HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        $finish;
    end

endmodule
