// =============================================================================
// File Name   : tb_sqp.sv
// Module Name : tb_sqp
// Project     : Sequential Quadratic Programming (SQP) Accelerator (Solver #7)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for the SQP Constrained Accelerator.
//   Verifies 2D upper bound active quadratic, 2D lower bound active coupled
//   quadratic, and 3D multi-axis constrained quadratic optimization.
// =============================================================================

`timescale 1ns / 1ps

import sqp_types_pkg::*;
`include "sqp_helpers.svh"

module tb_sqp;

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
    box_bounds_t bounds;
    q16_t        step_alpha;
    q16_t        tolerance;
    logic [7:0]  max_iters;

    // Results & Outputs
    vec_t        x_optimal;
    q16_t        f_optimal;
    q16_t        g_free_norm;
    active_mask_t active_mask;
    vec_t        vec_mu;
    logic [7:0]  iter_count;
    status_t     status;
    logic        done;
    logic        busy;

    // Instantiate Top Module
    sqp_top dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .prog_en      (prog_en),
        .prog_addr    (prog_addr),
        .prog_data    (prog_data),
        .start        (start),
        .num_params   (num_params),
        .x_init       (x_init),
        .bounds       (bounds),
        .step_alpha   (step_alpha),
        .tolerance    (tolerance),
        .max_iters    (max_iters),
        .x_optimal    (x_optimal),
        .f_optimal    (f_optimal),
        .g_free_norm  (g_free_norm),
        .active_mask  (active_mask),
        .vec_mu       (vec_mu),
        .iter_count   (iter_count),
        .status       (status),
        .done         (done),
        .busy         (busy)
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
        $display(" Sequential Quadratic Programming (SQP) Accelerator TB (Solver #7)");
        $display("==================================================================");

        // Reset
        rst_n       = 0;
        prog_en     = 0;
        prog_addr   = '0;
        prog_data   = '0;
        start       = 0;
        num_params  = 3'd2;
        x_init      = '0;
        bounds      = '0;
        step_alpha  = 0;
        tolerance   = 0;
        max_iters   = 0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Decoupled Paraboloid with Upper-Bound Constraints
        // Objective: f(x0, x1) = 2*(x0 - 3)^2 + 3*(x1 - 4)^2
        // Unconstrained Minimum: (3.0, 4.0)
        // Constraints: 0.0 <= x0 <= 1.5, 0.0 <= x1 <= 2.5
        // Constrained Minimum: x* = (1.5, 2.5), f(x*) = 11.25
        // Active Constraints: x0=1.5 (Upper Active), x1=2.5 (Upper Active) -> active_mask = 2'b11
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2D Paraboloid with Upper-Bound Box Constraints");
        $display("Objective: f(x0, x1) = 2*(x0 - 3)^2 + 3*(x1 - 4)^2");
        $display("Physical Box Bounds: 0.0 <= x0 <= 1.5, 0.0 <= x1 <= 2.5");
        $display("Target Constrained Minimum: x0* = 1.500000, x1* = 2.500000, f(x*) = 11.250000");

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
        num_params         = 3'd2;
        x_init             = set_vec(set_vec(x_init, 2'd0, real_to_q16(0.0)), 2'd1, real_to_q16(0.0));
        bounds.lower_bound = set_vec(set_vec(bounds.lower_bound, 2'd0, real_to_q16(0.0)), 2'd1, real_to_q16(0.0));
        bounds.upper_bound = set_vec(set_vec(bounds.upper_bound, 2'd0, real_to_q16(1.5)), 2'd1, real_to_q16(2.5));
        tolerance          = 32'h0000_0080;
        step_alpha         = Q16_ONE;
        max_iters          = 8'd30;
        start              = 1'b1;
        @(posedge clk);
        start              = 1'b0;

        $display("Starting SQP Solver from initial guess (x0, x1) = (0.0, 0.0)...");
        @(posedge done);
        #1;

        $display("--> SQP SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    x0_optimal:   %f (Expected: ~ 1.500000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:   %f (Expected: ~ 2.500000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    f_optimal:    %f (Expected: ~ 11.250000)", q16_to_real(f_optimal));
        $display("    active_mask:  %b (Expected: 2'b11)", active_mask[1:0]);
        $display("    mu0 (Shadow): %f", q16_to_real(get_vec(vec_mu, 2'd0)));
        $display("    mu1 (Shadow): %f", q16_to_real(get_vec(vec_mu, 2'd1)));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 1.48 && q16_to_real(get_vec(x_optimal, 2'd0)) < 1.52 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 2.48 && q16_to_real(get_vec(x_optimal, 2'd1)) < 2.52 &&
            active_mask[1:0] == 2'b11) begin
            $display("[TEST 1 PASSED] Successfully converged to constrained boundary via SQP!");
        end else begin
            $display("[TEST 1 FAILED] Did not converge to target constrained optimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2D Coupled Quadratic with Non-Negativity Constraints
        // Objective: f(x0, x1) = (x0 - 2)^2 + (x1 - 5)^2 + x0*x1
        // Unconstrained Minimum: x0* = -0.6667, x1* = 5.3333
        // Constraints: 0.0 <= x0 <= 10.0, 0.0 <= x1 <= 10.0
        // Constrained Minimum: x0* = 0.000000, x1* = 5.000000, f(x*) = 4.000000
        // Active Constraints: x0=0.0 (Lower Active), x1=5.0 (Free) -> active_mask = 2'b01
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 2D Coupled Quadratic with Lower-Bound Box Constraints");
        $display("Objective: f(x0, x1) = (x0 - 2)^2 + (x1 - 5)^2 + x0*x1");
        $display("Physical Box Bounds: 0.0 <= x0 <= 10.0, 0.0 <= x1 <= 10.0");
        $display("Target Constrained Minimum: x0* = 0.000000, x1* = 5.000000, f(x*) = 4.000000");

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
        num_params         = 3'd2;
        x_init             = set_vec(set_vec(x_init, 2'd0, real_to_q16(5.0)), 2'd1, real_to_q16(8.0));
        bounds.lower_bound = set_vec(set_vec(bounds.lower_bound, 2'd0, real_to_q16(0.0)), 2'd1, real_to_q16(0.0));
        bounds.upper_bound = set_vec(set_vec(bounds.upper_bound, 2'd0, real_to_q16(10.0)), 2'd1, real_to_q16(10.0));
        tolerance          = 32'h0000_0080;
        step_alpha         = Q16_ONE;
        max_iters          = 8'd30;
        start              = 1'b1;
        @(posedge clk);
        start              = 1'b0;

        $display("Starting SQP Solver from initial guess (x0, x1) = (5.0, 8.0)...");
        @(posedge done);
        #1;

        $display("--> SQP SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    x0_optimal:   %f (Expected: ~ 0.000000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:   %f (Expected: ~ 5.000000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    f_optimal:    %f (Expected: ~ 4.000000)", q16_to_real(f_optimal));
        $display("    active_mask:  %b (Expected: 2'b01)", active_mask[1:0]);
        $display("    mu0 (Shadow): %f", q16_to_real(get_vec(vec_mu, 2'd0)));

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > -0.05 && q16_to_real(get_vec(x_optimal, 2'd0)) < 0.05 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 4.95 && q16_to_real(get_vec(x_optimal, 2'd1)) < 5.05 &&
            active_mask[0] == 1'b1) begin
            $display("[TEST 2 PASSED] Successfully converged coupled system to constrained boundary!");
        end else begin
            $display("[TEST 2 FAILED] Did not converge to target constrained minimum.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 3D Coupled System with Multi-Axis Box Limits
        // Objective: f(x0, x1, x2) = (x0 - 1)^2 + (x1 - 2)^2 + (x2 - 3)^2 + x0*x1
        // Unconstrained Minimum: (0.0, 2.0, 3.0)
        // Physical Bounds: 1.0 <= x0 <= 5.0, 0.0 <= x1 <= 5.0, 0.0 <= x2 <= 2.0
        // Constrained Minimum: x0* = 1.0 (Lower Active), x1* = 1.5 (Free), x2* = 2.0 (Upper Active)
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 3D Coupled System with Multi-Axis Box Limits");
        $display("Objective: f(x0, x1, x2) = (x0 - 1)^2 + (x1 - 2)^2 + (x2 - 3)^2 + x0*x1");
        $display("Bounds: 1.0 <= x0 <= 5.0, 0.0 <= x1 <= 5.0, 0.0 <= x2 <= 2.0");
        $display("Target Constrained Minimum: x0* = 1.000, x1* = 1.500, x2* = 2.000");

        // Microcode Program for 3D Objective:
        // r0 = x0, r1 = x1, r2 = x2
        // r4 = 1.0, r5 = x0 - 1.0, r6 = (x0 - 1)^2
        // r7 = 2.0, r8 = x1 - 2.0, r9 = (x1 - 2)^2
        // r10 = 3.0, r11 = x2 - 3.0, r12 = (x2 - 3)^2
        // r13 = x0 * x1
        // r14 = r6 + r9
        // r15 = r14 + r12 + r13
        write_instr(5'd0,  OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd1);  // r4  = 1.0
        write_instr(5'd1,  OP_SUB,   4'd5,  4'd0, 4'd4, 16'sd0);  // r5  = x0 - 1.0
        write_instr(5'd2,  OP_MUL,   4'd6,  4'd5, 4'd5, 16'sd0);  // r6  = (x0 - 1)^2
        write_instr(5'd3,  OP_LOADC, 4'd7,  4'd0, 4'd0, 16'sd2);  // r7  = 2.0
        write_instr(5'd4,  OP_SUB,   4'd8,  4'd1, 4'd7, 16'sd0);  // r8  = x1 - 2.0
        write_instr(5'd5,  OP_MUL,   4'd9,  4'd8, 4'd8, 16'sd0);  // r9  = (x1 - 2)^2
        write_instr(5'd6,  OP_LOADC, 4'd10, 4'd0, 4'd0, 16'sd3);  // r10 = 3.0
        write_instr(5'd7,  OP_SUB,   4'd11, 4'd2, 4'd10, 16'sd0); // r11 = x2 - 3.0
        write_instr(5'd8,  OP_MUL,   4'd12, 4'd11, 4'd11, 16'sd0);// r12 = (x2 - 3)^2
        write_instr(5'd9,  OP_MUL,   4'd13, 4'd0, 4'd1, 16'sd0);  // r13 = x0 * x1
        write_instr(5'd10, OP_ADD,   4'd14, 4'd6, 4'd9, 16'sd0);  // r14 = (x0-1)^2 + (x1-2)^2
        write_instr(5'd11, OP_ADD,   4'd15, 4'd14, 4'd12, 16'sd0);// r15 = r14 + (x2-3)^2
        write_instr(5'd12, OP_ADD,   4'd15, 4'd15, 4'd13, 16'sd0);// r15 = r15 + x0*x1
        write_instr(5'd13, OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);  // END

        @(posedge clk);
        num_params         = 3'd3;
        x_init             = set_vec(set_vec(set_vec(x_init, 2'd0, real_to_q16(4.0)), 2'd1, real_to_q16(4.0)), 2'd2, real_to_q16(1.0));
        bounds.lower_bound = set_vec(set_vec(set_vec(bounds.lower_bound, 2'd0, real_to_q16(1.0)), 2'd1, real_to_q16(0.0)), 2'd2, real_to_q16(0.0));
        bounds.upper_bound = set_vec(set_vec(set_vec(bounds.upper_bound, 2'd0, real_to_q16(5.0)), 2'd1, real_to_q16(5.0)), 2'd2, real_to_q16(2.0));
        tolerance          = 32'h0000_0080;
        step_alpha         = Q16_ONE;
        max_iters          = 8'd30;
        start              = 1'b1;
        @(posedge clk);
        start              = 1'b0;

        $display("Starting 3D SQP Solver from guess (4.0, 4.0, 1.0)...");
        @(posedge done);
        #1;

        $display("--> SQP SOLVER FINISHED!");
        $display("    Status:       %0d (2 = CONVERGED)", status);
        $display("    Iterations:   %0d", iter_count);
        $display("    x0_optimal:   %f (Expected: ~ 1.000000)", q16_to_real(get_vec(x_optimal, 2'd0)));
        $display("    x1_optimal:   %f (Expected: ~ 1.500000)", q16_to_real(get_vec(x_optimal, 2'd1)));
        $display("    x2_optimal:   %f (Expected: ~ 2.000000)", q16_to_real(get_vec(x_optimal, 2'd2)));
        $display("    active_mask:  %b (Expected: 3'b101)", active_mask[2:0]);

        if (status == STATUS_CONVERGED &&
            q16_to_real(get_vec(x_optimal, 2'd0)) > 0.95 && q16_to_real(get_vec(x_optimal, 2'd0)) < 1.05 &&
            q16_to_real(get_vec(x_optimal, 2'd1)) > 1.45 && q16_to_real(get_vec(x_optimal, 2'd1)) < 1.55 &&
            q16_to_real(get_vec(x_optimal, 2'd2)) > 1.95 && q16_to_real(get_vec(x_optimal, 2'd2)) < 2.05) begin
            $display("[TEST 3 PASSED] Successfully converged 3D multi-axis constrained problem!");
        end else begin
            $display("[TEST 3 FAILED] Did not converge to target 3D constrained minimum.");
        end

        $display("\n==================================================================");
        $display(" ALL SQP CONSTRAINED HARDWARE TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
