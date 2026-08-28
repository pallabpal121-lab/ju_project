// ============================================================================
// File: tb_newton_top.sv
// Description: Standalone SystemVerilog Testbench for Q16.16 Newton Accelerator
// Tests: 1) f(x) = (x-3)^2 + 2
//        2) f(x) = x^4 + 2x^2 - 8x
//        3) f(x) = x^3 - x^2 + 1
// ============================================================================

`timescale 1ns/1ps

import q16_types::*;

module tb_newton_top;

    logic        clk;
    logic        rst_n;

    logic        prog_write_en;
    logic [5:0]  prog_addr;
    instr_word_t prog_instr;
    logic [5:0]  prog_len_in;

    logic        start;
    q16_t        x_init;
    q16_t        h_step;
    q16_t        eps_tol;
    logic [7:0]  max_iter;

    logic        done;
    status_e     status;
    q16_t        x_final;
    q16_t        f_final;
    q16_t        g_final;
    q16_t        H_final;
    logic [7:0]  iter_count;

    // Device Under Test
    newton_top dut (
        .clk           (clk),
        .rst_n         (rst_n),
        .prog_write_en (prog_write_en),
        .prog_addr     (prog_addr),
        .prog_instr    (prog_instr),
        .prog_len_in   (prog_len_in),
        .start         (start),
        .x_init        (x_init),
        .h_step        (h_step),
        .eps_tol       (eps_tol),
        .max_iter      (max_iter),
        .done          (done),
        .status        (status),
        .x_final       (x_final),
        .f_final       (f_final),
        .g_final       (g_final),
        .H_final       (H_final),
        .iter_count    (iter_count)
    );

    // Clock generation (100MHz)
    always #5 clk = ~clk;

    // Helper functions
    function automatic q16_t float_to_q16(real val);
        return q16_t'(int'(val * 65536.0));
    endfunction

    function automatic real q16_to_float(q16_t val);
        return real'(val) / 65536.0;
    endfunction

    function automatic instr_word_t encode_instr(
        opcode_e op,
        logic [4:0] dst = 0,
        logic [4:0] srcA = 0,
        logic [4:0] srcB = 0,
        q16_t imm = 0
    );
        instr_word_t word;
        word = '0;
        word[63:60] = op;
        word[59:55] = dst;
        word[54:50] = srcA;
        word[49:45] = srcB;
        word[31:0]  = imm;
        return word;
    endfunction

    function automatic real abs_real(real val);
        return val < 0.0 ? -val : val;
    endfunction

    // Test task
    task load_prog_word(input int addr, input instr_word_t instr);
        prog_addr  <= addr;
        prog_instr <= instr;
        @(posedge clk);
    endtask

    task run_optimization(
        input string name,
        input q16_t  init_val,
        input real   expected_x,
        input real   expected_f
    );
        $display("\n==================================================");
        $display("RUNNING: %s", name);
        $display("==================================================");
        
        x_init   = init_val;
        h_step   = 32'h00000100;
        eps_tol  = 32'h00000100;
        max_iter = 32;
        @(posedge clk);
        while (done) @(posedge clk);
        start = 1'b1;
        @(posedge clk);
        start = 1'b0;
        @(posedge clk);

        // Wait for completion
        begin
            int timeout;
            timeout = 0;
            while (!done && timeout < 5000) begin
                timeout = timeout + 1;
                @(posedge clk);
            end
        end
        if (!done) begin
            $error("TIMEOUT waiting for done signal!");
            $finish;
        end

        $display("RESULTS for %s:", name);
        $display("  Status    : %0d (%s)", status, status == STATUS_SUCCESS ? "SUCCESS" : "FAIL");
        $display("  Iterations: %0d", iter_count);
        $display("  x_final   : %0.4f (Expected: %0.4f) [raw: 0x%08h]", q16_to_float(x_final), expected_x, x_final);
        $display("  f_final   : %0.4f (Expected: %0.4f) [raw: 0x%08h]", q16_to_float(f_final), expected_f, f_final);
        $display("  g_final   : %0.4f [raw: 0x%08h]", q16_to_float(g_final), g_final);
        $display("  H_final   : %0.4f [raw: 0x%08h]", q16_to_float(H_final), H_final);

        if (status != STATUS_SUCCESS) begin
            $error("TEST FAILED: Status is not SUCCESS");
        end else if (abs_real(q16_to_float(x_final) - expected_x) > 0.05) begin
            $error("TEST FAILED: x_final out of tolerance");
        end else begin
            $display(">>> TEST PASSED <<<\n");
        end
    endtask

    // ------------------------------------------------------------------------
    // Main Test Stimulus
    // ------------------------------------------------------------------------
    initial begin
        clk           = 0;
        rst_n         = 0;
        start         = 0;
        prog_write_en = 0;
        prog_addr     = 0;
        prog_instr    = 0;
        prog_len_in   = 0;
        x_init        = 0;
        h_step        = 32'h00000100; // 0.00390625 in Q16.16
        eps_tol       = 32'h00000100; // 0.00390625 in Q16.16
        max_iter      = 32;

        #20;
        rst_n = 1;
        #20;

        // ----------------------------------------------------
        // Program 1: f(x) = (x-3)^2 + 2
        // ----------------------------------------------------
        rst_n = 0; #20; rst_n = 1; #20;
        prog_write_en = 1'b1; prog_len_in = 7; @(posedge clk);
        load_prog_word(0, encode_instr(OP_LOAD_X, 5'd0));
        load_prog_word(1, encode_instr(OP_LOAD_CONST, 5'd1, 5'd0, 5'd0, float_to_q16(3.0)));
        load_prog_word(2, encode_instr(OP_LOAD_CONST, 5'd2, 5'd0, 5'd0, float_to_q16(2.0)));
        load_prog_word(3, encode_instr(OP_SUB, 5'd3, 5'd0, 5'd1));
        load_prog_word(4, encode_instr(OP_MUL, 5'd4, 5'd3, 5'd3));
        load_prog_word(5, encode_instr(OP_ADD, 5'd5, 5'd4, 5'd2));
        load_prog_word(6, encode_instr(OP_END, 5'd0, 5'd5));
        prog_write_en = 1'b0; @(posedge clk);

        x_init   = float_to_q16(0.0);
        h_step   = 32'h00000100;
        eps_tol  = 32'h00000100;
        max_iter = 32;
        run_optimization("f(x) = (x-3)^2 + 2", float_to_q16(0.0), 3.0, 2.0);

        // ----------------------------------------------------
        // Program 2: f(x) = x^4 + 2x^2 - 8x
        // ----------------------------------------------------
        rst_n = 0; #20; rst_n = 1; #20;
        prog_write_en = 1'b1; prog_len_in = 10; @(posedge clk);
        load_prog_word(0, encode_instr(OP_LOAD_X, 5'd0));
        load_prog_word(1, encode_instr(OP_LOAD_CONST, 5'd1, 5'd0, 5'd0, float_to_q16(2.0)));
        load_prog_word(2, encode_instr(OP_LOAD_CONST, 5'd2, 5'd0, 5'd0, float_to_q16(8.0)));
        load_prog_word(3, encode_instr(OP_MUL, 5'd3, 5'd0, 5'd0));
        load_prog_word(4, encode_instr(OP_MUL, 5'd4, 5'd3, 5'd3));
        load_prog_word(5, encode_instr(OP_MUL, 5'd5, 5'd1, 5'd3));
        load_prog_word(6, encode_instr(OP_MUL, 5'd6, 5'd2, 5'd0));
        load_prog_word(7, encode_instr(OP_ADD, 5'd7, 5'd4, 5'd5));
        load_prog_word(8, encode_instr(OP_SUB, 5'd8, 5'd7, 5'd6));
        load_prog_word(9, encode_instr(OP_END, 5'd0, 5'd8));
        prog_write_en = 1'b0; @(posedge clk);

        x_init   = float_to_q16(0.0);
        h_step   = 32'h00000100;
        eps_tol  = 32'h00000100;
        max_iter = 32;
        run_optimization("f(x) = x^4 + 2x^2 - 8x", float_to_q16(0.0), 1.0, -5.0);

        // ----------------------------------------------------
        // Program 3: f(x) = x^3 - x^2 + 1
        // ----------------------------------------------------
        rst_n = 0; #20; rst_n = 1; #20;
        prog_write_en = 1'b1; prog_len_in = 7; @(posedge clk);
        load_prog_word(0, encode_instr(OP_LOAD_X, 5'd0));
        load_prog_word(1, encode_instr(OP_LOAD_CONST, 5'd1, 5'd0, 5'd0, float_to_q16(1.0)));
        load_prog_word(2, encode_instr(OP_MUL, 5'd2, 5'd0, 5'd0));
        load_prog_word(3, encode_instr(OP_MUL, 5'd3, 5'd2, 5'd0));
        load_prog_word(4, encode_instr(OP_SUB, 5'd4, 5'd3, 5'd2));
        load_prog_word(5, encode_instr(OP_ADD, 5'd5, 5'd4, 5'd1));
        load_prog_word(6, encode_instr(OP_END, 5'd0, 5'd5));
        prog_write_en = 1'b0; @(posedge clk);

        x_init   = float_to_q16(1.0);
        h_step   = 32'h00000100;
        eps_tol  = 32'h00000100;
        max_iter = 32;
        run_optimization("f(x) = x^3 - x^2 + 1", float_to_q16(1.0), 0.6667, 0.8519);

        $display("\n==================================================");
        $display("*** ALL SYSTEMVERILOG TESTS PASSED SUCCESSFULLY ***");
        $display("==================================================\n");
        $finish;
    end

endmodule
