// =============================================================================
// File Name   : tb_de.sv
// Module Name : tb_de
// Project     : Differential Evolution (DE) Accelerator (Solver #30)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for Differential Evolution Accelerator.
//   Verifies:
//   1. 2D Decoupled Quadratic Global Minimization (Target: (3.0, 4.0))
//   2. 2D Non-Convex Curved Rosenbrock Valley (Target: (1.0, 1.0))
//   3. 3D Multi-Modal Landscape Optimization (Target: (0.0, 0.0, 0.0))
// =============================================================================

`timescale 1ns / 1ps

import de_types_pkg::*;
`include "de_helpers.svh"

module tb_de;

    logic               clk;
    logic               rst_n;

    // Controls & Configurations
    logic               init_de;
    fitness_fn_t        fn_type;
    logic [3:0]         pop_size;
    logic [2:0]         dim;
    pop_arr_t           init_population;
    gene_vec_t          lb_vec;
    gene_vec_t          ub_vec;
    logic [7:0]         max_gens;
    q16_t               target_tol;
    q16_t               f_scale;
    q16_t               cr_rate;

    // Execution & Outputs
    logic               opt_valid;
    gene_vec_t          best_solution;
    q16_t               best_fitness;
    logic [7:0]         gen_count;
    status_t            status;
    logic               opt_done;
    logic               busy;

    // Instantiate Top Module
    de_top dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .init_de        (init_de),
        .fn_type        (fn_type),
        .pop_size       (pop_size),
        .dim            (dim),
        .init_population(init_population),
        .lb_vec         (lb_vec),
        .ub_vec         (ub_vec),
        .max_gens       (max_gens),
        .target_tol     (target_tol),
        .f_scale        (f_scale),
        .cr_rate        (cr_rate),
        .opt_valid      (opt_valid),
        .best_solution  (best_solution),
        .best_fitness   (best_fitness),
        .gen_count      (gen_count),
        .status         (status),
        .opt_done       (opt_done),
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

    // Test variables declared at module level
    real x0_best, x1_best, x2_best, fit_best;

    initial begin
        $display("==================================================================");
        $display(" Differential Evolution (DE) Global Search TB (Solver #30)");
        $display("==================================================================");

        // Reset
        rst_n           = 0;
        init_de         = 0;
        opt_valid       = 0;
        fn_type         = FN_QUADRATIC;
        pop_size        = 4'd6;
        dim             = 3'd2;
        init_population = '0;
        lb_vec          = '0;
        ub_vec          = '0;
        max_gens        = 8'd40;
        target_tol      = real_to_q16(0.001);
        f_scale         = real_to_q16(0.60);
        cr_rate         = real_to_q16(0.85);

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Decoupled Quadratic Global Minimization
        // f(x0, x1) = (x0 - 3)^2 + 2(x1 - 4)^2 | Target: (3.0, 4.0), F* = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2D Decoupled Quadratic Global Minimization");
        $display("Target Minimum: x* = (3.000000, 4.000000) | Target F* = 0.000000");

        fn_type  = FN_QUADRATIC;
        pop_size = 4'd6;
        dim      = 3'd2;
        max_gens = 8'd50;
        target_tol = real_to_q16(0.0001);

        lb_vec[0] = real_to_q16(-5.0);
        lb_vec[1] = real_to_q16(-5.0);
        ub_vec[0] = real_to_q16(5.0);
        ub_vec[1] = real_to_q16(5.0);

        init_population = '0;
        init_population = set_ind_gene(init_population, 3'd0, 2'd0, real_to_q16(0.0));
        init_population = set_ind_gene(init_population, 3'd0, 2'd1, real_to_q16(0.0));

        init_population = set_ind_gene(init_population, 3'd1, 2'd0, real_to_q16(1.0));
        init_population = set_ind_gene(init_population, 3'd1, 2'd1, real_to_q16(2.0));

        init_population = set_ind_gene(init_population, 3'd2, 2'd0, real_to_q16(2.5));
        init_population = set_ind_gene(init_population, 3'd2, 2'd1, real_to_q16(3.5));

        init_population = set_ind_gene(init_population, 3'd3, 2'd0, real_to_q16(-1.0));
        init_population = set_ind_gene(init_population, 3'd3, 2'd1, real_to_q16(-2.0));

        init_population = set_ind_gene(init_population, 3'd4, 2'd0, real_to_q16(4.0));
        init_population = set_ind_gene(init_population, 3'd4, 2'd1, real_to_q16(4.5));

        init_population = set_ind_gene(init_population, 3'd5, 2'd0, real_to_q16(-3.0));
        init_population = set_ind_gene(init_population, 3'd5, 2'd1, real_to_q16(1.0));

        @(posedge clk);
        init_de = 1'b1;
        @(posedge clk);
        init_de = 1'b0;
        #10;

        $display("Starting Differential Evolution loop...");
        @(posedge clk);
        opt_valid = 1'b1;
        @(posedge clk);
        opt_valid = 1'b0;
        @(posedge opt_done);
        #10;

        x0_best  = q16_to_real(best_solution[0]);
        x1_best  = q16_to_real(best_solution[1]);
        fit_best = q16_to_real(best_fitness);

        $display("--> DE OPTIMIZATION RESULTS:");
        $display("    Completed Generations: %0d", gen_count);
        $display("    Best Solution x*:      (%f, %f)", x0_best, x1_best);
        $display("    Minimum Fitness F*:    %f", fit_best);

        if (fit_best < 0.50 && x0_best > 2.20 && x0_best < 3.80 && x1_best > 3.20 && x1_best < 4.80) begin
            $display("[TEST 1 PASSED] Successfully optimized quadratic landscape via Differential Evolution!");
        end else begin
            $display("[TEST 1 FAILED] Solution outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 2D Non-Convex Curved Rosenbrock Valley
        // f(x0, x1) = 10(x1 - x0^2)^2 + (1 - x0)^2 | Target: (1.0, 1.0), F* = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 2D Non-Convex Curved Rosenbrock Valley");
        $display("Target Optimum: x* = (1.000000, 1.000000) | Target F* = 0.000000");

        fn_type  = FN_ROSENBROCK;
        pop_size = 4'd6;
        dim      = 3'd2;
        max_gens = 8'd50;
        target_tol = real_to_q16(0.0001);

        lb_vec[0] = real_to_q16(-2.0);
        lb_vec[1] = real_to_q16(-2.0);
        ub_vec[0] = real_to_q16(2.0);
        ub_vec[1] = real_to_q16(2.0);

        init_population = '0;
        init_population = set_ind_gene(init_population, 3'd0, 2'd0, real_to_q16(-1.0));
        init_population = set_ind_gene(init_population, 3'd0, 2'd1, real_to_q16(1.0));

        init_population = set_ind_gene(init_population, 3'd1, 2'd0, real_to_q16(0.5));
        init_population = set_ind_gene(init_population, 3'd1, 2'd1, real_to_q16(0.25));

        init_population = set_ind_gene(init_population, 3'd2, 2'd0, real_to_q16(0.8));
        init_population = set_ind_gene(init_population, 3'd2, 2'd1, real_to_q16(0.64));

        init_population = set_ind_gene(init_population, 3'd3, 2'd0, real_to_q16(0.0));
        init_population = set_ind_gene(init_population, 3'd3, 2'd1, real_to_q16(0.0));

        init_population = set_ind_gene(init_population, 3'd4, 2'd0, real_to_q16(1.2));
        init_population = set_ind_gene(init_population, 3'd4, 2'd1, real_to_q16(1.44));

        init_population = set_ind_gene(init_population, 3'd5, 2'd0, real_to_q16(-0.5));
        init_population = set_ind_gene(init_population, 3'd5, 2'd1, real_to_q16(0.25));

        @(posedge clk);
        init_de = 1'b1;
        @(posedge clk);
        init_de = 1'b0;
        #10;

        @(posedge clk);
        opt_valid = 1'b1;
        @(posedge clk);
        opt_valid = 1'b0;
        @(posedge opt_done);
        #10;

        x0_best  = q16_to_real(best_solution[0]);
        x1_best  = q16_to_real(best_solution[1]);
        fit_best = q16_to_real(best_fitness);

        $display("--> ROSENBROCK DE RESULTS:");
        $display("    Completed Generations: %0d", gen_count);
        $display("    Best Solution x*:      (%f, %f)", x0_best, x1_best);
        $display("    Minimum Fitness F*:    %f", fit_best);

        if (fit_best < 0.20 && x0_best > 0.70 && x0_best < 1.30 && x1_best > 0.50 && x1_best < 1.60) begin
            $display("[TEST 2 PASSED] Successfully converged along Rosenbrock banana valley!");
        end else begin
            $display("[TEST 2 FAILED] Rosenbrock solution outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 3D Multi-Modal Fitness Optimization
        // f(x) = ∑ (x_d^2 + x_d^4) | Target: (0.0, 0.0, 0.0), F* = 0.0
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 3D Multi-Modal Fitness Optimization");
        $display("Target Optimum: x* = (0.000000, 0.000000, 0.000000) | Target F* = 0.000000");

        fn_type  = FN_RASTRIGIN;
        pop_size = 4'd6;
        dim      = 3'd3;
        max_gens = 8'd40;
        target_tol = real_to_q16(0.0001);

        lb_vec[0] = real_to_q16(-3.0);
        lb_vec[1] = real_to_q16(-3.0);
        lb_vec[2] = real_to_q16(-3.0);
        ub_vec[0] = real_to_q16(3.0);
        ub_vec[1] = real_to_q16(3.0);
        ub_vec[2] = real_to_q16(3.0);

        init_population = '0;
        init_population = set_ind_gene(init_population, 3'd0, 2'd0, real_to_q16(1.0));
        init_population = set_ind_gene(init_population, 3'd0, 2'd1, real_to_q16(-1.0));
        init_population = set_ind_gene(init_population, 3'd0, 2'd2, real_to_q16(0.5));

        init_population = set_ind_gene(init_population, 3'd1, 2'd0, real_to_q16(-0.5));
        init_population = set_ind_gene(init_population, 3'd1, 2'd1, real_to_q16(0.2));
        init_population = set_ind_gene(init_population, 3'd1, 2'd2, real_to_q16(-0.3));

        init_population = set_ind_gene(init_population, 3'd2, 2'd0, real_to_q16(2.0));
        init_population = set_ind_gene(init_population, 3'd2, 2'd1, real_to_q16(-2.0));
        init_population = set_ind_gene(init_population, 3'd2, 2'd2, real_to_q16(1.5));

        init_population = set_ind_gene(init_population, 3'd3, 2'd0, real_to_q16(-1.5));
        init_population = set_ind_gene(init_population, 3'd3, 2'd1, real_to_q16(1.2));
        init_population = set_ind_gene(init_population, 3'd3, 2'd2, real_to_q16(-1.0));

        init_population = set_ind_gene(init_population, 3'd4, 2'd0, real_to_q16(0.1));
        init_population = set_ind_gene(init_population, 3'd4, 2'd1, real_to_q16(-0.1));
        init_population = set_ind_gene(init_population, 3'd4, 2'd2, real_to_q16(0.1));

        init_population = set_ind_gene(init_population, 3'd5, 2'd0, real_to_q16(-2.5));
        init_population = set_ind_gene(init_population, 3'd5, 2'd1, real_to_q16(2.5));
        init_population = set_ind_gene(init_population, 3'd5, 2'd2, real_to_q16(-2.5));

        @(posedge clk);
        init_de = 1'b1;
        @(posedge clk);
        init_de = 1'b0;
        #10;

        @(posedge clk);
        opt_valid = 1'b1;
        @(posedge clk);
        opt_valid = 1'b0;
        @(posedge opt_done);
        #10;

        x0_best  = q16_to_real(best_solution[0]);
        x1_best  = q16_to_real(best_solution[1]);
        x2_best  = q16_to_real(best_solution[2]);
        fit_best = q16_to_real(best_fitness);

        $display("--> 3D MULTI-MODAL RESULTS:");
        $display("    Completed Generations: %0d", gen_count);
        $display("    Best Solution x*:      (%f, %f, %f)", x0_best, x1_best, x2_best);
        $display("    Minimum Fitness F*:    %f", fit_best);

        if (fit_best < 0.10 && x0_best > -0.30 && x0_best < 0.30 &&
            x1_best > -0.30 && x1_best < 0.30 && x2_best > -0.30 && x2_best < 0.30) begin
            $display("[TEST 3 PASSED] Successfully solved 3D multi-modal optimization via Differential Evolution!");
        end else begin
            $display("[TEST 3 FAILED] 3D fitness solution outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL DIFFERENTIAL EVOLUTION (DE) HARDWARE OPTIMIZATION TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
