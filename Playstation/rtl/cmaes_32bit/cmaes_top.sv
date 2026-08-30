// =============================================================================
// File Name   : cmaes_top.sv
// Module Name : cmaes_top
// Project     : Covariance Matrix Adaptation Evolution Strategy (CMA-ES) (Solver #34)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Controller for Covariance Matrix Adaptation Evolution Strategy.
//   Manages generation loops, anisotropic sampling, parallel candidate fitness
//   evaluations, cumulative step-size adaptation, rank-1 covariance matrix learning,
//   and champion parameter extraction.
// =============================================================================

`timescale 1ns / 1ps

import cmaes_types_pkg::*;
`include "cmaes_helpers.svh"

module cmaes_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Initialization & Hyperparameters
    // -------------------------------------------------------------------------
    input  logic               init_cmaes,       // 1-cycle initialization strobe
    input  cmaes_fn_t          fn_type,          // Objective fitness function
    input  logic [2:0]         pop_size,         // Population size λ (2..4)
    input  logic [2:0]         dim,              // Dimension D (1..4)
    input  param_vec_t         init_mean,        // Initial distribution mean m_0
    input  q16_t               init_sigma,       // Initial exploration step size σ_0
    input  param_vec_t         lb_vec,           // Lower parameter bounds
    input  param_vec_t         ub_vec,           // Upper parameter bounds
    input  logic [7:0]         max_gens,         // Maximum generations
    input  q16_t               target_tol,       // Stopping fitness tolerance
    input  q16_t               c_sigma,          // Learning rate for p_σ
    input  q16_t               d_sigma,          // Damping for step-size σ
    input  q16_t               c_c,              // Learning rate for p_c
    input  q16_t               c_1,              // Learning rate for rank-1 C update
    input  q16_t               sigma_min,        // Minimum exploration step size

    // -------------------------------------------------------------------------
    // Interface 2: Execution & Results
    // -------------------------------------------------------------------------
    input  logic               opt_valid,        // Start CMA-ES optimization loop
    output param_vec_t         best_solution,    // Champion parameter vector x* (4x1)
    output q16_t               best_fitness,     // Champion minimum fitness f*
    output param_vec_t         final_mean,       // Final distribution mean m
    output q16_t               final_sigma,      // Final exploration step size σ
    output logic [7:0]         gen_count,        // Completed generations
    output status_t            status,           // Status code
    output logic               opt_done,         // Optimization complete strobe
    output logic               busy              // High while engine is active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        CMA_IDLE         = 3'd0,
        CMA_SAMPLE_START = 3'd1,
        CMA_SAMPLE_WAIT  = 3'd2,
        CMA_EVAL         = 3'd3,
        CMA_UPDATE_START = 3'd4,
        CMA_UPDATE_WAIT  = 3'd5,
        CMA_CHECK        = 3'd6,
        CMA_DONE         = 3'd7
    } cmaes_state_t;

    cmaes_state_t state;

    // Internal Configuration Registers
    cmaes_fn_t   fn_type_reg;
    logic [2:0]  pop_reg;
    logic [2:0]  dim_reg;
    param_vec_t  m_reg;
    q16_t        sigma_reg;
    param_vec_t  ps_reg;
    param_vec_t  pc_reg;
    cov_mat_t    C_reg;
    cov_mat_t    A_reg;
    param_vec_t  lb_reg;
    param_vec_t  ub_reg;
    logic [7:0]  max_g_reg;
    q16_t        tol_reg;
    q16_t        cs_reg;
    q16_t        ds_reg;
    q16_t        cc_reg;
    q16_t        c1_reg;
    q16_t        sig_min_reg;
    status_t     status_reg;
    logic [7:0]  g_cnt;

    // Champion Registers
    param_vec_t  best_x_reg;
    q16_t        best_f_reg;

    cand_arr_t   cand_x_reg;
    cand_arr_t   cand_y_reg;
    cand_arr_t   cand_z_reg;
    fitness_arr_t cand_f_reg;

    assign best_solution = best_x_reg;
    assign best_fitness  = best_f_reg;
    assign final_mean    = m_reg;
    assign final_sigma   = sigma_reg;
    assign gen_count     = g_cnt;
    assign status        = status_reg;

    // Sub-engine 1: Candidate Sampling Instance
    logic      start_smp;
    cand_arr_t smp_cand_x;
    cand_arr_t smp_cand_y;
    cand_arr_t smp_cand_z;
    logic      smp_done;
    logic      smp_busy;

    cmaes_sample_engine u_smp (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (start_smp),
        .mean_vec    (m_reg),
        .sigma       (sigma_reg),
        .A_mat       (A_reg),
        .pop_size    (pop_reg),
        .dim         (dim_reg),
        .lb_vec      (lb_reg),
        .ub_vec      (ub_reg),
        .cand_samples(smp_cand_x),
        .cand_y_vecs (smp_cand_y),
        .cand_z_vecs (smp_cand_z),
        .done        (smp_done),
        .busy        (smp_busy)
    );

    // Sub-engine 2: Distribution Update Instance
    logic       start_upd;
    param_vec_t upd_new_m;
    q16_t       upd_new_sig;
    param_vec_t upd_new_ps;
    param_vec_t upd_new_pc;
    cov_mat_t   upd_new_C;
    cov_mat_t   upd_new_A;
    logic [1:0] upd_best_idx;
    logic       upd_done;
    logic       upd_busy;

    cmaes_update_engine u_upd (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start_upd),
        .cand_y_vecs  (cand_y_reg),
        .cand_fitness (cand_f_reg),
        .mean_vec     (m_reg),
        .sigma        (sigma_reg),
        .p_sigma      (ps_reg),
        .p_c          (pc_reg),
        .C_mat        (C_reg),
        .A_mat        (A_reg),
        .pop_size     (pop_reg),
        .dim          (dim_reg),
        .c_sigma      (cs_reg),
        .d_sigma      (ds_reg),
        .c_c          (cc_reg),
        .c_1          (c1_reg),
        .sigma_min    (sig_min_reg),
        .lb_vec       (lb_reg),
        .ub_vec       (ub_reg),
        .new_mean     (upd_new_m),
        .new_sigma    (upd_new_sig),
        .new_p_sigma  (upd_new_ps),
        .new_p_c      (upd_new_pc),
        .new_C_mat    (upd_new_C),
        .new_A_mat    (upd_new_A),
        .best_cand_idx(upd_best_idx),
        .done         (upd_done),
        .busy         (upd_busy)
    );

    param_vec_t eval_cand;
    q16_t       eval_f;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= CMA_IDLE;
            fn_type_reg <= FN_QUADRATIC;
            pop_reg     <= 3'd4;
            dim_reg     <= 3'd2;
            m_reg       <= '0;
            sigma_reg   <= 32'h0000_8000;
            ps_reg      <= '0;
            pc_reg      <= '0;
            C_reg       <= '0;
            A_reg       <= '0;
            lb_reg      <= '0;
            ub_reg      <= '0;
            max_g_reg   <= 8'd40;
            tol_reg     <= 32'h0000_0010;
            cs_reg      <= 32'h0000_3333; // c_σ = 0.20
            ds_reg      <= 32'h0001_0000; // d_σ = 1.00
            cc_reg      <= 32'h0000_3333; // c_c = 0.20
            c1_reg      <= 32'h0000_1999; // c_1 = 0.10
            sig_min_reg <= 32'h0000_0800; // σ_min = 0.03125
            status_reg  <= STATUS_IDLE;
            g_cnt       <= 8'd0;
            best_x_reg  <= '0;
            best_f_reg  <= Q16_MAX_POS;
            cand_x_reg  <= '0;
            cand_y_reg  <= '0;
            cand_z_reg  <= '0;
            cand_f_reg  <= '0;
            start_smp   <= 1'b0;
            start_upd   <= 1'b0;
            opt_done    <= 1'b0;
            busy        <= 1'b0;
        end else begin
            start_smp <= 1'b0;
            start_upd <= 1'b0;
            opt_done  <= 1'b0;

            case (state)
                CMA_IDLE: begin
                    if (init_cmaes) begin
                        fn_type_reg <= fn_type;
                        pop_reg     <= (pop_size != 3'd0) ? pop_size : 3'd4;
                        dim_reg     <= (dim != 3'd0) ? dim : 3'd2;
                        m_reg       <= init_mean;
                        sigma_reg   <= (init_sigma != 32'sd0) ? init_sigma : 32'h0000_8000;
                        ps_reg      <= '0;
                        pc_reg      <= '0;
                        lb_reg      <= lb_vec;
                        ub_reg      <= ub_vec;
                        max_g_reg   <= (max_gens != 8'd0) ? max_gens : 8'd40;
                        tol_reg     <= (target_tol != 32'sd0) ? target_tol : 32'h0000_0010;
                        cs_reg      <= (c_sigma != 32'sd0) ? c_sigma : 32'h0000_3333;
                        ds_reg      <= (d_sigma != 32'sd0) ? d_sigma : 32'h0001_0000;
                        cc_reg      <= (c_c != 32'sd0) ? c_c : 32'h0000_3333;
                        c1_reg      <= (c_1 != 32'sd0) ? c_1 : 32'h0000_1999;
                        sig_min_reg <= (sigma_min != 32'sd0) ? sigma_min : 32'h0000_0800;
                        status_reg  <= STATUS_IDLE;
                        g_cnt       <= 8'd0;
                        best_x_reg  <= init_mean;
                        best_f_reg  <= evaluate_fitness(fn_type, init_mean, dim);

                        // Initialize C = I and A = I
                        for (int r = 0; r < MAX_DIM; r++) begin
                            for (int c = 0; c < MAX_DIM; c++) begin
                                C_reg[{2'(r), 2'(c)}] <= (r == c) ? Q16_ONE : Q16_ZERO;
                                A_reg[{2'(r), 2'(c)}] <= (r == c) ? Q16_ONE : Q16_ZERO;
                            end
                        end
                    end else if (opt_valid) begin
                        busy       <= 1'b1;
                        g_cnt      <= 8'd0;
                        status_reg <= STATUS_SAMPLING;
                        start_smp  <= 1'b1;
                        state      <= CMA_SAMPLE_WAIT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Wait for anisotropic candidate generator
                CMA_SAMPLE_WAIT: begin
                    if (smp_done) begin
                        cand_x_reg <= smp_cand_x;
                        cand_y_reg <= smp_cand_y;
                        cand_z_reg <= smp_cand_z;
                        status_reg <= STATUS_EVALUATING;
                        state      <= CMA_EVAL;
                    end
                end

                // Step 2: Evaluate objective fitness for all candidates
                CMA_EVAL: begin
                    for (int k = 0; k < MAX_POP; k++) begin
                        if (k < pop_reg) begin
                            for (int d = 0; d < MAX_DIM; d++) begin
                                eval_cand[d] = cand_x_reg[{2'(k), 2'(d)}];
                            end

                            eval_f = evaluate_fitness(fn_type_reg, eval_cand, dim_reg);
                            cand_f_reg[k] <= eval_f;

                            if (eval_f < best_f_reg) begin
                                best_f_reg <= eval_f;
                                best_x_reg <= eval_cand;
                            end
                        end
                    end

                    state <= CMA_UPDATE_START;
                end

                // Step 3: Trigger distribution update engine after fitness values are latched
                CMA_UPDATE_START: begin
                    status_reg <= STATUS_UPDATING;
                    start_upd  <= 1'b1;
                    state      <= CMA_UPDATE_WAIT;
                end

                // Step 4: Wait for distribution update engine
                CMA_UPDATE_WAIT: begin
                    if (upd_done) begin
                        m_reg     <= upd_new_m;
                        sigma_reg <= upd_new_sig;
                        ps_reg    <= upd_new_ps;
                        pc_reg    <= upd_new_pc;
                        C_reg     <= upd_new_C;
                        A_reg     <= upd_new_A;
                        state     <= CMA_CHECK;
                    end
                end

                // Step 5: Check stopping criteria and iteration limit
                CMA_CHECK: begin
                    if (best_f_reg <= tol_reg) begin
                        status_reg <= STATUS_CONVERGED;
                        state      <= CMA_DONE;
                    end else if (g_cnt + 1'b1 >= max_g_reg) begin
                        status_reg <= STATUS_MAX_ITERS;
                        state      <= CMA_DONE;
                    end else begin
                        g_cnt      <= g_cnt + 1'b1;
                        status_reg <= STATUS_SAMPLING;
                        start_smp  <= 1'b1;
                        state      <= CMA_SAMPLE_WAIT;
                    end
                end

                CMA_DONE: begin
                    opt_done <= 1'b1;
                    busy     <= 1'b0;
                    state    <= CMA_IDLE;
                end

                default: state <= CMA_IDLE;
            endcase
        end
    end

endmodule
