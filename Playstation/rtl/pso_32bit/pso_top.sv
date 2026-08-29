// =============================================================================
// File Name   : pso_top.sv
// Module Name : pso_top
// Project     : Particle Swarm Optimization (PSO) Accelerator (Solver #14)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Module for the Particle Swarm Optimization (PSO) Hardware
//   Accelerator. Manages multi-agent particle swarm memory, programmable DFG
//   fitness evaluation, hardware PRNG, velocity update pipelines, personal
//   best & global best tracking, and generation iteration loops.
// =============================================================================

`timescale 1ns / 1ps

import pso_types_pkg::*;
`include "pso_helpers.svh"

module pso_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Model Microcode Programming Port
    // -------------------------------------------------------------------------
    input  logic               prog_en,       // Microcode Write Enable
    input  logic [4:0]         prog_addr,     // Microcode Address (0..31)
    input  instr_t             prog_data,     // 32-bit Microcode Instruction

    // -------------------------------------------------------------------------
    // Interface 2: Swarm Particle Initialization Port
    // -------------------------------------------------------------------------
    input  logic               particle_we,   // Particle Initialization Write Enable
    input  logic [2:0]         particle_addr, // Particle Table Index (0..7)
    input  vec_t               particle_x_in, // Initial Position Vector
    input  vec_t               particle_v_in, // Initial Velocity Vector

    // -------------------------------------------------------------------------
    // Interface 3: Optimization Controls & Hyperparameters
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start trigger
    input  logic [2:0]         num_dims,      // Number of dimensions N (1..4)
    input  logic [3:0]         num_particles, // Number of particles P (1..8)
    input  q16_t               inertia_w,     // Inertia weight ω
    input  q16_t               c1_coeff,      // Cognitive coefficient c1
    input  q16_t               c2_coeff,      // Social coefficient c2
    input  q16_t               v_max,         // Max velocity magnitude limit
    input  vec_t               x_min_bound,   // Lower bounding box
    input  vec_t               x_max_bound,   // Upper bounding box
    input  q16_t               target_fitness,// Target global minimum fitness
    input  q16_t               tolerance,     // Convergence tolerance threshold
    input  logic [7:0]         max_iters,     // Maximum iteration/generation budget
    input  logic [31:0]        seed_val,      // PRNG Seed

    // -------------------------------------------------------------------------
    // Interface 4: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               g_best_pos,    // Global best position vector g*
    output q16_t               g_best_fitness,// Global best fitness f(g*)
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        PSO_IDLE            = 4'd0,
        PSO_INIT_EVAL_START = 4'd1,
        PSO_INIT_EVAL_WAIT  = 4'd2,
        PSO_GEN_CHECK       = 4'd3,
        PSO_STEP_PRNG1      = 4'd4,
        PSO_STEP_PRNG2      = 4'd5,
        PSO_VEL_START       = 4'd6,
        PSO_VEL_WAIT        = 4'd7,
        PSO_EVAL_START      = 4'd8,
        PSO_EVAL_WAIT       = 4'd9,
        PSO_DONE            = 4'd10
    } pso_state_t;

    pso_state_t state;

    // Swarm State Memory Arrays
    vec_t pos_table    [0:MAX_PARTICLES-1];
    vec_t vel_table    [0:MAX_PARTICLES-1];
    vec_t pbest_pos    [0:MAX_PARTICLES-1];
    q16_t pbest_fit    [0:MAX_PARTICLES-1];

    // Global Best Registers
    vec_t       g_best_pos_reg;
    q16_t       g_best_fit_reg;
    logic [7:0] iter_cnt;
    status_t    status_reg;

    // Latch Hyperparameters
    logic [2:0] num_dims_reg;
    logic [3:0] num_particles_reg;
    q16_t       omega_reg;
    q16_t       c1_reg;
    q16_t       c2_reg;
    q16_t       vmax_reg;
    vec_t       xmin_reg;
    vec_t       xmax_reg;
    q16_t       target_fit_reg;
    q16_t       tol_reg;
    logic [7:0] max_iters_reg;

    logic [3:0] p_idx; // Current particle index (0..7)

    // PRNG Submodule Interconnect
    logic        prng_seed_load;
    logic [31:0] prng_seed_in;
    logic        prng_next;
    q16_t        prng_rand_out;
    q16_t        r1_val, r2_val;

    pso_lfsr_prng u_prng (
        .clk       (clk),
        .rst_n     (rst_n),
        .seed_load (prng_seed_load),
        .seed_val  (prng_seed_in),
        .next_rand (prng_next),
        .rand_q16  (prng_rand_out)
    );

    // DFG Fitness Evaluator Interconnect
    logic start_dfg;
    vec_t dfg_x_in;
    q16_t dfg_f_out;
    logic dfg_done, dfg_busy;

    dfg_pso_engine u_dfg (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start_eval (start_dfg),
        .num_dims   (num_dims_reg),
        .x_vec      (dfg_x_in),
        .f_out      (dfg_f_out),
        .eval_done  (dfg_done),
        .busy       (dfg_busy)
    );

    // Velocity Update Engine Interconnect
    logic start_vel;
    vec_t vel_x_curr, vel_v_curr, vel_p_best, vel_g_best;
    vec_t vel_x_next, vel_v_next;
    logic vel_done, vel_busy;

    pso_velocity_engine u_vel_engine (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (start_vel),
        .num_dims    (num_dims_reg),
        .x_curr      (vel_x_curr),
        .v_curr      (vel_v_curr),
        .p_best      (vel_p_best),
        .g_best      (vel_g_best),
        .inertia_w   (omega_reg),
        .c1_coeff    (c1_reg),
        .c2_coeff    (c2_reg),
        .v_max       (vmax_reg),
        .x_min_bound (xmin_reg),
        .x_max_bound (xmax_reg),
        .r1_rand     (r1_val),
        .r2_rand     (r2_val),
        .x_next      (vel_x_next),
        .v_next      (vel_v_next),
        .done        (vel_done),
        .busy        (vel_busy)
    );

    // Output assignments
    assign g_best_pos     = g_best_pos_reg;
    assign g_best_fitness = g_best_fit_reg;
    assign iter_count     = iter_cnt;
    assign status         = status_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= PSO_IDLE;
            g_best_pos_reg   <= '0;
            g_best_fit_reg   <= Q16_INF_POS;
            iter_cnt         <= 8'd0;
            status_reg       <= STATUS_IDLE;
            num_dims_reg     <= 3'd2;
            num_particles_reg<= 4'd8;
            omega_reg        <= Q16_INERTIA_DEF;
            c1_reg           <= Q16_C1_DEF;
            c2_reg           <= Q16_C2_DEF;
            vmax_reg         <= Q16_VMAX_DEF;
            xmin_reg         <= '0;
            xmax_reg         <= '0;
            target_fit_reg   <= Q16_ZERO;
            tol_reg          <= Q16_EPS_DEF;
            max_iters_reg    <= 8'd30;
            p_idx            <= 4'd0;
            prng_seed_load   <= 1'b0;
            prng_seed_in     <= 32'd0;
            prng_next        <= 1'b0;
            r1_val           <= Q16_HALF;
            r2_val           <= Q16_HALF;
            start_dfg        <= 1'b0;
            dfg_x_in         <= '0;
            start_vel        <= 1'b0;
            vel_x_curr       <= '0;
            vel_v_curr       <= '0;
            vel_p_best       <= '0;
            vel_g_best       <= '0;
            done             <= 1'b0;
            busy             <= 1'b0;
            for (int i = 0; i < MAX_PARTICLES; i++) begin
                pos_table[i] <= '0;
                vel_table[i] <= '0;
                pbest_pos[i] <= '0;
                pbest_fit[i] <= Q16_INF_POS;
            end
        end else begin
            // Particle Initialization Port
            if (particle_we) begin
                pos_table[particle_addr] <= particle_x_in;
                vel_table[particle_addr] <= particle_v_in;
                pbest_pos[particle_addr] <= particle_x_in;
                pbest_fit[particle_addr] <= Q16_INF_POS;
            end

            prng_seed_load <= 1'b0;
            prng_next      <= 1'b0;
            start_dfg      <= 1'b0;
            start_vel      <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: PSO_IDLE - Latch User Configuration & Initialize
                // -------------------------------------------------------------
                PSO_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy             <= 1'b1;
                        num_dims_reg     <= num_dims;
                        num_particles_reg<= num_particles;
                        omega_reg        <= (inertia_w != Q16_ZERO) ? inertia_w : Q16_INERTIA_DEF;
                        c1_reg           <= (c1_coeff != Q16_ZERO) ? c1_coeff : Q16_C1_DEF;
                        c2_reg           <= (c2_coeff != Q16_ZERO) ? c2_coeff : Q16_C2_DEF;
                        vmax_reg         <= (v_max != Q16_ZERO) ? v_max : Q16_VMAX_DEF;
                        xmin_reg         <= x_min_bound;
                        xmax_reg         <= x_max_bound;
                        target_fit_reg   <= target_fitness;
                        tol_reg          <= (tolerance != Q16_ZERO) ? tolerance : Q16_EPS_DEF;
                        max_iters_reg    <= max_iters;
                        iter_cnt         <= 8'd0;
                        g_best_fit_reg   <= Q16_INF_POS;
                        status_reg       <= STATUS_RUNNING;
                        p_idx            <= 4'd0;

                        // Load PRNG Seed
                        prng_seed_in     <= (seed_val != 32'd0) ? seed_val : 32'hACE1_54A9;
                        prng_seed_load   <= 1'b1;

                        state            <= PSO_INIT_EVAL_START;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: PSO_INIT_EVAL_START - Evaluate Initial Swarm Fitness
                // -------------------------------------------------------------
                PSO_INIT_EVAL_START: begin
                    dfg_x_in  <= pos_table[p_idx[2:0]];
                    start_dfg <= 1'b1;
                    state     <= PSO_INIT_EVAL_WAIT;
                end

                // -------------------------------------------------------------
                // STATE 2: PSO_INIT_EVAL_WAIT - Latch Personal Best & Global Best
                // -------------------------------------------------------------
                PSO_INIT_EVAL_WAIT: begin
                    if (dfg_done) begin
                        pbest_pos[p_idx[2:0]] <= pos_table[p_idx[2:0]];
                        pbest_fit[p_idx[2:0]] <= dfg_f_out;

                        // Check and update global best
                        if (dfg_f_out < g_best_fit_reg) begin
                            g_best_pos_reg <= pos_table[p_idx[2:0]];
                            g_best_fit_reg <= dfg_f_out;
                        end

                        if (p_idx + 1'b1 < num_particles_reg) begin
                            p_idx <= p_idx + 1'b1;
                            state <= PSO_INIT_EVAL_START;
                        end else begin
                            state <= PSO_GEN_CHECK;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE 3: PSO_GEN_CHECK - Check Convergence & Iteration Budget
                // -------------------------------------------------------------
                PSO_GEN_CHECK: begin
                    if (g_best_fit_reg <= tol_reg || q16_abs(g_best_fit_reg - target_fit_reg) <= tol_reg) begin
                        status_reg <= STATUS_CONVERGED;
                        state      <= PSO_DONE;
                    end else if (iter_cnt >= max_iters_reg) begin
                        status_reg <= STATUS_MAX_ITERS;
                        state      <= PSO_DONE;
                    end else begin
                        p_idx <= 4'd0;
                        state <= PSO_STEP_PRNG1;
                    end
                end

                // -------------------------------------------------------------
                // STATE 4: PSO_STEP_PRNG1 - Sample Random Number r1
                // -------------------------------------------------------------
                PSO_STEP_PRNG1: begin
                    prng_next <= 1'b1;
                    r1_val    <= (prng_rand_out != Q16_ZERO) ? prng_rand_out : Q16_HALF;
                    state     <= PSO_STEP_PRNG2;
                end

                // -------------------------------------------------------------
                // STATE 5: PSO_STEP_PRNG2 - Sample Random Number r2
                // -------------------------------------------------------------
                PSO_STEP_PRNG2: begin
                    prng_next <= 1'b1;
                    r2_val    <= (prng_rand_out != Q16_ZERO) ? prng_rand_out : Q16_HALF;
                    state     <= PSO_VEL_START;
                end

                // -------------------------------------------------------------
                // STATE 6: PSO_VEL_START - Update Particle Velocity & Position
                // -------------------------------------------------------------
                PSO_VEL_START: begin
                    vel_x_curr <= pos_table[p_idx[2:0]];
                    vel_v_curr <= vel_table[p_idx[2:0]];
                    vel_p_best <= pbest_pos[p_idx[2:0]];
                    vel_g_best <= g_best_pos_reg;
                    start_vel  <= 1'b1;
                    state      <= PSO_VEL_WAIT;
                end

                // -------------------------------------------------------------
                // STATE 7: PSO_VEL_WAIT - Latch New Position & Velocity
                // -------------------------------------------------------------
                PSO_VEL_WAIT: begin
                    if (vel_done) begin
                        pos_table[p_idx[2:0]] <= vel_x_next;
                        vel_table[p_idx[2:0]] <= vel_v_next;
                        state                 <= PSO_EVAL_START;
                    end
                end

                // -------------------------------------------------------------
                // STATE 8: PSO_EVAL_START - Evaluate New Position Fitness f(x)
                // -------------------------------------------------------------
                PSO_EVAL_START: begin
                    dfg_x_in  <= pos_table[p_idx[2:0]];
                    start_dfg <= 1'b1;
                    state     <= PSO_EVAL_WAIT;
                end

                // -------------------------------------------------------------
                // STATE 9: PSO_EVAL_WAIT - Update Personal & Global Bests
                // -------------------------------------------------------------
                PSO_EVAL_WAIT: begin
                    if (dfg_done) begin
                        // 1. Personal best update
                        if (dfg_f_out < pbest_fit[p_idx[2:0]]) begin
                            pbest_pos[p_idx[2:0]] <= pos_table[p_idx[2:0]];
                            pbest_fit[p_idx[2:0]] <= dfg_f_out;
                        end

                        // 2. Global best update
                        if (dfg_f_out < g_best_fit_reg) begin
                            g_best_pos_reg <= pos_table[p_idx[2:0]];
                            g_best_fit_reg <= dfg_f_out;
                        end

                        // Loop through all particles
                        if (p_idx + 1'b1 < num_particles_reg) begin
                            p_idx <= p_idx + 1'b1;
                            state <= PSO_STEP_PRNG1;
                        end else begin
                            iter_cnt <= iter_cnt + 1'b1;
                            state    <= PSO_GEN_CHECK;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE 10: PSO_DONE - Output Converged Results
                // -------------------------------------------------------------
                PSO_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= PSO_IDLE;
                end

                default: state <= PSO_IDLE;
            endcase
        end
    end

endmodule
