// =============================================================================
// File Name   : qubo_top.sv
// Module Name : qubo_top
// Project     : QUBO / Simulated Annealing Ising Accelerator (Solver #18)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Module for Quadratic Unconstrained Binary Optimization (QUBO)
//   and Simulated Annealing Ising ground-state search.
//   Coordinates thermal cooling cycles, stochastic spin sampling, Metropolis-Hastings
//   acceptance, hardware Boltzmann probability evaluations, and
//   global ground-state memory.
// =============================================================================

`timescale 1ns / 1ps

import qubo_types_pkg::*;
`include "qubo_helpers.svh"

module qubo_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Problem Formulation & Controls
    // -------------------------------------------------------------------------
    input  logic               start,            // 1-cycle start trigger
    input  logic [3:0]         num_spins,        // Number of spins N (1..8)
    input  qubo_mat_t          q_matrix,         // 8x8 QUBO Matrix Q
    input  spin_vec_t          q_init,           // Initial spin state q_0
    input  q16_t               t_start,          // Initial temperature T_start
    input  q16_t               t_end,            // Final temperature T_end
    input  q16_t               gamma_cool,       // Cooling rate γ (e.g. 0.95)
    input  logic [5:0]         sweeps_per_temp,  // Sweeps per temp level
    input  logic [7:0]         max_temp_steps,   // Maximum thermal steps
    input  logic [31:0]        prng_seed,        // Random seed for Xorshift PRNG

    // -------------------------------------------------------------------------
    // Interface 2: Results & Status Outputs
    // -------------------------------------------------------------------------
    output spin_vec_t          q_optimal,        // Optimal binary spin state q*
    output q16_t               energy_optimal,   // Minimum Hamiltonian energy E(q*)
    output logic [7:0]         steps_count,      // Total temperature steps
    output logic [15:0]        flips_accepted,   // Total flips accepted
    output status_t            status,           // Convergence status code
    output logic               done,             // 1-cycle completion strobe
    output logic               busy              // High while active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        SA_IDLE          = 4'd0,
        SA_START_INIT_E  = 4'd1,
        SA_WAIT_INIT_E   = 4'd2,
        SA_START_FLIP    = 4'd3,
        SA_WAIT_DELTA    = 4'd4,
        SA_WAIT_DIV_U    = 4'd5,
        SA_WAIT_EXP      = 4'd6,
        SA_ADVANCE_SPIN  = 4'd7,
        SA_EVAL_FINAL_E  = 4'd8,
        SA_WAIT_FINAL_E  = 4'd9,
        SA_DONE          = 4'd10
    } sa_state_t;

    sa_state_t state;

    // Registers
    spin_vec_t  q_curr;
    spin_vec_t  q_best;
    q16_t       e_curr;
    q16_t       e_best;
    q16_t       t_curr;

    logic [2:0]  spin_idx;
    logic [7:0]  trial_cnt;
    logic [7:0]  step_cnt;
    logic [15:0] accepted_cnt;
    status_t     status_reg;

    // Latched Parameters
    logic [3:0] num_spins_reg;
    qubo_mat_t  q_mat_reg;
    q16_t       t_end_reg;
    q16_t       gamma_reg;
    logic [7:0] trials_per_temp;
    logic [7:0] max_steps_reg;

    // Sub-Engine 1: PRNG
    logic next_rand_strobe, load_seed_strobe;
    q16_t rand_num;

    qubo_lfsr_prng u_prng (
        .clk       (clk),
        .rst_n     (rst_n),
        .next_rand (next_rand_strobe),
        .seed      (prng_seed),
        .load_seed (load_seed_strobe),
        .rand_q16  (rand_num)
    );

    // Sub-Engine 2: Energy Engine
    logic start_e_total, start_e_delta;
    spin_vec_t e_spin_in;
    logic [2:0] e_flip_idx;
    q16_t energy_out;
    logic e_done, e_busy;

    qubo_energy_engine u_energy (
        .clk        (clk),
        .rst_n      (rst_n),
        .start_total(start_e_total),
        .start_delta(start_e_delta),
        .num_spins  (num_spins_reg),
        .q_matrix   (q_mat_reg),
        .spin_state (e_spin_in),
        .flip_idx   (e_flip_idx),
        .energy_out (energy_out),
        .done       (e_done),
        .busy       (e_busy)
    );

    // Sub-Engine 3: Divider for u = ΔE / T
    logic div_start;
    q16_t div_dividend, div_divisor, div_quotient;
    logic div_done, div_by_zero, div_busy;

    q16_divider u_div (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (div_start),
        .dividend   (div_dividend),
        .divisor    (div_divisor),
        .quotient   (div_quotient),
        .done       (div_done),
        .div_by_zero(div_by_zero),
        .busy       (div_busy)
    );

    // Sub-Engine 4: Exponential Boltzmann Prob Evaluator
    logic exp_start;
    q16_t exp_u_in, exp_prob_out;
    logic exp_done, exp_busy;

    qubo_exp_unit u_exp (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (exp_start),
        .u_val   (exp_u_in),
        .prob_out(exp_prob_out),
        .done    (exp_done),
        .busy    (exp_busy)
    );

    // Outputs
    assign q_optimal      = q_best;
    assign energy_optimal = e_best;
    assign steps_count    = step_cnt;
    assign flips_accepted = accepted_cnt;
    assign status         = status_reg;

    q16_t delta_e_reg;
    q16_t new_e;
    logic [2:0] chosen_spin;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= SA_IDLE;
            q_curr           <= '0;
            q_best           <= '0;
            e_curr           <= Q16_ZERO;
            e_best           <= Q16_ZERO;
            t_curr           <= Q16_T_START_DEF;
            spin_idx         <= 3'd0;
            trial_cnt        <= 8'd0;
            step_cnt         <= 8'd0;
            accepted_cnt     <= 16'd0;
            status_reg       <= STATUS_IDLE;
            num_spins_reg    <= 4'd4;
            q_mat_reg        <= '0;
            t_end_reg        <= Q16_T_END_DEF;
            gamma_reg        <= Q16_GAMMA_DEF;
            trials_per_temp  <= 8'd32;
            max_steps_reg    <= 8'd50;
            start_e_total    <= 1'b0;
            start_e_delta    <= 1'b0;
            e_spin_in        <= '0;
            e_flip_idx       <= 3'd0;
            div_start        <= 1'b0;
            div_dividend     <= Q16_ZERO;
            div_divisor      <= Q16_ZERO;
            exp_start        <= 1'b0;
            exp_u_in         <= Q16_ZERO;
            next_rand_strobe <= 1'b0;
            load_seed_strobe <= 1'b0;
            delta_e_reg      <= Q16_ZERO;
            done             <= 1'b0;
            busy             <= 1'b0;
        end else begin
            start_e_total    <= 1'b0;
            start_e_delta    <= 1'b0;
            div_start        <= 1'b0;
            exp_start        <= 1'b0;
            next_rand_strobe <= 1'b0;
            load_seed_strobe <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: SA_IDLE - Latch User Configuration & Initialize
                // -------------------------------------------------------------
                SA_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy             <= 1'b1;
                        num_spins_reg    <= num_spins;
                        q_mat_reg        <= q_matrix;
                        q_curr           <= q_init;
                        q_best           <= q_init;
                        t_curr           <= (t_start != Q16_ZERO) ? t_start : Q16_T_START_DEF;
                        t_end_reg        <= (t_end != Q16_ZERO)   ? t_end   : Q16_T_END_DEF;
                        gamma_reg        <= (gamma_cool != Q16_ZERO) ? gamma_cool : Q16_GAMMA_DEF;
                        trials_per_temp  <= (sweeps_per_temp != 6'd0) ? {2'b00, sweeps_per_temp} * 8'(num_spins) : 8'd32;
                        max_steps_reg    <= (max_temp_steps != 8'd0) ? max_temp_steps : 8'd50;
                        spin_idx         <= 3'd0;
                        trial_cnt        <= 8'd0;
                        step_cnt         <= 8'd0;
                        accepted_cnt     <= 16'd0;
                        load_seed_strobe <= 1'b1;
                        status_reg       <= STATUS_RUNNING;
                        state            <= SA_START_INIT_E;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: SA_START_INIT_E - Evaluate Initial State Energy
                // -------------------------------------------------------------
                SA_START_INIT_E: begin
                    e_spin_in     <= q_curr;
                    start_e_total <= 1'b1;
                    state         <= SA_WAIT_INIT_E;
                end

                // -------------------------------------------------------------
                // STATE 2: SA_WAIT_INIT_E - Latch Initial Energy & Start Sweeps
                // -------------------------------------------------------------
                SA_WAIT_INIT_E: begin
                    if (e_done) begin
                        e_curr    <= energy_out;
                        e_best    <= energy_out;
                        trial_cnt <= 8'd0;
                        state     <= SA_START_FLIP;
                    end
                end

                // -------------------------------------------------------------
                // STATE 3: SA_START_FLIP - Stochastically Select Spin & Evaluate ΔE
                // -------------------------------------------------------------
                SA_START_FLIP: begin
                    next_rand_strobe <= 1'b1;
                    chosen_spin = (num_spins_reg == 4'd4) ? {1'b0, rand_num[1:0]} : (rand_num[2:0] % num_spins_reg[2:0]);
                    spin_idx    <= chosen_spin;
                    e_spin_in   <= q_curr;
                    e_flip_idx  <= chosen_spin;
                    start_e_delta <= 1'b1;
                    state       <= SA_WAIT_DELTA;
                end

                // -------------------------------------------------------------
                // STATE 4: SA_WAIT_DELTA - Metropolis Decision
                // -------------------------------------------------------------
                SA_WAIT_DELTA: begin
                    if (e_done) begin
                        delta_e_reg <= energy_out;

                        if (energy_out < Q16_ZERO) begin
                            // Strictly downhill move -> Unconditional Acceptance
                            new_e = e_curr + energy_out;
                            q_curr <= flip_spin(q_curr, spin_idx);
                            e_curr <= new_e;
                            accepted_cnt <= accepted_cnt + 1'b1;

                            if (new_e < e_best) begin
                                q_best <= flip_spin(q_curr, spin_idx);
                                e_best <= new_e;
                            end

                            state <= SA_ADVANCE_SPIN;
                        end else begin
                            // Neutral or Uphill move -> Compute u = ΔE / T
                            div_dividend <= energy_out;
                            div_divisor  <= t_curr;
                            div_start    <= 1'b1;
                            state        <= SA_WAIT_DIV_U;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE 5: SA_WAIT_DIV_U - Compute P = exp(-u)
                // -------------------------------------------------------------
                SA_WAIT_DIV_U: begin
                    if (div_done) begin
                        exp_u_in         <= (div_by_zero) ? 32'h0010_0000 : div_quotient;
                        exp_start        <= 1'b1;
                        next_rand_strobe <= 1'b1; // Advance PRNG for threshold R
                        state            <= SA_WAIT_EXP;
                    end
                end

                // -------------------------------------------------------------
                // STATE 6: SA_WAIT_EXP - Stochastic Metropolis Acceptance
                // -------------------------------------------------------------
                SA_WAIT_EXP: begin
                    if (exp_done) begin
                        // If uniform random R < P -> Accept jump
                        if (rand_num < exp_prob_out) begin
                            new_e = e_curr + delta_e_reg;
                            q_curr <= flip_spin(q_curr, spin_idx);
                            e_curr <= new_e;
                            accepted_cnt <= accepted_cnt + 1'b1;

                            if (new_e < e_best) begin
                                q_best <= flip_spin(q_curr, spin_idx);
                                e_best <= new_e;
                            end
                        end

                        state <= SA_ADVANCE_SPIN;
                    end
                end

                // -------------------------------------------------------------
                // STATE 7: SA_ADVANCE_SPIN - Coordinate Trials & Temperature
                // -------------------------------------------------------------
                SA_ADVANCE_SPIN: begin
                    if (trial_cnt + 1'b1 < trials_per_temp) begin
                        trial_cnt <= trial_cnt + 1'b1;
                        state     <= SA_START_FLIP;
                    end else begin
                        // Thermal Cooling Step: T_next = γ * T_curr
                        trial_cnt <= 8'd0;
                        t_curr    <= q16_mul(t_curr, gamma_reg);
                        step_cnt  <= step_cnt + 1'b1;

                        if (q16_mul(t_curr, gamma_reg) <= t_end_reg) begin
                            status_reg <= STATUS_CONVERGED;
                            state      <= SA_EVAL_FINAL_E;
                        end else if (step_cnt + 1'b1 >= max_steps_reg) begin
                            status_reg <= STATUS_MAX_ITERS;
                            state      <= SA_EVAL_FINAL_E;
                        end else begin
                            state <= SA_START_FLIP;
                        end
                    end
                end

                // -------------------------------------------------------------
                // STATE 8: SA_EVAL_FINAL_E - Verify Ground State Energy E(q*)
                // -------------------------------------------------------------
                SA_EVAL_FINAL_E: begin
                    e_spin_in     <= q_best;
                    start_e_total <= 1'b1;
                    state         <= SA_WAIT_FINAL_E;
                end

                // -------------------------------------------------------------
                // STATE 9: SA_WAIT_FINAL_E - Latch Final Optimal Ground Energy
                // -------------------------------------------------------------
                SA_WAIT_FINAL_E: begin
                    if (e_done) begin
                        e_best <= energy_out;
                        state  <= SA_DONE;
                    end
                end

                // -------------------------------------------------------------
                // STATE 10: SA_DONE - Completion Strobe
                // -------------------------------------------------------------
                SA_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= SA_IDLE;
                end

                default: state <= SA_IDLE;
            endcase
        end
    end

endmodule
