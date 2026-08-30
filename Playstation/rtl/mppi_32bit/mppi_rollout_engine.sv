// =============================================================================
// File Name   : mppi_rollout_engine.sv
// Module Name : mppi_rollout_engine
// Project     : Model Predictive Path Integral (MPPI) Accelerator (Solver #33)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined Stochastic Forward Dynamics & Trajectory Cost Evaluator:
//   1. Generates stochastic control perturbations δu_{k, t} ~ N(0, σ^2 I)
//      - Rollout 0: Nominal anchor (δu = 0)
//      - Rollout 1: Positive perturbation (+δu) with decorrelated channels
//      - Rollout 2: Mirrored antithetic perturbation (-δu)
//      - Rollout 3: Random exploration perturbation
//   2. Propagates dynamics: x_{t+1} = A * x_t + B * (u_t + δu_{k, t})
//   3. Accumulates total path cost S_k = ∑ stage_cost + terminal_cost
// =============================================================================

`timescale 1ns / 1ps

import mppi_types_pkg::*;
`include "mppi_helpers.svh"

module mppi_rollout_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  state_vec_t         x_init,           // Current robot state x_curr (2x1)
    input  state_vec_t         x_ref,            // Target reference state x_ref (2x1)
    input  ctrl_seq_t          nominal_seq,      // Nominal control sequence U (8x1 packed)
    input  logic [2:0]         num_rollouts,     // Total rollouts K (2..4)
    input  logic [2:0]         horizon,          // Time horizon T (1..4)
    input  q16_t               noise_sigma,      // Perturbation std dev σ
    input  ctrl_vec_t          lb_ctrl,          // Control lower bounds (2x1)
    input  ctrl_vec_t          ub_ctrl,          // Control upper bounds (2x1)
    input  mat22_t             A_mat,            // Dynamics matrix A (2x2 packed)
    input  mat22_t             B_mat,            // Input matrix B (2x2 packed)
    input  state_vec_t         Q_diag,           // State stage weight Q (2x1)
    input  ctrl_vec_t          R_diag,           // Control stage weight R (2x1)
    input  state_vec_t         Qf_diag,          // Terminal state weight Qf (2x1)

    output noise_arr_t         noise_rollouts,   // Perturbations δu (32x1 packed)
    output cost_arr_t          rollout_costs,    // Evaluated costs S_k (4x1)
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        ROL_IDLE     = 3'd0,
        ROL_GEN_NOISE= 3'd1,
        ROL_STEP_DYN = 3'd2,
        ROL_TERMINAL = 3'd3,
        ROL_NEXT_K   = 3'd4,
        ROL_DONE     = 3'd5
    } rol_state_t;

    rol_state_t state;

    noise_arr_t noise_reg;
    cost_arr_t  costs_reg;

    assign noise_rollouts = noise_reg;
    assign rollout_costs  = costs_reg;

    // PRNG Instance
    logic        prng_next;
    logic [31:0] prng_rand_out;
    q16_t        prng_q16;

    xorshift32_prng u_prng (
        .clk      (clk),
        .rst_n    (rst_n),
        .next_rand(prng_next),
        .seed     (32'h55AA_AA55),
        .rand_out (prng_rand_out),
        .rand_q16 (prng_q16)
    );

    logic [1:0] curr_k;
    logic [1:0] curr_t;
    state_vec_t sim_x;
    ctrl_vec_t  pert_u;
    q16_t       stage_c, term_c;
    q16_t       z_noise_0, z_noise_1, delta_val_0, delta_val_1, u_nom_0, u_nom_1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ROL_IDLE;
            noise_reg <= '0;
            costs_reg <= '0;
            curr_k    <= 2'd0;
            curr_t    <= 2'd0;
            sim_x     <= '0;
            pert_u    <= '0;
            prng_next <= 1'b0;
            done      <= 1'b0;
            busy      <= 1'b0;
        end else begin
            prng_next <= 1'b0;

            case (state)
                ROL_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        noise_reg <= '0;
                        costs_reg <= '0;
                        curr_k    <= 2'd0;
                        curr_t    <= 2'd0;
                        sim_x     <= x_init;
                        prng_next <= 1'b1;
                        state     <= ROL_GEN_NOISE;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Generate noise δu_{k, t} for both control channels independently
                ROL_GEN_NOISE: begin
                    u_nom_0 = nominal_seq[{curr_t, 1'b0}];
                    u_nom_1 = nominal_seq[{curr_t, 1'b1}];

                    z_noise_0 = q16_t'((prng_rand_out[31:16] << 1) - 32'h0001_0000);
                    z_noise_1 = q16_t'((prng_rand_out[15:0] << 1)  - 32'h0001_0000);

                    if (curr_k == 2'd0) begin
                        // Rollout 0: Nominal anchor (δu = 0)
                        delta_val_0 = 32'sd0;
                        delta_val_1 = 32'sd0;
                    end else if (curr_k == 2'd1) begin
                        // Rollout 1: Positive perturbation (+δu)
                        delta_val_0 = q16_mul(noise_sigma, z_noise_0);
                        delta_val_1 = q16_mul(noise_sigma, z_noise_1);
                    end else if (curr_k == 2'd2) begin
                        // Rollout 2: Antithetic mirrored perturbation (-δu)
                        delta_val_0 = -noise_reg[{2'd1, curr_t, 1'b0}];
                        delta_val_1 = -noise_reg[{2'd1, curr_t, 1'b1}];
                    end else begin
                        // Rollout 3: Random perturbation
                        delta_val_0 = q16_mul(noise_sigma, z_noise_0);
                        delta_val_1 = q16_mul(noise_sigma, z_noise_1);
                    end

                    noise_reg[{curr_k, curr_t, 1'b0}] <= delta_val_0;
                    noise_reg[{curr_k, curr_t, 1'b1}] <= delta_val_1;

                    pert_u[0] <= clamp_ctrl(u_nom_0 + delta_val_0, lb_ctrl[0], ub_ctrl[0]);
                    pert_u[1] <= clamp_ctrl(u_nom_1 + delta_val_1, lb_ctrl[1], ub_ctrl[1]);

                    prng_next <= 1'b1;
                    state     <= ROL_STEP_DYN;
                end

                // Step 2: Propagate dynamics and accumulate stage cost
                ROL_STEP_DYN: begin
                    stage_c = eval_stage_cost(sim_x, x_ref, pert_u, Q_diag, R_diag);
                    costs_reg[curr_k] <= costs_reg[curr_k] + stage_c;

                    sim_x <= step_dynamics(sim_x, pert_u, A_mat, B_mat);

                    if (curr_t + 1'b1 < horizon) begin
                        curr_t <= curr_t + 1'b1;
                        state  <= ROL_GEN_NOISE;
                    end else begin
                        state  <= ROL_TERMINAL;
                    end
                end

                // Step 3: Add terminal cost phi(x_T)
                ROL_TERMINAL: begin
                    term_c = eval_terminal_cost(sim_x, x_ref, Qf_diag);
                    costs_reg[curr_k] <= costs_reg[curr_k] + term_c;
                    state  <= ROL_NEXT_K;
                end

                // Step 4: Advance to next rollout or finish
                ROL_NEXT_K: begin
                    if (curr_k + 1'b1 < num_rollouts) begin
                        curr_k <= curr_k + 1'b1;
                        curr_t <= 2'd0;
                        sim_x  <= x_init;
                        state  <= ROL_GEN_NOISE;
                    end else begin
                        state  <= ROL_DONE;
                    end
                end

                ROL_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= ROL_IDLE;
                end

                default: state <= ROL_IDLE;
            endcase
        end
    end

endmodule
