// =============================================================================
// File Name   : dd_top.sv
// Module Name : dd_top
// Project     : Dual Decomposition Engine (Solver #22)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Multi-Agent Coordinator for Dual Decomposition.
//   Coordinates parallel agent execution and master price updates:
//   1. Broadcast shadow prices λ_k to S local agents
//   2. Agents compute local decisions x_s*(λ_k) = Q_s^{-1}(p_s - A_s^T λ_k) in parallel
//   3. Master aggregates total consumption sum_{s=1}^S A_s x_s
//   4. Master updates shadow prices λ_{k+1} and checks resource balance
// =============================================================================

`timescale 1ns / 1ps

import dd_types_pkg::*;
`include "dd_helpers.svh"

module dd_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Problem Formulation & Controls
    // -------------------------------------------------------------------------
    input  logic               start,            // 1-cycle start trigger
    input  logic [2:0]         num_agents,       // Number of active agents S (1..4)
    input  logic [1:0]         local_dim,        // Local dimension N_s (1..2)
    input  logic [1:0]         num_res,          // Number of coupled resources M (1..2)
    input  dd_couple_mode_t    couple_mode,      // Equality or inequality coupling

    input  agent_mat_t         q_inv_mats[MAX_AGENTS], // Inverse Hessians Q_s^{-1}
    input  agent_vec_t         p_vectors[MAX_AGENTS],  // Local cost/profit vectors p_s
    input  couple_mat_t        a_matrices[MAX_AGENTS], // Coupling matrices A_s (M x N_s)
    input  res_vec_t           c_budget,         // Global resource capacity c (2x1)
    input  res_vec_t           lambda_init,      // Initial shadow price guess λ_0
    input  q16_t               step_alpha,       // Dual step size α

    input  q16_t               tol_feas,         // Feasibility tolerance ε_feas
    input  q16_t               tol_opt,          // Price consensus tolerance ε_opt
    input  logic [15:0]        max_iters,        // Maximum outer iterations

    // -------------------------------------------------------------------------
    // Interface 2: Results & Status Outputs
    // -------------------------------------------------------------------------
    output all_agents_vec_t    x_optimal,        // Optimal local decisions x_s*
    output res_vec_t           lambda_optimal,   // Optimal shadow market prices λ*
    output q16_t               feas_error,       // Constraint feasibility violation ||r||_inf
    output logic [15:0]        iter_count,       // Outer iterations completed
    output status_t            status,           // Convergence status code
    output logic               done,             // 1-cycle completion strobe
    output logic               busy              // High while active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        DD_IDLE         = 3'd0,
        DD_START_AGENTS = 3'd1,
        DD_WAIT_AGENTS  = 3'd2,
        DD_START_MASTER = 3'd3,
        DD_WAIT_MASTER  = 3'd4,
        DD_CHECK_CONV   = 3'd5,
        DD_DONE_STATE   = 3'd6
    } dd_state_t;

    dd_state_t state;

    // Registers
    all_agents_vec_t x_opt_regs;
    res_vec_t        res_usage_regs[MAX_AGENTS];
    res_vec_t        lambda_curr;
    q16_t            viol_reg;
    q16_t            pdelta_reg;
    logic [15:0]     iters_cnt;
    status_t         status_reg;

    // Latched inputs
    logic [2:0]      num_agents_reg;
    logic [1:0]      local_dim_reg;
    logic [1:0]      num_res_reg;
    dd_couple_mode_t couple_mode_reg;
    agent_mat_t      q_inv_regs[MAX_AGENTS];
    agent_vec_t      p_vec_regs[MAX_AGENTS];
    couple_mat_t     a_mat_regs[MAX_AGENTS];
    res_vec_t        c_budget_reg;
    q16_t            alpha_reg;
    q16_t            tol_feas_reg;
    q16_t            tol_opt_reg;
    logic [15:0]     max_iters_reg;

    // Parallel Agent Node Control & Signals
    logic       start_agents;
    agent_vec_t agent_x_out[MAX_AGENTS];
    res_vec_t   agent_usage_out[MAX_AGENTS];
    logic       agent_done[MAX_AGENTS];
    logic       agent_busy[MAX_AGENTS];

    generate
        for (genvar s = 0; s < MAX_AGENTS; s++) begin : gen_agents
            dd_agent_node #(
                .AGENT_ID(s)
            ) u_agent (
                .clk       (clk),
                .rst_n     (rst_n),
                .start     (start_agents),
                .local_dim (local_dim_reg),
                .num_res   (num_res_reg),
                .q_inv_mat (q_inv_regs[s]),
                .p_vec     (p_vec_regs[s]),
                .a_mat     (a_mat_regs[s]),
                .lambda_bus(lambda_curr),
                .x_opt     (agent_x_out[s]),
                .res_usage (agent_usage_out[s]),
                .done      (agent_done[s]),
                .busy      (agent_busy[s])
            );
        end
    endgenerate

    // Master Engine Control & Signals
    logic     start_master;
    res_vec_t master_lam_next;
    res_vec_t master_r_err;
    q16_t     master_viol;
    q16_t     master_pdelta;
    logic     master_done;
    logic     master_busy;

    dd_master_engine u_master (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start_master),
        .num_agents     (num_agents_reg),
        .num_res        (num_res_reg),
        .couple_mode    (couple_mode_reg),
        .c_budget       (c_budget_reg),
        .agent_res_usage(res_usage_regs),
        .lambda_curr    (lambda_curr),
        .step_alpha     (alpha_reg),
        .lambda_next    (master_lam_next),
        .res_error      (master_r_err),
        .viol_norm      (master_viol),
        .price_delta    (master_pdelta),
        .done           (master_done),
        .busy           (master_busy)
    );

    // Outputs
    assign x_optimal      = x_opt_regs;
    assign lambda_optimal = lambda_curr;
    assign feas_error     = viol_reg;
    assign iter_count     = iters_cnt;
    assign status         = status_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= DD_IDLE;
            for (int s = 0; s < MAX_AGENTS; s++) begin
                x_opt_regs[s]     <= '0;
                res_usage_regs[s] <= '0;
                q_inv_regs[s]     <= '0;
                p_vec_regs[s]     <= '0;
                a_mat_regs[s]     <= '0;
            end
            lambda_curr     <= '0;
            viol_reg        <= Q16_ZERO;
            pdelta_reg      <= Q16_ZERO;
            iters_cnt       <= 16'd0;
            status_reg      <= STATUS_IDLE;
            num_agents_reg  <= 3'd2;
            local_dim_reg   <= 2'd1;
            num_res_reg     <= 2'd1;
            couple_mode_reg <= COUPLE_EQ;
            c_budget_reg    <= '0;
            alpha_reg       <= Q16_ALPHA_DEF;
            tol_feas_reg    <= Q16_FEAS_TOL;
            tol_opt_reg     <= Q16_OPT_TOL;
            max_iters_reg   <= 16'd100;
            start_agents    <= 1'b0;
            start_master    <= 1'b0;
            done            <= 1'b0;
            busy            <= 1'b0;
        end else begin
            start_agents <= 1'b0;
            start_master <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: DD_IDLE - Latch Parameters & Initialize Shadow Prices
                // -------------------------------------------------------------
                DD_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy            <= 1'b1;
                        num_agents_reg  <= num_agents;
                        local_dim_reg   <= local_dim;
                        num_res_reg     <= num_res;
                        couple_mode_reg <= couple_mode;
                        for (int s = 0; s < MAX_AGENTS; s++) begin
                            q_inv_regs[s] <= q_inv_mats[s];
                            p_vec_regs[s] <= p_vectors[s];
                            a_mat_regs[s] <= a_matrices[s];
                        end
                        c_budget_reg    <= c_budget;
                        lambda_curr     <= lambda_init;
                        alpha_reg       <= (step_alpha != Q16_ZERO) ? step_alpha : Q16_ALPHA_DEF;
                        tol_feas_reg    <= (tol_feas != Q16_ZERO) ? tol_feas : Q16_FEAS_TOL;
                        tol_opt_reg     <= (tol_opt != Q16_ZERO) ? tol_opt : Q16_OPT_TOL;
                        max_iters_reg   <= (max_iters != 16'd0) ? max_iters : 16'd100;
                        iters_cnt       <= 16'd0;
                        status_reg      <= STATUS_RUNNING;
                        state           <= DD_START_AGENTS;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: DD_START_AGENTS - Trigger Parallel Local Optimizers
                // -------------------------------------------------------------
                DD_START_AGENTS: begin
                    start_agents <= 1'b1;
                    state        <= DD_WAIT_AGENTS;
                end

                // -------------------------------------------------------------
                // STATE 2: DD_WAIT_AGENTS - Latch Decisions & Resource Usages
                // -------------------------------------------------------------
                DD_WAIT_AGENTS: begin
                    if (agent_done[0]) begin
                        for (int s = 0; s < MAX_AGENTS; s++) begin
                            x_opt_regs[s]     <= agent_x_out[s];
                            res_usage_regs[s] <= agent_usage_out[s];
                        end
                        state <= DD_START_MASTER;
                    end
                end

                // -------------------------------------------------------------
                // STATE 3: DD_START_MASTER - Trigger Master Coordinator
                // -------------------------------------------------------------
                DD_START_MASTER: begin
                    start_master <= 1'b1;
                    state        <= DD_WAIT_MASTER;
                end

                // -------------------------------------------------------------
                // STATE 4: DD_WAIT_MASTER - Latch Price Updates & Violations
                // -------------------------------------------------------------
                DD_WAIT_MASTER: begin
                    if (master_done) begin
                        lambda_curr <= master_lam_next;
                        viol_reg    <= master_viol;
                        pdelta_reg  <= master_pdelta;
                        iters_cnt   <= iters_cnt + 1'b1;
                        state       <= DD_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // STATE 5: DD_CHECK_CONV - Convergence & Max-Iter Decision
                // -------------------------------------------------------------
                DD_CHECK_CONV: begin
                    if (viol_reg <= tol_feas_reg && pdelta_reg <= tol_opt_reg) begin
                        status_reg <= STATUS_CONVERGED;
                        state      <= DD_DONE_STATE;
                    end else if (iters_cnt >= max_iters_reg) begin
                        status_reg <= STATUS_MAX_ITERS;
                        state      <= DD_DONE_STATE;
                    end else begin
                        state <= DD_START_AGENTS;
                    end
                end

                // -------------------------------------------------------------
                // STATE 6: DD_DONE_STATE - Completion Strobe
                // -------------------------------------------------------------
                DD_DONE_STATE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= DD_IDLE;
                end

                default: state <= DD_IDLE;
            endcase
        end
    end

endmodule
