// =============================================================================
// File Name   : mppi_top.sv
// Module Name : mppi_top
// Project     : Model Predictive Path Integral (MPPI) Accelerator (Solver #33)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Controller for Model Predictive Path Integral (MPPI) Optimizer.
//   Manages stochastic trajectory rollouts, forward linear/non-linear dynamics,
//   softmax importance weights, path-integral weighted control aggregation,
//   real-time actuator streaming u0*, and receding horizon buffer shifting.
// =============================================================================

`timescale 1ns / 1ps

import mppi_types_pkg::*;
`include "mppi_helpers.svh"

module mppi_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Initialization & System Parameters
    // -------------------------------------------------------------------------
    input  logic               init_mppi,        // 1-cycle initialization strobe
    input  logic [2:0]         horizon,          // Time horizon T (1..4)
    input  logic [2:0]         num_rollouts,     // Total rollouts K (2..4)
    input  q16_t               lambda_inv,       // Softmax temperature 1 / λ
    input  q16_t               noise_sigma,      // Perturbation std dev σ
    input  ctrl_vec_t          lb_ctrl,          // Control lower bounds (2x1)
    input  ctrl_vec_t          ub_ctrl,          // Control upper bounds (2x1)
    input  mat22_t             A_mat,            // Dynamics matrix A (2x2 packed)
    input  mat22_t             B_mat,            // Input matrix B (2x2 packed)
    input  state_vec_t         Q_diag,           // State stage weight Q (2x1)
    input  ctrl_vec_t          R_diag,           // Control stage weight R (2x1)
    input  state_vec_t         Qf_diag,          // Terminal state weight Qf (2x1)
    input  ctrl_seq_t          init_nominal,     // Initial nominal control sequence (8x1)

    // -------------------------------------------------------------------------
    // Interface 2: Execution & Real-Time Control Output
    // -------------------------------------------------------------------------
    input  logic               step_mppi,        // 1-cycle trigger per control interval
    input  state_vec_t         x_curr,           // Current robot state x_curr (2x1)
    input  state_vec_t         x_ref,            // Reference target state x_ref (2x1)

    output ctrl_vec_t          u_opt_current,    // Optimal immediate actuator command u0* (2x1)
    output ctrl_seq_t          u_sequence,       // Full optimized horizon sequence U* (8x1)
    output q16_t               best_cost,        // Minimum trajectory cost β
    output weight_arr_t        weights,          // Rollout importance weights (4x1)
    output status_t            status,           // Status code
    output logic               mppi_done,        // MPPI optimization complete strobe
    output logic               busy              // High while engine is active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        MPPI_IDLE      = 3'd0,
        MPPI_ROL_START = 3'd1,
        MPPI_ROL_WAIT  = 3'd2,
        MPPI_WGT_START = 3'd3,
        MPPI_WGT_WAIT  = 3'd4,
        MPPI_UPDATE    = 3'd5,
        MPPI_DONE      = 3'd6
    } mppi_state_t;

    mppi_state_t state;

    // Internal Configuration Registers
    logic [2:0]   hor_reg;
    logic [2:0]   k_rol_reg;
    q16_t         lam_inv_reg;
    q16_t         sigma_reg;
    ctrl_vec_t    lb_reg;
    ctrl_vec_t    ub_reg;
    mat22_t       A_reg;
    mat22_t       B_reg;
    state_vec_t   Q_reg;
    ctrl_vec_t    R_reg;
    state_vec_t   Qf_reg;
    ctrl_seq_t    nominal_seq_reg;
    status_t      status_reg;

    // Outputs
    ctrl_vec_t    u_opt_reg;
    ctrl_seq_t    u_seq_reg;
    q16_t         best_cost_reg;
    weight_arr_t  weights_reg;

    assign u_opt_current = u_opt_reg;
    assign u_sequence    = u_seq_reg;
    assign best_cost     = best_cost_reg;
    assign weights       = weights_reg;
    assign status        = status_reg;

    // Sub-engine 1: Rollout Instance
    logic        start_rol;
    state_vec_t  x_init_in;
    state_vec_t  x_ref_in;
    noise_arr_t  rol_noise_out;
    cost_arr_t   rol_costs_out;
    logic        rol_done;
    logic        rol_busy;

    mppi_rollout_engine u_rol (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start_rol),
        .x_init        (x_init_in),
        .x_ref         (x_ref_in),
        .nominal_seq   (nominal_seq_reg),
        .num_rollouts  (k_rol_reg),
        .horizon       (hor_reg),
        .noise_sigma   (sigma_reg),
        .lb_ctrl       (lb_reg),
        .ub_ctrl       (ub_reg),
        .A_mat         (A_reg),
        .B_mat         (B_reg),
        .Q_diag        (Q_reg),
        .R_diag        (R_reg),
        .Qf_diag       (Qf_reg),
        .noise_rollouts(rol_noise_out),
        .rollout_costs (rol_costs_out),
        .done          (rol_done),
        .busy          (rol_busy)
    );

    // Sub-engine 2: Softmax Weights Instance
    logic        start_wgt;
    weight_arr_t wgt_out;
    q16_t        wgt_min_cost;
    logic        wgt_done;
    logic        wgt_busy;

    mppi_weights_engine u_wgt (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (start_wgt),
        .rollout_costs(rol_costs_out),
        .num_rollouts (k_rol_reg),
        .lambda_inv   (lam_inv_reg),
        .weights      (wgt_out),
        .min_cost     (wgt_min_cost),
        .done         (wgt_done),
        .busy         (wgt_busy)
    );

    q16_t weighted_noise, u_star_val, u_nom_val;
    q16_t u_star_arr [0:MAX_HORIZON-1][0:MAX_CTRL-1];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= MPPI_IDLE;
            hor_reg         <= 3'd4;
            k_rol_reg       <= 3'd4;
            lam_inv_reg     <= 32'h0001_0000; // 1 / λ = 1.0
            sigma_reg       <= 32'h0000_8000; // σ = 0.50
            lb_reg          <= '0;
            ub_reg          <= '0;
            A_reg           <= '0;
            B_reg           <= '0;
            Q_reg           <= '0;
            R_reg           <= '0;
            Qf_reg          <= '0;
            nominal_seq_reg <= '0;
            status_reg      <= STATUS_IDLE;
            u_opt_reg       <= '0;
            u_seq_reg       <= '0;
            best_cost_reg   <= Q16_MAX_POS;
            weights_reg     <= '0;
            start_rol       <= 1'b0;
            start_wgt       <= 1'b0;
            x_init_in       <= '0;
            x_ref_in        <= '0;
            mppi_done       <= 1'b0;
            busy            <= 1'b0;
        end else begin
            start_rol <= 1'b0;
            start_wgt <= 1'b0;
            mppi_done <= 1'b0;

            case (state)
                MPPI_IDLE: begin
                    if (init_mppi) begin
                        hor_reg         <= (horizon != 3'd0) ? horizon : 3'd4;
                        k_rol_reg       <= (num_rollouts != 3'd0) ? num_rollouts : 3'd4;
                        lam_inv_reg     <= (lambda_inv != 32'sd0) ? lambda_inv : 32'h0001_0000;
                        sigma_reg       <= (noise_sigma != 32'sd0) ? noise_sigma : 32'h0000_8000;
                        lb_reg          <= lb_ctrl;
                        ub_reg          <= ub_ctrl;
                        A_reg           <= A_mat;
                        B_reg           <= B_mat;
                        Q_reg           <= Q_diag;
                        R_reg           <= R_diag;
                        Qf_reg          <= Qf_diag;
                        nominal_seq_reg <= init_nominal;
                        status_reg      <= STATUS_IDLE;
                    end else if (step_mppi) begin
                        busy       <= 1'b1;
                        x_init_in  <= x_curr;
                        x_ref_in   <= x_ref;
                        status_reg <= STATUS_ROLLOUT;
                        start_rol  <= 1'b1;
                        state      <= MPPI_ROL_WAIT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Wait for stochastic rollout engine
                MPPI_ROL_WAIT: begin
                    if (rol_done) begin
                        status_reg <= STATUS_WEIGHTS;
                        start_wgt  <= 1'b1;
                        state      <= MPPI_WGT_WAIT;
                    end
                end

                // Step 2: Wait for importance weights engine
                MPPI_WGT_WAIT: begin
                    if (wgt_done) begin
                        weights_reg   <= wgt_out;
                        best_cost_reg <= wgt_min_cost;
                        status_reg    <= STATUS_UPDATE;
                        state         <= MPPI_UPDATE;
                    end
                end

                // Step 3: Compute optimal control u_t* = u_t + ∑ w_k δu_{k, t}
                MPPI_UPDATE: begin
                    for (int t = 0; t < MAX_HORIZON; t++) begin
                        for (int m = 0; m < MAX_CTRL; m++) begin
                            if (t < hor_reg) begin
                                weighted_noise = 32'sd0;
                                for (int k = 0; k < MAX_ROLLOUTS; k++) begin
                                    if (k < k_rol_reg) begin
                                        weighted_noise = weighted_noise +
                                            q16_mul(weights_reg[k], rol_noise_out[{2'(k), 2'(t), 1'(m)}]);
                                    end
                                end

                                u_nom_val  = nominal_seq_reg[{2'(t), 1'(m)}];
                                u_star_val = clamp_ctrl(u_nom_val + weighted_noise, lb_reg[m], ub_reg[m]);
                                u_star_arr[t][m] = u_star_val;

                                u_seq_reg[{2'(t), 1'(m)}] <= u_star_val;

                                if (t == 0) begin
                                    u_opt_reg[m] <= u_star_val;
                                end
                            end else begin
                                u_star_arr[t][m] = 32'sd0;
                                u_seq_reg[{2'(t), 1'(m)}] <= 32'sd0;
                            end
                        end
                    end

                    // Receding Horizon Shift: u_t <- u_{t+1}
                    for (int t = 0; t < MAX_HORIZON - 1; t++) begin
                        for (int m = 0; m < MAX_CTRL; m++) begin
                            nominal_seq_reg[{2'(t), 1'(m)}] <= u_star_arr[t + 1][m];
                        end
                    end
                    for (int m = 0; m < MAX_CTRL; m++) begin
                        nominal_seq_reg[{2'(MAX_HORIZON - 1), 1'(m)}] <= 32'sd0;
                    end

                    state <= MPPI_DONE;
                end

                MPPI_DONE: begin
                    status_reg <= STATUS_DONE;
                    mppi_done  <= 1'b1;
                    busy       <= 1'b0;
                    state      <= MPPI_IDLE;
                end

                default: state <= MPPI_IDLE;
            endcase
        end
    end

endmodule
