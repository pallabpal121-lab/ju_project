// =============================================================================
// File Name   : ekf_top.sv
// Module Name : ekf_top
// Project     : Extended Kalman Filter (EKF) Accelerator (Solver #24)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Controller for Real-Time Extended Kalman Filtering.
//   Supports flexible predict-correct cycles:
//   1. Initialization: Resets state x_0 and error covariance P_0
//   2. Predict Phase:  x^- = f(x, u), P^- = F P F^T + Q
//   3. Correct Phase:  y = z - h(x^-), S = H P^- H^T + R, K = P^- H^T S^-1,
//                      x = x^- + K y, P = (I - K H) P^-
// =============================================================================

`timescale 1ns / 1ps

import ekf_types_pkg::*;
`include "ekf_helpers.svh"

module ekf_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Configuration & Initialization
    // -------------------------------------------------------------------------
    input  logic               init_ekf,         // 1-cycle reset & initialization strobe
    input  logic [2:0]         state_dim,        // State dimension N (1..4)
    input  logic [1:0]         meas_dim,         // Measurement dimension M (1..2)
    input  state_vec_t         x_init,           // Initial state vector x_0
    input  state_mat_t         p_init,           // Initial covariance P_0

    // -------------------------------------------------------------------------
    // Interface 2: Prediction Step Inputs
    // -------------------------------------------------------------------------
    input  logic               predict_valid,    // 1-cycle strobe for time-update
    input  state_vec_t         f_x_pred,         // Predicted state f(x, u)
    input  state_mat_t         f_mat,            // State transition Jacobian F_k
    input  state_mat_t         q_mat,            // Process noise covariance Q_k

    // -------------------------------------------------------------------------
    // Interface 3: Correction Step Inputs
    // -------------------------------------------------------------------------
    input  logic               correct_valid,    // 1-cycle strobe for measurement update
    input  meas_vec_t          z_meas,           // Actual measurement z_k
    input  meas_vec_t          h_x_pred,         // Predicted measurement h(x^-)
    input  meas_mat_t          h_mat,            // Measurement Jacobian H_k
    input  innov_mat_t         r_mat,            // Measurement noise covariance R_k

    // -------------------------------------------------------------------------
    // Interface 4: Results & Status Outputs
    // -------------------------------------------------------------------------
    output state_vec_t         x_estimated,      // Current state estimate x_k
    output state_mat_t         p_covariance,     // Current error covariance P_k
    output meas_vec_t          innov_residual,   // Innovation residual y_k
    output gain_mat_t          k_gain_out,       // Kalman gain matrix K_k
    output status_t            status,           // Status code
    output logic               predict_done,     // 1-cycle prediction complete strobe
    output logic               correct_done,     // 1-cycle correction complete strobe
    output logic               busy              // High while engine is active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        EKF_IDLE         = 3'd0,
        EKF_WAIT_PREDICT = 3'd1,
        EKF_WAIT_CORRECT = 3'd2
    } ekf_state_t;

    ekf_state_t state;

    // Internal State & Covariance Registers
    state_vec_t  x_reg;
    state_mat_t  p_reg;
    logic [2:0]  state_dim_reg;
    logic [1:0]  meas_dim_reg;
    status_t     status_reg;
    meas_vec_t   innov_latched;
    gain_mat_t   k_latched;

    // Sub-engine 1: ekf_predict_engine
    logic       start_pred;
    state_vec_t pred_x_out;
    state_mat_t pred_p_out;
    logic       pred_done;
    logic       pred_busy;

    ekf_predict_engine u_pred (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (start_pred),
        .state_dim(state_dim_reg),
        .f_x_pred (f_x_pred),
        .p_curr   (p_reg),
        .f_mat    (f_mat),
        .q_mat    (q_mat),
        .x_prior  (pred_x_out),
        .p_prior  (pred_p_out),
        .done     (pred_done),
        .busy     (pred_busy)
    );

    // Sub-engine 2: ekf_correct_engine
    logic       start_corr;
    state_vec_t corr_x_out;
    state_mat_t corr_p_out;
    meas_vec_t  corr_y_out;
    gain_mat_t  corr_k_out;
    logic       corr_done;
    logic       corr_busy;

    ekf_correct_engine u_corr (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (start_corr),
        .state_dim(state_dim_reg),
        .meas_dim (meas_dim_reg),
        .x_prior  (x_reg),
        .p_prior  (p_reg),
        .z_meas   (z_meas),
        .h_x_pred (h_x_pred),
        .h_mat    (h_mat),
        .r_mat    (r_mat),
        .x_post   (corr_x_out),
        .p_post   (corr_p_out),
        .innov_y  (corr_y_out),
        .k_gain   (corr_k_out),
        .done     (corr_done),
        .busy     (corr_busy)
    );

    // Outputs
    assign x_estimated    = x_reg;
    assign p_covariance   = p_reg;
    assign innov_residual = innov_latched;
    assign k_gain_out     = k_latched;
    assign status         = status_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= EKF_IDLE;
            x_reg         <= '0;
            p_reg         <= '0;
            state_dim_reg <= 3'd2;
            meas_dim_reg  <= 2'd1;
            status_reg    <= STATUS_IDLE;
            innov_latched <= '0;
            k_latched     <= '0;
            start_pred    <= 1'b0;
            start_corr    <= 1'b0;
            predict_done  <= 1'b0;
            correct_done  <= 1'b0;
            busy          <= 1'b0;
        end else begin
            start_pred   <= 1'b0;
            start_corr   <= 1'b0;
            predict_done <= 1'b0;
            correct_done <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: EKF_IDLE - Handle Init, Predict, or Correct Strobes
                // -------------------------------------------------------------
                EKF_IDLE: begin
                    if (init_ekf) begin
                        state_dim_reg <= state_dim;
                        meas_dim_reg  <= meas_dim;
                        x_reg         <= x_init;
                        p_reg         <= p_init;
                        status_reg    <= STATUS_IDLE;
                        busy          <= 1'b0;
                    end else if (predict_valid) begin
                        busy       <= 1'b1;
                        start_pred <= 1'b1;
                        state      <= EKF_WAIT_PREDICT;
                    end else if (correct_valid) begin
                        busy       <= 1'b1;
                        start_corr <= 1'b1;
                        state      <= EKF_WAIT_CORRECT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: EKF_WAIT_PREDICT - Wait for Predict Engine
                // -------------------------------------------------------------
                EKF_WAIT_PREDICT: begin
                    if (pred_done) begin
                        x_reg        <= pred_x_out;
                        p_reg        <= pred_p_out;
                        predict_done <= 1'b1;
                        status_reg   <= STATUS_PREDICTED;
                        busy         <= 1'b0;
                        state        <= EKF_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // STATE 2: EKF_WAIT_CORRECT - Wait for Correct Engine
                // -------------------------------------------------------------
                EKF_WAIT_CORRECT: begin
                    if (corr_done) begin
                        x_reg         <= corr_x_out;
                        p_reg         <= corr_p_out;
                        innov_latched <= corr_y_out;
                        k_latched     <= corr_k_out;
                        correct_done  <= 1'b1;
                        status_reg    <= STATUS_UPDATED;
                        busy          <= 1'b0;
                        state         <= EKF_IDLE;
                    end
                end

                default: state <= EKF_IDLE;
            endcase
        end
    end

endmodule
