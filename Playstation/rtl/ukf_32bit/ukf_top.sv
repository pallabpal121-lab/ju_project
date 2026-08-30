// =============================================================================
// File Name   : ukf_top.sv
// Module Name : ukf_top
// Project     : Unscented Kalman Filter (UKF) Accelerator (Solver #25)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Controller for Real-Time Unscented Kalman Filtering (UKF).
//   Supports complete predict-correct Sigma-Point workflow:
//   1. Initialization: Configures x_0, P_0, weights W_m, W_c, and γ^2 = N + λ
//   2. Generate Sigmas: ukf_sigma_gen evaluates Cholesky L and forms χ_0..χ_2N
//   3. Time Update:    ukf_predict_engine computes x_prior and P_prior + Q
//   4. Correct Step:   ukf_correct_engine computes z_hat, P_zz, P_xz, K,
//                      x_post = x_prior + K(z - z_hat), P_post = P_prior - K P_zz K^T
// =============================================================================

`timescale 1ns / 1ps

import ukf_types_pkg::*;
`include "ukf_helpers.svh"

module ukf_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Configuration & Initialization
    // -------------------------------------------------------------------------
    input  logic               init_ukf,         // 1-cycle reset & initialization strobe
    input  logic [2:0]         state_dim,        // State dimension N (1..4)
    input  logic [1:0]         meas_dim,         // Measurement dimension M (1..2)
    input  state_vec_t         x_init,           // Initial state vector x_0
    input  state_mat_t         p_init,           // Initial covariance P_0
    input  q16_t               gamma_sq,         // Scaling factor γ^2 = N + λ_ukf
    input  weights_arr_t       weights_m,        // Mean weights W_i^(m) [9]
    input  weights_arr_t       weights_c,        // Covariance weights W_i^(c) [9]

    // -------------------------------------------------------------------------
    // Interface 2: Sigma Points Generation
    // -------------------------------------------------------------------------
    input  logic               gen_sigmas_valid, // Trigger sigma point generation
    output sigma_state_arr_t   sigma_points_out, // Generated 9 sigma points
    output logic               sigmas_ready,     // Sigma points ready strobe

    // -------------------------------------------------------------------------
    // Interface 3: Prediction Step Inputs
    // -------------------------------------------------------------------------
    input  logic               predict_valid,    // Trigger prediction step
    input  sigma_state_arr_t   sigma_propagated, // Propagated state sigma points f(χ_i, u)
    input  state_mat_t         q_mat,            // Process noise covariance Q
    output state_vec_t         x_prior_out,      // A priori mean
    output state_mat_t         p_prior_out,      // A priori covariance
    output logic               predict_done,     // Prediction complete strobe

    // -------------------------------------------------------------------------
    // Interface 4: Correction Step Inputs
    // -------------------------------------------------------------------------
    input  logic               correct_valid,    // Trigger correction step
    input  sigma_meas_arr_t    sigma_meas_in,    // Measurement sigma points h(χ_i)
    input  meas_vec_t          z_meas,           // Actual measurement z
    input  innov_mat_t         r_mat,            // Measurement noise covariance R
    output state_vec_t         x_estimated,      // Current state estimate
    output state_mat_t         p_covariance,     // Current error covariance
    output meas_vec_t          innov_residual,   // Innovation residual (z - z_hat)
    output cross_mat_t         k_gain_out,       // Kalman gain matrix K
    output status_t            status,           // Status code
    output logic               correct_done,     // Correction complete strobe
    output logic               busy              // High while engine is active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        UKF_IDLE         = 3'd0,
        UKF_WAIT_SIGMAS  = 3'd1,
        UKF_WAIT_PREDICT = 3'd2,
        UKF_WAIT_CORRECT = 3'd3
    } ukf_state_t;

    ukf_state_t state;

    // Internal State & Covariance Registers
    state_vec_t       x_reg;
    state_mat_t       p_reg;
    logic [2:0]       state_dim_reg;
    logic [1:0]       meas_dim_reg;
    q16_t             gamma_sq_reg;
    weights_arr_t     wm_reg;
    weights_arr_t     wc_reg;
    status_t          status_reg;

    sigma_state_arr_t sigmas_latched;
    state_vec_t       x_prior_latched;
    state_mat_t       p_prior_latched;
    sigma_state_arr_t sigmas_prior_latched;
    meas_vec_t        innov_latched;
    cross_mat_t       k_latched;

    // Sub-engine 1: ukf_sigma_gen
    logic             start_sig;
    sigma_state_arr_t sig_gen_pts;
    state_mat_t       sig_gen_l;
    logic             sig_gen_done;
    logic             sig_gen_busy;

    ukf_sigma_gen u_sig_gen (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (start_sig),
        .state_dim   (state_dim_reg),
        .x_mean      (x_reg),
        .p_cov       (p_reg),
        .gamma_sq    (gamma_sq_reg),
        .sigma_points(sig_gen_pts),
        .l_cholesky  (sig_gen_l),
        .done        (sig_gen_done),
        .busy        (sig_gen_busy)
    );

    // Sub-engine 2: ukf_predict_engine
    logic             start_pred;
    state_vec_t       pred_x_out;
    state_mat_t       pred_p_out;
    sigma_state_arr_t pred_sig_out;
    logic             pred_done;
    logic             pred_busy;

    ukf_predict_engine u_pred (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (start_pred),
        .state_dim       (state_dim_reg),
        .sigma_points_in (sigma_propagated),
        .weights_m       (wm_reg),
        .weights_c       (wc_reg),
        .q_mat           (q_mat),
        .x_prior         (pred_x_out),
        .p_prior         (pred_p_out),
        .sigma_points_out(pred_sig_out),
        .done            (pred_done),
        .busy            (pred_busy)
    );

    // Sub-engine 3: ukf_correct_engine
    logic        start_corr;
    state_vec_t  corr_x_out;
    state_mat_t  corr_p_out;
    meas_vec_t   corr_y_out;
    cross_mat_t  corr_k_out;
    logic        corr_done;
    logic        corr_busy;

    ukf_correct_engine u_corr (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (start_corr),
        .state_dim     (state_dim_reg),
        .meas_dim      (meas_dim_reg),
        .x_prior       (x_prior_latched),
        .p_prior       (p_prior_latched),
        .sigma_state_in(sigmas_prior_latched),
        .sigma_meas_in (sigma_meas_in),
        .z_meas        (z_meas),
        .weights_m     (wm_reg),
        .weights_c     (wc_reg),
        .r_mat         (r_mat),
        .x_post        (corr_x_out),
        .p_post        (corr_p_out),
        .innov_y       (corr_y_out),
        .k_gain        (corr_k_out),
        .done          (corr_done),
        .busy          (corr_busy)
    );

    // Outputs
    assign sigma_points_out = sigmas_latched;
    assign x_prior_out      = x_prior_latched;
    assign p_prior_out      = p_prior_latched;
    assign x_estimated      = x_reg;
    assign p_covariance     = p_reg;
    assign innov_residual   = innov_latched;
    assign k_gain_out       = k_latched;
    assign status           = status_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                <= UKF_IDLE;
            x_reg                <= '0;
            p_reg                <= '0;
            state_dim_reg        <= 3'd2;
            meas_dim_reg         <= 2'd1;
            gamma_sq_reg         <= 32'h0003_0000;
            wm_reg               <= '0;
            wc_reg               <= '0;
            status_reg           <= STATUS_IDLE;
            sigmas_latched       <= '0;
            x_prior_latched      <= '0;
            p_prior_latched      <= '0;
            sigmas_prior_latched <= '0;
            innov_latched        <= '0;
            k_latched            <= '0;
            start_sig            <= 1'b0;
            start_pred           <= 1'b0;
            start_corr           <= 1'b0;
            sigmas_ready         <= 1'b0;
            predict_done         <= 1'b0;
            correct_done         <= 1'b0;
            busy                 <= 1'b0;
        end else begin
            start_sig    <= 1'b0;
            start_pred   <= 1'b0;
            start_corr   <= 1'b0;
            sigmas_ready <= 1'b0;
            predict_done <= 1'b0;
            correct_done <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: UKF_IDLE - Handle Init, Gen Sigmas, Predict, Correct
                // -------------------------------------------------------------
                UKF_IDLE: begin
                    if (init_ukf) begin
                        state_dim_reg <= state_dim;
                        meas_dim_reg  <= meas_dim;
                        x_reg         <= x_init;
                        p_reg         <= p_init;
                        gamma_sq_reg  <= (gamma_sq != 32'sd0) ? gamma_sq : 32'h0003_0000;
                        wm_reg        <= weights_m;
                        wc_reg        <= weights_c;
                        status_reg    <= STATUS_IDLE;
                        busy          <= 1'b0;
                    end else if (gen_sigmas_valid) begin
                        busy      <= 1'b1;
                        start_sig <= 1'b1;
                        state     <= UKF_WAIT_SIGMAS;
                    end else if (predict_valid) begin
                        busy       <= 1'b1;
                        start_pred <= 1'b1;
                        state      <= UKF_WAIT_PREDICT;
                    end else if (correct_valid) begin
                        busy       <= 1'b1;
                        start_corr <= 1'b1;
                        state      <= UKF_WAIT_CORRECT;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: UKF_WAIT_SIGMAS - Wait for Sigma-Point Generator
                // -------------------------------------------------------------
                UKF_WAIT_SIGMAS: begin
                    if (sig_gen_done) begin
                        sigmas_latched <= sig_gen_pts;
                        sigmas_ready   <= 1'b1;
                        status_reg     <= STATUS_SIGMAS;
                        busy           <= 1'b0;
                        state          <= UKF_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // STATE 2: UKF_WAIT_PREDICT - Wait for Predict Engine
                // -------------------------------------------------------------
                UKF_WAIT_PREDICT: begin
                    if (pred_done) begin
                        x_prior_latched      <= pred_x_out;
                        p_prior_latched      <= pred_p_out;
                        sigmas_prior_latched <= pred_sig_out;
                        x_reg                <= pred_x_out;
                        p_reg                <= pred_p_out;
                        predict_done         <= 1'b1;
                        status_reg           <= STATUS_PREDICTED;
                        busy                 <= 1'b0;
                        state                <= UKF_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // STATE 3: UKF_WAIT_CORRECT - Wait for Correct Engine
                // -------------------------------------------------------------
                UKF_WAIT_CORRECT: begin
                    if (corr_done) begin
                        x_reg         <= corr_x_out;
                        p_reg         <= corr_p_out;
                        innov_latched <= corr_y_out;
                        k_latched     <= corr_k_out;
                        correct_done  <= 1'b1;
                        status_reg    <= STATUS_UPDATED;
                        busy          <= 1'b0;
                        state         <= UKF_IDLE;
                    end
                end

                default: state <= UKF_IDLE;
            endcase
        end
    end

endmodule
