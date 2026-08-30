// =============================================================================
// File Name   : pca_top.sv
// Module Name : pca_top
// Project     : Principal Component Analysis (PCA) / Streaming SVD Accelerator
//               (Solver #28)
// -----------------------------------------------------------------------------
// Description:
//   Top-level SoC Controller for Principal Component Analysis (PCA).
//   Supports complete streaming dimensionality reduction & reconstruction workflow:
//   1. Initialization & Covariance: pca_cov_engine evaluates mean x_mean and Σ_0
//   2. Multi-Component Extraction:  pca_power_iter_engine finds (v_k, λ_k) and
//                                   Hotelling deflation computes Σ_k+1 = Σ_k - λ v v^T
//   3. Online Encoding:             z = V_K^T * (x - x_mean)
//   4. Online Decoding:             x_hat = x_mean + V_K * z
// =============================================================================

`timescale 1ns / 1ps

import pca_types_pkg::*;
`include "pca_helpers.svh"

module pca_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,              // Primary System Clock
    input  logic               rst_n,            // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Initialization & Training Dataset
    // -------------------------------------------------------------------------
    input  logic               init_pca,         // 1-cycle initialization strobe
    input  logic [3:0]         num_samples,      // Total samples M (2..8)
    input  logic [2:0]         feat_dim,         // Feature dimension D (1..4)
    input  logic [2:0]         num_comp,         // Components to extract K (1..4)
    input  dataset_arr_t       dataset,          // Training dataset [32]
    input  logic [7:0]         max_iters,        // Power iterations per component
    input  q16_t               tol_eps,          // Power iteration tolerance

    // -------------------------------------------------------------------------
    // Interface 2: Eigen-Extraction Execution & Results
    // -------------------------------------------------------------------------
    input  logic               extract_valid,    // Trigger eigen-extraction
    output feature_vec_t       mean_vector,      // Sample mean x_mean (4x1)
    output eigen_mat_t         eigen_vectors,    // Orthonormal basis matrix V (4x4)
    output lambda_vec_t        eigen_values,     // Eigenvalue spectrum λ (4x1)
    output status_t            status,           // Status code
    output logic               extract_done,     // Extraction complete strobe

    // -------------------------------------------------------------------------
    // Interface 3: Online Encoding (Projection) & Decoding (Reconstruction)
    // -------------------------------------------------------------------------
    input  logic               project_valid,    // Trigger encoding projection
    input  feature_vec_t       x_input,          // D-dimensional feature input
    output latent_vec_t        z_projected,      // K-dimensional compressed latent
    output logic               project_done,     // Projection complete strobe

    input  logic               reconstruct_valid,// Trigger decoding reconstruction
    input  latent_vec_t        z_input,          // K-dimensional latent input
    output feature_vec_t       x_reconstructed,  // D-dimensional reconstructed output
    output logic               reconstruct_done, // Reconstruction complete strobe
    output logic               busy              // High while engine is active
);

    // -------------------------------------------------------------------------
    // Master FSM State Definitions
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        PCA_IDLE          = 4'd0,
        PCA_WAIT_COV      = 4'd1,
        PCA_EXTRACT_START = 4'd2,
        PCA_PI_START      = 4'd3,
        PCA_PI_WAIT       = 4'd4,
        PCA_DEFLATE       = 4'd5,
        PCA_EXTRACT_DONE  = 4'd6,
        PCA_PROJECT_CALC  = 4'd7,
        PCA_PROJECT_DONE  = 4'd8,
        PCA_RECON_CALC    = 4'd9,
        PCA_RECON_DONE    = 4'd10
    } pca_state_t;

    pca_state_t state;

    // Internal Configuration Registers
    logic [3:0]   m_samples_reg;
    logic [2:0]   d_feat_reg;
    logic [2:0]   k_comp_reg;
    dataset_arr_t ds_reg;
    logic [7:0]   max_i_reg;
    q16_t         tol_reg;
    status_t      status_reg;

    // Learned Model Registers
    feature_vec_t mean_reg;
    cov_mat_t     curr_cov_reg;
    eigen_mat_t   v_mat_reg;
    lambda_vec_t  lambda_reg;
    logic [1:0]   comp_k_idx;

    // Projection & Reconstruction Registers
    latent_vec_t  z_latched;
    feature_vec_t x_hat_latched;

    assign mean_vector      = mean_reg;
    assign eigen_vectors    = v_mat_reg;
    assign eigen_values     = lambda_reg;
    assign status           = status_reg;
    assign z_projected      = z_latched;
    assign x_reconstructed  = x_hat_latched;

    // Sub-engine 1: pca_cov_engine
    logic         start_cov;
    feature_vec_t cov_mean_out;
    cov_mat_t     cov_mat_out;
    logic         cov_done;
    logic         cov_busy;

    pca_cov_engine u_cov (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start_cov),
        .num_samples(m_samples_reg),
        .feat_dim   (d_feat_reg),
        .dataset    (ds_reg),
        .mean_vec   (cov_mean_out),
        .cov_matrix (cov_mat_out),
        .done       (cov_done),
        .busy       (cov_busy)
    );

    // Sub-engine 2: pca_power_iter_engine
    logic         start_pi;
    feature_vec_t pi_v_out;
    q16_t         pi_lambda_out;
    logic         pi_done;
    logic         pi_busy;

    pca_power_iter_engine u_pi (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (start_pi),
        .cov_mat   (curr_cov_reg),
        .v_mat_prev(v_mat_reg),
        .comp_idx  (comp_k_idx),
        .feat_dim  (d_feat_reg),
        .max_iters (max_i_reg),
        .tol_eps   (tol_reg),
        .eigen_vec (pi_v_out),
        .eigen_val (pi_lambda_out),
        .done      (pi_done),
        .busy      (pi_busy)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= PCA_IDLE;
            m_samples_reg    <= 4'd6;
            d_feat_reg       <= 3'd2;
            k_comp_reg       <= 3'd2;
            ds_reg           <= '0;
            max_i_reg        <= 8'd25;
            tol_reg          <= 32'h0000_0010;
            status_reg       <= STATUS_IDLE;
            mean_reg         <= '0;
            curr_cov_reg     <= '0;
            v_mat_reg        <= '0;
            lambda_reg       <= '0;
            comp_k_idx       <= 2'd0;
            z_latched        <= '0;
            x_hat_latched    <= '0;
            start_cov        <= 1'b0;
            start_pi         <= 1'b0;
            extract_done     <= 1'b0;
            project_done     <= 1'b0;
            reconstruct_done <= 1'b0;
            busy             <= 1'b0;
        end else begin
            start_cov        <= 1'b0;
            start_pi         <= 1'b0;
            extract_done     <= 1'b0;
            project_done     <= 1'b0;
            reconstruct_done <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE 0: PCA_IDLE
                // -------------------------------------------------------------
                PCA_IDLE: begin
                    if (init_pca) begin
                        m_samples_reg <= num_samples;
                        d_feat_reg    <= feat_dim;
                        k_comp_reg    <= num_comp;
                        ds_reg        <= dataset;
                        max_i_reg     <= (max_iters != 8'd0) ? max_iters : 8'd25;
                        tol_reg       <= (tol_eps != 32'sd0) ? tol_eps : 32'h0000_0010;
                        v_mat_reg     <= '0;
                        lambda_reg    <= '0;
                        status_reg    <= STATUS_IDLE;
                        busy          <= 1'b1;
                        start_cov     <= 1'b1;
                        state         <= PCA_WAIT_COV;
                    end else if (extract_valid) begin
                        busy       <= 1'b1;
                        comp_k_idx <= 2'd0;
                        status_reg <= STATUS_EXTRACTING;
                        state      <= PCA_EXTRACT_START;
                    end else if (project_valid) begin
                        busy  <= 1'b1;
                        state <= PCA_PROJECT_CALC;
                    end else if (reconstruct_valid) begin
                        busy  <= 1'b1;
                        state <= PCA_RECON_CALC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE 1: PCA_WAIT_COV - Wait for Covariance Matrix Accumulation
                // -------------------------------------------------------------
                PCA_WAIT_COV: begin
                    if (cov_done) begin
                        mean_reg     <= cov_mean_out;
                        curr_cov_reg <= cov_mat_out;
                        status_reg   <= STATUS_COV_READY;
                        busy         <= 1'b0;
                        state        <= PCA_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // STATE 2: PCA_EXTRACT_START - Start extraction for component k
                // -------------------------------------------------------------
                PCA_EXTRACT_START: begin
                    start_pi <= 1'b1;
                    state    <= PCA_PI_WAIT;
                end

                // -------------------------------------------------------------
                // STATE 3: PCA_PI_WAIT - Wait for dominant eigenvector (v_k, λ_k)
                // -------------------------------------------------------------
                PCA_PI_WAIT: begin
                    if (pi_done) begin
                        eigen_mat_t v_next;
                        v_next = v_mat_reg;
                        for (int r = 0; r < MAX_FEATURES; r++) begin
                            v_next = set_eigen_elem(v_next, 2'(r), comp_k_idx, pi_v_out[r]);
                        end
                        v_mat_reg <= v_next;
                        lambda_reg[comp_k_idx] <= pi_lambda_out;

                        // Deflate covariance: Σ_new = Σ - λ_k * (v_k * v_k^T)
                        curr_cov_reg <= deflate_cov(curr_cov_reg, pi_lambda_out, pi_v_out, d_feat_reg);
                        state        <= PCA_DEFLATE;
                    end
                end

                // -------------------------------------------------------------
                // STATE 4: PCA_DEFLATE - Advance to next component or finish
                // -------------------------------------------------------------
                PCA_DEFLATE: begin
                    if (comp_k_idx + 1'b1 < k_comp_reg) begin
                        comp_k_idx <= comp_k_idx + 1'b1;
                        state      <= PCA_EXTRACT_START;
                    end else begin
                        status_reg <= STATUS_CONVERGED;
                        state      <= PCA_EXTRACT_DONE;
                    end
                end

                PCA_EXTRACT_DONE: begin
                    extract_done <= 1'b1;
                    busy         <= 1'b0;
                    state        <= PCA_IDLE;
                end

                // -------------------------------------------------------------
                // STATE 5: PCA_PROJECT_CALC - Latent encoding z = V^T * (x - x_mean)
                // -------------------------------------------------------------
                PCA_PROJECT_CALC: begin
                    z_latched  <= project_latent(v_mat_reg, x_input, mean_reg, d_feat_reg, k_comp_reg);
                    status_reg <= STATUS_PROJECTED;
                    state      <= PCA_PROJECT_DONE;
                end

                PCA_PROJECT_DONE: begin
                    project_done <= 1'b1;
                    busy         <= 1'b0;
                    state        <= PCA_IDLE;
                end

                // -------------------------------------------------------------
                // STATE 6: PCA_RECON_CALC - Decoding x_hat = x_mean + V * z
                // -------------------------------------------------------------
                PCA_RECON_CALC: begin
                    x_hat_latched <= reconstruct_feature(v_mat_reg, z_input, mean_reg, d_feat_reg, k_comp_reg);
                    state         <= PCA_RECON_DONE;
                end

                PCA_RECON_DONE: begin
                    reconstruct_done <= 1'b1;
                    busy             <= 1'b0;
                    state            <= PCA_IDLE;
                end

                default: state <= PCA_IDLE;
            endcase
        end
    end

endmodule
