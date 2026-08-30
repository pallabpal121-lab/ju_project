// =============================================================================
// File Name   : tb_pca.sv
// Module Name : tb_pca
// Project     : Principal Component Analysis (PCA) / Streaming SVD Accelerator
//               (Solver #28)
// -----------------------------------------------------------------------------
// Description:
//   Comprehensive SystemVerilog testbench for PCA Accelerator.
//   Verifies:
//   1. 2D Correlated Point Cloud Dimensionality Reduction & Variance Maximization
//   2. 3D Ellipsoid Principal Axes Extraction and Orthogonality
//   3. 4D Physical Sensor Data Compression & High-Fidelity Reconstruction
// =============================================================================

`timescale 1ns / 1ps

import pca_types_pkg::*;
`include "pca_helpers.svh"

module tb_pca;

    logic               clk;
    logic               rst_n;

    // Controls & Configurations
    logic               init_pca;
    logic [3:0]         num_samples;
    logic [2:0]         feat_dim;
    logic [2:0]         num_comp;
    dataset_arr_t       dataset;
    logic [7:0]         max_iters;
    q16_t               tol_eps;

    // Extraction Interface
    logic               extract_valid;
    feature_vec_t       mean_vector;
    eigen_mat_t         eigen_vectors;
    lambda_vec_t        eigen_values;
    status_t            status;
    logic               extract_done;

    // Online Projection & Reconstruction Interface
    logic               project_valid;
    feature_vec_t       x_input;
    latent_vec_t        z_projected;
    logic               project_done;

    logic               reconstruct_valid;
    latent_vec_t        z_input;
    feature_vec_t       x_reconstructed;
    logic               reconstruct_done;
    logic               busy;

    // Instantiate Top Module
    pca_top dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .init_pca        (init_pca),
        .num_samples     (num_samples),
        .feat_dim        (feat_dim),
        .num_comp        (num_comp),
        .dataset         (dataset),
        .max_iters       (max_iters),
        .tol_eps         (tol_eps),
        .extract_valid   (extract_valid),
        .mean_vector     (mean_vector),
        .eigen_vectors   (eigen_vectors),
        .eigen_values    (eigen_values),
        .status          (status),
        .extract_done    (extract_done),
        .project_valid   (project_valid),
        .x_input         (x_input),
        .z_projected     (z_projected),
        .project_done    (project_done),
        .reconstruct_valid(reconstruct_valid),
        .z_input         (z_input),
        .x_reconstructed (x_reconstructed),
        .reconstruct_done(reconstruct_done),
        .busy            (busy)
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

    // Task to run online projection
    task automatic do_project(
        input feature_vec_t x_in,
        output latent_vec_t z_out
    );
        @(posedge clk);
        x_input       = x_in;
        project_valid = 1'b1;
        @(posedge clk);
        project_valid = 1'b0;
        @(posedge project_done);
        z_out = z_projected;
        #1;
    endtask

    // Task to run online reconstruction
    task automatic do_reconstruct(
        input latent_vec_t z_in,
        output feature_vec_t x_out
    );
        @(posedge clk);
        z_input           = z_in;
        reconstruct_valid = 1'b1;
        @(posedge clk);
        reconstruct_valid = 1'b0;
        @(posedge reconstruct_done);
        x_out = x_reconstructed;
        #1;
    endtask

    // Test variables declared at module level
    real v1_0, v1_1, v2_0, v2_1, l1, l2;
    real dot_v1_v2, dot_v1_v3, dot_v2_v3;
    feature_vec_t test_sample, recon_sample;
    latent_vec_t latent_z;
    real err_sq, rmse;

    initial begin
        $display("==================================================================");
        $display(" Principal Component Analysis (PCA) Accelerator TB (Solver #28)");
        $display("==================================================================");

        // Reset
        rst_n             = 0;
        init_pca          = 0;
        extract_valid     = 0;
        project_valid     = 0;
        reconstruct_valid = 0;
        num_samples       = 4'd6;
        feat_dim          = 3'd2;
        num_comp          = 3'd2;
        dataset           = '0;
        max_iters         = 8'd25;
        tol_eps           = real_to_q16(0.0001);
        x_input           = '0;
        z_input           = '0;

        #30;
        rst_n = 1;
        #20;

        // ---------------------------------------------------------------------
        // TEST 1: 2D Correlated Point Cloud Dimensionality Reduction
        // Points along y = 2x: (1,2), (2,4), (3,6), (-1,-2), (-2,-4), (-3,-6)
        // Target PC1: [1/sqrt(5), 2/sqrt(5)] = [0.4472, 0.8944]
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] 2D Correlated Point Cloud Dimensionality Reduction");
        $display("Dataset: 6 points along y = 2x line | Target PC1: [0.4472, 0.8944]");

        num_samples = 4'd6;
        feat_dim    = 3'd2;
        num_comp    = 3'd2;
        dataset     = '0;

        dataset = set_sample_feat(dataset, 3'd0, 2'd0, real_to_q16(1.0));
        dataset = set_sample_feat(dataset, 3'd0, 2'd1, real_to_q16(2.0));

        dataset = set_sample_feat(dataset, 3'd1, 2'd0, real_to_q16(2.0));
        dataset = set_sample_feat(dataset, 3'd1, 2'd1, real_to_q16(4.0));

        dataset = set_sample_feat(dataset, 3'd2, 2'd0, real_to_q16(3.0));
        dataset = set_sample_feat(dataset, 3'd2, 2'd1, real_to_q16(6.0));

        dataset = set_sample_feat(dataset, 3'd3, 2'd0, real_to_q16(-1.0));
        dataset = set_sample_feat(dataset, 3'd3, 2'd1, real_to_q16(-2.0));

        dataset = set_sample_feat(dataset, 3'd4, 2'd0, real_to_q16(-2.0));
        dataset = set_sample_feat(dataset, 3'd4, 2'd1, real_to_q16(-4.0));

        dataset = set_sample_feat(dataset, 3'd5, 2'd0, real_to_q16(-3.0));
        dataset = set_sample_feat(dataset, 3'd5, 2'd1, real_to_q16(-6.0));

        max_iters = 8'd25;
        tol_eps   = real_to_q16(0.0001);

        // Initialization & Covariance Matrix Generation
        @(posedge clk);
        init_pca = 1'b1;
        @(posedge clk);
        init_pca = 1'b0;
        @(negedge busy);
        #10;

        $display("Sample Mean: x_mean = [%f, %f]",
                 q16_to_real(mean_vector[0]), q16_to_real(mean_vector[1]));

        // Trigger Power Iteration & Deflation
        $display("Starting multi-component eigen-extraction...");
        @(posedge clk);
        extract_valid = 1'b1;
        @(posedge clk);
        extract_valid = 1'b0;
        @(posedge extract_done);
        #10;

        v1_0 = q16_to_real(get_eigen_elem(eigen_vectors, 2'd0, 2'd0));
        v1_1 = q16_to_real(get_eigen_elem(eigen_vectors, 2'd1, 2'd0));
        v2_0 = q16_to_real(get_eigen_elem(eigen_vectors, 2'd0, 2'd1));
        v2_1 = q16_to_real(get_eigen_elem(eigen_vectors, 2'd1, 2'd1));
        l1   = q16_to_real(eigen_values[0]);
        l2   = q16_to_real(eigen_values[1]);

        $display("--> PCA EIGEN-EXTRACTION RESULTS:");
        $display("    PC 1: v_1 = [%f, %f] | Eigenvalue λ_1 = %f", v1_0, v1_1, l1);
        $display("    PC 2: v_2 = [%f, %f] | Eigenvalue λ_2 = %f", v2_0, v2_1, l2);

        // Online Projection & Reconstruction of Sample (2.0, 4.0)
        test_sample = '0;
        test_sample[0] = real_to_q16(2.0);
        test_sample[1] = real_to_q16(4.0);
        do_project(test_sample, latent_z);
        do_reconstruct(latent_z, recon_sample);

        $display("    Test Sample:  x = [2.000000, 4.000000]");
        $display("    Latent Code:  z = [%f, %f]", q16_to_real(latent_z[0]), q16_to_real(latent_z[1]));
        $display("    Reconstructed: x_hat = [%f, %f]", q16_to_real(recon_sample[0]), q16_to_real(recon_sample[1]));

        if ((v1_0 > 0.40 && v1_0 < 0.50 && v1_1 > 0.80 && v1_1 < 0.95) ||
            (v1_0 < -0.40 && v1_0 > -0.50 && v1_1 < -0.80 && v1_1 > -0.95)) begin
            $display("[TEST 1 PASSED] Successfully extracted 2D principal component along y=2x direction!");
        end else begin
            $display("[TEST 1 FAILED] PC1 vector outside tolerance.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 2: 3D Ellipsoid Principal Axes Extraction and Orthogonality
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] 3D Ellipsoid Principal Axes Extraction and Orthogonality");

        num_samples = 4'd6;
        feat_dim    = 3'd3;
        num_comp    = 3'd3;
        dataset     = '0;

        dataset = set_sample_feat(dataset, 3'd0, 2'd0, real_to_q16(2.0));
        dataset = set_sample_feat(dataset, 3'd0, 2'd1, real_to_q16(0.0));
        dataset = set_sample_feat(dataset, 3'd0, 2'd2, real_to_q16(0.0));

        dataset = set_sample_feat(dataset, 3'd1, 2'd0, real_to_q16(-2.0));
        dataset = set_sample_feat(dataset, 3'd1, 2'd1, real_to_q16(0.0));
        dataset = set_sample_feat(dataset, 3'd1, 2'd2, real_to_q16(0.0));

        dataset = set_sample_feat(dataset, 3'd2, 2'd0, real_to_q16(0.0));
        dataset = set_sample_feat(dataset, 3'd2, 2'd1, real_to_q16(1.0));
        dataset = set_sample_feat(dataset, 3'd2, 2'd2, real_to_q16(0.0));

        dataset = set_sample_feat(dataset, 3'd3, 2'd0, real_to_q16(0.0));
        dataset = set_sample_feat(dataset, 3'd3, 2'd1, real_to_q16(-1.0));
        dataset = set_sample_feat(dataset, 3'd3, 2'd2, real_to_q16(0.0));

        dataset = set_sample_feat(dataset, 3'd4, 2'd0, real_to_q16(0.0));
        dataset = set_sample_feat(dataset, 3'd4, 2'd1, real_to_q16(0.0));
        dataset = set_sample_feat(dataset, 3'd4, 2'd2, real_to_q16(0.5));

        dataset = set_sample_feat(dataset, 3'd5, 2'd0, real_to_q16(0.0));
        dataset = set_sample_feat(dataset, 3'd5, 2'd1, real_to_q16(0.0));
        dataset = set_sample_feat(dataset, 3'd5, 2'd2, real_to_q16(-0.5));

        @(posedge clk);
        init_pca = 1'b1;
        @(posedge clk);
        init_pca = 1'b0;
        @(negedge busy);
        #10;

        @(posedge clk);
        extract_valid = 1'b1;
        @(posedge clk);
        extract_valid = 1'b0;
        @(posedge extract_done);
        #10;

        $display("--> 3D EIGENVECTORS & EIGENVALUES:");
        for (int k = 0; k < 3; k++) begin
            $display("    PC %0d: v_%0d = [%f, %f, %f] | λ_%0d = %f",
                     k+1, k+1,
                     q16_to_real(get_eigen_elem(eigen_vectors, 2'd0, 2'(k))),
                     q16_to_real(get_eigen_elem(eigen_vectors, 2'd1, 2'(k))),
                     q16_to_real(get_eigen_elem(eigen_vectors, 2'd2, 2'(k))),
                     k+1, q16_to_real(eigen_values[k]));
        end

        // Check Orthogonality: v_1^T * v_2, v_1^T * v_3, v_2^T * v_3
        dot_v1_v2 = q16_to_real(get_eigen_elem(eigen_vectors, 2'd0, 2'd0)) * q16_to_real(get_eigen_elem(eigen_vectors, 2'd0, 2'd1)) +
                    q16_to_real(get_eigen_elem(eigen_vectors, 2'd1, 2'd0)) * q16_to_real(get_eigen_elem(eigen_vectors, 2'd1, 2'd1)) +
                    q16_to_real(get_eigen_elem(eigen_vectors, 2'd2, 2'd0)) * q16_to_real(get_eigen_elem(eigen_vectors, 2'd2, 2'd1));

        dot_v1_v3 = q16_to_real(get_eigen_elem(eigen_vectors, 2'd0, 2'd0)) * q16_to_real(get_eigen_elem(eigen_vectors, 2'd0, 2'd2)) +
                    q16_to_real(get_eigen_elem(eigen_vectors, 2'd1, 2'd0)) * q16_to_real(get_eigen_elem(eigen_vectors, 2'd1, 2'd2)) +
                    q16_to_real(get_eigen_elem(eigen_vectors, 2'd2, 2'd0)) * q16_to_real(get_eigen_elem(eigen_vectors, 2'd2, 2'd2));

        $display("    Orthogonality: v1·v2 = %f | v1·v3 = %f", dot_v1_v2, dot_v1_v3);

        if (dot_v1_v2 > -0.05 && dot_v1_v2 < 0.05 && dot_v1_v3 > -0.05 && dot_v1_v3 < 0.05) begin
            $display("[TEST 2 PASSED] Successfully extracted 3D orthogonal principal axes!");
        end else begin
            $display("[TEST 2 FAILED] 3D Eigenvectors not orthogonal.");
        end

        #50;

        // ---------------------------------------------------------------------
        // TEST 3: 4D Physical Sensor Data Compression & High-Fidelity Reconstruction
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] 4D Physical Sensor Data Compression & High-Fidelity Reconstruction");

        num_samples = 4'd6;
        feat_dim    = 3'd4;
        num_comp    = 3'd2; // Compress 4D -> 2D
        dataset     = '0;

        dataset = set_sample_feat(dataset, 3'd0, 2'd0, real_to_q16(1.0));
        dataset = set_sample_feat(dataset, 3'd0, 2'd1, real_to_q16(2.0));
        dataset = set_sample_feat(dataset, 3'd0, 2'd2, real_to_q16(3.0));
        dataset = set_sample_feat(dataset, 3'd0, 2'd3, real_to_q16(4.0));

        dataset = set_sample_feat(dataset, 3'd1, 2'd0, real_to_q16(2.0));
        dataset = set_sample_feat(dataset, 3'd1, 2'd1, real_to_q16(4.0));
        dataset = set_sample_feat(dataset, 3'd1, 2'd2, real_to_q16(6.0));
        dataset = set_sample_feat(dataset, 3'd1, 2'd3, real_to_q16(8.0));

        dataset = set_sample_feat(dataset, 3'd2, 2'd0, real_to_q16(3.0));
        dataset = set_sample_feat(dataset, 3'd2, 2'd1, real_to_q16(6.0));
        dataset = set_sample_feat(dataset, 3'd2, 2'd2, real_to_q16(9.0));
        dataset = set_sample_feat(dataset, 3'd2, 2'd3, real_to_q16(12.0));

        dataset = set_sample_feat(dataset, 3'd3, 2'd0, real_to_q16(-1.0));
        dataset = set_sample_feat(dataset, 3'd3, 2'd1, real_to_q16(-2.0));
        dataset = set_sample_feat(dataset, 3'd3, 2'd2, real_to_q16(-3.0));
        dataset = set_sample_feat(dataset, 3'd3, 2'd3, real_to_q16(-4.0));

        dataset = set_sample_feat(dataset, 3'd4, 2'd0, real_to_q16(-2.0));
        dataset = set_sample_feat(dataset, 3'd4, 2'd1, real_to_q16(-4.0));
        dataset = set_sample_feat(dataset, 3'd4, 2'd2, real_to_q16(-6.0));
        dataset = set_sample_feat(dataset, 3'd4, 2'd3, real_to_q16(-8.0));

        dataset = set_sample_feat(dataset, 3'd5, 2'd0, real_to_q16(-3.0));
        dataset = set_sample_feat(dataset, 3'd5, 2'd1, real_to_q16(-6.0));
        dataset = set_sample_feat(dataset, 3'd5, 2'd2, real_to_q16(-9.0));
        dataset = set_sample_feat(dataset, 3'd5, 2'd3, real_to_q16(-12.0));

        @(posedge clk);
        init_pca = 1'b1;
        @(posedge clk);
        init_pca = 1'b0;
        @(negedge busy);
        #10;

        @(posedge clk);
        extract_valid = 1'b1;
        @(posedge clk);
        extract_valid = 1'b0;
        @(posedge extract_done);
        #10;

        test_sample = '0;
        test_sample[0] = real_to_q16(2.0);
        test_sample[1] = real_to_q16(4.0);
        test_sample[2] = real_to_q16(6.0);
        test_sample[3] = real_to_q16(8.0);

        do_project(test_sample, latent_z);
        do_reconstruct(latent_z, recon_sample);

        $display("    Original 4D Input:      [%f, %f, %f, %f]",
                 q16_to_real(test_sample[0]), q16_to_real(test_sample[1]),
                 q16_to_real(test_sample[2]), q16_to_real(test_sample[3]));
        $display("    Compressed 2D Latent:   [%f, %f]",
                 q16_to_real(latent_z[0]), q16_to_real(latent_z[1]));
        $display("    Reconstructed 4D Output:[%f, %f, %f, %f]",
                 q16_to_real(recon_sample[0]), q16_to_real(recon_sample[1]),
                 q16_to_real(recon_sample[2]), q16_to_real(recon_sample[3]));

        err_sq = 0.0;
        for (int d = 0; d < 4; d++) begin
            err_sq += (q16_to_real(recon_sample[d]) - q16_to_real(test_sample[d])) *
                      (q16_to_real(recon_sample[d]) - q16_to_real(test_sample[d]));
        end
        rmse = $sqrt(err_sq / 4.0);
        $display("    Reconstruction RMSE:    %f", rmse);

        if (rmse < 0.05) begin
            $display("[TEST 3 PASSED] Successfully compressed and reconstructed 4D sensor stream via 2D latent PCA!");
        end else begin
            $display("[TEST 3 FAILED] Reconstruction error outside tolerance.");
        end

        $display("\n==================================================================");
        $display(" ALL PCA / STREAMING SVD HARDWARE ACCELERATOR TESTS COMPLETED!");
        $display("==================================================================");
        #100;
        $finish;
    end

endmodule
