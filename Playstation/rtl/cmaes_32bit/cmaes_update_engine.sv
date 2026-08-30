// =============================================================================
// File Name   : cmaes_update_engine.sv
// Module Name : cmaes_update_engine
// Project     : Covariance Matrix Adaptation Evolution Strategy (CMA-ES) (Solver #34)
// -----------------------------------------------------------------------------
// Description:
//   Pipelined CMA-ES Distribution Update Engine:
//   1. Identifies champion elite candidates via combinational sort (μ = 2)
//   2. Updates distribution mean: m_{t+1} = m_t + c_m * σ * y_w (c_m = 1.50)
//   3. Accumulates evolution path p_σ and p_c
//   4. Diagonal Covariance Adaptation: C_{d,d} = (1 - c_1)*C_{d,d} + c_1 * p_{c,d}^2
//   5. Positive Diagonal Scaling: A_{d,d} = sqrt(C_{d,d}) >= 0.10
//   6. Step size annealing: σ_{t+1} = max(σ_t * 0.98, σ_min)
// =============================================================================

`timescale 1ns / 1ps

import cmaes_types_pkg::*;
`include "cmaes_helpers.svh"

module cmaes_update_engine (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,

    input  cand_arr_t          cand_y_vecs,  // Anisotropic steps y_k (16x1 packed)
    input  fitness_arr_t       cand_fitness, // Evaluated fitnesses f(x_k) (4x1)
    input  param_vec_t         mean_vec,     // Current mean m (4x1)
    input  q16_t               sigma,        // Current step size σ
    input  param_vec_t         p_sigma,      // Current step-size path p_σ
    input  param_vec_t         p_c,          // Current covariance path p_c
    input  cov_mat_t           C_mat,        // Current covariance matrix C (4x4)
    input  cov_mat_t           A_mat,        // Current coordinate matrix A (4x4)
    input  logic [2:0]         pop_size,     // Population size λ (2..4)
    input  logic [2:0]         dim,          // Dimension D (1..4)
    input  q16_t               c_sigma,      // Learning rate for p_σ
    input  q16_t               d_sigma,      // Damping for step-size σ
    input  q16_t               c_c,          // Learning rate for p_c
    input  q16_t               c_1,          // Learning rate for rank-1 C update
    input  q16_t               sigma_min,    // Minimum step size clamp
    input  param_vec_t         lb_vec,       // Parameter lower bounds
    input  param_vec_t         ub_vec,       // Parameter upper bounds

    output param_vec_t         new_mean,     // Updated mean m_{t+1} (4x1)
    output q16_t               new_sigma,    // Updated step size σ_{t+1}
    output param_vec_t         new_p_sigma,  // Updated step-size path p_σ
    output param_vec_t         new_p_c,      // Updated covariance path p_c
    output cov_mat_t           new_C_mat,    // Updated covariance matrix C (4x4)
    output cov_mat_t           new_A_mat,    // Updated coordinate matrix A (4x4)
    output logic [1:0]         best_cand_idx,// Champion candidate index
    output logic               done,
    output logic               busy
);

    typedef enum logic [2:0] {
        UPD_IDLE   = 3'd0,
        UPD_RANK   = 3'd1,
        UPD_MEAN_Y = 3'd2,
        UPD_PATHS  = 3'd3,
        UPD_COV_C  = 3'd4,
        UPD_SQRT_A = 3'd5,
        UPD_DONE   = 3'd6
    } upd_state_t;

    upd_state_t state;

    param_vec_t m_reg;
    q16_t       sig_reg;
    param_vec_t ps_reg;
    param_vec_t pc_reg;
    cov_mat_t   C_reg;
    cov_mat_t   A_reg;
    logic [1:0] best_idx_reg;

    assign new_mean      = m_reg;
    assign new_sigma     = sig_reg;
    assign new_p_sigma   = ps_reg;
    assign new_p_c       = pc_reg;
    assign new_C_mat     = C_reg;
    assign new_A_mat     = A_reg;
    assign best_cand_idx = best_idx_reg;

    // Hardware Square Root Unit Instance
    logic sqrt_start;
    q16_t sqrt_rad, sqrt_root;
    logic sqrt_done, sqrt_busy;

    q16_sqrt u_sqrt (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (sqrt_start),
        .rad_in  (sqrt_rad),
        .sqrt_out(sqrt_root),
        .done    (sqrt_done),
        .busy    (sqrt_busy)
    );

    logic [1:0] elite0_idx, elite1_idx;
    param_vec_t y_w_reg;
    logic [1:0] curr_diag;
    q16_t       w0_wt, w1_wt, c_m_wt;
    q16_t       cs_factor, cc_factor;
    q16_t       c_old, pc_prod, new_c_val, new_a_val, new_sig_val;
    logic [1:0] temp_e0, temp_e1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= UPD_IDLE;
            m_reg        <= '0;
            sig_reg      <= 32'h0000_8000;
            ps_reg       <= '0;
            pc_reg       <= '0;
            C_reg        <= '0;
            A_reg        <= '0;
            best_idx_reg <= 2'd0;
            elite0_idx   <= 2'd0;
            elite1_idx   <= 2'd1;
            y_w_reg      <= '0;
            curr_diag    <= 2'd0;
            sqrt_start   <= 1'b0;
            sqrt_rad     <= 32'sd0;
            done         <= 1'b0;
            busy         <= 1'b0;
        end else begin
            sqrt_start <= 1'b0;

            case (state)
                UPD_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy    <= 1'b1;
                        m_reg   <= mean_vec;
                        sig_reg <= sigma;
                        ps_reg  <= p_sigma;
                        pc_reg  <= p_c;
                        C_reg   <= C_mat;
                        A_reg   <= A_mat;
                        state   <= UPD_RANK;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // Step 1: Combinational sort of top 2 elites (μ = 2)
                UPD_RANK: begin
                    if (cand_fitness[0] <= cand_fitness[1]) begin
                        temp_e0 = 2'd0;
                        temp_e1 = 2'd1;
                    end else begin
                        temp_e0 = 2'd1;
                        temp_e1 = 2'd0;
                    end

                    for (int k = 2; k < MAX_POP; k++) begin
                        if (k < pop_size) begin
                            if (cand_fitness[k] < cand_fitness[temp_e0]) begin
                                temp_e1 = temp_e0;
                                temp_e0 = 2'(k);
                            end else if (cand_fitness[k] < cand_fitness[temp_e1]) begin
                                temp_e1 = 2'(k);
                            end
                        end
                    end

                    elite0_idx   <= temp_e0;
                    elite1_idx   <= temp_e1;
                    best_idx_reg <= temp_e0;
                    state        <= UPD_MEAN_Y;
                end

                // Step 2: Compute recombination y_w = 0.85 * y_{elite0} + 0.15 * y_{elite1}
                // Mean update with acceleration factor c_m = 1.50
                UPD_MEAN_Y: begin
                    w0_wt  = 32'h0000_D999; // 0.85
                    w1_wt  = 32'h0000_2666; // 0.15
                    c_m_wt = 32'h0001_8000; // 1.50

                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim) begin
                            y_w_reg[d] <= q16_mul(w0_wt, cand_y_vecs[{elite0_idx, 2'(d)}]) +
                                          q16_mul(w1_wt, cand_y_vecs[{elite1_idx, 2'(d)}]);

                            m_reg[d]   <= clamp_param(
                                m_reg[d] + q16_mul(c_m_wt, q16_mul(sig_reg,
                                    q16_mul(w0_wt, cand_y_vecs[{elite0_idx, 2'(d)}]) +
                                    q16_mul(w1_wt, cand_y_vecs[{elite1_idx, 2'(d)}])
                                )),
                                lb_vec[d], ub_vec[d]
                            );
                        end
                    end
                    state <= UPD_PATHS;
                end

                // Step 3: Update evolution paths p_σ and p_c
                UPD_PATHS: begin
                    cs_factor = 32'h0000_9999; // 0.60
                    cc_factor = 32'h0000_9999; // 0.60

                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim) begin
                            ps_reg[d] <= q16_mul(32'h0001_0000 - c_sigma, ps_reg[d]) +
                                         q16_mul(cs_factor, y_w_reg[d]);

                            pc_reg[d] <= q16_mul(32'h0001_0000 - c_c, pc_reg[d]) +
                                         q16_mul(cc_factor, y_w_reg[d]);
                        end
                    end

                    // Step size annealing: σ_{t+1} = max(σ_t * 0.98, σ_min)
                    new_sig_val = q16_mul(sig_reg, 32'h0000_FA64); // 0.978
                    if (new_sig_val < sigma_min) begin
                        sig_reg <= sigma_min;
                    end else begin
                        sig_reg <= new_sig_val;
                    end

                    state <= UPD_COV_C;
                end

                // Step 4: Diagonal Covariance Update: C_{d, d} = (1 - c_1)*C_{d, d} + c_1 * p_{c, d}^2
                UPD_COV_C: begin
                    for (int d = 0; d < MAX_DIM; d++) begin
                        if (d < dim) begin
                            c_old     = C_reg[{2'(d), 2'(d)}];
                            pc_prod   = q16_mul(pc_reg[d], pc_reg[d]);
                            new_c_val = q16_mul(32'h0001_0000 - c_1, c_old) + q16_mul(c_1, pc_prod);

                            if (new_c_val < 32'h0000_1000) new_c_val = 32'h0000_1000; // min variance clamp 0.0625

                            C_reg[{2'(d), 2'(d)}] <= new_c_val;
                        end
                    end

                    curr_diag  <= 2'd0;
                    c_old      = C_reg[4'd0];
                    pc_prod    = q16_mul(pc_reg[0], pc_reg[0]);
                    new_c_val  = q16_mul(32'h0001_0000 - c_1, c_old) + q16_mul(c_1, pc_prod);
                    sqrt_rad   <= (new_c_val > 32'h0000_1000) ? new_c_val : 32'h0000_1000;
                    sqrt_start <= 1'b1;
                    state      <= UPD_SQRT_A;
                end

                // Step 5: Update diagonal coordinate scaling: A_{d, d} = sqrt(C_{d, d})
                UPD_SQRT_A: begin
                    if (sqrt_done) begin
                        new_a_val = (sqrt_root > 32'h0000_2000) ? sqrt_root : 32'h0000_2000; // min scale 0.125
                        A_reg[{curr_diag, curr_diag}] <= new_a_val;

                        if (curr_diag + 1'b1 < dim) begin
                            curr_diag  <= curr_diag + 1'b1;
                            c_old      = C_reg[{curr_diag + 1'b1, curr_diag + 1'b1}];
                            pc_prod    = q16_mul(pc_reg[curr_diag + 1'b1], pc_reg[curr_diag + 1'b1]);
                            new_c_val  = q16_mul(32'h0001_0000 - c_1, c_old) + q16_mul(c_1, pc_prod);
                            sqrt_rad   <= (new_c_val > 32'h0000_1000) ? new_c_val : 32'h0000_1000;
                            sqrt_start <= 1'b1;
                        end else begin
                            state <= UPD_DONE;
                        end
                    end
                end

                UPD_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= UPD_IDLE;
                end

                default: state <= UPD_IDLE;
            endcase
        end
    end

endmodule
