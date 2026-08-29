// =============================================================================
// File Name   : admm_primal_x_engine.sv
// Module Name : admm_primal_x_engine
// Project     : Alternating Direction Method of Multipliers (ADMM) Accelerator (Solver #11)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Constructs the Primal Normal System and solves for x_{k+1}:
//     1. Builds H = A^T*A + rho*I (N x N symmetric matrix)
//     2. Computes base projection A^T*b
//     3. Evaluates RHS vector v_k = A^T*b + rho*(z_k - u_k)
//     4. Solves (A^T*A + rho*I) * x_{k+1} = v_k via Cholesky Linear Solver!
// =============================================================================

`timescale 1ns / 1ps

import admm_types_pkg::*;
`include "admm_helpers.svh"

module admm_primal_x_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Inputs
    input  logic               start_build,    // 1-cycle pulse to build H and A^T*b
    input  logic               start_solve,    // 1-cycle pulse to solve x_{k+1}
    input  logic [2:0]         num_params,     // Number of features N (1..4)
    input  logic [3:0]         num_obs,        // Number of samples M (1..8)
    input  dataset_mat_t       a_matrix,       // Dataset matrix A (M x N)
    input  obs_vec_t           b_obs,          // Target observation vector b (M x 1)
    input  q16_t               rho_val,        // Augmented Lagrangian penalty rho
    input  vec_t               z_cur,          // Current primal vector z_k
    input  vec_t               u_cur,          // Current dual multiplier u_k

    // Outputs
    output vec_t               x_next_out,     // Solved primal vector x_{k+1}
    output mat_t               h_mat_out,      // Built matrix H = A^T*A + rho*I
    output vec_t               at_b_out,       // Built vector A^T*b
    output logic               build_done,
    output logic               solve_done,
    output logic               busy
);

    typedef enum logic [2:0] {
        PRIMAL_IDLE       = 3'd0,
        PRIMAL_BUILD_H    = 3'd1,
        PRIMAL_BUILD_ATB  = 3'd2,
        PRIMAL_FORM_RHS   = 3'd3,
        PRIMAL_SOLVE_WAIT = 3'd4,
        PRIMAL_DONE       = 3'd5
    } primal_state_t;

    primal_state_t state;

    mat_t h_mat;
    vec_t at_b;
    vec_t rhs_v;

    // Cholesky Solver Interconnect
    logic chol_start;
    vec_t chol_p_out;
    logic chol_done, chol_err, chol_busy;

    cholesky_solver_engine u_chol (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (chol_start),
        .n_dim           (num_params),
        .h_mat           (h_mat),
        .g_vec           (rhs_v),
        .p_vec           (chol_p_out),
        .done            (chol_done),
        .error_not_posdef(chol_err),
        .busy            (chol_busy)
    );

    logic [2:0] j_idx, k_idx;
    logic [3:0] m_idx;
    logic signed [63:0] dot_acc_64;
    q16_t a_mj, a_mk, b_m;
    q16_t diff_z_u;

    assign h_mat_out = h_mat;
    assign at_b_out  = at_b;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= PRIMAL_IDLE;
            h_mat       <= '0;
            at_b        <= '0;
            rhs_v       <= '0;
            x_next_out  <= '0;
            j_idx       <= 2'd0;
            k_idx       <= 2'd0;
            m_idx       <= 4'd0;
            dot_acc_64  <= 64'sd0;
            chol_start  <= 1'b0;
            build_done  <= 1'b0;
            solve_done  <= 1'b0;
            busy        <= 1'b0;
        end else begin
            chol_start <= 1'b0;

            case (state)
                PRIMAL_IDLE: begin
                    build_done <= 1'b0;
                    solve_done <= 1'b0;
                    if (start_build) begin
                        busy       <= 1'b1;
                        j_idx      <= 2'd0;
                        k_idx      <= 2'd0;
                        m_idx      <= 4'd0;
                        dot_acc_64 <= 64'sd0;
                        h_mat      <= '0;
                        at_b       <= '0;
                        state      <= PRIMAL_BUILD_H;
                    end else if (start_solve) begin
                        busy  <= 1'b1;
                        state <= PRIMAL_FORM_RHS;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // 1. BUILD H = A^T*A + rho*I (N x N)
                // -------------------------------------------------------------
                PRIMAL_BUILD_H: begin
                    if (j_idx < num_params) begin
                        if (k_idx < num_params) begin
                            if (m_idx < num_obs) begin
                                a_mj = get_data_elem(a_matrix, 3'(m_idx[2:0]), j_idx);
                                a_mk = get_data_elem(a_matrix, 3'(m_idx[2:0]), k_idx);
                                dot_acc_64 <= dot_acc_64 + (64'(a_mj) * 64'(a_mk));
                                m_idx      <= m_idx + 1'b1;
                            end else begin
                                // Add rho on diagonal (j == k)
                                if (j_idx == k_idx) begin
                                    h_mat <= set_mat(h_mat, j_idx, k_idx, q16_t'(dot_acc_64 >>> 16) + rho_val);
                                end else begin
                                    h_mat <= set_mat(h_mat, j_idx, k_idx, q16_t'(dot_acc_64 >>> 16));
                                end
                                m_idx      <= 4'd0;
                                dot_acc_64 <= 64'sd0;
                                k_idx      <= k_idx + 1'b1;
                            end
                        end else begin
                            k_idx <= 2'd0;
                            j_idx <= j_idx + 1'b1;
                        end
                    end else begin
                        j_idx <= 2'd0;
                        m_idx <= 4'd0;
                        state <= PRIMAL_BUILD_ATB;
                    end
                end

                // -------------------------------------------------------------
                // 2. BUILD A^T*b (N x 1)
                // -------------------------------------------------------------
                PRIMAL_BUILD_ATB: begin
                    if (j_idx < num_params) begin
                        if (m_idx < num_obs) begin
                            a_mj = get_data_elem(a_matrix, 3'(m_idx[2:0]), j_idx);
                            b_m  = get_obs(b_obs, 3'(m_idx[2:0]));
                            dot_acc_64 <= dot_acc_64 + (64'(a_mj) * 64'(b_m));
                            m_idx      <= m_idx + 1'b1;
                        end else begin
                            at_b       <= set_vec(at_b, j_idx, q16_t'(dot_acc_64 >>> 16));
                            m_idx      <= 4'd0;
                            dot_acc_64 <= 64'sd0;
                            j_idx      <= j_idx + 1'b1;
                        end
                    end else begin
                        build_done <= 1'b1;
                        busy       <= 1'b0;
                        state      <= PRIMAL_IDLE;
                    end
                end

                // -------------------------------------------------------------
                // 3. FORM RHS VECTOR: v_k = A^T*b + rho*(z_k - u_k)
                // -------------------------------------------------------------
                PRIMAL_FORM_RHS: begin
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params) begin
                            diff_z_u = get_vec(z_cur, 2'(k)) - get_vec(u_cur, 2'(k));
                            rhs_v    = set_vec(rhs_v, 2'(k), get_vec(at_b, 2'(k)) + q16_t'((64'(rho_val) * 64'(diff_z_u)) >>> 16));
                        end
                    end
                    chol_start <= 1'b1;
                    state      <= PRIMAL_SOLVE_WAIT;
                end

                // -------------------------------------------------------------
                // 4. SOLVE H * x_{k+1} = v_k VIA CHOLESKY
                // -------------------------------------------------------------
                PRIMAL_SOLVE_WAIT: begin
                    if (chol_done) begin
                        x_next_out <= chol_p_out;
                        solve_done <= 1'b1;
                        busy       <= 1'b0;
                        state      <= PRIMAL_IDLE;
                    end
                end

                default: state <= PRIMAL_IDLE;
            endcase
        end
    end

endmodule
