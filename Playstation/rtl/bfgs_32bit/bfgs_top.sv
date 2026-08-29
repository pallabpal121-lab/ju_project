// =============================================================================
// File Name   : bfgs_top.sv
// Module Name : bfgs_top
// Project     : Quasi-Newton BFGS Optimization Accelerator (Solver #4)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Top-level SoC Module for the Quasi-Newton BFGS Optimization Accelerator.
//   Directly computes and updates the Inverse Hessian Matrix B in silicon without
//   matrix inversions or second-derivative computations.
// =============================================================================

`timescale 1ns / 1ps

import bfgs_types_pkg::*;
`include "bfgs_helpers.svh"

module bfgs_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Model Microcode Programming Port
    // -------------------------------------------------------------------------
    input  logic               prog_en,       // Microcode Write Enable
    input  logic [4:0]         prog_addr,     // Microcode Address (0..31)
    input  instr_t             prog_data,     // 32-bit Microcode Instruction

    // -------------------------------------------------------------------------
    // Interface 2: Optimization Controls & Parameters
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start pulse
    input  logic [2:0]         num_vars,      // Number of variables N (1..4)
    input  vec_t               x_init,        // Initial parameter guess [x3, x2, x1, x0]
    input  q16_t               step_alpha,    // Step size / learning rate α
    input  q16_t               tolerance,     // Convergence threshold on gradient infinity norm
    input  logic [7:0]         max_iters,     // Maximum iteration limit

    // -------------------------------------------------------------------------
    // Interface 3: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               x_optimal,     // Converged parameter vector x*
    output q16_t               f_optimal,     // Final function value f(x*)
    output q16_t               g_norm_inf,    // Final max gradient component max_j(|g_j|)
    output logic [7:0]         iter_count,    // Total iterations executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while solving
);

    // -------------------------------------------------------------------------
    // Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        BFGS_IDLE             = 3'd0,
        BFGS_START_GRAD       = 3'd1,
        BFGS_WAIT_GRAD        = 3'd2,
        BFGS_CHECK_CONV       = 3'd3,
        BFGS_START_TRIAL_GRAD = 3'd4,
        BFGS_WAIT_TRIAL_GRAD  = 3'd5,
        BFGS_WAIT_B_UPDATE    = 3'd6,
        BFGS_DONE             = 3'd7
    } bfgs_state_t;

    bfgs_state_t state;

    // Configuration & State Registers
    vec_t       x_curr;
    vec_t       x_next;
    vec_t       g_curr;
    vec_t       g_next;
    vec_t       s_disp;
    vec_t       y_grad_diff;
    mat_t       B_matrix;
    q16_t       f_curr_val;
    q16_t       f_next_val;
    logic [2:0] num_vars_reg;
    q16_t       alpha_reg;
    q16_t       tol_reg;
    logic [7:0] max_iters_reg;

    // Gradient Engine Interconnect
    logic grad_start;
    vec_t grad_x_in;
    q16_t grad_f_val;
    vec_t grad_vec_g;
    logic grad_done, grad_busy;

    // Matrix Update Engine Interconnect
    logic update_start;
    mat_t update_B_out;
    logic update_done, update_busy;

    // Helper fixed-point multiplier
    function automatic q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Submodule 1: Gradient Vector Sweeper Engine
    bfgs_gradient_engine u_grad_engine (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (grad_start),
        .num_vars   (num_vars_reg),
        .x_curr     (grad_x_in),
        .f_val      (grad_f_val),
        .vec_g      (grad_vec_g),
        .done       (grad_done),
        .busy       (grad_busy)
    );

    // Submodule 2: Rank-2 Inverse Hessian Matrix Update Engine
    bfgs_matrix_update_engine u_mat_engine (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (update_start),
        .num_vars   (num_vars_reg),
        .B_in       (B_matrix),
        .vec_s      (s_disp),
        .vec_y      (y_grad_diff),
        .B_out      (update_B_out),
        .done       (update_done),
        .busy       (update_busy)
    );

    vec_t tmp_p;
    vec_t tmp_s;
    vec_t tmp_x_next;
    vec_t tmp_y;
    q16_t sum_p_j;
    q16_t max_grad_calc;
    q16_t cur_g_val;
    function automatic mat_t get_identity_mat();
        mat_t m;
        m = '0;
        m = set_mat(m, 2'd0, 2'd0, Q16_ONE);
        m = set_mat(m, 2'd1, 2'd1, Q16_ONE);
        m = set_mat(m, 2'd2, 2'd2, Q16_ONE);
        m = set_mat(m, 2'd3, 2'd3, Q16_ONE);
        return m;
    endfunction

    // Master BFGS Loop FSM
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= BFGS_IDLE;
            num_vars_reg <= 3'd2;
            alpha_reg    <= Q16_ONE;
            tol_reg      <= Q16_EPS_DEF;
            max_iters_reg<= 8'd50;
            iter_count   <= 8'd0;
            status       <= STATUS_IDLE;
            done         <= 1'b0;
            busy         <= 1'b0;
            f_optimal    <= Q16_ZERO;
            g_norm_inf   <= Q16_ZERO;
            x_optimal    <= '0;
            x_curr       <= '0;
            x_next       <= '0;
            g_curr       <= '0;
            g_next       <= '0;
            s_disp       <= '0;
            y_grad_diff  <= '0;
            B_matrix     <= '0;
            f_curr_val   <= Q16_ZERO;
            f_next_val   <= Q16_ZERO;
            grad_start   <= 1'b0;
            grad_x_in    <= '0;
            update_start <= 1'b0;
        end else begin
            grad_start   <= 1'b0;
            update_start <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: BFGS_IDLE
                // -------------------------------------------------------------
                BFGS_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy          <= 1'b1;
                        num_vars_reg  <= (num_vars != 3'd0)        ? num_vars    : 3'd2;
                        alpha_reg     <= (step_alpha != Q16_ZERO)  ? step_alpha  : Q16_ONE;
                        tol_reg       <= (tolerance != Q16_ZERO)   ? tolerance   : Q16_EPS_DEF;
                        max_iters_reg <= (max_iters != 8'd0)       ? max_iters   : 8'd50;
                        iter_count    <= 8'd0;
                        status        <= STATUS_RUNNING;
                        x_curr        <= x_init;
                        B_matrix      <= get_identity_mat(); // Initialize B_0 = I
                        state         <= BFGS_START_GRAD;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE GRADIENT AT CURRENT POSITION x_curr
                // -------------------------------------------------------------
                BFGS_START_GRAD: begin
                    grad_x_in  <= x_curr;
                    grad_start <= 1'b1;
                    state      <= BFGS_WAIT_GRAD;
                end

                BFGS_WAIT_GRAD: begin
                    if (grad_done) begin
                        f_curr_val <= grad_f_val;
                        g_curr     <= grad_vec_g;
                        state      <= BFGS_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // CHECK CONVERGENCE (||g||_inf <= tolerance or max iterations)
                // -------------------------------------------------------------
                BFGS_CHECK_CONV: begin
                    max_grad_calc = 32'sd0;
                    for (int j = 0; j < MAX_VARS; j++) begin
                        if (j < num_vars_reg) begin
                            cur_g_val = get_vec(g_curr, 2'(j));
                            if (cur_g_val < 32'sd0) cur_g_val = -cur_g_val;
                            if (cur_g_val > max_grad_calc) max_grad_calc = cur_g_val;
                        end
                    end

                    // Condition 1: Converged
                    if (max_grad_calc <= tol_reg) begin
                        status     <= STATUS_CONVERGED;
                        x_optimal  <= x_curr;
                        f_optimal  <= f_curr_val;
                        g_norm_inf <= max_grad_calc;
                        state      <= BFGS_DONE;

                    // Condition 2: Max iterations reached
                    end else if (iter_count >= max_iters_reg) begin
                        status     <= STATUS_MAX_ITERS;
                        x_optimal  <= x_curr;
                        f_optimal  <= f_curr_val;
                        g_norm_inf <= max_grad_calc;
                        state      <= BFGS_DONE;

                    // Condition 3: Compute Search Direction p = -B · g and Step s = α · p
                    end else begin
                        tmp_p      = '0;
                        tmp_s      = '0;
                        tmp_x_next = x_curr;
                        for (int j = 0; j < MAX_VARS; j++) begin
                            if (j < num_vars_reg) begin
                                sum_p_j = 32'sd0;
                                for (int k = 0; k < MAX_VARS; k++) begin
                                    if (k < num_vars_reg) begin
                                        sum_p_j = sum_p_j + q16_mul(get_mat(B_matrix, 2'(j), 2'(k)), get_vec(g_curr, 2'(k)));
                                    end
                                end
                                tmp_p      = set_vec(tmp_p, 2'(j), -sum_p_j); // p_j = -(B·g)_j
                                tmp_s      = set_vec(tmp_s, 2'(j), q16_mul(alpha_reg, -sum_p_j)); // s_j = α · p_j
                                tmp_x_next = set_vec(tmp_x_next, 2'(j), get_vec(x_curr, 2'(j)) + q16_mul(alpha_reg, -sum_p_j));
                            end
                        end
                        s_disp     <= tmp_s;
                        x_next     <= tmp_x_next;
                        state      <= BFGS_START_TRIAL_GRAD;
                    end
                end

                // -------------------------------------------------------------
                // EVALUATE GRADIENT AT NEW POSITION x_next
                // -------------------------------------------------------------
                BFGS_START_TRIAL_GRAD: begin
                    grad_x_in  <= x_next;
                    grad_start <= 1'b1;
                    state      <= BFGS_WAIT_TRIAL_GRAD;
                end

                BFGS_WAIT_TRIAL_GRAD: begin
                    if (grad_done) begin
                        f_next_val <= grad_f_val;
                        g_next     <= grad_vec_g;

                        // Gradient change: y = g_(k+1) - g_k
                        tmp_y = '0;
                        for (int j = 0; j < MAX_VARS; j++) begin
                            if (j < num_vars_reg) begin
                                tmp_y = set_vec(tmp_y, 2'(j), get_vec(grad_vec_g, 2'(j)) - get_vec(g_curr, 2'(j)));
                            end
                        end
                        y_grad_diff  <= tmp_y;
                        update_start <= 1'b1; // Trigger Rank-2 Inverse Hessian update
                        state        <= BFGS_WAIT_B_UPDATE;
                    end
                end

                // -------------------------------------------------------------
                // WAIT FOR RANK-2 UPDATE ON B & ADVANCE TO NEXT ITERATION
                // -------------------------------------------------------------
                BFGS_WAIT_B_UPDATE: begin
                    if (update_done) begin
                        B_matrix   <= update_B_out;
                        x_curr     <= x_next;
                        g_curr     <= g_next;
                        f_curr_val <= f_next_val;
                        iter_count <= iter_count + 1'b1;
                        state      <= BFGS_CHECK_CONV;
                    end
                end

                // -------------------------------------------------------------
                // STATE: BFGS_DONE
                // -------------------------------------------------------------
                BFGS_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= BFGS_IDLE;
                end

                default: state <= BFGS_IDLE;
            endcase
        end
    end

endmodule
