// =============================================================================
// File Name   : lasso_top.sv
// Module Name : lasso_top
// Project     : Coordinate Descent / LASSO L1 Sparsity Accelerator (Solver #10)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Top-level SoC Controller for the LASSO Coordinate Descent Accelerator.
//   Iteratively sweeps all feature coordinates j in cyclical order, solving
//   the exact 1D soft-thresholded subproblem for each coordinate until the
//   entire weight vector w converges to optimal sparse solutions.
// =============================================================================

`timescale 1ns / 1ps

import lasso_types_pkg::*;
`include "lasso_helpers.svh"

module lasso_top (
    // -------------------------------------------------------------------------
    // Clock & Asynchronous Reset
    // -------------------------------------------------------------------------
    input  logic               clk,           // Primary System Clock
    input  logic               rst_n,         // Active-Low Global Reset

    // -------------------------------------------------------------------------
    // Interface 1: Dataset & Optimization Controls
    // -------------------------------------------------------------------------
    input  logic               start,         // 1-cycle start pulse
    input  logic [2:0]         num_params,    // Number of features N (1..4)
    input  logic [3:0]         num_obs,       // Number of samples M (1..8)
    input  dataset_mat_t       x_matrix,      // Dataset matrix X (M x N)
    input  obs_vec_t           y_obs,         // Target observation vector y (M x 1)
    input  vec_t               w_init,        // Starting weight vector w_0
    input  q16_t               lambda_reg,    // L1 regularization penalty lambda
    input  q16_t               tolerance,     // Convergence threshold on max_j |delta w_j|
    input  logic [7:0]         max_iters,     // Maximum full coordinate cycles

    // -------------------------------------------------------------------------
    // Interface 2: Results & Status Outputs
    // -------------------------------------------------------------------------
    output vec_t               w_optimal,     // Converged sparse weight vector w*
    output q16_t               cost_optimal,  // Final Residual Sum of Squares 0.5*||y - Xw||^2
    output q16_t               max_delta_w,   // Final cycle max coordinate shift
    output logic [2:0]         sparsity_count,// Number of exact zero weights (sparsity)
    output logic [7:0]         iter_count,    // Total coordinate cycles executed
    output status_t            status,        // Convergence status code
    output logic               done,          // 1-cycle completion strobe
    output logic               busy           // High while solving
);

    // -------------------------------------------------------------------------
    // Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        TOP_IDLE         = 4'd0,
        TOP_INIT_RES_1   = 4'd1,
        TOP_INIT_RES_2   = 4'd2,
        TOP_CYCLE_START  = 4'd3,
        TOP_COORD_START  = 4'd4,
        TOP_COORD_WAIT   = 4'd5,
        TOP_CHECK_CONV   = 4'd6,
        TOP_CALC_COST    = 4'd7,
        TOP_DONE         = 4'd8
    } top_state_t;

    top_state_t state;

    // Registers
    logic [2:0] num_params_reg;
    logic [3:0] num_obs_reg;
    q16_t       lambda_reg_val;
    q16_t       tol_reg;
    logic [7:0] max_iters_reg;

    vec_t       w_cur;
    obs_vec_t   r_cur;
    logic [1:0] cur_coord;
    q16_t       cycle_max_dw;

    // Coordinate Engine Interconnect
    logic     coord_start;
    q16_t     engine_w_out;
    obs_vec_t engine_r_out;
    q16_t     engine_dw_abs;
    logic     coord_done, coord_busy;

    lasso_coordinate_engine u_coord_engine (
        .clk        (clk),
        .rst_n      (rst_n),
        .start_coord(coord_start),
        .coord_idx  (cur_coord),
        .num_obs    (num_obs_reg),
        .x_matrix   (x_matrix),
        .r_in       (r_cur),
        .w_j_in     (get_vec(w_cur, cur_coord)),
        .lambda_reg (lambda_reg_val),
        .w_j_out    (engine_w_out),
        .r_out      (engine_r_out),
        .delta_w_abs(engine_dw_abs),
        .coord_done (coord_done),
        .busy       (coord_busy)
    );

    logic [3:0] init_m;
    q16_t       pred_val;
    logic signed [63:0] rss_accum_64;
    logic [2:0] zero_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= TOP_IDLE;
            num_params_reg <= 3'd2;
            num_obs_reg    <= 4'd4;
            lambda_reg_val <= Q16_ZERO;
            tol_reg        <= Q16_EPS_DEF;
            max_iters_reg  <= 8'd50;
            iter_count     <= 8'd0;
            status         <= STATUS_IDLE;
            done           <= 1'b0;
            busy           <= 1'b0;
            w_optimal      <= '0;
            cost_optimal   <= Q16_ZERO;
            max_delta_w    <= Q16_ZERO;
            sparsity_count <= 3'd0;
            w_cur          <= '0;
            r_cur          <= '0;
            cur_coord      <= 2'd0;
            cycle_max_dw   <= Q16_ZERO;
            coord_start    <= 1'b0;
            init_m         <= 4'd0;
            pred_val       <= Q16_ZERO;
            rss_accum_64   <= 64'sd0;
            zero_cnt       <= 3'd0;
        end else begin
            coord_start <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: TOP_IDLE
                // -------------------------------------------------------------
                TOP_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy           <= 1'b1;
                        num_params_reg <= (num_params != 3'd0) ? num_params : 3'd2;
                        num_obs_reg    <= (num_obs != 4'd0)    ? num_obs    : 4'd4;
                        lambda_reg_val <= lambda_reg;
                        tol_reg        <= (tolerance != Q16_ZERO) ? tolerance : Q16_EPS_DEF;
                        max_iters_reg  <= (max_iters != 8'd0)     ? max_iters : 8'd50;
                        iter_count     <= 8'd0;
                        status         <= STATUS_RUNNING;
                        w_cur          <= w_init;
                        init_m         <= 4'd0;
                        state          <= TOP_INIT_RES_1;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // INITIALIZE RESIDUAL: r_m = y_m - sum_{j} X_{m,j} * w_j
                // -------------------------------------------------------------
                TOP_INIT_RES_1: begin
                    if (init_m < num_obs_reg) begin
                        pred_val = Q16_ZERO;
                        for (int k = 0; k < MAX_PARAMS; k++) begin
                            if (k < num_params_reg) begin
                                pred_val = pred_val + q16_t'((64'(get_mat_elem(x_matrix, 3'(init_m[2:0]), 2'(k))) * 64'(get_vec(w_cur, 2'(k)))) >>> 16);
                            end
                        end
                        r_cur   <= set_obs(r_cur, 3'(init_m[2:0]), get_obs(y_obs, 3'(init_m[2:0])) - pred_val);
                        init_m  <= init_m + 1'b1;
                        state   <= TOP_INIT_RES_1;
                    end else begin
                        state <= TOP_CYCLE_START;
                    end
                end

                // -------------------------------------------------------------
                // 1. START FULL COORDINATE CYCLE (j = 0..N-1)
                // -------------------------------------------------------------
                TOP_CYCLE_START: begin
                    cur_coord    <= 2'd0;
                    cycle_max_dw <= Q16_ZERO;
                    state        <= TOP_COORD_START;
                end

                TOP_COORD_START: begin
                    coord_start <= 1'b1;
                    state       <= TOP_COORD_WAIT;
                end

                TOP_COORD_WAIT: begin
                    if (coord_done) begin
                        w_cur <= set_vec(w_cur, cur_coord, engine_w_out);
                        r_cur <= engine_r_out;

                        if (engine_dw_abs > cycle_max_dw) begin
                            cycle_max_dw <= engine_dw_abs;
                        end

                        if (cur_coord + 1'b1 < num_params_reg) begin
                            cur_coord <= cur_coord + 1'b1;
                            state     <= TOP_COORD_START;
                        end else begin
                            state <= TOP_CHECK_CONV;
                        end
                    end
                end

                // -------------------------------------------------------------
                // 2. CHECK CONVERGENCE AFTER FULL CYCLE
                // -------------------------------------------------------------
                TOP_CHECK_CONV: begin
                    max_delta_w <= cycle_max_dw;

                    // Condition 1: Converged if max coordinate shift <= tolerance
                    if (cycle_max_dw <= tol_reg) begin
                        status <= STATUS_CONVERGED;
                        state  <= TOP_CALC_COST;

                    // Condition 2: Max cycles reached
                    end else if (iter_count >= max_iters_reg) begin
                        status <= STATUS_MAX_ITERS;
                        state  <= TOP_CALC_COST;

                    // Condition 3: Continue cyclical sweeps
                    end else begin
                        iter_count <= iter_count + 1'b1;
                        state      <= TOP_CYCLE_START;
                    end
                end

                // -------------------------------------------------------------
                // 3. COMPUTE FINAL METRICS: RSS & Sparsity Count
                // -------------------------------------------------------------
                TOP_CALC_COST: begin
                    w_optimal <= w_cur;

                    // Calculate RSS = 0.5 * sum r_m^2
                    rss_accum_64 = 64'sd0;
                    for (int m = 0; m < MAX_OBS; m++) begin
                        if (m < num_obs_reg) begin
                            rss_accum_64 = rss_accum_64 + (64'(get_obs(r_cur, 3'(m))) * 64'(get_obs(r_cur, 3'(m))));
                        end
                    end
                    cost_optimal <= q16_t'((rss_accum_64 >>> 16) >>> 1); // 0.5 * RSS

                    // Count zero coefficients
                    zero_cnt = 3'd0;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params_reg) begin
                            if (get_vec(w_cur, 2'(k)) == Q16_ZERO) begin
                                zero_cnt = zero_cnt + 1'b1;
                            end
                        end
                    end
                    sparsity_count <= zero_cnt;

                    state <= TOP_DONE;
                end

                // -------------------------------------------------------------
                // STATE: TOP_DONE
                // -------------------------------------------------------------
                TOP_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= TOP_IDLE;
                end

                default: state <= TOP_IDLE;
            endcase
        end
    end

endmodule
