// =============================================================================
// File Name   : newton_bcd_top.sv
// Module Name : newton_bcd_top
// Project     : Universal Multivariable Newton 2nd-Order Accelerator
// -----------------------------------------------------------------------------
// Description: Block Coordinate Descent (BCD) Top-Level SoC Module.
//              Optimizes an N-variable equation by repeatedly batching pairs
//              of variables (x_i, x_j) through a dedicated 2-variable Newton core.
//              Implements alternating disjoint and overlapping sweeps.
// =============================================================================

`timescale 1ns / 1ps

import newton_multivar_pkg::*;
`include "multivar_helpers.svh"

module newton_bcd_top (
    input  logic                          clk,
    input  logic                          rst_n,

    // Interface 1: Equation Microcode Programming Port
    input  logic                          prog_en,
    input  logic [$clog2(PROG_DEPTH)-1:0] prog_addr,
    input  instr_t                        prog_data,

    // Interface 2: Optimization Controls & Parameters
    input  logic                          start,
    input  logic [NUM_VARS_BITS-1:0]      num_vars,      // Active dimension N (2..MAX_VARS)
    input  vec_t                          x_init,        // Initial guess vector
    input  q16_t                          tolerance,     // Convergence threshold on step size
    input  q16_t                          step_alpha,    // Step size / learning rate alpha
    input  q16_t                          lambda_reg,    // Damping factor lambda
    input  logic [7:0]                    max_sweeps,    // Maximum number of full sweeps

    // Interface 3: Results & Status Outputs
    output vec_t                          x_optimal,     // Converged optimal vector x*
    output q16_t                          f_optimal,     // Final function value f(x*)
    output q16_t                          max_delta_last,// Max parameter update in last sweep
    output logic [7:0]                    sweep_count,   // Total sweeps executed
    output status_t                       status,        // Convergence status code
    output logic                          done,          // 1-cycle completion pulse
    output logic                          busy           // High while solver is running
);

    // -------------------------------------------------------------------------
    // BCD Master FSM States
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        BCD_IDLE         = 3'd0,
        BCD_SETUP_SWEEP  = 3'd1,
        BCD_START_PAIR   = 3'd2,
        BCD_WAIT_PAIR    = 3'd3,
        BCD_ADVANCE_PAIR = 3'd4,
        BCD_CHECK_SWEEP  = 3'd5,
        BCD_DONE         = 3'd6
    } bcd_state_t;

    bcd_state_t state;

    // Registers
    vec_t                          x_reg;
    logic [NUM_VARS_BITS-1:0]      num_vars_reg;
    q16_t                          tol_reg;
    q16_t                          alpha_reg;
    q16_t                          lambda_reg_in;
    logic [7:0]                    max_sweeps_reg;
    logic                          phase; // 0: disjoint pairs (0,1), (2,3)... 1: interleaved (1,2), (3,4)...
    logic [NUM_VARS_BITS-1:0]      pair_i, pair_j;
    q16_t                          max_delta_sweep;

    // 2-Variable Core Signals
    logic                          core_start;
    logic [NUM_VARS_BITS-1:0]      core_idx_i, core_idx_j;
    q16_t                          core_delta_i, core_delta_j;
    q16_t                          core_f_val;
    logic                          core_done, core_busy;

    // Instantiate 2-Variable Newton Core
    newton_2var_core u_core (
        .clk        (clk),
        .rst_n      (rst_n),
        .prog_en    (prog_en),
        .prog_addr  (prog_addr),
        .prog_data  (prog_data),
        .start      (core_start),
        .num_vars   (num_vars_reg),
        .x_full     (x_reg),
        .idx_i      (core_idx_i),
        .idx_j      (core_idx_j),
        .lambda_reg (lambda_reg_in),
        .step_alpha (alpha_reg),
        .delta_i    (core_delta_i),
        .delta_j    (core_delta_j),
        .f_val      (core_f_val),
        .done       (core_done),
        .busy       (core_busy)
    );

    q16_t abs_di, abs_dj;
    vec_t tmp_x;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= BCD_IDLE;
            x_reg           <= '0;
            num_vars_reg    <= { {(NUM_VARS_BITS-2){1'b0}}, 2'd2 };
            tol_reg         <= Q16_EPS_DEF;
            alpha_reg       <= Q16_ONE;
            lambda_reg_in   <= Q16_LAMBDA_DEF;
            max_sweeps_reg  <= 8'd50;
            sweep_count     <= 8'd0;
            phase           <= 1'b0;
            pair_i          <= '0;
            pair_j          <= '0;
            core_idx_i      <= '0;
            core_idx_j      <= '0;
            core_start      <= 1'b0;
            max_delta_sweep <= Q16_ZERO;
            max_delta_last  <= Q16_ZERO;
            x_optimal       <= '0;
            f_optimal       <= Q16_ZERO;
            status          <= STATUS_IDLE;
            done            <= 1'b0;
            busy            <= 1'b0;
        end else begin
            core_start <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                // STATE: BCD_IDLE
                // -------------------------------------------------------------
                BCD_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy            <= 1'b1;
                        num_vars_reg    <= (num_vars >= 2) ? num_vars : { {(NUM_VARS_BITS-2){1'b0}}, 2'd2 };
                        tol_reg         <= (tolerance != Q16_ZERO)  ? tolerance  : Q16_EPS_DEF;
                        alpha_reg       <= (step_alpha != Q16_ZERO) ? step_alpha : Q16_ONE;
                        lambda_reg_in   <= (lambda_reg != Q16_ZERO) ? lambda_reg : Q16_LAMBDA_DEF;
                        max_sweeps_reg  <= (max_sweeps != 8'd0)     ? max_sweeps : 8'd50;
                        sweep_count     <= 8'd0;
                        x_reg           <= x_init;
                        phase           <= 1'b0;
                        status          <= STATUS_RUNNING;
                        state           <= BCD_SETUP_SWEEP;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // STATE: BCD_SETUP_SWEEP
                // -------------------------------------------------------------
                BCD_SETUP_SWEEP: begin
                    max_delta_sweep <= Q16_ZERO;
                    if (!phase) begin
                        // Phase 0: Disjoint pairs (0, 1), (2, 3), ...
                        pair_i <= '0;
                        pair_j <= { {(NUM_VARS_BITS-1){1'b0}}, 1'b1 };
                    end else begin
                        // Phase 1: Interleaved pairs (1, 2), (3, 4), ...
                        pair_i <= { {(NUM_VARS_BITS-1){1'b0}}, 1'b1 };
                        pair_j <= { {(NUM_VARS_BITS-2){1'b0}}, 2'd2 };
                    end
                    state <= BCD_START_PAIR;
                end

                // -------------------------------------------------------------
                // STATE: BCD_START_PAIR
                // -------------------------------------------------------------
                BCD_START_PAIR: begin
                    core_idx_i <= pair_i;
                    core_idx_j <= pair_j;
                    core_start <= 1'b1;
                    state      <= BCD_WAIT_PAIR;
                end

                // -------------------------------------------------------------
                // STATE: BCD_WAIT_PAIR
                // -------------------------------------------------------------
                BCD_WAIT_PAIR: begin
                    if (core_done) begin
                        // Update state vector X with new values
                        tmp_x = x_reg;
                        tmp_x = set_vec(tmp_x, pair_i, get_vec(tmp_x, pair_i) + core_delta_i);
                        tmp_x = set_vec(tmp_x, pair_j, get_vec(tmp_x, pair_j) + core_delta_j);
                        x_reg <= tmp_x;

                        // Track maximum step in this sweep
                        abs_di = (core_delta_i < 32'sd0) ? -core_delta_i : core_delta_i;
                        abs_dj = (core_delta_j < 32'sd0) ? -core_delta_j : core_delta_j;
                        if (abs_di > max_delta_sweep) max_delta_sweep <= abs_di;
                        if (abs_dj > max_delta_sweep && abs_dj > abs_di) max_delta_sweep <= abs_dj;

                        state <= BCD_ADVANCE_PAIR;
                    end
                end

                // -------------------------------------------------------------
                // STATE: BCD_ADVANCE_PAIR
                // -------------------------------------------------------------
                BCD_ADVANCE_PAIR: begin
                    if (phase && pair_j == '0) begin
                        // Just completed ring-closing pair (num_vars-1, 0)
                        state <= BCD_CHECK_SWEEP;
                    end else if (pair_j + 2 < num_vars_reg) begin
                        pair_i <= pair_i + 2;
                        pair_j <= pair_j + 2;
                        state  <= BCD_START_PAIR;
                    end else if (phase && num_vars_reg > 2 && pair_j < num_vars_reg) begin
                        // Close the ring in phase 1: couple (num_vars-1, 0)
                        pair_i <= num_vars_reg - 1'b1;
                        pair_j <= '0;
                        state  <= BCD_START_PAIR;
                    end else begin
                        state <= BCD_CHECK_SWEEP;
                    end
                end

                // -------------------------------------------------------------
                // STATE: BCD_CHECK_SWEEP
                // -------------------------------------------------------------
                BCD_CHECK_SWEEP: begin
                    sweep_count    <= sweep_count + 1'b1;
                    max_delta_last <= max_delta_sweep;

                    // Convergence condition
                    if (max_delta_sweep <= tol_reg) begin
                        status    <= STATUS_CONVERGED;
                        x_optimal <= x_reg;
                        f_optimal <= core_f_val;
                        state     <= BCD_DONE;
                    end else if (sweep_count + 1'b1 >= max_sweeps_reg) begin
                        status    <= STATUS_MAX_ITERS;
                        x_optimal <= x_reg;
                        f_optimal <= core_f_val;
                        state     <= BCD_DONE;
                    end else begin
                        phase <= ~phase; // Alternate between disjoint and interleaved
                        state <= BCD_SETUP_SWEEP;
                    end
                end

                // -------------------------------------------------------------
                // STATE: BCD_DONE
                // -------------------------------------------------------------
                BCD_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= BCD_IDLE;
                end

                default: state <= BCD_IDLE;
            endcase
        end
    end

endmodule
