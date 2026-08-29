// =============================================================================
// File Name   : pso_velocity_engine.sv
// Module Name : pso_velocity_engine
// Project     : Particle Swarm Optimization (PSO) Accelerator (Solver #14)
// -----------------------------------------------------------------------------
// Description:
//   Evaluates velocity and position updates for a single particle:
//   v_{d}(t+1) = clamp(ω·v_{d} + c1·r1·(p_{d} - x_{d}) + c2·r2·(g*_{d} - x_{d}), -v_max, +v_max)
//   x_{d}(t+1) = clamp(x_{d} + v_{d}(t+1), x_min_d, x_max_d)
// =============================================================================

`timescale 1ns / 1ps

import pso_types_pkg::*;
`include "pso_helpers.svh"

module pso_velocity_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Inputs
    input  logic               start,
    input  logic [2:0]         num_dims,      // Number of dimensions N (1..4)
    input  vec_t               x_curr,        // Current position vector
    input  vec_t               v_curr,        // Current velocity vector
    input  vec_t               p_best,        // Personal best position vector
    input  vec_t               g_best,        // Global best position vector
    input  q16_t               inertia_w,     // Inertia weight ω
    input  q16_t               c1_coeff,      // Cognitive coefficient c1
    input  q16_t               c2_coeff,      // Social coefficient c2
    input  q16_t               v_max,         // Max velocity magnitude
    input  vec_t               x_min_bound,   // Lower bounding box
    input  vec_t               x_max_bound,   // Upper bounding box
    input  q16_t               r1_rand,       // Random scalar r1 in [0, 1)
    input  q16_t               r2_rand,       // Random scalar r2 in [0, 1)

    // Outputs
    output vec_t               x_next,        // Updated position vector
    output vec_t               v_next,        // Updated velocity vector
    output logic               done,
    output logic               busy
);

    typedef enum logic [1:0] {
        V_IDLE = 2'd0,
        V_CALC = 2'd1,
        V_DONE = 2'd2
    } v_state_t;

    v_state_t state;

    vec_t v_out_reg;
    vec_t x_out_reg;

    assign v_next = v_out_reg;
    assign x_next = x_out_reg;

    // Temporary calculation variables
    q16_t x_d, v_d, p_d, g_d;
    q16_t v_inertia, v_cog, v_soc, v_sum, v_clamped;
    q16_t x_sum, x_clamped;
    q16_t c1_r1, c2_r2;
    vec_t temp_v, temp_x;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= V_IDLE;
            v_out_reg <= '0;
            x_out_reg <= '0;
            done      <= 1'b0;
            busy      <= 1'b0;
        end else begin
            case (state)
                V_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= V_CALC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                V_CALC: begin
                    // Compute c1 * r1 and c2 * r2
                    c1_r1 = q16_mul(c1_coeff, r1_rand);
                    c2_r2 = q16_mul(c2_coeff, r2_rand);

                    temp_v = '0;
                    temp_x = '0;

                    for (int d = 0; d < MAX_DIMS; d++) begin
                        if (d < num_dims) begin
                            x_d = get_vec(x_curr, 2'(d));
                            v_d = get_vec(v_curr, 2'(d));
                            p_d = get_vec(p_best, 2'(d));
                            g_d = get_vec(g_best, 2'(d));

                            // 1. Inertia component: ω * v_d
                            v_inertia = q16_mul(inertia_w, v_d);

                            // 2. Cognitive component: c1 * r1 * (p_d - x_d)
                            v_cog = q16_mul(c1_r1, p_d - x_d);

                            // 3. Social component: c2 * r2 * (g_d - x_d)
                            v_soc = q16_mul(c2_r2, g_d - x_d);

                            // 4. Sum velocity
                            v_sum = v_inertia + v_cog + v_soc;

                            // 5. Clamp velocity: [-v_max, +v_max]
                            v_clamped = q16_clamp(v_sum, -v_max, v_max);
                            temp_v    = set_vec(temp_v, 2'(d), v_clamped);

                            // 6. Update position and clamp to [x_min, x_max]
                            x_sum     = x_d + v_clamped;
                            x_clamped = q16_clamp(x_sum, get_vec(x_min_bound, 2'(d)), get_vec(x_max_bound, 2'(d)));
                            temp_x    = set_vec(temp_x, 2'(d), x_clamped);
                        end
                    end

                    v_out_reg <= temp_v;
                    x_out_reg <= temp_x;
                    state     <= V_DONE;
                end

                V_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= V_IDLE;
                end

                default: state <= V_IDLE;
            endcase
        end
    end

endmodule
