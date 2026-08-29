// =============================================================================
// File Name   : fw_lmo_engine.sv
// Module Name : fw_lmo_engine
// Project     : Frank-Wolfe / Conditional Gradient Accelerator (Solver #20)
// -----------------------------------------------------------------------------
// Description:
//   Hardware Linear Minimization Oracle (LMO) evaluating s = argmin_{s in C} g^T s:
//   1. L1 Ball (||x||_1 <= R): s_i* = -sign(g_i*) * R at i* = argmax |g_i|
//   2. Box (l <= x <= u): s_i = (g_i > 0) ? l_i : u_i
//   3. Simplex (sum x_i = 1, x_i >= 0): s_i* = 1.0 at i* = argmin g_i
// =============================================================================

`timescale 1ns / 1ps

import fw_types_pkg::*;
`include "fw_helpers.svh"

module fw_lmo_engine (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,

    input  logic [2:0]  dim_n,         // Problem dimension N (1..4)
    input  fw_geom_t    geom,          // Constraint geometry
    input  vec_t        grad_in,       // Gradient vector g = ∇f(x)
    input  q16_t        radius_l1,     // L1 ball radius R
    input  vec_t        box_lower,     // Box lower bounds l
    input  vec_t        box_upper,     // Box upper bounds u

    output vec_t        s_lmo,         // LMO extreme point s in C
    output logic        done,
    output logic        busy
);

    typedef enum logic [1:0] {
        LMO_IDLE = 2'd0,
        LMO_CALC = 2'd1,
        LMO_DONE = 2'd2
    } lmo_state_t;

    lmo_state_t state;

    vec_t s_reg;
    assign s_lmo = s_reg;

    q16_t max_abs_g, min_g, abs_gi;
    logic [1:0] best_idx_l1, best_idx_simplex;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= LMO_IDLE;
            s_reg <= '0;
            done  <= 1'b0;
            busy  <= 1'b0;
        end else begin
            case (state)
                LMO_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        state <= LMO_CALC;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                LMO_CALC: begin
                    case (geom)
                        // -----------------------------------------------------
                        // 1. L1 Ball: s = -sign(g_i*) * R at i* = argmax |g_i|
                        // -----------------------------------------------------
                        GEOM_L1_BALL: begin
                            max_abs_g   = q16_abs(grad_in[0]);
                            best_idx_l1 = 2'd0;
                            for (int i = 1; i < MAX_DIM; i++) begin
                                if (i < dim_n) begin
                                    abs_gi = q16_abs(grad_in[i]);
                                    if (abs_gi > max_abs_g) begin
                                        max_abs_g   = abs_gi;
                                        best_idx_l1 = 2'(i);
                                    end
                                end
                            end

                            s_reg <= '0;
                            if (grad_in[best_idx_l1] > 0) begin
                                s_reg[best_idx_l1] <= -radius_l1;
                            end else begin
                                s_reg[best_idx_l1] <= radius_l1;
                            end
                        end

                        // -----------------------------------------------------
                        // 2. Hyperbox: s_i = (g_i > 0) ? l_i : u_i
                        // -----------------------------------------------------
                        GEOM_BOX: begin
                            for (int i = 0; i < MAX_DIM; i++) begin
                                if (i < dim_n) begin
                                    if (grad_in[i] > 0) begin
                                        s_reg[i] <= box_lower[i];
                                    end else begin
                                        s_reg[i] <= box_upper[i];
                                    end
                                end else begin
                                    s_reg[i] <= Q16_ZERO;
                                end
                            end
                        end

                        // -----------------------------------------------------
                        // 3. Probability Simplex: s_i* = 1.0 at i* = argmin g_i
                        // -----------------------------------------------------
                        GEOM_SIMPLEX: begin
                            min_g            = grad_in[0];
                            best_idx_simplex = 2'd0;
                            for (int i = 1; i < MAX_DIM; i++) begin
                                if (i < dim_n) begin
                                    if (grad_in[i] < min_g) begin
                                        min_g            = grad_in[i];
                                        best_idx_simplex = 2'(i);
                                    end
                                end
                            end

                            s_reg <= '0;
                            s_reg[best_idx_simplex] <= Q16_ONE;
                        end

                        default: s_reg <= '0;
                    endcase

                    state <= LMO_DONE;
                end

                LMO_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= LMO_IDLE;
                end

                default: state <= LMO_IDLE;
            endcase
        end
    end

endmodule
