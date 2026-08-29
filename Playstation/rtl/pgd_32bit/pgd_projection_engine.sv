// =============================================================================
// File Name   : pgd_projection_engine.sv
// Module Name : pgd_projection_engine
// Project     : Projected Gradient Descent (PGD) Accelerator (Solver #12)
// -----------------------------------------------------------------------------
// Description for Freshers & Beginners:
//   Unified Multi-Geometry Convex Hardware Projection Engine:
//     1. PROJ_NONE         : Bypass (Unconstrained)
//     2. PROJ_NON_NEGATIVE : Clamps negative coordinates to exact silicon zero
//     3. PROJ_BOX          : Physical hyperbox clamping: min(max(x, l), u)
//     4. PROJ_L2_BALL      : Euclidean Ball boundary projection: x * (R / ||x||_2)
//     5. PROJ_SIMPLEX      : Exact Probability Simplex projection (sum x_i = 1, x_i >= 0)
// =============================================================================

`timescale 1ns / 1ps

import pgd_types_pkg::*;
`include "pgd_helpers.svh"

module pgd_projection_engine (
    input  logic               clk,
    input  logic               rst_n,

    // Control & Inputs
    input  logic               start,
    input  proj_mode_t         proj_mode,     // Projection geometry selector
    input  logic [2:0]         num_params,    // Number of variables N (1..4)
    input  vec_t               x_in,          // Input vector to project y_{k+1}
    input  vec_t               box_lower,     // Box lower bounds l
    input  vec_t               box_upper,     // Box upper bounds u
    input  q16_t               ball_radius,   // L2 Ball radius R

    // Outputs
    output vec_t               x_out,         // Projected point Pi_C(y)
    output logic               done,
    output logic               busy
);

    typedef enum logic [3:0] {
        PROJ_IDLE        = 4'd0,
        PROJ_DIRECT      = 4'd1,
        PROJ_L2_SUM      = 4'd2,
        PROJ_L2_SQRT     = 4'd3,
        PROJ_L2_DIV      = 4'd4,
        PROJ_L2_SCALE    = 4'd5,
        PROJ_SMPX_SORT   = 4'd6,
        PROJ_SMPX_RHO    = 4'd7,
        PROJ_SMPX_DIV    = 4'd8,
        PROJ_SMPX_APPLY  = 4'd9,
        PROJ_DONE        = 4'd10
    } proj_state_t;

    proj_state_t state;

    vec_t x_res;

    // Divider Interconnect
    logic div_start;
    q16_t div_dividend, div_divisor, div_quotient;
    logic div_done, div_by_zero, div_busy;

    q16_divider u_div (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (div_start),
        .dividend   (div_dividend),
        .divisor    (div_divisor),
        .quotient   (div_quotient),
        .done       (div_done),
        .div_by_zero(div_by_zero),
        .busy       (div_busy)
    );

    // Sqrt Interconnect
    logic sqrt_start;
    q16_t sqrt_val, sqrt_root;
    logic sqrt_done, sqrt_busy;

    q16_sqrt u_sqrt (
        .clk  (clk),
        .rst_n(rst_n),
        .start(sqrt_start),
        .val  (sqrt_val),
        .root (sqrt_root),
        .done (sqrt_done),
        .busy (sqrt_busy)
    );

    // Internal Variables
    logic signed [63:0] norm_sq_acc;
    q16_t norm_val, scale_factor;
    q16_t coord_val, low_val, up_val;

    // Simplex sort & water-filling variables
    q16_t sorted_mu [3:0];
    q16_t cum_sum [3:0];
    logic [1:0] rho_idx;
    q16_t theta_val;
    q16_t tmp_swap;

    assign x_out = x_res;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= PROJ_IDLE;
            x_res        <= '0;
            norm_sq_acc  <= 64'sd0;
            norm_val     <= Q16_ZERO;
            scale_factor <= Q16_ONE;
            div_start    <= 1'b0;
            div_dividend <= Q16_ZERO;
            div_divisor  <= Q16_ZERO;
            sqrt_start   <= 1'b0;
            sqrt_val     <= Q16_ZERO;
            rho_idx      <= 2'd0;
            theta_val    <= Q16_ZERO;
            done         <= 1'b0;
            busy         <= 1'b0;
            for (int i = 0; i < 4; i++) begin
                sorted_mu[i] <= Q16_ZERO;
                cum_sum[i]   <= Q16_ZERO;
            end
        end else begin
            div_start  <= 1'b0;
            sqrt_start <= 1'b0;

            case (state)
                PROJ_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        busy <= 1'b1;
                        case (proj_mode)
                            PROJ_NONE, PROJ_NON_NEGATIVE, PROJ_BOX: state <= PROJ_DIRECT;
                            PROJ_L2_BALL:                           state <= PROJ_L2_SUM;
                            PROJ_SIMPLEX:                           state <= PROJ_SMPX_SORT;
                            default:                                state <= PROJ_DIRECT;
                        endcase
                    end else begin
                        busy <= 1'b0;
                    end
                end

                // -------------------------------------------------------------
                // 1. DIRECT 1-CYCLE PROJECTIONS: NONE, NON-NEGATIVE, BOX
                // -------------------------------------------------------------
                PROJ_DIRECT: begin
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params) begin
                            coord_val = get_vec(x_in, 2'(k));
                            low_val   = get_vec(box_lower, 2'(k));
                            up_val    = get_vec(box_upper, 2'(k));

                            if (proj_mode == PROJ_NON_NEGATIVE) begin
                                x_res = set_vec(x_res, 2'(k), (coord_val < Q16_ZERO) ? Q16_ZERO : coord_val);
                            end else if (proj_mode == PROJ_BOX) begin
                                if (coord_val < low_val)      x_res = set_vec(x_res, 2'(k), low_val);
                                else if (coord_val > up_val)  x_res = set_vec(x_res, 2'(k), up_val);
                                else                          x_res = set_vec(x_res, 2'(k), coord_val);
                            end else begin
                                x_res = set_vec(x_res, 2'(k), coord_val);
                            end
                        end
                    end
                    state <= PROJ_DONE;
                end

                // -------------------------------------------------------------
                // 2. EUCLIDEAN L2 BALL PROJECTION: ||x||_2 <= R
                // -------------------------------------------------------------
                PROJ_L2_SUM: begin
                    norm_sq_acc = 64'sd0;
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params) begin
                            coord_val   = get_vec(x_in, 2'(k));
                            norm_sq_acc = norm_sq_acc + (64'(coord_val) * 64'(coord_val));
                        end
                    end
                    sqrt_val   <= q16_t'(norm_sq_acc >>> 16);
                    sqrt_start <= 1'b1;
                    state      <= PROJ_L2_SQRT;
                end

                PROJ_L2_SQRT: begin
                    if (sqrt_done) begin
                        norm_val <= sqrt_root;
                        if (sqrt_root <= ball_radius || sqrt_root == Q16_ZERO) begin
                            // Inside ball -> no scaling
                            x_res <= x_in;
                            state <= PROJ_DONE;
                        end else begin
                            // Outside ball -> scale = R / ||x||_2
                            div_dividend <= ball_radius;
                            div_divisor  <= sqrt_root;
                            div_start    <= 1'b1;
                            state        <= PROJ_L2_DIV;
                        end
                    end
                end

                PROJ_L2_DIV: begin
                    if (div_done) begin
                        scale_factor <= div_quotient;
                        state        <= PROJ_L2_SCALE;
                    end
                end

                PROJ_L2_SCALE: begin
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params) begin
                            coord_val = get_vec(x_in, 2'(k));
                            x_res     = set_vec(x_res, 2'(k), q16_t'((64'(coord_val) * 64'(scale_factor)) >>> 16));
                        end
                    end
                    state <= PROJ_DONE;
                end

                // -------------------------------------------------------------
                // 3. PROBABILITY SIMPLEX PROJECTION: sum x_i = 1, x_i >= 0
                // -------------------------------------------------------------
                PROJ_SMPX_SORT: begin
                    // Load and sort coordinates descending
                    sorted_mu[0] <= get_vec(x_in, 2'd0);
                    sorted_mu[1] <= (num_params > 3'd1) ? get_vec(x_in, 2'd1) : Q16_NEG_ONE;
                    sorted_mu[2] <= (num_params > 3'd2) ? get_vec(x_in, 2'd2) : Q16_NEG_ONE;
                    sorted_mu[3] <= (num_params > 3'd3) ? get_vec(x_in, 2'd3) : Q16_NEG_ONE;
                    state        <= PROJ_SMPX_RHO;
                end

                PROJ_SMPX_RHO: begin
                    // Sorting network (Bubble sort on 4 elements)
                    for (int step = 0; step < 4; step++) begin
                        for (int i = 0; i < 3; i++) begin
                            if (sorted_mu[i] < sorted_mu[i+1]) begin
                                tmp_swap = sorted_mu[i];
                                sorted_mu[i]   <= sorted_mu[i+1];
                                sorted_mu[i+1] <= tmp_swap;
                            end
                        end
                    end

                    // Running cumulative sums
                    cum_sum[0] <= sorted_mu[0];
                    cum_sum[1] <= sorted_mu[0] + sorted_mu[1];
                    cum_sum[2] <= sorted_mu[0] + sorted_mu[1] + sorted_mu[2];
                    cum_sum[3] <= sorted_mu[0] + sorted_mu[1] + sorted_mu[2] + sorted_mu[3];

                    // Find rho index: (j+1)*mu_j + 1 - cum_sum_j > 0
                    if (num_params >= 3'd4 && (q16_t'(4 * sorted_mu[3]) + Q16_ONE - (sorted_mu[0] + sorted_mu[1] + sorted_mu[2] + sorted_mu[3])) > Q16_ZERO) begin
                        rho_idx      <= 2'd3;
                        div_dividend <= (sorted_mu[0] + sorted_mu[1] + sorted_mu[2] + sorted_mu[3]) - Q16_ONE;
                        div_divisor  <= 32'h0004_0000; // 4.0
                    end else if (num_params >= 3'd3 && (q16_t'(3 * sorted_mu[2]) + Q16_ONE - (sorted_mu[0] + sorted_mu[1] + sorted_mu[2])) > Q16_ZERO) begin
                        rho_idx      <= 2'd2;
                        div_dividend <= (sorted_mu[0] + sorted_mu[1] + sorted_mu[2]) - Q16_ONE;
                        div_divisor  <= 32'h0003_0000; // 3.0
                    end else if (num_params >= 3'd2 && (q16_t'(2 * sorted_mu[1]) + Q16_ONE - (sorted_mu[0] + sorted_mu[1])) > Q16_ZERO) begin
                        rho_idx      <= 2'd1;
                        div_dividend <= (sorted_mu[0] + sorted_mu[1]) - Q16_ONE;
                        div_divisor  <= 32'h0002_0000; // 2.0
                    end else begin
                        rho_idx      <= 2'd0;
                        div_dividend <= sorted_mu[0] - Q16_ONE;
                        div_divisor  <= 32'h0001_0000; // 1.0
                    end

                    div_start <= 1'b1;
                    state     <= PROJ_SMPX_DIV;
                end

                PROJ_SMPX_DIV: begin
                    if (div_done) begin
                        theta_val <= div_quotient;
                        state     <= PROJ_SMPX_APPLY;
                    end
                end

                PROJ_SMPX_APPLY: begin
                    for (int k = 0; k < MAX_PARAMS; k++) begin
                        if (k < num_params) begin
                            coord_val = get_vec(x_in, 2'(k));
                            if (coord_val > theta_val) begin
                                x_res = set_vec(x_res, 2'(k), coord_val - theta_val);
                            end else begin
                                x_res = set_vec(x_res, 2'(k), Q16_ZERO);
                            end
                        end
                    end
                    state <= PROJ_DONE;
                end

                // -------------------------------------------------------------
                // STATE: PROJ_DONE
                // -------------------------------------------------------------
                PROJ_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= PROJ_IDLE;
                end

                default: state <= PROJ_IDLE;
            endcase
        end
    end

endmodule
