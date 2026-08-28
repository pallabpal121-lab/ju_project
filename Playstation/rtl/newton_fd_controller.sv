// ============================================================================
// File: newton_fd_controller.sv
// Description: Finite-Difference Gradient & Hessian Calculation Engine
// Compatible with Icarus Verilog and standard SystemVerilog
// ============================================================================

import q16_types::*;

module newton_fd_controller (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         start,
    input  q16_t         x_val,
    input  q16_t         h_step,
    
    // DFG Evaluator Interface
    output logic         eval_start,
    output q16_t         eval_x,
    input  q16_t         eval_f,
    input  logic         eval_done,
    input  logic         eval_ovf,

    // Divider Interface
    output logic         div_start,
    output q16_t         div_num,
    output q16_t         div_den,
    input  q16_t         div_quot,
    input  logic         div_done,
    input  logic         div_by_zero,

    // Outputs
    output q16_t         f_zero_out,
    output q16_t         g_out,
    output q16_t         H_out,
    output logic         done,
    output logic         fd_error
);

    typedef enum logic [3:0] {
        IDLE,
        INIT_MINUS,
        EVAL_MINUS,
        WAIT_MINUS,
        INIT_ZERO,
        EVAL_ZERO,
        WAIT_ZERO,
        INIT_PLUS,
        EVAL_PLUS,
        WAIT_PLUS,
        CALC_G,
        WAIT_G,
        CALC_H,
        WAIT_H,
        FINISH
    } fd_state_e;

    fd_state_e state;

    q16_t f_minus, f_zero, f_plus;
    q16_t h2;
    logic alu_ovf;

    // Fixed-point multiply for h^2 = h * h
    q16_alu u_h2_alu (
        .op       (opcode_e'(OP_MUL)),
        .a        (h_step),
        .b        (h_step),
        .res      (h2),
        .overflow (alu_ovf)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            eval_start <= 1'b0;
            eval_x     <= '0;
            div_start  <= 1'b0;
            div_num    <= '0;
            div_den    <= '0;
            f_minus    <= '0;
            f_zero     <= '0;
            f_plus     <= '0;
            f_zero_out <= '0;
            g_out      <= '0;
            H_out      <= '0;
            done       <= 1'b0;
            fd_error   <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    done       <= 1'b0;
                    eval_start <= 1'b0;
                    div_start  <= 1'b0;
                    if (start) begin
                        fd_error <= 1'b0;
                        eval_x   <= x_val - h_step;
                        state    <= EVAL_MINUS;
                    end
                end

                EVAL_MINUS: begin
                    eval_start <= 1'b1;
                    state      <= WAIT_MINUS;
                end

                WAIT_MINUS: begin
                    eval_start <= 1'b0;
                    if (eval_done && !eval_start) begin
                        f_minus <= eval_f;
                        eval_x  <= x_val;
                        state   <= EVAL_ZERO;
                    end
                end

                EVAL_ZERO: begin
                    eval_start <= 1'b1;
                    state      <= WAIT_ZERO;
                end

                WAIT_ZERO: begin
                    eval_start <= 1'b0;
                    if (eval_done && !eval_start) begin
                        f_zero     <= eval_f;
                        f_zero_out <= eval_f;
                        eval_x     <= x_val + h_step;
                        state      <= EVAL_PLUS;
                    end
                end

                EVAL_PLUS: begin
                    eval_start <= 1'b1;
                    state      <= WAIT_PLUS;
                end

                WAIT_PLUS: begin
                    eval_start <= 1'b0;
                    if (eval_done && !eval_start) begin
                        f_plus <= eval_f;
                        state  <= CALC_G;
                    end
                end

                CALC_G: begin
                    // g = (f_plus - f_minus) / (2 * h)
                    div_num   <= f_plus - f_minus;
                    div_den   <= h_step <<< 1;
                    div_start <= 1'b1;
                    state     <= WAIT_G;
                end

                WAIT_G: begin
                    div_start <= 1'b0;
                    if (div_done && !div_start) begin
                        if (div_by_zero) begin
                            fd_error <= 1'b1;
                        end
                        g_out <= div_quot;
                        state <= CALC_H;
                    end
                end

                CALC_H: begin
                    // H = (f_plus - 2*f_zero + f_minus) / (h^2)
                    div_num   <= f_plus - (f_zero <<< 1) + f_minus;
                    div_den   <= h2;
                    div_start <= 1'b1;
                    state     <= WAIT_H;
                end

                WAIT_H: begin
                    div_start <= 1'b0;
                    if (div_done && !div_start) begin
                        if (div_by_zero) begin
                            fd_error <= 1'b1;
                        end
                        H_out <= div_quot;
                        done  <= 1'b1;
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
