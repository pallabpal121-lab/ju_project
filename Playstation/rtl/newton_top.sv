// ============================================================================
// File: newton_top.sv
// Description: Top-Level Reconfigurable Q16.16 Newton Optimization Accelerator
// Compatible with Icarus Verilog and standard SystemVerilog
// ============================================================================

import q16_types::*;

module newton_top (
    input  logic         clk,
    input  logic         rst_n,

    // DFG Program Memory Interface
    input  logic         prog_write_en,
    input  logic [5:0]   prog_addr,
    input  instr_word_t  prog_instr,
    input  logic [5:0]   prog_len_in,

    // Control Registers
    input  logic         start,
    input  q16_t         x_init,
    input  q16_t         h_step,
    input  q16_t         eps_tol,
    input  logic [7:0]   max_iter,

    // Output Status & Results
    output logic         done,
    output status_e      status,
    output q16_t         x_final,
    output q16_t         f_final,
    output q16_t         g_final,
    output q16_t         H_final,
    output logic [7:0]   iter_count
);

    // DFG Program Memory Array
    instr_word_t prog_mem [0:PROG_DEPTH-1];
    logic [5:0]  prog_len;
    logic [5:0]  eval_pc;
    instr_word_t current_eval_instr;

    assign current_eval_instr = prog_mem[eval_pc];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prog_len <= '0;
            for (int i = 0; i < PROG_DEPTH; i++) begin
                prog_mem[i] <= '0;
            end
        end else if (prog_write_en) begin
            prog_mem[prog_addr] <= prog_instr;
            prog_len            <= prog_len_in;
        end
    end

    // DFG Evaluator Instantiation
    logic   eval_start;
    q16_t   eval_x, eval_f;
    logic   eval_done, eval_ovf;

    dfg_evaluator u_eval (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (eval_start),
        .x_val         (eval_x),
        .prog_len      (prog_len),
        .pc            (eval_pc),
        .current_instr (current_eval_instr),
        .f_out         (eval_f),
        .done          (eval_done),
        .overflow_flag (eval_ovf)
    );

    // Shared Divider Mux & Signals
    logic   div_start, top_div_start, fd_div_start;
    q16_t   div_num, top_div_num, fd_div_num;
    q16_t   div_den, top_div_den, fd_div_den;
    q16_t   div_quot;
    logic   div_done, div_by_zero;

    q16_divider u_divider (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (div_start),
        .dividend_a  (div_num),
        .divisor_b   (div_den),
        .quotient    (div_quot),
        .done        (div_done),
        .div_by_zero (div_by_zero)
    );

    // Finite-Difference Controller Instantiation
    logic   fd_start;
    q16_t   fd_x;
    q16_t   f_zero, g_val, H_val;
    logic   fd_done, fd_error;

    newton_fd_controller u_fd_ctrl (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (fd_start),
        .x_val      (fd_x),
        .h_step     (h_step),

        .eval_start (eval_start),
        .eval_x     (eval_x),
        .eval_f     (eval_f),
        .eval_done  (eval_done),
        .eval_ovf   (eval_ovf),

        .div_start  (fd_div_start),
        .div_num    (fd_div_num),
        .div_den    (fd_div_den),
        .div_quot   (div_quot),
        .div_done   (div_done),
        .div_by_zero(div_by_zero),

        .f_zero_out (f_zero),
        .g_out      (g_val),
        .H_out      (H_val),
        .done       (fd_done),
        .fd_error   (fd_error)
    );

    // Main FSM Controller
    typedef enum logic [3:0] {
        IDLE,
        INIT_ITER,
        WAIT_FD,
        SOLVE_STEP,
        WAIT_SOLVE,
        UPDATE_X,
        FINISH
    } fsm_state_e;

    fsm_state_e state;
    q16_t x_curr, f_curr, g_curr, H_curr, delta_x;
    logic select_fd_div;

    // Divider Mux Selection
    always @(*) begin
        if (select_fd_div) begin
            div_start = fd_div_start;
            div_num   = fd_div_num;
            div_den   = fd_div_den;
        end else begin
            div_start = top_div_start;
            div_num   = top_div_num;
            div_den   = top_div_den;
        end
    end

    function automatic q16_t q16_abs(input q16_t val);
        return val[31] ? -val : val;
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= IDLE;
            done          <= 1'b0;
            status        <= STATUS_IDLE;
            x_curr        <= '0;
            f_curr        <= '0;
            g_curr        <= '0;
            H_curr        <= '0;
            delta_x       <= '0;
            x_final       <= '0;
            f_final       <= '0;
            g_final       <= '0;
            H_final       <= '0;
            iter_count    <= '0;
            fd_start      <= 1'b0;
            fd_x          <= '0;
            top_div_start <= 1'b0;
            top_div_num   <= '0;
            top_div_den   <= '0;
            select_fd_div <= 1'b1;
        end else begin
            case (state)
                IDLE: begin
                    done          <= 1'b0;
                    top_div_start <= 1'b0;
                    fd_start      <= 1'b0;
                    select_fd_div <= 1'b1;
                    if (start) begin
                        x_curr     <= x_init;
                        iter_count <= '0;
                        status     <= STATUS_BUSY;
                        state      <= INIT_ITER;
                    end
                end

                INIT_ITER: begin
                    if (iter_count >= max_iter) begin
                        status <= STATUS_MAX_ITER;
                        state  <= FINISH;
                    end else begin
                        fd_x          <= x_curr;
                        fd_start      <= 1'b1;
                        select_fd_div <= 1'b1;
                        state         <= WAIT_FD;
                    end
                end

                WAIT_FD: begin
                    fd_start <= 1'b0;
                    if (fd_done && !fd_start) begin
                        f_curr <= f_zero;
                        g_curr <= g_val;
                        H_curr <= H_val;

                        if (fd_error) begin
                            status <= STATUS_DIV_BY_ZERO;
                            state  <= FINISH;
                        end else if (q16_abs(g_val) < eps_tol) begin
                            status <= STATUS_SUCCESS;
                            state  <= FINISH;
                        end else if (H_val == 32'sd0) begin
                            status <= STATUS_INVALID_HESSIAN;
                            state  <= FINISH;
                        end else begin
                            state <= SOLVE_STEP;
                        end
                    end
                end

                SOLVE_STEP: begin
                    select_fd_div <= 1'b0;
                    top_div_num   <= -g_curr;
                    top_div_den   <= H_curr;
                    top_div_start <= 1'b1;
                    state         <= WAIT_SOLVE;
                end

                WAIT_SOLVE: begin
                    top_div_start <= 1'b0;
                    if (div_done && !top_div_start) begin
                        if (div_by_zero) begin
                            status <= STATUS_DIV_BY_ZERO;
                            state  <= FINISH;
                        end else begin
                            delta_x <= div_quot;
                            state   <= UPDATE_X;
                        end
                    end
                end

                UPDATE_X: begin
                    x_curr     <= x_curr + delta_x;
                    iter_count <= iter_count + 1'b1;

                    if (q16_abs(delta_x) < eps_tol) begin
                        status <= STATUS_SUCCESS;
                        state  <= FINISH;
                    end else begin
                        state <= INIT_ITER;
                    end
                end

                FINISH: begin
                    done       <= 1'b1;
                    x_final    <= x_curr;
                    f_final    <= f_curr;
                    g_final    <= g_curr;
                    H_final    <= H_curr;
                    state      <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
