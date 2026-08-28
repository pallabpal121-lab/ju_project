// ============================================================================
// File: q16_divider.sv
// Description: Multi-cycle Signed Q16.16 Fixed-Point Divider
// Computes quotient = (a << 16) / b
// ============================================================================

import q16_types::*;

module q16_divider (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  q16_t        dividend_a,
    input  q16_t        divisor_b,
    output q16_t        quotient,
    output logic        done,
    output logic        div_by_zero
);

    typedef enum logic [1:0] {
        IDLE,
        CALC,
        FINISH
    } div_state_e;

    div_state_e state;

    logic [5:0]  count;
    logic        sign_q;

    logic [63:0] abs_num;
    logic [31:0] abs_den;
    logic [63:0] rem_quot;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            count       <= '0;
            sign_q      <= 1'b0;
            abs_den     <= '0;
            rem_quot    <= '0;
            quotient    <= '0;
            done        <= 1'b0;
            div_by_zero <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        if (divisor_b == 32'sd0) begin
                            div_by_zero <= 1'b1;
                            quotient    <= 32'sd0;
                            done        <= 1'b1;
                            state       <= IDLE;
                        end else begin
                            div_by_zero <= 1'b0;
                            sign_q      <= dividend_a[31] ^ divisor_b[31];

                            // Absolute values with full 64-bit sign-extended dividend shift
                            abs_den     <= divisor_b[31] ? -divisor_b : divisor_b;
                            
                            begin
                                logic signed [63:0] div_ext;
                                div_ext = $signed({{32{dividend_a[31]}}, dividend_a}) <<< 16;
                                abs_num = div_ext[63] ? -div_ext : div_ext;
                            end

                            rem_quot    <= '0;
                            count       <= 6'd48; // 48 steps for 64-bit dividend / 32-bit divisor
                            state       <= CALC;
                        end
                    end
                end

                CALC: begin
                    if (count > 0) begin
                        // Shift left and subtract
                        logic [63:0] next_rq;
                        logic [31:0] high_rem;

                        next_rq  = rem_quot << 1;
                        high_rem = next_rq[63:32] | {31'b0, abs_num[count - 1]};

                        if (high_rem >= abs_den) begin
                            next_rq[63:32] = high_rem - abs_den;
                            next_rq[0]     = 1'b1;
                        end else begin
                            next_rq[63:32] = high_rem;
                            next_rq[0]     = 1'b0;
                        end

                        rem_quot <= next_rq;
                        count    <= count - 1'b1;
                    end else begin
                        state <= FINISH;
                    end
                end

                FINISH: begin
                    logic signed [31:0] final_q;
                    final_q = rem_quot[31:0];
                    quotient <= sign_q ? -final_q : final_q;
                    done     <= 1'b1;
                    state    <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
