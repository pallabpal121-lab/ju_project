// ============================================================================
// File: q16_alu.sv
// Description: Q16.16 Fixed-Point Arithmetic Unit (ADD, SUB, MUL)
// Compatible with Icarus Verilog and standard SystemVerilog
// ============================================================================

import q16_types::*;

module q16_alu (
    input  opcode_e   op,
    input  q16_t      a,
    input  q16_t      b,
    output q16_t      res,
    output logic      overflow
);

    logic signed [63:0] prod;
    logic signed [32:0] sum;
    logic signed [32:0] diff;

    assign prod = $signed(a) * $signed(b);
    assign sum  = {a[31], a} + {b[31], b};
    assign diff = {a[31], a} - {b[31], b};

    always @(*) begin
        res      = 32'sd0;
        overflow = 1'b0;

        case (op)
            OP_ADD: begin
                res = sum[31:0];
                if ((a[31] == b[31]) && (res[31] != a[31])) begin
                    overflow = 1'b1;
                end
            end

            OP_SUB: begin
                res = diff[31:0];
                if ((a[31] != b[31]) && (res[31] != a[31])) begin
                    overflow = 1'b1;
                end
            end

            OP_MUL: begin
                res = prod[47:16];
                if ((prod[63:47] != 17'd0) && (prod[63:47] != 17'h1FFFF)) begin
                    overflow = 1'b1;
                end
            end

            default: begin
                res      = 32'sd0;
                overflow = 1'b0;
            end
        endcase
    end

endmodule
