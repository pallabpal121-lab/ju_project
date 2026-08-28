// ============================================================================
// File: dfg_evaluator.sv
// Description: Executes DFG program instructions on Q16.16 ALU
// Compatible with Icarus Verilog and standard SystemVerilog
// ============================================================================

import q16_types::*;

module dfg_evaluator (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         start,
    input  q16_t         x_val,
    input  logic [5:0]   prog_len,
    output logic [5:0]   pc,
    input  instr_word_t  current_instr,

    output q16_t         f_out,
    output logic         done,
    output logic         overflow_flag
);

    typedef enum logic [1:0] {
        IDLE,
        EXEC,
        FINISH
    } eval_state_e;

    eval_state_e state;

    q16_t reg_file [0:REG_DEPTH-1];
    
    // Instruction decoding wires
    logic [3:0]  inst_opcode;
    logic [4:0]  inst_dst;
    logic [4:0]  inst_srcA;
    logic [4:0]  inst_srcB;
    q16_t        inst_imm;

    assign inst_opcode = current_instr[63:60];
    assign inst_dst    = current_instr[59:55];
    assign inst_srcA   = current_instr[54:50];
    assign inst_srcB   = current_instr[49:45];
    assign inst_imm    = $signed(current_instr[31:0]);

    // ALU Interface
    opcode_e alu_op;
    q16_t    alu_a, alu_b, alu_res;
    logic    alu_ovf;

    q16_alu u_alu (
        .op       (alu_op),
        .a        (alu_a),
        .b        (alu_b),
        .res      (alu_res),
        .overflow (alu_ovf)
    );

    always @(*) begin
        alu_op = opcode_e'(inst_opcode);
        alu_a  = reg_file[inst_srcA];
        alu_b  = reg_file[inst_srcB];
    end

    logic [4:0] last_written_dst;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= IDLE;
            pc               <= '0;
            f_out            <= '0;
            done             <= 1'b0;
            overflow_flag    <= 1'b0;
            last_written_dst <= '0;
            for (int i = 0; i < REG_DEPTH; i++) begin
                reg_file[i] = '0;
            end
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    pc   <= '0;
                    if (start) begin
                        overflow_flag    <= 1'b0;
                        last_written_dst <= '0;
                        state            <= EXEC;
                    end
                end

                EXEC: begin
                    if (pc >= prog_len || inst_opcode == 4'd15) begin
                        if (inst_opcode == 4'd15 && inst_srcA != 5'd0) begin
                            f_out <= reg_file[inst_srcA];
                        end else begin
                            f_out <= reg_file[last_written_dst];
                        end
                        done  <= 1'b1;
                        state <= IDLE;
                    end else begin
                        if (alu_ovf) begin
                            overflow_flag <= 1'b1;
                        end

                        case (inst_opcode)
                            4'd0: begin // OP_NOP
                                // No action
                            end

                            4'd1: begin // OP_LOAD_CONST
                                reg_file[inst_dst] <= inst_imm;
                                last_written_dst   <= inst_dst;
                            end

                            4'd2: begin // OP_LOAD_X
                                reg_file[inst_dst] <= x_val;
                                last_written_dst   <= inst_dst;
                            end

                            4'd3, 4'd4, 4'd5: begin // OP_ADD, OP_SUB, OP_MUL
                                reg_file[inst_dst] <= alu_res;
                                last_written_dst   <= inst_dst;
                            end

                            default: begin
                                // Unhandled opcode
                            end
                        endcase

                        pc <= pc + 1'b1;
                    end
                end

                FINISH: begin
                    done  <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
