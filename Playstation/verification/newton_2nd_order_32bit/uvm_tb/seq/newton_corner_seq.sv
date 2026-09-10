// =============================================================================
// File Name   : newton_corner_seq.sv
// Class Name  : newton_corner_seq
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// -----------------------------------------------------------------------------
// Tests corner conditions:
// 1. Reaching max_iters limit with very tight tolerance and small alpha
// 2. Fractional step alpha (alpha = 0.5)
// =============================================================================

`ifndef NEWTON_CORNER_SEQ_SV
`define NEWTON_CORNER_SEQ_SV

class newton_corner_seq extends newton_base_seq;
    `uvm_object_utils(newton_corner_seq)

    function new(string name = "newton_corner_seq");
        super.new(name);
    endfunction

    virtual task body();
        newton_seq_item item;

        `uvm_info(get_type_name(), "Executing Corner Case Sequence ...", UVM_LOW)

        // Case 1: Max iterations limit hit (pos_large)
        item = newton_seq_item::type_id::create("corner_max_iters");
        start_item(item);
        item.reprogram   = 1'b1;
        item.prog_length = 7;
        item.program_mem[0] = make_instr(OP_MUL,   4'd1,  4'd0, 4'd0, 16'sd0);  // r1 = x^2
        item.program_mem[1] = make_instr(OP_LOADC, 4'd2,  4'd0, 4'd0, -16'sd6); // r2 = -6.0
        item.program_mem[2] = make_instr(OP_MUL,   4'd3,  4'd2, 4'd0, 16'sd0);  // r3 = -6x
        item.program_mem[3] = make_instr(OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd9);  // r4 = +9.0
        item.program_mem[4] = make_instr(OP_ADD,   4'd5,  4'd1, 4'd3, 16'sd0);  // r5 = x^2 - 6x
        item.program_mem[5] = make_instr(OP_ADD,   4'd15, 4'd5, 4'd4, 16'sd0);  // r15 = x^2 - 6x + 9
        item.program_mem[6] = make_instr(OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);

        item.x_init     = 32'h0032_0000; // x_0 = 50.0 (pos_large)
        item.tolerance  = 32'h0000_0001; // Ultra tight tolerance
        item.step_alpha = 32'h0000_2000; // Very small step alpha = 0.125
        item.lambda_reg = Q16_LAMBDA_DEF;
        item.max_iters  = 8'd3;          // Max iters = 3 (will trigger STATUS_MAX_ITERS)
        finish_item(item);

        // Case 1b: Max iterations with zero_near
        item = newton_seq_item::type_id::create("corner_max_iters_zero");
        start_item(item);
        item.reprogram   = 1'b0;
        item.prog_length = 0;
        item.x_init      = 32'h0000_8000; // x_0 = 0.5 (zero_near)
        item.tolerance   = 32'h0000_0001;
        item.step_alpha  = 32'h0000_1000;
        item.lambda_reg  = Q16_LAMBDA_DEF;
        item.max_iters   = 8'd2;
        finish_item(item);

        // Case 1c: Max iterations with pos_small
        item = newton_seq_item::type_id::create("corner_max_iters_pos_small");
        start_item(item);
        item.reprogram   = 1'b0;
        item.prog_length = 0;
        item.x_init      = 32'h0005_0000; // x_0 = 5.0 (pos_small)
        item.tolerance   = 32'h0000_0001;
        item.step_alpha  = 32'h0000_1000;
        item.lambda_reg  = Q16_LAMBDA_DEF;
        item.max_iters   = 8'd2;
        finish_item(item);

        // Case 2: Half step size convergence
        item = newton_seq_item::type_id::create("corner_half_step");
        start_item(item);
        item.reprogram   = 1'b0;
        item.prog_length = 0;
        item.x_init      = 32'h0007_0000; // x_0 = 7.0
        item.tolerance   = Q16_EPS_DEF;
        item.step_alpha  = Q16_HALF;      // alpha = 0.5
        item.lambda_reg  = Q16_LAMBDA_DEF;
        item.max_iters   = 8'd50;
        finish_item(item);

        // Case 3: Singular Hessian Curvature (f(x) = 2x, f''(x) = 0) across all 5 initial guess bins
        // 3a. Singular with pos_small
        item = newton_seq_item::type_id::create("corner_sing_pos_small");
        start_item(item);
        item.reprogram   = 1'b1;
        item.prog_length = 3;
        item.program_mem[0] = make_instr(OP_LOADC, 4'd2,  4'd0, 4'd0, 16'sd2); // r2 = 2.0
        item.program_mem[1] = make_instr(OP_MUL,   4'd15, 4'd2, 4'd0, 16'sd0); // r15 = 2.0 * x
        item.program_mem[2] = make_instr(OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);
        item.x_init      = 32'h0005_0000; // x_0 = 5.0 (pos_small)
        item.tolerance   = 32'h0000_0001;
        item.step_alpha  = Q16_ONE;
        item.lambda_reg  = -32'sd1;       // Damping floor disabled -> divisor = 0 -> STATUS_SINGULAR
        item.max_iters   = 8'd50;
        finish_item(item);

        // 3b. Singular with pos_large
        item = newton_seq_item::type_id::create("corner_sing_pos_large");
        start_item(item);
        item.reprogram   = 1'b0;
        item.prog_length = 0;
        item.x_init      = 32'h0014_0000; // x_0 = 20.0 (pos_large)
        item.tolerance   = 32'h0000_0001;
        item.step_alpha  = Q16_ONE;
        item.lambda_reg  = -32'sd1;
        item.max_iters   = 8'd50;
        finish_item(item);

        // 3c. Singular with zero_near
        item = newton_seq_item::type_id::create("corner_sing_zero_near");
        start_item(item);
        item.reprogram   = 1'b0;
        item.prog_length = 0;
        item.x_init      = 32'h0000_0000; // x_0 = 0.0 (zero_near)
        item.tolerance   = 32'h0000_0001;
        item.step_alpha  = Q16_ONE;
        item.lambda_reg  = -32'sd1;
        item.max_iters   = 8'd50;
        finish_item(item);

        // 3d. Singular with neg_small
        item = newton_seq_item::type_id::create("corner_sing_neg_small");
        start_item(item);
        item.reprogram   = 1'b0;
        item.prog_length = 0;
        item.x_init      = -32'sd327680;  // x_0 = -5.0 (neg_small)
        item.tolerance   = 32'h0000_0001;
        item.step_alpha  = Q16_ONE;
        item.lambda_reg  = -32'sd1;
        item.max_iters   = 8'd50;
        finish_item(item);

        // 3e. Singular with neg_large
        item = newton_seq_item::type_id::create("corner_sing_neg_large");
        start_item(item);
        item.reprogram   = 1'b0;
        item.prog_length = 0;
        item.x_init      = -32'sd1310720; // x_0 = -20.0 (neg_large)
        item.tolerance   = 32'h0000_0001;
        item.step_alpha  = Q16_ONE;
        item.lambda_reg  = -32'sd1;
        item.max_iters   = 8'd50;
        finish_item(item);

        // Case 4: Complete ALU Opcode & Arithmetic Overflow Stress
        // Tests: OP_NOP, OP_MOV, OP_NEG, OP_ADD (+/+ and -/- overflow), OP_SUB (+/- and -/+ overflow)
        item = newton_seq_item::type_id::create("corner_alu_stress");
        start_item(item);
        item.reprogram   = 1'b1;
        item.prog_length = 13;

        // 1. OP_NOP
        item.program_mem[0]  = make_instr(OP_NOP,   4'd1,  4'd0, 4'd0, 16'sd0);

        // 2. OP_MOV
        item.program_mem[1]  = make_instr(OP_MOV,   4'd2,  4'd0, 4'd0, 16'sd0);     // r2 = x

        // 3. OP_NEG
        item.program_mem[2]  = make_instr(OP_NEG,   4'd3,  4'd2, 4'd0, 16'sd0);     // r3 = -x

        // 4. OP_ADD positive overflow: (+30000) + (+30000)
        item.program_mem[3]  = make_instr(OP_LOADC, 4'd4,  4'd0, 4'd0, 16'sd30000); // r4 = +30000.0
        item.program_mem[4]  = make_instr(OP_ADD,   4'd5,  4'd4, 4'd4, 16'sd0);     // r5 = +60000 (overflows to negative)

        // 5. OP_ADD negative overflow: (-30000) + (-30000)
        item.program_mem[5]  = make_instr(OP_LOADC, 4'd6,  4'd0, 4'd0, -16'sd30000);// r6 = -30000.0
        item.program_mem[6]  = make_instr(OP_ADD,   4'd7,  4'd6, 4'd6, 16'sd0);     // r7 = -60000 (overflows to positive)

        // 6. OP_SUB pos - neg overflow: (+30000) - (-30000)
        item.program_mem[7]  = make_instr(OP_SUB,   4'd8,  4'd4, 4'd6, 16'sd0);     // r8 = pos - neg overflows to negative

        // 7. OP_SUB neg - pos overflow: (-30000) - (+30000)
        item.program_mem[8]  = make_instr(OP_SUB,   4'd9,  4'd6, 4'd4, 16'sd0);     // r9 = neg - pos overflows to positive

        // 8. Normal quadratic objective: f(x) = (x - 3)^2 in r15
        item.program_mem[9]  = make_instr(OP_LOADC, 4'd10, 4'd0, 4'd0, -16'sd3);   // r10 = -3.0
        item.program_mem[10] = make_instr(OP_ADD,   4'd11, 4'd0, 4'd10, 16'sd0);   // r11 = x - 3
        item.program_mem[11] = make_instr(OP_MUL,   4'd15, 4'd11, 4'd11, 16'sd0);  // r15 = (x - 3)^2
        item.program_mem[12] = make_instr(OP_END,   4'd0,  4'd0, 4'd0, 16'sd0);

        item.x_init     = 32'h0004_0000; // x_0 = 4.0
        item.tolerance  = Q16_EPS_DEF;
        item.step_alpha = Q16_ONE;
        item.lambda_reg = Q16_LAMBDA_DEF;
        item.max_iters  = 8'd50;
        finish_item(item);
    endtask

endclass : newton_corner_seq

`endif // NEWTON_CORNER_SEQ_SV
