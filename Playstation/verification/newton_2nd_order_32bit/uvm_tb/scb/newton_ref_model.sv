// =============================================================================
// File Name   : newton_ref_model.sv
// Class Name  : newton_ref_model
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_REF_MODEL_SV
`define NEWTON_REF_MODEL_SV

class newton_ref_model extends uvm_object;
    `uvm_object_utils(newton_ref_model)

    function new(string name = "newton_ref_model");
        super.new(name);
    endfunction

    // Bit-Accurate Q16.16 Division
    static function q16_t q16_div(q16_t dividend, q16_t divisor, output bit div_by_zero);
        logic sign_res;
        logic [31:0] abs_dend, abs_div;
        logic [63:0] rem_acc;
        logic [63:0] shifted;

        div_by_zero = 1'b0;
        if (divisor == 32'sd0) begin
            div_by_zero = 1'b1;
            return 32'sd0;
        end

        sign_res = dividend[31] ^ divisor[31];
        abs_dend = dividend[31] ? 32'(-dividend) : dividend;
        abs_div  = divisor[31]  ? 32'(-divisor)  : divisor;

        rem_acc = {32'd0, abs_dend};

        for (int i = 0; i < 48; i++) begin
            shifted = {rem_acc[62:0], 1'b0};
            if (shifted[63:32] >= abs_div) begin
                rem_acc = {(shifted[63:32] - abs_div), shifted[31:1], 1'b1};
            end else begin
                rem_acc = shifted;
            end
        end

        if (sign_res) begin
            return -32'(rem_acc[31:0]);
        end else begin
            return 32'(rem_acc[31:0]);
        end
    endfunction

    // Bit-Accurate Q16.16 Multiplication
    static function q16_t q16_mul(q16_t a, q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // Evaluate DFG Equation for Given x
    static function q16_t eval_dfg(const ref instr_t prog[PROG_DEPTH], input q16_t x_val, output bit div_err);
        q16_t reg_file[NUM_REGS];
        div_err = 1'b0;

        for (int i = 0; i < NUM_REGS; i++) reg_file[i] = Q16_ZERO;
        reg_file[REG_X] = x_val;

        for (int pc = 0; pc < PROG_DEPTH; pc++) begin
            instr_t instr = prog[pc];

            case (instr.op)
                OP_NOP:   ;
                OP_ADD:   reg_file[instr.dst] = reg_file[instr.src_a] + reg_file[instr.src_b];
                OP_SUB:   reg_file[instr.dst] = reg_file[instr.src_a] - reg_file[instr.src_b];
                OP_MUL:   reg_file[instr.dst] = q16_mul(reg_file[instr.src_a], reg_file[instr.src_b]);
                OP_DIV:   begin
                    bit dbz;
                    reg_file[instr.dst] = q16_div(reg_file[instr.src_a], reg_file[instr.src_b], dbz);
                    if (dbz) div_err = 1'b1;
                end
                OP_NEG:   reg_file[instr.dst] = -reg_file[instr.src_a];
                OP_MOV:   reg_file[instr.dst] = reg_file[instr.src_a];
                OP_LOADC: reg_file[instr.dst] = {instr.imm, 16'h0000};
                OP_END:   return reg_file[REG_RESULT];
                default:  ;
            endcase
        end

        return reg_file[REG_RESULT];
    endfunction

    // Full Solver Reference Simulation
    static function void solve_golden(
        const ref instr_t prog[PROG_DEPTH],
        input  q16_t       x_init,
        input  q16_t       tolerance,
        input  q16_t       step_alpha,
        input  q16_t       lambda_reg,
        input  logic [7:0] max_iters,
        output q16_t       x_optimal,
        output q16_t       f_optimal,
        output q16_t       g_final,
        output logic [7:0] iter_count,
        output status_t    status
    );
        q16_t x_curr;
        q16_t tol_val;
        q16_t alpha_val;
        q16_t lam_val;
        int   max_it;
        bit   div_err;

        x_curr    = x_init;
        tol_val   = (tolerance  != Q16_ZERO) ? tolerance  : Q16_EPS_DEF;
        alpha_val = (step_alpha != Q16_ZERO) ? step_alpha : Q16_ONE;
        lam_val   = (lambda_reg != Q16_ZERO) ? lambda_reg : Q16_LAMBDA_DEF;
        max_it    = (max_iters  != 8'd0)     ? int'(max_iters) : 50;
        iter_count = 8'd0;

        forever begin
            q16_t f_0, f_plus, f_minus;
            logic signed [31:0] diff_1st, diff_2nd;
            q16_t grad, step_num, step_den, abs_g;
            q16_t div_step, scaled_step;
            q16_t raw_curv, abs_curv, clamped_step;
            logic signed [32:0] x_full_sum;
            bit dbz;

            // 1. Sample 3 points
            f_0     = eval_dfg(prog, x_curr, div_err);
            f_plus  = eval_dfg(prog, x_curr + Q16_H_STEP, div_err);
            f_minus = eval_dfg(prog, x_curr - Q16_H_STEP, div_err);

            // 2. Finite difference terms
            diff_1st = f_plus - f_minus;
            diff_2nd = f_plus - (f_0 <<< 1) + f_minus;

            grad     = diff_1st <<< 3;
            step_num = diff_1st >>> 4;

            // Floor-clamped denominator: pure Newton when >= lam_val, clamped to lam_val otherwise
            raw_curv = diff_2nd <<< 1;
            abs_curv = (raw_curv >= 0) ? raw_curv : -raw_curv;
            if (abs_curv >= lam_val)
                step_den = raw_curv;
            else
                step_den = (raw_curv >= 0) ? lam_val : -lam_val;

            abs_g = (grad[31]) ? -grad : grad;

            // 3. Convergence & stopping checks
            if (abs_g <= tol_val) begin
                status    = STATUS_CONVERGED;
                x_optimal = x_curr;
                f_optimal = f_0;
                g_final   = grad;
                return;
            end else if (iter_count >= max_it) begin
                status    = STATUS_MAX_ITERS;
                x_optimal = x_curr;
                f_optimal = f_0;
                g_final   = grad;
                return;
            end

            // 4. Newton step solve
            div_step = q16_div(-step_num, step_den, dbz);
            if (dbz) begin
                status    = STATUS_SINGULAR;
                x_optimal = x_curr;
                f_optimal = f_0;
                g_final   = grad;
                return;
            end

            // 5. Scaled update with step bounding & saturating addition
            scaled_step = q16_mul(alpha_val, div_step);

            if (scaled_step > 32'sh000A_0000)
                clamped_step = 32'sh000A_0000;
            else if (scaled_step < -32'sh000A_0000)
                clamped_step = -32'sh000A_0000;
            else
                clamped_step = scaled_step;

            x_full_sum = {x_curr[31], x_curr} + {clamped_step[31], clamped_step};

            if ((~x_curr[31]) && (~clamped_step[31]) && x_full_sum[31])
                x_curr = 32'sh7FFF_FFFF;
            else if (x_curr[31] && clamped_step[31] && (~x_full_sum[31]))
                x_curr = 32'sh8000_0000;
            else
                x_curr = x_full_sum[31:0];

            iter_count  = iter_count + 1'b1;
        end
    endfunction

endclass : newton_ref_model

`endif // NEWTON_REF_MODEL_SV
