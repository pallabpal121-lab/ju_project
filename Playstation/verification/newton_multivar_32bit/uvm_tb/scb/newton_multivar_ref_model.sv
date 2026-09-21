// =============================================================================
// File Name   : newton_multivar_ref_model.sv
// Class Name  : newton_multivar_ref_model
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Bit-accurate Golden Reference Model for the Multivariable
//               Block Coordinate Descent (BCD) Newton Optimization Accelerator.
// =============================================================================

`ifndef NEWTON_MULTIVAR_REF_MODEL_SV
`define NEWTON_MULTIVAR_REF_MODEL_SV

class newton_multivar_ref_model extends uvm_object;
    `uvm_object_utils(newton_multivar_ref_model)

    function new(string name = "newton_multivar_ref_model");
        super.new(name);
    endfunction

    // -------------------------------------------------------------------------
    // Bit-Accurate Q16.16 Multiplication
    // -------------------------------------------------------------------------
    static function q16_t q16_mul(input q16_t a, input q16_t b);
        logic signed [63:0] prod;
        prod = 64'(a) * 64'(b);
        return prod[47:16];
    endfunction

    // -------------------------------------------------------------------------
    // Bit-Accurate Q16.16 Restoring Division
    // -------------------------------------------------------------------------
    static function q16_t q16_div(input q16_t dividend, input q16_t divisor, output bit div_by_zero);
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

    // -------------------------------------------------------------------------
    // Evaluate Multivariable DFG Equation
    // -------------------------------------------------------------------------
    static function q16_t eval_dfg(
        const ref instr_t prog[PROG_DEPTH],
        input int         n_vars,
        const ref q16_t   x_in[MAX_VARS]
    );
        q16_t reg_file[NUM_REGS];
        bit   dbz;

        for (int i = 0; i < NUM_REGS; i++) reg_file[i] = Q16_ZERO;
        for (int i = 0; i < n_vars; i++)    reg_file[i] = x_in[i];

        for (int pc = 0; pc < PROG_DEPTH; pc++) begin
            instr_t instr = prog[pc];
            case (instr.op)
                OP_NOP:   ;
                OP_ADD:   reg_file[instr.dst] = reg_file[instr.src_a] + reg_file[instr.src_b];
                OP_SUB:   reg_file[instr.dst] = reg_file[instr.src_a] - reg_file[instr.src_b];
                OP_MUL:   reg_file[instr.dst] = q16_mul(reg_file[instr.src_a], reg_file[instr.src_b]);
                OP_DIV:   reg_file[instr.dst] = q16_div(reg_file[instr.src_a], reg_file[instr.src_b], dbz);
                OP_NEG:   reg_file[instr.dst] = -reg_file[instr.src_a];
                OP_MOV:   reg_file[instr.dst] = reg_file[instr.src_a];
                OP_LOADC: reg_file[instr.dst] = {instr.imm, 16'h0000};
                OP_END:   return reg_file[REG_RESULT];
                default:  ;
            endcase
        end
        return reg_file[REG_RESULT];
    endfunction

    // -------------------------------------------------------------------------
    // 2-Variable Newton Step Solver for Pair (idx_i, idx_j)
    // -------------------------------------------------------------------------
    static function void solve_2var_step(
        const ref instr_t prog[PROG_DEPTH],
        input int         n_vars,
        const ref q16_t   x_curr[MAX_VARS],
        input int         idx_i,
        input int         idx_j,
        input q16_t       lambda_val,
        input q16_t       alpha_val,
        output q16_t      delta_i,
        output q16_t      delta_j,
        output q16_t      f_val
    );
        q16_t tmp_x[MAX_VARS];
        q16_t f_0, f_pi, f_mi, f_pj, f_mj;
        q16_t f_pp, f_pm, f_mp, f_mm;
        q16_t g_i, g_j;
        q16_t A_00, A_11, A_01;
        q16_t det, num_i, num_j;
        q16_t p_i, p_j;
        bit   dbz;

        // 1. Center sample
        f_0   = eval_dfg(prog, n_vars, x_curr);
        f_val = f_0;

        // 2. Sample x_i + h
        tmp_x = x_curr; tmp_x[idx_i] = x_curr[idx_i] + Q16_H_STEP;
        f_pi  = eval_dfg(prog, n_vars, tmp_x);

        // 3. Sample x_i - h
        tmp_x = x_curr; tmp_x[idx_i] = x_curr[idx_i] - Q16_H_STEP;
        f_mi  = eval_dfg(prog, n_vars, tmp_x);

        // 4. Sample x_j + h
        tmp_x = x_curr; tmp_x[idx_j] = x_curr[idx_j] + Q16_H_STEP;
        f_pj  = eval_dfg(prog, n_vars, tmp_x);

        // 5. Sample x_j - h
        tmp_x = x_curr; tmp_x[idx_j] = x_curr[idx_j] - Q16_H_STEP;
        f_mj  = eval_dfg(prog, n_vars, tmp_x);

        // 6. Sample x_i + h, x_j + h
        tmp_x = x_curr; tmp_x[idx_i] = x_curr[idx_i] + Q16_H_STEP; tmp_x[idx_j] = x_curr[idx_j] + Q16_H_STEP;
        f_pp  = eval_dfg(prog, n_vars, tmp_x);

        // 7. Sample x_i + h, x_j - h
        tmp_x = x_curr; tmp_x[idx_i] = x_curr[idx_i] + Q16_H_STEP; tmp_x[idx_j] = x_curr[idx_j] - Q16_H_STEP;
        f_pm  = eval_dfg(prog, n_vars, tmp_x);

        // 8. Sample x_i - h, x_j + h
        tmp_x = x_curr; tmp_x[idx_i] = x_curr[idx_i] - Q16_H_STEP; tmp_x[idx_j] = x_curr[idx_j] + Q16_H_STEP;
        f_mp  = eval_dfg(prog, n_vars, tmp_x);

        // 9. Sample x_i - h, x_j - h
        tmp_x = x_curr; tmp_x[idx_i] = x_curr[idx_i] - Q16_H_STEP; tmp_x[idx_j] = x_curr[idx_j] - Q16_H_STEP;
        f_mm  = eval_dfg(prog, n_vars, tmp_x);

        // Gradient
        g_i = (f_pi - f_mi) <<< 3;
        g_j = (f_pj - f_mj) <<< 3;

        // Hessian components
        A_00 = ((f_pi - (f_0 <<< 1) + f_mi) <<< 8) + lambda_val;
        A_11 = ((f_pj - (f_0 <<< 1) + f_mj) <<< 8) + lambda_val;
        A_01 = (f_pp - f_pm - f_mp + f_mm) <<< 6;

        // Determinant
        det = q16_mul(A_00, A_11) - q16_mul(A_01, A_01);

        // Cramer's rule numerators
        num_i = -q16_mul(g_i, A_11) + q16_mul(g_j, A_01);
        num_j = -q16_mul(g_j, A_00) + q16_mul(g_i, A_01);

        if (det <= 32'sd16) begin
            // Fallback to gradient descent
            p_i = -g_i;
            p_j = -g_j;
            delta_i = -q16_mul(alpha_val, g_i);
            delta_j = -q16_mul(alpha_val, g_j);
        end else begin
            p_i     = q16_div(num_i, det, dbz);
            p_j     = q16_div(num_j, det, dbz);
            delta_i = q16_mul(alpha_val, p_i);
            delta_j = q16_mul(alpha_val, p_j);
        end
    endfunction

    // -------------------------------------------------------------------------
    // Full Multivariable BCD Solver
    // -------------------------------------------------------------------------
    static function void solve_bcd_golden(
        const ref instr_t prog[PROG_DEPTH],
        input int         n_vars,
        const ref q16_t   x_init[MAX_VARS],
        input q16_t       tolerance,
        input q16_t       step_alpha,
        input q16_t       lambda_reg,
        input int         max_sweeps,
        output q16_t      x_optimal[MAX_VARS],
        output q16_t      f_optimal,
        output q16_t      max_delta_last,
        output logic [7:0] sweep_count,
        output status_t   status
    );
        q16_t x_curr[MAX_VARS];
        q16_t tol_val;
        q16_t alpha_val;
        q16_t lam_val;
        int   max_sw;
        int   active_n;
        bit   phase;

        active_n  = (n_vars >= 2) ? n_vars : 2;
        tol_val   = (tolerance  != Q16_ZERO) ? tolerance  : Q16_EPS_DEF;
        alpha_val = (step_alpha != Q16_ZERO) ? step_alpha : Q16_ONE;
        lam_val   = (lambda_reg != Q16_ZERO) ? lambda_reg : Q16_LAMBDA_DEF;
        max_sw    = (max_sweeps != 0)        ? max_sweeps : 50;

        x_curr    = x_init;
        sweep_count = 8'd0;
        phase     = 1'b0;
        status    = STATUS_RUNNING;

        forever begin
            q16_t max_delta_sweep;
            int   pair_i, pair_j;
            bit   sweep_done;

            max_delta_sweep = Q16_ZERO;
            sweep_done      = 0;

            if (!phase) begin
                pair_i = 0;
                pair_j = 1;
            end else begin
                pair_i = 1;
                pair_j = 2;
            end

            while (!sweep_done) begin
                q16_t di, dj, f_val_core;
                q16_t abs_di, abs_dj;

                solve_2var_step(prog, active_n, x_curr, pair_i, pair_j, lam_val, alpha_val, di, dj, f_val_core);
                f_optimal = f_val_core;

                x_curr[pair_i] = x_curr[pair_i] + di;
                x_curr[pair_j] = x_curr[pair_j] + dj;

                abs_di = (di < 32'sd0) ? -di : di;
                abs_dj = (dj < 32'sd0) ? -dj : dj;
                if (abs_di > max_delta_sweep) max_delta_sweep = abs_di;
                if (abs_dj > max_delta_sweep && abs_dj > abs_di) max_delta_sweep = abs_dj;

                // Advance pair logic matching RTL
                if (phase && pair_j == 0) begin
                    sweep_done = 1;
                end else if (pair_j + 2 < active_n) begin
                    pair_i = pair_i + 2;
                    pair_j = pair_j + 2;
                end else if (phase && active_n > 2 && pair_j < active_n) begin
                    pair_i = active_n - 1;
                    pair_j = 0;
                end else begin
                    sweep_done = 1;
                end
            end

            sweep_count++;
            max_delta_last = max_delta_sweep;

            // Check sweep termination
            if (max_delta_sweep <= tol_val) begin
                status    = STATUS_CONVERGED;
                x_optimal = x_curr;
                return;
            end else if (sweep_count >= max_sw) begin
                status    = STATUS_MAX_ITERS;
                x_optimal = x_curr;
                return;
            end else begin
                phase = ~phase;
            end
        end
    endfunction

endclass : newton_multivar_ref_model

`endif // NEWTON_MULTIVAR_REF_MODEL_SV
