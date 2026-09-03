// =============================================================================
// File Name   : newton_seq_item.sv
// Class Name  : newton_seq_item
// Project     : Universal Newton 2nd-Order Optimization Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_SEQ_ITEM_SV
`define NEWTON_SEQ_ITEM_SV

class newton_seq_item extends uvm_sequence_item;

    // Microcode Program Payload
    instr_t             program_mem [PROG_DEPTH];
    int                 prog_length;
    bit                 reprogram;

    // Optimization Input Parameters
    rand q16_t          x_init;
    rand q16_t          tolerance;
    rand q16_t          step_alpha;
    rand q16_t          lambda_reg;
    rand logic [7:0]    max_iters;

    // Output Response (Sampled on 'done')
    q16_t               x_optimal;
    q16_t               f_optimal;
    q16_t               g_final;
    logic [7:0]         iter_count;
    status_t            status;

    // Constraints with Distribution Weights & Fallback Testing
    constraint c_tol {
        tolerance dist {
            32'h0000_0000                   := 5,  // Exercises RTL fallback to Q16_EPS_DEF
            [32'h0000_0010 : 32'h0000_003F] := 30, // Tight tolerance (tol_tight coverage bin)
            [32'h0000_0040 : 32'h0000_0100] := 35, // Medium tolerance (tol_med coverage bin)
            [32'h0000_0101 : 32'h0000_0800] := 30  // Loose tolerance (tol_loose coverage bin)
        };
    }

    constraint c_alpha {
        step_alpha dist {
            32'h0000_0000 := 5,  // Exercises RTL fallback to Q16_ONE (1.0)
            32'h0001_0000 := 35, // 1.0 (full_step)
            32'h0000_C000 := 15, // 0.75
            32'h0000_8000 := 25, // 0.5 (half_step)
            32'h0000_4000 := 10, // 0.25
            32'h0000_2000 := 10  // 0.125
        };
    }

    // Denominator Safety Floor Threshold (Clamping Limit)
    constraint c_lambda {
        lambda_reg dist {
            32'h0000_0000                   := 5,  // Exercises RTL fallback to Q16_LAMBDA_DEF
            [32'h0000_0080 : 32'h0000_0100] := 35, // Lower clamp range (0.002 - 0.004)
            [32'h0000_0101 : 32'h0000_0200] := 35, // Medium clamp range (0.004 - 0.008)
            [32'h0000_0201 : 32'h0000_0400] := 25  // Upper clamp range (0.008 - 0.016)
        };
    }

    constraint c_max_iters {
        max_iters dist {
            8'd0           := 5,  // Exercises RTL fallback to 8'd50
            [8'd5  : 8'd10]:= 20, // Low iteration limits (it_low bin)
            [8'd11 : 8'd40]:= 45, // Medium iteration limits (it_med bin)
            [8'd41 : 8'd80]:= 30  // High iteration limits (it_high bin)
        };
    }

    constraint c_x_init {
        x_init dist {
            [-32'sd3276800 : -32'sd655360] := 20, // neg_large (< -10.0)
            [-32'sd655359  : -32'sd65536]  := 20, // neg_small (-10.0 to -1.0)
            [-32'sd65535   :  32'sd65535]  := 25, // zero_near (-1.0 to +1.0)
            [ 32'sd65536   :  32'sd655360] := 20, // pos_small (+1.0 to +10.0)
            [ 32'sd655361  :  32'sd3276800] := 15  // pos_large (> +10.0)
        };
    }

    `uvm_object_utils_begin(newton_seq_item)
        `uvm_field_int(prog_length, UVM_ALL_ON | UVM_DEC)
        `uvm_field_int(reprogram,   UVM_ALL_ON | UVM_BIN)
        `uvm_field_int(x_init,      UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(tolerance,   UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(step_alpha,  UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(lambda_reg,  UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(max_iters,   UVM_ALL_ON | UVM_DEC)
        `uvm_field_int(x_optimal,   UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(f_optimal,   UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(g_final,     UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(iter_count,  UVM_ALL_ON | UVM_DEC)
        `uvm_field_enum(status_t, status, UVM_ALL_ON)
    `uvm_object_utils_end

    function new(string name = "newton_seq_item");
        super.new(name);
        prog_length = 0;
        reprogram   = 1'b0;
        tolerance   = Q16_EPS_DEF;
        step_alpha  = Q16_ONE;
        lambda_reg  = Q16_LAMBDA_DEF;
        max_iters   = 8'd50;
        for (int i = 0; i < PROG_DEPTH; i++) begin
            program_mem[i] = '0;
        end
    endfunction

    virtual function string convert2string();
        real r_x_init    = real'(x_init) / 65536.0;
        real r_x_opt     = real'(x_optimal) / 65536.0;
        real r_f_opt     = real'(f_optimal) / 65536.0;
        real r_g_final   = real'(g_final) / 65536.0;
        real r_alpha     = real'(step_alpha) / 65536.0;
        real r_tol       = real'(tolerance) / 65536.0;

        return $sformatf("x_init=%0.4f (0x%08h), tol=%0.5f, alpha=%0.4f, max_iters=%0d | RES: status=%s, iters=%0d, x*=%0.4f (0x%08h), f(x*)=%0.4f, g=%0.4f",
                         r_x_init, x_init, r_tol, r_alpha, max_iters, status.name(), iter_count, r_x_opt, x_optimal, r_f_opt, r_g_final);
    endfunction

endclass : newton_seq_item

`endif // NEWTON_SEQ_ITEM_SV
