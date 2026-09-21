// =============================================================================
// File Name   : newton_axi_seq_item.sv
// Class Name  : newton_axi_seq_item
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// =============================================================================

`ifndef NEWTON_AXI_SEQ_ITEM_SV
`define NEWTON_AXI_SEQ_ITEM_SV

class newton_axi_seq_item extends uvm_sequence_item;

    // Operation mode flag
    bit                 is_raw_axi;

    // -------------------------------------------------------------------------
    // Raw AXI4-Lite Transfer Fields
    // -------------------------------------------------------------------------
    typedef enum bit {
        AXI_READ  = 1'b0,
        AXI_WRITE = 1'b1
    } axi_op_e;

    typedef enum bit [1:0] {
        TRANSFER_NORMAL      = 2'b00,
        TRANSFER_SPLIT_AW    = 2'b01,
        TRANSFER_PIPELINE    = 2'b10,
        TRANSFER_RESET_WDATA = 2'b11
    } transfer_mode_e;

    rand axi_op_e       axi_op;
    rand logic [11:0]   axi_addr;
    rand logic [31:0]   axi_data;
    rand logic [2:0]    axi_prot;
    rand logic [3:0]    axi_wstrb;
    logic [1:0]         axi_resp;
    bit                 delay_ready;
    transfer_mode_e     transfer_mode;
    bit                 do_mid_reset;
    int                 reset_delay_cycles;

    // -------------------------------------------------------------------------
    // High-Level Newton Optimization Scenario Fields
    // -------------------------------------------------------------------------
    // Microcode Program Payload
    instr_t             program_mem [PROG_DEPTH];
    int                 prog_length;
    bit                 reprogram;

    // Optimization Input Parameters
    rand logic [4:0]    num_vars;
    rand q16_t          x_init [MAX_VARS];
    rand q16_t          tolerance;
    rand q16_t          step_alpha;
    rand q16_t          lambda_reg;
    rand logic [7:0]    max_sweeps;

    // Output Response (Sampled from AXI readouts after completion)
    q16_t               x_optimal [MAX_VARS];
    q16_t               f_optimal;
    q16_t               max_delta_last;
    logic [7:0]         sweep_count;
    status_t            status;

    // -------------------------------------------------------------------------
    // Constraints
    // -------------------------------------------------------------------------
    constraint c_num_vars {
        num_vars inside {[5'd2 : 5'd16]};
    }

    constraint c_tol {
        tolerance dist {
            32'h0000_0000                   := 5,  // Exercises RTL fallback to Q16_EPS_DEF
            [32'h0000_0010 : 32'h0000_003F] := 30, // Tight tolerance
            [32'h0000_0040 : 32'h0000_0100] := 35, // Medium tolerance
            [32'h0000_0101 : 32'h0000_0800] := 30  // Loose tolerance
        };
    }

    constraint c_alpha {
        step_alpha dist {
            32'h0000_0000 := 5,  // Exercises RTL fallback to Q16_ONE (1.0)
            32'h0001_0000 := 40, // 1.0 (full step)
            32'h0000_C000 := 15, // 0.75
            32'h0000_8000 := 25, // 0.5 (half step)
            32'h0000_4000 := 15  // 0.25
        };
    }

    constraint c_lambda {
        lambda_reg dist {
            32'h0000_0000                   := 5,  // Exercises RTL fallback to Q16_LAMBDA_DEF
            [32'h0000_0100 : 32'h0000_0400] := 50, // Standard damping (0.0039 - 0.0156)
            [32'h0000_0401 : 32'h0000_1000] := 45  // Heavy damping (0.0156 - 0.0625)
        };
    }

    constraint c_max_sweeps {
        max_sweeps dist {
            8'd0           := 5,  // Exercises RTL fallback to 8'd50
            [8'd2  : 8'd10]:= 35, // Low sweep limits
            [8'd11 : 8'd30]:= 40, // Medium sweep limits
            [8'd31 : 8'd50]:= 20  // High sweep limits
        };
    }

    constraint c_x_init {
        foreach (x_init[i]) {
            x_init[i] dist {
                [-32'sd327680 : -32'sd65536] := 25, // [-5.0, -1.0]
                [-32'sd65535  :  32'sd65535] := 40, // [-1.0, +1.0]
                [ 32'sd65536  :  32'sd327680] := 25, // [+1.0, +5.0]
                32'sd0                       := 10  // Exactly 0.0
            };
        }
    }

    `uvm_object_utils_begin(newton_axi_seq_item)
        `uvm_field_int(is_raw_axi,     UVM_ALL_ON | UVM_BIN)
        `uvm_field_enum(axi_op_e, axi_op, UVM_ALL_ON)
        `uvm_field_int(axi_addr,       UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(axi_data,       UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(axi_resp,       UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(prog_length,    UVM_ALL_ON | UVM_DEC)
        `uvm_field_int(reprogram,      UVM_ALL_ON | UVM_BIN)
        `uvm_field_int(num_vars,       UVM_ALL_ON | UVM_DEC)
        `uvm_field_sarray_int(x_init,  UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(tolerance,      UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(step_alpha,     UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(lambda_reg,     UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(max_sweeps,     UVM_ALL_ON | UVM_DEC)
        `uvm_field_sarray_int(x_optimal, UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(f_optimal,      UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(max_delta_last, UVM_ALL_ON | UVM_HEX)
        `uvm_field_int(sweep_count,    UVM_ALL_ON | UVM_DEC)
        `uvm_field_enum(status_t, status, UVM_ALL_ON)
    `uvm_object_utils_end

    function new(string name = "newton_axi_seq_item");
        super.new(name);
        is_raw_axi     = 1'b0;
        axi_op         = AXI_READ;
        axi_addr       = 12'h000;
        axi_data       = 32'h0000_0000;
        axi_prot       = 3'b000;
        axi_wstrb      = 4'hF;
        axi_resp       = 2'b00;
        do_mid_reset   = 1'b0;
        reset_delay_cycles = 0;
        prog_length    = 0;
        reprogram      = 1'b0;
        num_vars       = 5'd2;
        tolerance      = Q16_EPS_DEF;
        step_alpha     = Q16_ONE;
        lambda_reg     = Q16_LAMBDA_DEF;
        max_sweeps     = 8'd20;
        f_optimal      = Q16_ZERO;
        max_delta_last = Q16_ZERO;
        sweep_count    = 8'd0;
        status         = STATUS_IDLE;

        for (int i = 0; i < PROG_DEPTH; i++) program_mem[i] = '0;
        for (int i = 0; i < MAX_VARS; i++) begin
            x_init[i]    = Q16_ZERO;
            x_optimal[i] = Q16_ZERO;
        end
    endfunction

    virtual function string convert2string();
        if (is_raw_axi) begin
            return $sformatf("RAW AXI %s: addr=0x%03h, data=0x%08h, resp=2'b%02b",
                             (axi_op == AXI_WRITE) ? "WRITE" : "READ", axi_addr, axi_data, axi_resp);
        end else begin
            string s;
            s = $sformatf("NEWTON BCD: N=%0d, tol=0x%08h, alpha=0x%08h, max_sw=%0d | RES: status=%s, sweeps=%0d, f*=0x%08h (%0.4f), max_delta=0x%08h\n",
                          num_vars, tolerance, step_alpha, max_sweeps, status.name(), sweep_count,
                          f_optimal, real'(f_optimal) / 65536.0, max_delta_last);
            s = {s, "    x_init   = ["};
            for (int i = 0; i < num_vars; i++) begin
                s = {s, $sformatf("%0.4f%s", real'(x_init[i]) / 65536.0, (i == num_vars-1) ? "" : ", ")};
            end
            s = {s, "]\n    x_optimal= ["};
            for (int i = 0; i < num_vars; i++) begin
                s = {s, $sformatf("%0.4f%s", real'(x_optimal[i]) / 65536.0, (i == num_vars-1) ? "" : ", ")};
            end
            s = {s, "]"};
            return s;
        end
    endfunction

endclass : newton_axi_seq_item

`endif // NEWTON_AXI_SEQ_ITEM_SV
