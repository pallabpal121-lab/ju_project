// =============================================================================
// File Name   : newton_base_seq.sv
// Class Name  : newton_base_seq
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// Description : Base sequence providing reusable high-level optimization
//               routines and low-level AXI register access tasks.
// =============================================================================

`ifndef NEWTON_BASE_SEQ_SV
`define NEWTON_BASE_SEQ_SV

class newton_base_seq extends uvm_sequence #(newton_axi_seq_item);
    `uvm_object_utils(newton_base_seq)

    function new(string name = "newton_base_seq");
        super.new(name);
    endfunction

    // -------------------------------------------------------------------------
    // Helper Task: Low-Level AXI Register Write
    // -------------------------------------------------------------------------
    virtual task write_reg(input logic [11:0] addr, input logic [31:0] data, input bit delay_rdy = 0);
        newton_axi_seq_item item;
        item = newton_axi_seq_item::type_id::create("axi_write_item");
        start_item(item);
        item.is_raw_axi   = 1'b1;
        item.axi_op       = newton_axi_seq_item::AXI_WRITE;
        item.axi_addr     = addr;
        item.axi_data     = data;
        item.delay_ready  = delay_rdy;
        finish_item(item);
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Low-Level AXI Register Read
    // -------------------------------------------------------------------------
    virtual task read_reg(input logic [11:0] addr, output logic [31:0] data, output logic [1:0] resp, input bit delay_rdy = 0);
        newton_axi_seq_item item;
        item = newton_axi_seq_item::type_id::create("axi_read_item");
        start_item(item);
        item.is_raw_axi   = 1'b1;
        item.axi_op       = newton_axi_seq_item::AXI_READ;
        item.axi_addr     = addr;
        item.delay_ready  = delay_rdy;
        finish_item(item);
        data = item.axi_data;
        resp = item.axi_resp;
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Extended AXI Write (prot and wstrb support)
    // -------------------------------------------------------------------------
    virtual task write_reg_ext(input logic [11:0] addr, input logic [31:0] data, input bit delay_rdy = 0, input logic [2:0] prot = 3'b000, input logic [3:0] wstrb = 4'hF);
        newton_axi_seq_item item;
        item = newton_axi_seq_item::type_id::create("axi_write_ext_item");
        start_item(item);
        item.is_raw_axi   = 1'b1;
        item.axi_op       = newton_axi_seq_item::AXI_WRITE;
        item.axi_addr     = addr;
        item.axi_data     = data;
        item.delay_ready  = delay_rdy;
        item.axi_prot     = prot;
        item.axi_wstrb    = wstrb;
        finish_item(item);
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Extended AXI Read (prot support)
    // -------------------------------------------------------------------------
    virtual task read_reg_ext(input logic [11:0] addr, output logic [31:0] data, output logic [1:0] resp, input bit delay_rdy = 0, input logic [2:0] prot = 3'b000);
        newton_axi_seq_item item;
        item = newton_axi_seq_item::type_id::create("axi_read_ext_item");
        start_item(item);
        item.is_raw_axi   = 1'b1;
        item.axi_op       = newton_axi_seq_item::AXI_READ;
        item.axi_addr     = addr;
        item.delay_ready  = delay_rdy;
        item.axi_prot     = prot;
        finish_item(item);
        data = item.axi_data;
        resp = item.axi_resp;
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Split AW/W AXI Write (AWVALID before WVALID)
    // -------------------------------------------------------------------------
    virtual task write_reg_split(input logic [11:0] addr, input logic [31:0] data);
        newton_axi_seq_item item;
        item = newton_axi_seq_item::type_id::create("axi_write_split_item");
        start_item(item);
        item.is_raw_axi    = 1'b1;
        item.axi_op        = newton_axi_seq_item::AXI_WRITE;
        item.axi_addr      = addr;
        item.axi_data      = data;
        item.transfer_mode = newton_axi_seq_item::TRANSFER_SPLIT_AW;
        finish_item(item);
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Split AW/W AXI Write with Mid-Flight Reset during W_DATA
    // -------------------------------------------------------------------------
    virtual task write_reg_split_reset(input logic [11:0] addr, input logic [31:0] data);
        newton_axi_seq_item item;
        item = newton_axi_seq_item::type_id::create("axi_write_split_reset_item");
        start_item(item);
        item.is_raw_axi    = 1'b1;
        item.axi_op        = newton_axi_seq_item::AXI_WRITE;
        item.axi_addr      = addr;
        item.axi_data      = data;
        item.transfer_mode = newton_axi_seq_item::TRANSFER_RESET_WDATA;
        finish_item(item);
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Pipelined AXI Write (Hold AW/W stable while slave busy)
    // -------------------------------------------------------------------------
    virtual task write_reg_pipelined(input logic [11:0] addr, input logic [31:0] data);
        newton_axi_seq_item item;
        item = newton_axi_seq_item::type_id::create("axi_write_pipelined_item");
        start_item(item);
        item.is_raw_axi    = 1'b1;
        item.axi_op        = newton_axi_seq_item::AXI_WRITE;
        item.axi_addr      = addr;
        item.axi_data      = data;
        item.transfer_mode = newton_axi_seq_item::TRANSFER_PIPELINE;
        finish_item(item);
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Pipelined AXI Read (Hold AR stable while slave busy)
    // -------------------------------------------------------------------------
    virtual task read_reg_pipelined(input logic [11:0] addr, output logic [31:0] data, output logic [1:0] resp);
        newton_axi_seq_item item;
        item = newton_axi_seq_item::type_id::create("axi_read_pipelined_item");
        start_item(item);
        item.is_raw_axi    = 1'b1;
        item.axi_op        = newton_axi_seq_item::AXI_READ;
        item.axi_addr      = addr;
        item.transfer_mode = newton_axi_seq_item::TRANSFER_PIPELINE;
        finish_item(item);
        data = item.axi_data;
        resp = item.axi_resp;
    endtask

    // -------------------------------------------------------------------------
    // Helper: Build Quadratic Bowl Microcode for arbitrary N variables
    // -------------------------------------------------------------------------
    function void build_quad_microcode(output instr_t prog[PROG_DEPTH], output int len, input int n);
        int idx = 0;
        prog[idx++] = '{op: OP_MUL, dst: 5'd16, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        if (n == 2) begin
            prog[idx++] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1, src_b: 5'd1, imm: 13'sd0};
            prog[idx++] = '{op: OP_ADD, dst: 5'd31, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
        end else begin
            prog[idx++] = '{op: OP_MUL, dst: 5'd17, src_a: 5'd1, src_b: 5'd1, imm: 13'sd0};
            prog[idx++] = '{op: OP_ADD, dst: 5'd18, src_a: 5'd16, src_b: 5'd17, imm: 13'sd0};
            for (int i = 2; i < n; i++) begin
                prog[idx++] = '{op: OP_MUL, dst: 5'd16, src_a: 5'(i), src_b: 5'(i), imm: 13'sd0};
                prog[idx++] = '{op: OP_ADD, dst: (i == n-1) ? 5'd31 : 5'd18, src_a: 5'd18, src_b: 5'd16, imm: 13'sd0};
            end
        end
        prog[idx++] = '{op: OP_END, dst: 5'd0, src_a: 5'd0, src_b: 5'd0, imm: 13'sd0};
        len = idx;
    endfunction

    // -------------------------------------------------------------------------
    // Helper Task: Load Microcode into DFG Engine
    // -------------------------------------------------------------------------
    virtual task load_microcode(const ref instr_t prog[PROG_DEPTH], input int len);
        for (int i = 0; i < len; i++) begin
            write_reg(AXI_REG_PROG_ADDR, 32'(i));
            write_reg(AXI_REG_PROG_DATA, prog[i]);
        end
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Write Initial State Vector
    // -------------------------------------------------------------------------
    virtual task write_state_vector(const ref q16_t x[MAX_VARS], input int num_v);
        for (int i = 0; i < num_v; i++) begin
            write_reg(AXI_STATE_VEC_BASE + (i * 4), x[i]);
        end
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Read Optimal State Vector
    // -------------------------------------------------------------------------
    virtual task read_state_vector(output q16_t x[MAX_VARS], input int num_v);
        logic [31:0] data;
        logic [1:0]  resp;
        for (int i = 0; i < num_v; i++) begin
            read_reg(AXI_STATE_VEC_BASE + (i * 4), data, resp);
            x[i] = data;
        end
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Configure Solver Hyperparameters
    // -------------------------------------------------------------------------
    virtual task configure_optimizer(
        input logic [4:0] num_v,
        input q16_t       tol,
        input q16_t       alpha,
        input q16_t       lambda_val,
        input logic [7:0] max_sw
    );
        write_reg(AXI_REG_NUM_VARS,   32'(num_v));
        write_reg(AXI_REG_TOLERANCE,  tol);
        write_reg(AXI_REG_ALPHA,      alpha);
        write_reg(AXI_REG_LAMBDA,     lambda_val);
        write_reg(AXI_REG_MAX_SWEEPS, 32'(max_sw));
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Trigger Optimization Start
    // -------------------------------------------------------------------------
    virtual task trigger_start();
        write_reg(AXI_REG_CTRL, CTRL_START_MASK);
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Poll Solver Status
    // -------------------------------------------------------------------------
    virtual task poll_status(output logic [31:0] status_val);
        logic [1:0] resp;
        read_reg(AXI_REG_STATUS, status_val, resp);
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Exercise Complete AXI Register Map & Unmapped Addresses
    // -------------------------------------------------------------------------
    virtual task ping_axi_bus_map();
        logic [31:0] rdata;
        logic [1:0]  resp;

        // Reads on configuration & status registers
        read_reg(AXI_REG_CTRL,        rdata, resp);
        read_reg(AXI_REG_STATUS,      rdata, resp);
        read_reg(AXI_REG_NUM_VARS,    rdata, resp);
        read_reg(AXI_REG_TOLERANCE,   rdata, resp);
        read_reg(AXI_REG_ALPHA,       rdata, resp);
        read_reg(AXI_REG_LAMBDA,      rdata, resp);
        read_reg(AXI_REG_MAX_SWEEPS,  rdata, resp);
        read_reg(AXI_REG_SWEEP_COUNT, rdata, resp);
        read_reg(AXI_REG_F_OPTIMAL,   rdata, resp);
        read_reg(AXI_REG_MAX_DELTA,   rdata, resp);
        read_reg(AXI_REG_PROG_ADDR,   rdata, resp);

        // State vector window access
        read_reg(AXI_STATE_VEC_BASE,  rdata, resp);
        write_reg(AXI_STATE_VEC_BASE, 32'h0001_0000);

        // Unmapped address accesses
        read_reg(12'h048, rdata, resp);
        read_reg(12'h800, rdata, resp);
        read_reg(12'hFFC, rdata, resp);
        write_reg(12'h048, 32'hDEAD_BEEF);
        write_reg(12'h800, 32'hDEAD_BEEF);
        write_reg(12'hFFC, 32'hDEAD_BEEF);
    endtask

    // -------------------------------------------------------------------------
    // Helper Task: Execute Full High-Level Optimization Item
    // -------------------------------------------------------------------------
    virtual task execute_optimization(newton_axi_seq_item item);
        start_item(item);
        finish_item(item);
    endtask

endclass : newton_base_seq

`endif // NEWTON_BASE_SEQ_SV
