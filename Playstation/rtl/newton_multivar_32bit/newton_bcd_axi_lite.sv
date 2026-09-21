// =============================================================================
// File Name   : newton_bcd_axi_lite.sv
// Module Name : newton_bcd_axi_lite
// Project     : Universal Multivariable Newton 2nd-Order Accelerator
// -----------------------------------------------------------------------------
// Description: Standard 32-bit AXI4-Lite Slave SoC Wrapper for the
//              2-Variable Block Coordinate Descent (BCD) Newton Accelerator.
//              Provides memory-mapped register access for host CPUs (RISC-V/ARM).
// =============================================================================

`timescale 1ns / 1ps

import newton_multivar_pkg::*;
`include "multivar_helpers.svh"
`include "newton_bcd_axi_regs.svh"

module newton_bcd_axi_lite (
    // Clock & Reset
    input  logic               s_axi_aclk,
    input  logic               s_axi_aresetn,

    // Write Address Channel
    input  logic [11:0]        s_axi_awaddr,
    input  logic [2:0]         s_axi_awprot,
    input  logic               s_axi_awvalid,
    output logic               s_axi_awready,

    // Write Data Channel
    input  logic [31:0]        s_axi_wdata,
    input  logic [3:0]         s_axi_wstrb,
    input  logic               s_axi_wvalid,
    output logic               s_axi_wready,

    // Write Response Channel
    output logic [1:0]         s_axi_bresp,
    output logic               s_axi_bvalid,
    input  logic               s_axi_bready,

    // Read Address Channel
    input  logic [11:0]        s_axi_araddr,
    input  logic [2:0]         s_axi_arprot,
    input  logic               s_axi_arvalid,
    output logic               s_axi_arready,

    // Read Data Channel
    output logic [31:0]        s_axi_rdata,
    output logic [1:0]         s_axi_rresp,
    output logic               s_axi_rvalid,
    input  logic               s_axi_rready,

    // Interrupt Output
    output logic               irq_done
);

    // -------------------------------------------------------------------------
    // Register Address Map Offsets (Imported from newton_bcd_axi_regs.svh)
    // -------------------------------------------------------------------------
    localparam logic [11:0] ADDR_REG_CTRL        = AXI_REG_CTRL;
    localparam logic [11:0] ADDR_REG_STATUS      = AXI_REG_STATUS;
    localparam logic [11:0] ADDR_REG_NUM_VARS    = AXI_REG_NUM_VARS;
    localparam logic [11:0] ADDR_REG_TOLERANCE   = AXI_REG_TOLERANCE;
    localparam logic [11:0] ADDR_REG_ALPHA       = AXI_REG_ALPHA;
    localparam logic [11:0] ADDR_REG_LAMBDA      = AXI_REG_LAMBDA;
    localparam logic [11:0] ADDR_REG_MAX_SWEEPS  = AXI_REG_MAX_SWEEPS;
    localparam logic [11:0] ADDR_REG_SWEEP_COUNT = AXI_REG_SWEEP_COUNT;
    localparam logic [11:0] ADDR_REG_F_OPTIMAL   = AXI_REG_F_OPTIMAL;
    localparam logic [11:0] ADDR_REG_MAX_DELTA   = AXI_REG_MAX_DELTA;
    localparam logic [11:0] ADDR_REG_PROG_ADDR   = AXI_REG_PROG_ADDR;
    localparam logic [11:0] ADDR_REG_PROG_DATA   = AXI_REG_PROG_DATA;
    localparam logic [11:0] ADDR_STATE_VEC_BASE  = AXI_STATE_VEC_BASE; // 0x100 to 0x13C for 16 vars

    // Internal Control / Status Registers
    logic                      reg_start_pulse;
    logic [NUM_VARS_BITS-1:0]  reg_num_vars;
    q16_t                      reg_tolerance;
    q16_t                      reg_alpha;
    q16_t                      reg_lambda;
    logic [7:0]                reg_max_sweeps;
    vec_t                      reg_x_init;

    // Microcode programming port registers
    logic                      reg_prog_en;
    logic [$clog2(PROG_DEPTH)-1:0] reg_prog_addr;
    instr_t                    reg_prog_data;

    // Core Interconnect
    vec_t                      core_x_optimal;
    q16_t                      core_f_optimal;
    q16_t                      core_max_delta_last;
    logic [7:0]                core_sweep_count;
    status_t                   core_status;
    logic                      core_done;
    logic                      core_busy;

    // AXI Handshake Registers
    logic                      aw_ready_reg;
    logic                      w_ready_reg;
    logic                      b_valid_reg;
    logic [11:0]               aw_addr_latched;
    logic                      ar_ready_reg;
    logic                      r_valid_reg;
    logic [31:0]               r_data_reg;
    logic [11:0]               ar_addr_latched;

    assign s_axi_awready = aw_ready_reg;
    assign s_axi_wready  = w_ready_reg;
    assign s_axi_bvalid  = b_valid_reg;
    assign s_axi_bresp   = 2'b00; // OKAY
    assign s_axi_arready = ar_ready_reg;
    assign s_axi_rvalid  = r_valid_reg;
    assign s_axi_rdata   = r_data_reg;
    assign s_axi_rresp   = 2'b00; // OKAY

    assign irq_done      = core_done;

    // Instantiate 2-Variable BCD Optimizer Core
    newton_bcd_top u_bcd_core (
        .clk           (s_axi_aclk),
        .rst_n         (s_axi_aresetn),
        .prog_en       (reg_prog_en),
        .prog_addr     (reg_prog_addr),
        .prog_data     (reg_prog_data),
        .start         (reg_start_pulse),
        .num_vars      (reg_num_vars),
        .x_init        (reg_x_init),
        .tolerance     (reg_tolerance),
        .step_alpha    (reg_alpha),
        .lambda_reg    (reg_lambda),
        .max_sweeps    (reg_max_sweeps),
        .x_optimal     (core_x_optimal),
        .f_optimal     (core_f_optimal),
        .max_delta_last(core_max_delta_last),
        .sweep_count   (core_sweep_count),
        .status        (core_status),
        .done          (core_done),
        .busy          (core_busy)
    );

    // -------------------------------------------------------------------------
    // AXI4-Lite Write Channel State Machine
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        W_IDLE = 2'd0,
        W_DATA = 2'd1,
        W_RESP = 2'd2
    } w_state_t;

    w_state_t w_state;

    always_ff @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            w_state         <= W_IDLE;
            aw_ready_reg    <= 1'b0;
            w_ready_reg     <= 1'b0;
            b_valid_reg     <= 1'b0;
            aw_addr_latched <= '0;
            reg_start_pulse <= 1'b0;
            reg_prog_en     <= 1'b0;
            reg_prog_addr   <= '0;
            reg_prog_data   <= '0;
            reg_num_vars    <= 5'd16;
            reg_tolerance   <= Q16_EPS_DEF;
            reg_alpha       <= Q16_ONE;
            reg_lambda      <= Q16_LAMBDA_DEF;
            reg_max_sweeps  <= 8'd20;
            reg_x_init      <= '0;
        end else begin
            reg_start_pulse <= 1'b0;
            reg_prog_en     <= 1'b0;

            case (w_state)
                W_IDLE: begin
                    aw_ready_reg <= 1'b1;
                    w_ready_reg  <= 1'b1;
                    if (s_axi_awvalid && aw_ready_reg) begin
                        aw_addr_latched <= s_axi_awaddr;
                        aw_ready_reg    <= 1'b0;
                        if (s_axi_wvalid && w_ready_reg) begin
                            w_ready_reg <= 1'b0;
                            b_valid_reg <= 1'b1;
                            w_state     <= W_RESP;
                            // Write execution
                            execute_write(s_axi_awaddr, s_axi_wdata);
                        end else begin
                            w_state <= W_DATA;
                        end
                    end
                end

                W_DATA: begin
                    if (s_axi_wvalid) begin
                        w_ready_reg <= 1'b0;
                        b_valid_reg <= 1'b1;
                        w_state     <= W_RESP;
                        execute_write(aw_addr_latched, s_axi_wdata);
                    end
                end

                W_RESP: begin
                    if (s_axi_bready) begin
                        b_valid_reg  <= 1'b0;
                        aw_ready_reg <= 1'b1;
                        w_ready_reg  <= 1'b1;
                        w_state      <= W_IDLE;
                    end
                end

                default: w_state <= W_IDLE;
            endcase
        end
    end

    task automatic execute_write(input logic [11:0] addr, input logic [31:0] data);
        if (addr == ADDR_REG_CTRL) begin
            if (data[0]) reg_start_pulse <= 1'b1;
        end else if (addr == ADDR_REG_NUM_VARS) begin
            reg_num_vars <= data[NUM_VARS_BITS-1:0];
        end else if (addr == ADDR_REG_TOLERANCE) begin
            reg_tolerance <= data;
        end else if (addr == ADDR_REG_ALPHA) begin
            reg_alpha <= data;
        end else if (addr == ADDR_REG_LAMBDA) begin
            reg_lambda <= data;
        end else if (addr == ADDR_REG_MAX_SWEEPS) begin
            reg_max_sweeps <= data[7:0];
        end else if (addr == ADDR_REG_PROG_ADDR) begin
            reg_prog_addr <= data[$clog2(PROG_DEPTH)-1:0];
        end else if (addr == ADDR_REG_PROG_DATA) begin
            reg_prog_data <= data;
            reg_prog_en   <= 1'b1; // Auto-strobe write enable on data write
        end else if (addr >= ADDR_STATE_VEC_BASE && addr < (ADDR_STATE_VEC_BASE + (MAX_VARS * 4))) begin
            // State variable write: x_init[i]
            reg_x_init <= set_vec(reg_x_init, (addr - ADDR_STATE_VEC_BASE) >> 2, data);
        end
    endtask

    // -------------------------------------------------------------------------
    // AXI4-Lite Read Channel State Machine
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        R_IDLE = 2'd0,
        R_READ = 2'd1
    } r_state_t;

    r_state_t r_state;

    always_ff @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            r_state         <= R_IDLE;
            ar_ready_reg    <= 1'b0;
            r_valid_reg     <= 1'b0;
            r_data_reg      <= '0;
            ar_addr_latched <= '0;
        end else begin
            case (r_state)
                R_IDLE: begin
                    ar_ready_reg <= 1'b1;
                    if (s_axi_arvalid && ar_ready_reg) begin
                        ar_addr_latched <= s_axi_araddr;
                        ar_ready_reg    <= 1'b0;
                        r_valid_reg     <= 1'b1;
                        r_data_reg      <= execute_read(s_axi_araddr);
                        r_state         <= R_READ;
                    end
                end

                R_READ: begin
                    if (s_axi_rready) begin
                        r_valid_reg  <= 1'b0;
                        ar_ready_reg <= 1'b1;
                        r_state      <= R_IDLE;
                    end
                end

                default: r_state <= R_IDLE;
            endcase
        end
    end

    logic status_done_latch;

    always_ff @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            status_done_latch <= 1'b0;
        end else begin
            if (reg_start_pulse) begin
                status_done_latch <= 1'b0;
            end else if (core_done) begin
                status_done_latch <= 1'b1;
            end
        end
    end

    function automatic logic [31:0] execute_read(input logic [11:0] addr);
        if (addr == ADDR_REG_CTRL) begin
            return 32'd0;
        end else if (addr == ADDR_REG_STATUS) begin
            // [0: busy, 1: done, 4:2: status_code]
            return {27'd0, core_status, status_done_latch, core_busy};
        end else if (addr == ADDR_REG_NUM_VARS) begin
            return {{(32-NUM_VARS_BITS){1'b0}}, reg_num_vars};
        end else if (addr == ADDR_REG_TOLERANCE) begin
            return reg_tolerance;
        end else if (addr == ADDR_REG_ALPHA) begin
            return reg_alpha;
        end else if (addr == ADDR_REG_LAMBDA) begin
            return reg_lambda;
        end else if (addr == ADDR_REG_MAX_SWEEPS) begin
            return {24'd0, reg_max_sweeps};
        end else if (addr == ADDR_REG_SWEEP_COUNT) begin
            return {24'd0, core_sweep_count};
        end else if (addr == ADDR_REG_F_OPTIMAL) begin
            return core_f_optimal;
        end else if (addr == ADDR_REG_MAX_DELTA) begin
            return core_max_delta_last;
        end else if (addr >= ADDR_STATE_VEC_BASE && addr < (ADDR_STATE_VEC_BASE + (MAX_VARS * 4))) begin
            // Read optimal variable: x_optimal[i]
            return get_vec(core_x_optimal, (addr - ADDR_STATE_VEC_BASE) >> 2);
        end else begin
            return 32'hDEAD_BEEF;
        end
    endfunction

endmodule
