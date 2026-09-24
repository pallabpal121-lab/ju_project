// =============================================================================
// File Name   : newton_axi_if.sv
// Module Name : newton_axi_if (SystemVerilog AXI4-Lite & IRQ Interface)
// Project     : Universal Multivariable Newton BCD Accelerator UVM TB
// =============================================================================

`timescale 1ns / 1ps

interface newton_axi_if (
    input logic s_axi_aclk,
    input logic s_axi_aresetn
);
    import uvm_pkg::*;
    `include "uvm_macros.svh"
    import newton_multivar_pkg::*;
    import newton_axi_regs_pkg::*;

    // Controllable reset force for mid-flight reset verification
    logic rst_force_n = 1'b1;
    wire  s_axi_aresetn_eff = s_axi_aresetn & rst_force_n;

    // -------------------------------------------------------------------------
    // AXI4-Lite Write Channels
    // -------------------------------------------------------------------------
    logic [11:0] s_axi_awaddr;
    logic [2:0]  s_axi_awprot;
    logic        s_axi_awvalid;
    logic        s_axi_awready;

    logic [31:0] s_axi_wdata;
    logic [3:0]  s_axi_wstrb;
    logic        s_axi_wvalid;
    logic        s_axi_wready;

    logic [1:0]  s_axi_bresp;
    logic        s_axi_bvalid;
    logic        s_axi_bready;

    // -------------------------------------------------------------------------
    // AXI4-Lite Read Channels
    // -------------------------------------------------------------------------
    logic [11:0] s_axi_araddr;
    logic [2:0]  s_axi_arprot;
    logic        s_axi_arvalid;
    logic        s_axi_arready;

    logic [31:0] s_axi_rdata;
    logic [1:0]  s_axi_rresp;
    logic        s_axi_rvalid;
    logic        s_axi_rready;

    // -------------------------------------------------------------------------
    // Interrupt Output
    // -------------------------------------------------------------------------
    logic        irq_done;

    // -------------------------------------------------------------------------
    // Clocking Block: Driver
    // -------------------------------------------------------------------------
    clocking drv_cb @(posedge s_axi_aclk);
        default input #1ns output #1ns;
        output s_axi_awaddr;
        output s_axi_awprot;
        output s_axi_awvalid;
        input  s_axi_awready;

        output s_axi_wdata;
        output s_axi_wstrb;
        output s_axi_wvalid;
        input  s_axi_wready;

        input  s_axi_bresp;
        input  s_axi_bvalid;
        output s_axi_bready;

        output s_axi_araddr;
        output s_axi_arprot;
        output s_axi_arvalid;
        input  s_axi_arready;

        input  s_axi_rdata;
        input  s_axi_rresp;
        input  s_axi_rvalid;
        output s_axi_rready;

        input  irq_done;
        output rst_force_n;
    endclocking

    // -------------------------------------------------------------------------
    // Clocking Block: Monitor
    // -------------------------------------------------------------------------
    clocking mon_cb @(posedge s_axi_aclk);
        default input #1ns output #1ns;
        input s_axi_awaddr;
        input s_axi_awprot;
        input s_axi_awvalid;
        input s_axi_awready;

        input s_axi_wdata;
        input s_axi_wstrb;
        input s_axi_wvalid;
        input s_axi_wready;

        input s_axi_bresp;
        input s_axi_bvalid;
        input s_axi_bready;

        input s_axi_araddr;
        input s_axi_arprot;
        input s_axi_arvalid;
        input s_axi_arready;

        input s_axi_rdata;
        input s_axi_rresp;
        input s_axi_rvalid;
        input s_axi_rready;

        input irq_done;
    endclocking

    modport drv_mp (clocking drv_cb, input s_axi_aclk, input s_axi_aresetn);
    modport mon_mp (clocking mon_cb, input s_axi_aclk, input s_axi_aresetn);

    // =========================================================================
    // Industry-Grade Protocol Assertions (SVA)
    // =========================================================================

    // 1. Reset check: Bus valid/ready and IRQ signals must remain deasserted during reset
    property p_reset_state;
        @(posedge s_axi_aclk)
        !s_axi_aresetn_eff |-> (!s_axi_awready && !s_axi_wready && !s_axi_bvalid &&
                                !s_axi_arready && !s_axi_rvalid && !irq_done);
    endproperty
    A_RESET_STATE: assert property (p_reset_state)
        else `uvm_error("SVA_AXI", "Protocol violation: Bus control or IRQ asserted during active reset!")

    // 2. Write address stability: awaddr & awprot must remain stable until awready handshake
    property p_aw_stable;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_awvalid && !s_axi_awready |=> s_axi_awvalid && $stable(s_axi_awaddr) && $stable(s_axi_awprot);
    endproperty
    A_AW_STABLE: assert property (p_aw_stable)
        else `uvm_error("SVA_AXI", "Protocol violation: AWADDR or AWPROT changed while AWVALID was asserted without AWREADY!")

    // 3. Write data stability: wdata & wstrb must remain stable until wready handshake
    property p_w_stable;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_wvalid && !s_axi_wready |=> s_axi_wvalid && $stable(s_axi_wdata) && $stable(s_axi_wstrb);
    endproperty
    A_W_STABLE: assert property (p_w_stable)
        else `uvm_error("SVA_AXI", "Protocol violation: WDATA or WSTRB changed while WVALID was asserted without WREADY!")

    // 4. Write response stability: bvalid & bresp must remain stable until bready handshake
    property p_b_stable;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_bvalid && !s_axi_bready |=> s_axi_bvalid && $stable(s_axi_bresp);
    endproperty
    A_B_STABLE: assert property (p_b_stable)
        else `uvm_error("SVA_AXI", "Protocol violation: BRESP or BVALID changed while waiting for BREADY!")

    // 5. Read address stability: araddr & arprot must remain stable until arready handshake
    property p_ar_stable;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_arvalid && !s_axi_arready |=> s_axi_arvalid && $stable(s_axi_araddr) && $stable(s_axi_arprot);
    endproperty
    A_AR_STABLE: assert property (p_ar_stable)
        else `uvm_error("SVA_AXI", "Protocol violation: ARADDR or ARPROT changed while ARVALID was asserted without ARREADY!")

    // 6. Read data stability: rvalid, rdata, and rresp must remain stable until rready handshake
    property p_r_stable;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_rvalid && !s_axi_rready |=> s_axi_rvalid && $stable(s_axi_rdata) && $stable(s_axi_rresp);
    endproperty
    A_R_STABLE: assert property (p_r_stable)
        else `uvm_error("SVA_AXI", "Protocol violation: RDATA, RRESP or RVALID changed while waiting for RREADY!")

    // 7. Data integrity: Unknown checks on handshakes
    property p_no_x_on_awvalid;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_awvalid |-> !$isunknown({s_axi_awaddr, s_axi_awprot});
    endproperty
    A_NO_X_AWVALID: assert property (p_no_x_on_awvalid)
        else `uvm_error("SVA_AXI", "Protocol violation: Unknown (X/Z) value detected on AW bus when AWVALID is high!")

    property p_no_x_on_wvalid;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_wvalid |-> !$isunknown({s_axi_wdata, s_axi_wstrb});
    endproperty
    A_NO_X_WVALID: assert property (p_no_x_on_wvalid)
        else `uvm_error("SVA_AXI", "Protocol violation: Unknown (X/Z) value detected on W bus when WVALID is high!")

    property p_no_x_on_arvalid;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_arvalid |-> !$isunknown({s_axi_araddr, s_axi_arprot});
    endproperty
    A_NO_X_ARVALID: assert property (p_no_x_on_arvalid)
        else `uvm_error("SVA_AXI", "Protocol violation: Unknown (X/Z) value detected on AR bus when ARVALID is high!")

    property p_no_x_on_rvalid;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_rvalid |-> !$isunknown({s_axi_rdata, s_axi_rresp});
    endproperty
    A_NO_X_RVALID: assert property (p_no_x_on_rvalid)
        else `uvm_error("SVA_AXI", "Protocol violation: Unknown (X/Z) value detected on R bus when RVALID is high!")

    // 8. Interrupt check: irq_done must be 1-cycle pulse
    property p_irq_pulse;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        irq_done |=> !irq_done;
    endproperty
    A_IRQ_PULSE: assert property (p_irq_pulse)
        else `uvm_error("SVA_AXI", "Protocol violation: irq_done remained asserted for more than 1 clock cycle!")

    // 9. Tapeout-Grade Address Alignment (4-byte word aligned)
    property p_aw_align;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_awvalid |-> (s_axi_awaddr[1:0] == 2'b00);
    endproperty
    A_AW_ALIGN: assert property (p_aw_align)
        else `uvm_error("SVA_AXI", "Protocol violation: s_axi_awaddr is not 4-byte aligned!")

    property p_ar_align;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_arvalid |-> (s_axi_araddr[1:0] == 2'b00);
    endproperty
    A_AR_ALIGN: assert property (p_ar_align)
        else `uvm_error("SVA_AXI", "Protocol violation: s_axi_araddr is not 4-byte aligned!")

    // 10. Tapeout-Grade Legal Response Encoding
    property p_bresp_valid;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_bvalid |-> (s_axi_bresp inside {2'b00, 2'b01, 2'b10, 2'b11});
    endproperty
    A_BRESP_VALID: assert property (p_bresp_valid)
        else `uvm_error("SVA_AXI", "Protocol violation: Illegal BRESP encoding detected!")

    property p_rresp_valid;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_rvalid |-> (s_axi_rresp inside {2'b00, 2'b01, 2'b10, 2'b11});
    endproperty
    A_RRESP_VALID: assert property (p_rresp_valid)
        else `uvm_error("SVA_AXI", "Protocol violation: Illegal RRESP encoding detected!")

    // 11. Tapeout-Grade Reset Recovery Check: Ready lines armed and IRQ low post-reset
    property p_reset_recovery;
        @(posedge s_axi_aclk)
        $rose(s_axi_aresetn_eff) |=> (s_axi_awready && s_axi_wready && s_axi_arready && !irq_done);
    endproperty
    A_RESET_RECOVERY: assert property (p_reset_recovery)
        else `uvm_error("SVA_AXI", "Protocol violation: AXI ready lines not armed or IRQ asserted post-reset!")

    // 12. Write Strobe Validity: WSTRB must be non-zero on active write handshakes
    property p_wstrb_nonzero;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_wvalid && s_axi_wready |-> (s_axi_wstrb != 4'b0000);
    endproperty
    A_WSTRB_NONZERO: assert property (p_wstrb_nonzero)
        else `uvm_error("SVA_AXI", "Protocol violation: WSTRB is 4'b0000 during active write handshake!")

    // 13. Protection signals known check
    property p_prot_known;
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_awvalid |-> !$isunknown(s_axi_awprot);
    endproperty
    A_PROT_KNOWN: assert property (p_prot_known)
        else `uvm_error("SVA_AXI", "Protocol violation: Unknown value detected on AWPROT!")

    // =========================================================================
    // Protocol Coverage Properties
    // =========================================================================
    C_AXI_WRITE_HANDSHAKE: cover property (
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_awvalid && s_axi_awready && s_axi_wvalid && s_axi_wready
    );

    C_AXI_READ_HANDSHAKE: cover property (
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_arvalid && s_axi_arready ##[1:5] s_axi_rvalid && s_axi_rready
    );

    C_AW_HANDSHAKE: cover property (
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_awvalid && s_axi_awready
    );

    C_W_HANDSHAKE: cover property (
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_wvalid && s_axi_wready
    );

    C_B_HANDSHAKE: cover property (
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_bvalid && s_axi_bready
    );

    C_AR_HANDSHAKE: cover property (
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_arvalid && s_axi_arready
    );

    C_R_HANDSHAKE: cover property (
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        s_axi_rvalid && s_axi_rready
    );

    C_IRQ_DONE_TRIGGER: cover property (
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        irq_done
    );

    C_IRQ_DONE_PULSE: cover property (
        @(posedge s_axi_aclk) disable iff (!s_axi_aresetn_eff)
        irq_done ##1 !irq_done
    );

    C_RESET_RECOVERY: cover property (
        @(posedge s_axi_aclk)
        $rose(s_axi_aresetn_eff) ##1 (s_axi_awready && s_axi_wready && s_axi_arready)
    );

endinterface : newton_axi_if

// -----------------------------------------------------------------------------
// Industry Standard: Virtual Interface Type Definition
// -----------------------------------------------------------------------------
typedef virtual newton_axi_if newton_vif_t;
