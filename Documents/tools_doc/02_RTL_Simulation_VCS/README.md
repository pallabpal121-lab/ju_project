# 02_RTL_Simulation_VCS: Logic Verification & Simulation Guide

This folder contains the complete Synopsys VCS simulation collateral, verification architecture guides, bus standards, and testbench examples.

---

## 1. Core Verification Manuals (Sequential Reading Path)

All legacy `.doc` files have been converted into standard `.pdf` format so they open natively with full zoom, navigation, and search directly inside the Antigravity IDE.

| Seq | Document Name | Description | Key Focus Topics |
| :--- | :--- | :--- | :--- |
| `01` | [01_primer_UVM_Register_Abstraction_Layer.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/01_primer_UVM_Register_Abstraction_Layer.pdf) | UVM RAL Primer | Register abstraction, frontdoor/backdoor access, adapter classes |
| `02` | [02_svtb_training_labs.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/02_svtb_training_labs.pdf) | SVTB Training Labs *(Converted from .doc)* | SystemVerilog testbench OOP, interfaces, virtual interfaces, mailboxes |
| `03` | [03_VC_SpyGlass_Lint_and_CDC_Guide.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/03_VC_SpyGlass_Lint_and_CDC_Guide.pdf) | VC SpyGlass Lint & CDC Guide | RTL static signoff, lint rules, clock domain crossing (CDC) |
| `04` | [04_WishBone_b3_Bus_Specification.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/04_WishBone_b3_Bus_Specification.pdf) | Wishbone B3 Specification | Open bus architecture, handshakes, standard bus cycles, pipelining |
| `05` | [05_vcsSwift_cosimulation_manual.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/05_vcsSwift_cosimulation_manual.pdf) | VCS Swift Co-simulation Manual *(Converted from .doc)* | Hardware modeling, SmartModel integration, Swift cosimulation |
| `06` | [06_AEware_Application_Note_for_FSM_Livelock.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/06_AEware_Application_Note_for_FSM_Livelock.pdf) | FSM Livelock Detection | Formal detection of unreachable states, livelocks, and deadlocks |
| `07` | [07_AEware_compute_formal_core_all_UG.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/07_AEware_compute_formal_core_all_UG.pdf) | AEware Compute Formal Core UG | Formal property checking engines and assertion analysis |
| `08` | [08_AEware_compute_fta_all_UG.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/08_AEware_compute_fta_all_UG.pdf) | AEware Compute FTA UG | Formal timing and protocol analysis |
| `09` | [09_oc_ethernet_core_architecture.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/09_oc_ethernet_core_architecture.pdf) | OpenCores Ethernet MAC Architecture | 10/100 Mbps MAC RTL architecture, DMA engine, MII interface |
| `10` | [10_UTF_Commands_Doxygen.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/10_UTF_Commands_Doxygen.pdf) | UTF Commands & Doxygen Reference | Unit testing framework command reference |
| `11` | [11_CBug_GUTS_instructions.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/11_CBug_GUTS_instructions.pdf) | CBug GUTS Instructions *(Converted from .doc)* | C/C++ testbench integration with VCS simulation engine |
| `12` | [12_CBug_GUTS_instructions_PLI.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/12_CBug_GUTS_instructions_PLI.pdf) | CBug PLI Instructions *(Converted from .doc)* | Programming Language Interface (PLI/VPI) debugging in VCS |
| `13` | [13_wb_dma_architecture.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/13_wb_dma_architecture.pdf) | Wishbone DMA Architecture *(Converted from .doc)* | Direct Memory Access controller design, channels, FIFO buffers |
| `14` | [14_wb_design.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/14_wb_design.pdf) | Wishbone SoC Interconnect *(Converted from .doc)* | Interconnect topologies, crossbars, arbiters, wait-state logic |
| `15` | [15_regfile.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/15_regfile.pdf) | Register File Architecture *(Converted from .doc)* | Multi-port register file design, read/write hazard handling |
| `16` | [16_wbif_wishbone_interface.pdf](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/16_wbif_wishbone_interface.pdf) | Wishbone Interface IP *(Converted from .doc)* | Master/slave interface wrappers, bridge logic, timing constraints |

---

## 2. Example Testbench Collateral

Complete simulation collateral has been partitioned into 4 structured subdirectories inside [`Example_Testbench_Code/`](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/Example_Testbench_Code):

1. **[`01_DUT_RTL_Design/`](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/Example_Testbench_Code/01_DUT_RTL_Design)**: Verilog and VHDL Design Under Test source files (`dut.v.pdf`, `top.v.pdf`, DesignWare synchronous FIFO controllers `dw_fifo_s1_sf.v.pdf`, and RAM blocks).
2. **[`02_Verification_Environment/`](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/Example_Testbench_Code/02_Verification_Environment)**: Verification transactors, scoreboards, monitors, generators, and interfaces (`env.vr.pdf`, `intel_mgmt_master.vr.pdf`, `scoreboard.vr.pdf`).
3. **[`03_Test_Scenarios_and_Suites/`](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/Example_Testbench_Code/03_Test_Scenarios_and_Suites)**: Directed and constrained-random test cases (`test_00_trivial.vr.pdf` to `test_04_error_injection.vr.pdf`).
4. **[`04_Makefiles_and_Simulation_Setup/`](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS/Example_Testbench_Code/04_Makefiles_and_Simulation_Setup)**: Simulation execution scripts, compile flags, `synopsys_sim.setup`, and regression run scripts.
